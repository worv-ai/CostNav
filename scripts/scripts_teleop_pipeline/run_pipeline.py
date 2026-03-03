import argparse
import os
import signal
import subprocess
import sys
import zipfile
from datetime import datetime
import math
from typing import List

from dotenv import load_dotenv


SCRIPT_DIR = os.path.dirname(__file__)
ENV_PATH = os.path.join(SCRIPT_DIR, ".env")
load_dotenv(ENV_PATH)
_AUTH_SUB = None
_FIXER_RAN_FLAG = os.path.join(SCRIPT_DIR, ".fixer_ran")


def _expand_env_vars(keys: List[str]) -> None:
    for key in keys:
        val = os.getenv(key)
        if val:
            os.environ[key] = os.path.expandvars(val)


_expand_env_vars(
    [
        "DEFAULT_MAP_PATH",
        "MESH_PATH",
        "MESH_USD_CACHE_DIR",
        "SAVE_MAP_PATH",
        "ROBOT_USD_PATH",
        "ROS_DISTRO",
        "ROS_ROOT",
        "ROS_SETUP_SCRIPT",
        "ROS_SETUP_ZSH",
        "ROS_PYTHON_SITE_PACKAGES",
        "ISAAC_SIM_PYTHON",
        "JOYSTICK_TELEOP_CONFIG",
    ]
)


def _is_omniverse_path(path: str) -> bool:
    return isinstance(path, str) and path.startswith("omniverse://")


def _path_exists(path: str) -> bool:
    if _is_omniverse_path(path):
        return True
    return os.path.exists(path)


def parse_args():
    parser = argparse.ArgumentParser(description="Teleop Pipeline (USDZ → Isaac Sim → Teleop)")
    parser.add_argument("--map", type=str, default=os.getenv("DEFAULT_MAP_PATH"), help="Path to USDZ map")
    parser.add_argument("--mesh", type=str, default=os.getenv("MESH_PATH"), help="Optional mesh path (PLY/OBJ/USD)")
    parser.add_argument(
        "--mesh-mode",
        type=str,
        choices=["auto", "skip"],
        default=os.getenv("MESH_MODE", "skip"),
        help="Mesh handling mode",
    )
    parser.add_argument("--mesh-prim-path", type=str, default=os.getenv("MESH_PRIM_PATH", "/World/NvbloxMesh"))
    parser.add_argument("--mesh-offset", type=str, default=os.getenv("MESH_OFFSET", "0,0,0"))
    parser.add_argument("--mesh-rot", type=str, default=os.getenv("MESH_ROT", "0,0,0"))
    parser.add_argument("--mesh-scale", type=str, default=os.getenv("MESH_SCALE", "1,1,1"))
    parser.add_argument(
        "--mesh-cache",
        type=str,
        default=os.getenv("MESH_USD_CACHE_DIR") or os.path.join(SCRIPT_DIR, "_mesh_cache"),
        help="Cache dir for converted mesh USD",
    )
    parser.add_argument(
        "--mesh-force-convert",
        action="store_true",
        default=os.getenv("MESH_FORCE_CONVERT", "0") == "1",
        help="Force mesh conversion",
    )
    parser.add_argument(
        "--save-map",
        type=str,
        default=os.getenv("SAVE_MAP_PATH", ""),
        help="Save stage path after mesh import",
    )
    parser.add_argument(
        "--save-map-on-start",
        action="store_true",
        default=os.getenv("SAVE_MAP_ON_START", "0") == "1",
        help="Auto save stage after mesh import",
    )
    parser.add_argument("--robot", type=str, default=os.getenv("ROBOT_USD_PATH"), help="Path to robot USD")
    parser.add_argument(
        "--teleop",
        type=str,
        choices=["joystick", "keyboard", "none"],
        default=os.getenv("TELEOP_MODE", "joystick"),
    )
    parser.add_argument(
        "--keyboard-teleop-cmd",
        type=str,
        default=os.getenv("KEYBOARD_TELEOP_CMD", "ros2 run teleop_twist_keyboard teleop_twist_keyboard"),
    )
    parser.add_argument(
        "--joystick-teleop-config",
        type=str,
        default=os.getenv("JOYSTICK_TELEOP_CONFIG", os.path.join(SCRIPT_DIR, "teleop_twist_joy.yaml")),
    )
    parser.add_argument(
        "--joystick-teleop-cmd",
        type=str,
        default=os.getenv("JOYSTICK_TELEOP_CMD", ""),
    )
    parser.add_argument("--headless", action="store_true", default=False)
    parser.add_argument("--capture-dir", type=str, default=os.getenv("CAPTURE_DIR", ""))
    parser.add_argument(
        "--capture-camera",
        type=str,
        default=os.getenv("CAPTURE_CAMERA_PATH", "/World/Segway_E1_ROS2/base_link/sensor_link/rgb_left"),
    )
    parser.add_argument("--capture-res", type=str, default=os.getenv("CAPTURE_RES", "1920,1080"))
    parser.add_argument("--capture-every", type=int, default=int(os.getenv("CAPTURE_EVERY", "1")))
    parser.add_argument("--capture-max-frames", type=int, default=int(os.getenv("CAPTURE_MAX_FRAMES", "0")))
    return parser.parse_args()


args = parse_args()

# Map path must be provided (no implicit base path).
if not args.map:
    print("[Error] --map is required. Provide a USDZ path (or set DEFAULT_MAP_PATH).")
    sys.exit(1)
if not _path_exists(args.map):
    print(f"[Error] Map not found: {args.map}")
    sys.exit(1)

# Isaac Sim 앱 시작 (모든 Isaac Sim 모듈 임포트보다 먼저)
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

from omni.isaac.core.world import World
from omni.isaac.core.utils.extensions import enable_extension
import omni.timeline
import omni.usd
import carb
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema

from mesh_utils import ensure_usd_mesh
from teleop_utils import (
    build_joystick_teleop_cmd,
    detect_joystick_device,
    launch_background_teleop,
    launch_terminal_teleop,
)


ROS2_BRIDGE_EXTENSIONS = ("isaacsim.ros2.bridge", "omni.isaac.ros2_bridge")
NUREC_EXTENSIONS = ("omni.volume", "omni.usd.schema.omni_nurec_types")


def _usdz_contains_nurec(path: str) -> bool:
    if not path or not path.lower().endswith(".usdz"):
        return False
    try:
        with zipfile.ZipFile(path, "r") as zf:
            return any(name.lower().endswith(".nurec") for name in zf.namelist())
    except Exception as exc:
        carb.log_warn(f"[Pipeline] USDZ scan failed: {exc}")
        return False


def _enable_nurec_extensions_if_needed(map_path: str) -> None:
    if not _usdz_contains_nurec(map_path):
        return
    for ext_name in NUREC_EXTENSIONS:
        if enable_extension(ext_name):
            carb.log_info(f"[Pipeline] Enabled NuRec extension: {ext_name}")
        else:
            carb.log_warn(f"[Pipeline] NuRec extension not available: {ext_name}")


def _enable_ros2_bridge_extension():
    for ext_name in ROS2_BRIDGE_EXTENSIONS:
        if enable_extension(ext_name):
            carb.log_info(f"[Pipeline] Enabled ROS2 bridge extension: {ext_name}")
            return ext_name
    carb.log_error("[Pipeline] Failed to enable ROS2 bridge extension.")
    return None


def _nucleus_login_from_env() -> bool:
    server = os.getenv("OMNI_SERVER")
    user = os.getenv("OMNI_USER")
    password = os.getenv("OMNI_PASS")

    if not server or not user or not password:
        carb.log_warn("[Pipeline] OMNI_SERVER/OMNI_USER/OMNI_PASS not set. Skipping Nucleus login.")
        return False

    if not server.startswith("omniverse://"):
        server = f"omniverse://{server}"

    try:
        import omni.client
    except Exception as exc:
        carb.log_error(f"[Pipeline] omni.client import failed: {exc}")
        return False

    try:
        omni.client.initialize()
    except Exception:
        pass

    try:
        server_host = omni.client.break_url(server).host
    except Exception:
        server_host = None

    try:
        def _auth_cb(url_prefix: str):
            try:
                url = omni.client.break_url(url_prefix)
            except Exception:
                return None
            if url.scheme and url.scheme != "omniverse":
                return None
            if server_host and url.host and url.host != server_host:
                return None
            return (user, password)

        global _AUTH_SUB
        _AUTH_SUB = omni.client.register_authentication_callback(_auth_cb)

        result, _ = omni.client.stat(server)
        if result == omni.client.Result.OK:
            carb.log_info(f"[Pipeline] Nucleus login success: {server}")
            return True
        carb.log_error(f"[Pipeline] Nucleus login failed: {server} (result={result})")
        return False
    except Exception as exc:
        carb.log_error(f"[Pipeline] Nucleus login exception: {exc}")
        return False


def _parse_vec3(value: str, default: List[float]) -> List[float]:
    if not value:
        return default
    parts = [p.strip() for p in value.split(",") if p.strip()]
    if not parts:
        return default
    nums = [float(p) for p in parts]
    if len(nums) == 1:
        return [nums[0], nums[0], nums[0]]
    if len(nums) >= 3:
        return [nums[0], nums[1], nums[2]]
    while len(nums) < 3:
        nums.append(default[len(nums)])
    return nums


def _parse_quat(value: str) -> List[float] | None:
    if not value:
        return None
    parts = [p.strip() for p in value.split(",") if p.strip()]
    if len(parts) != 4:
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def _quat_from_euler_deg(roll: float, pitch: float, yaw: float) -> List[float]:
    r = math.radians(roll)
    p = math.radians(pitch)
    y = math.radians(yaw)
    cr = math.cos(r / 2.0)
    sr = math.sin(r / 2.0)
    cp = math.cos(p / 2.0)
    sp = math.sin(p / 2.0)
    cy = math.cos(y / 2.0)
    sy = math.sin(y / 2.0)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yv = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, yv, z]


def _parse_res(value: str, default=(1920, 1080)) -> tuple[int, int]:
    if not value:
        return default
    parts = [p.strip() for p in value.split(",") if p.strip()]
    if len(parts) < 2:
        return default
    try:
        return int(parts[0]), int(parts[1])
    except ValueError:
        return default


def _load_map(map_path: str) -> None:
    if not map_path:
        carb.log_warn("[Pipeline] No map path provided.")
        return
    if not _path_exists(map_path):
        carb.log_error(f"[Pipeline] Map not found: {map_path}")
        return
    import omni.isaac.core.utils.stage as stage_utils
    stage_utils.add_reference_to_stage(usd_path=map_path, prim_path="/World/Map")
    carb.log_info(f"[Pipeline] Loaded map: {map_path}")


def _map_has_mesh_prim(stage, root_path: str = "/World/Map") -> bool:
    if stage is None:
        return False
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim or not root_prim.IsValid():
        return False
    try:
        root_prim.Load()
    except Exception:
        pass
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Mesh) or prim.GetTypeName() == "Mesh":
            return True
    return False


def _set_prim_invisible(stage, prim_path: str) -> bool:
    if stage is None or not prim_path:
        return False
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    try:
        imageable = UsdGeom.Imageable(prim)
        imageable.MakeInvisible()
        return True
    except Exception:
        return False


def _hide_mesh_geometry(stage, root_path: str = "/World/Map") -> int:
    if stage is None:
        return 0
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim or not root_prim.IsValid():
        return 0
    try:
        root_prim.Load()
    except Exception:
        pass
    hidden = set()
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Mesh) or prim.GetTypeName() == "Mesh":
            target = prim
            parent = prim.GetParent()
            if parent and parent.IsValid() and parent.IsA(UsdGeom.Xform):
                parent_name = (parent.GetName() or "").lower()
                if "mesh" in parent_name or "nvblox" in parent_name:
                    target = parent
            try:
                UsdGeom.Imageable(target).MakeInvisible()
                hidden.add(target.GetPath().pathString)
            except Exception:
                pass
    return len(hidden)


def _apply_collider(stage, root_path: str) -> None:
    for prim in stage.Traverse():
        if not prim.GetPath().pathString.startswith(root_path):
            continue
        if prim.GetTypeName() != "Mesh":
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        PhysxSchema.PhysxCollisionAPI.Apply(prim)


def _add_mesh(usd_path: str, prim_path: str, offset, rot, scale) -> bool:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        carb.log_error("[Pipeline] No USD stage available.")
        return False

    xform = UsdGeom.Xform.Define(stage, prim_path)
    prim = xform.GetPrim()
    prim.GetReferences().AddReference(usd_path)

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*offset))
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(*rot))
    xformable.AddScaleOp().Set(Gf.Vec3f(*scale))

    _apply_collider(stage, prim_path)
    carb.log_info(f"[Pipeline] Mesh added: {usd_path} → {prim_path}")
    return True


def _save_stage(path: str) -> None:
    if not path:
        return
    try:
        omni.usd.get_context().save_as_stage(path)
        carb.log_info(f"[Pipeline] Stage saved: {path}")
    except Exception as exc:
        carb.log_error(f"[Pipeline] Failed to save stage: {exc}")


def _prepare_trial_dirs(base_dir: str) -> tuple[str, str, str]:
    if not base_dir:
        return "", "", ""
    os.makedirs(base_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    trial_name = f"teleop_{stamp}"
    trial_dir = os.path.join(base_dir, trial_name)
    suffix = 1
    while os.path.exists(trial_dir):
        trial_dir = os.path.join(base_dir, f"{trial_name}_{suffix}")
        suffix += 1
    capture_dir = os.path.join(trial_dir, "capture")
    fixer_dir = os.path.join(trial_dir, "fixer")
    os.makedirs(capture_dir, exist_ok=False)
    os.makedirs(fixer_dir, exist_ok=False)
    return trial_dir, capture_dir, fixer_dir


def _run_fixer_to_video(input_dir: str, output_dir: str | None = None, video_dir: str | None = None) -> bool:
    fixer_script = os.path.join(SCRIPT_DIR, "fixer_to_video.sh")
    if not os.path.isfile(fixer_script) or not os.access(fixer_script, os.X_OK):
        carb.log_warn("[Pipeline] fixer_to_video.sh not found or not executable. Skipping Fixer.")
        return False

    if not input_dir or not os.path.isdir(input_dir):
        carb.log_warn("[Pipeline] Fixer input dir is missing. Skipping Fixer.")
        return False

    args = [fixer_script, "--input", input_dir]
    out_dir = output_dir or os.getenv("FIXER_OUT_DIR") or ""
    if out_dir:
        args += ["--output", out_dir]
    vid_dir = video_dir or os.getenv("FIXER_VIDEO_DIR") or ""
    if vid_dir:
        args += ["--video-dir", vid_dir]
    fixer_fps = os.getenv("FIXER_FPS")
    if fixer_fps:
        args += ["--fps", fixer_fps]
    fixer_timestep = os.getenv("FIXER_TIMESTEP")
    if fixer_timestep:
        args += ["--timestep", fixer_timestep]

    carb.log_info(f"[Pipeline] Running Fixer: {args}")
    result = subprocess.run(args, check=False)
    if result.returncode == 0:
        try:
            with open(_FIXER_RAN_FLAG, "w") as f:
                f.write("ok\n")
        except Exception:
            pass
        carb.log_info("[Pipeline] Fixer completed.")
        return True
    carb.log_error(f"[Pipeline] Fixer failed (exit={result.returncode}).")
    return False


def main():
    _enable_ros2_bridge_extension()

    # World 생성
    world = World(stage_units_in_meters=1.0)
    simulation_app.update()

    _nucleus_login_from_env()
    _enable_nurec_extensions_if_needed(args.map)
    simulation_app.update()

    # Map 로드
    if args.map:
        _load_map(args.map)

    # Mesh 처리
    mesh_mode = (args.mesh_mode or "skip").lower()
    if mesh_mode not in {"auto", "skip"}:
        carb.log_error("[Pipeline] Invalid mesh mode. Use auto or skip.")
        sys.exit(1)

    stage = omni.usd.get_context().get_stage()
    map_has_mesh = _map_has_mesh_prim(stage, root_path="/World/Map")

    if mesh_mode == "auto":
        if not args.mesh:
            carb.log_error("[Pipeline] --mesh (or MESH_PATH) is required for auto mode.")
            sys.exit(1)
        if map_has_mesh:
            carb.log_error("[Pipeline] Map already contains mesh geometry. Auto import disabled to prevent duplicates.")
            sys.exit(1)
        if not _path_exists(args.mesh):
            carb.log_error(f"[Pipeline] Mesh not found: {args.mesh}")
            sys.exit(1)

        mesh_usd = ensure_usd_mesh(args.mesh, args.mesh_cache, force=args.mesh_force_convert)
        if not mesh_usd:
            carb.log_error(
                "[Pipeline] Mesh auto-import failed. If the mesh axes are not aligned, align the map manually and run with --mesh-mode skip."
            )
            sys.exit(1)

        offset = _parse_vec3(args.mesh_offset, [0.0, 0.0, 0.0])
        rot = _parse_vec3(args.mesh_rot, [0.0, 0.0, 0.0])
        scale = _parse_vec3(args.mesh_scale, [1.0, 1.0, 1.0])
        if not _add_mesh(mesh_usd, args.mesh_prim_path, offset, rot, scale):
            carb.log_error(
                "[Pipeline] Mesh auto-import failed. If the mesh axes are not aligned, align the map manually and run with --mesh-mode skip."
            )
            sys.exit(1)
        _set_prim_invisible(stage, args.mesh_prim_path)

        if args.save_map_on_start or args.save_map:
            save_path = args.save_map or os.getenv("SAVE_MAP_PATH", "")
            _save_stage(save_path)

    elif mesh_mode == "skip":
        if not map_has_mesh:
            carb.log_error(
                "[Pipeline] No mesh geometry found in the map. Use --mesh-mode auto with MESH_PATH, or use an already-aligned map."
            )
            sys.exit(1)
        hidden = _hide_mesh_geometry(stage, root_path="/World/Map")
        if hidden:
            carb.log_info(f"[Pipeline] Hid mesh geometry: {hidden} prim(s).")
        carb.log_info("[Pipeline] Mesh import skipped.")

    # 로봇 소환
    if args.robot and _path_exists(args.robot):
        import omni.isaac.core.utils.stage as stage_utils
        from omni.isaac.core.robots import Robot
        stage_utils.add_reference_to_stage(usd_path=args.robot, prim_path="/World/Segway_E1_ROS2")
        pos = _parse_vec3(os.getenv("ROBOT_SPAWN_POS", "0,0,0"), [0.0, 0.0, 0.0])
        quat = _parse_quat(os.getenv("ROBOT_SPAWN_QUAT", ""))
        if quat is None:
            rot = os.getenv("ROBOT_SPAWN_ROT", "")
            if rot:
                rpy = _parse_vec3(rot, [0.0, 0.0, 0.0])
                quat = _quat_from_euler_deg(rpy[0], rpy[1], rpy[2])
        if quat is None:
            yaw = os.getenv("ROBOT_SPAWN_YAW", "")
            if yaw:
                try:
                    quat = _quat_from_euler_deg(0.0, 0.0, float(yaw))
                except ValueError:
                    quat = None
        world.scene.add(
            Robot(
                prim_path="/World/Segway_E1_ROS2",
                name="segway_e1",
                position=pos,
                orientation=quat,
            )
        )

    # Teleop 실행
    teleop_proc = None
    teleop_mode = (args.teleop or "joystick").lower()
    if teleop_mode == "joystick":
        joystick_device = None
        if not args.joystick_teleop_cmd:
            existing_id = os.getenv("XBOX_ID")
            joystick_device, detected_id = detect_joystick_device(existing_id)
            if detected_id and detected_id != existing_id:
                os.environ["XBOX_ID"] = detected_id
            if joystick_device:
                carb.log_info(f"[Pipeline] Joystick device: {joystick_device}")
        if args.joystick_teleop_cmd:
            teleop_cmd = args.joystick_teleop_cmd
        else:
            yaml_path = args.joystick_teleop_config
            if os.path.isdir(yaml_path):
                yaml_path = os.path.join(yaml_path, "teleop_twist_joy.yaml")
            teleop_cmd = build_joystick_teleop_cmd(yaml_path, device=joystick_device)
        teleop_proc = launch_background_teleop(teleop_cmd, "joystick")
    elif teleop_mode == "keyboard":
        launch_terminal_teleop(args.keyboard_teleop_cmd, "keyboard")
        carb.log_info("[Pipeline] Keyboard teleop is handled by external terminal.")
    else:
        carb.log_info("[Pipeline] Teleop disabled.")

    world.reset()
    carb.log_info(f"[Pipeline] Ready! Teleop mode: {teleop_mode}")

    timeline = omni.timeline.get_timeline_interface()
    if timeline is None:
        carb.log_warn("[Pipeline] Timeline interface not available. Running without play/stop control.")
    else:
        try:
            timeline.set_looping(True)
            start_time = timeline.get_start_time()
            end_time = timeline.get_end_time()
            if end_time <= start_time:
                timeline.set_end_time(start_time + 1000.0)
            timeline.set_current_time(start_time)
        except Exception:
            pass
        timeline.play()

    # Capture setup
    capture_enabled = bool(args.capture_dir)
    capture_every = max(1, int(args.capture_every))
    capture_max = max(0, int(args.capture_max_frames))
    captured = 0
    frame_idx = 0
    trial_dir = ""
    fixer_dir = ""

    if capture_enabled:
        trial_dir, capture_dir, fixer_dir = _prepare_trial_dirs(args.capture_dir)
        args.capture_dir = capture_dir
        os.environ["CAPTURE_DIR"] = capture_dir
        os.environ["FIXER_INPUT_DIR"] = capture_dir
        os.environ["FIXER_OUT_DIR"] = fixer_dir
        os.environ["FIXER_VIDEO_DIR"] = trial_dir
        import omni.replicator.core as rep

        os.makedirs(args.capture_dir, exist_ok=True)
        res = _parse_res(args.capture_res)
        render_product = rep.create.render_product(args.capture_camera, res)
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=args.capture_dir, rgb=True)
        writer.attach([render_product])
        carb.log_info(f"[Capture] Enabled: dir={args.capture_dir} cam={args.capture_camera} res={res}")
        carb.log_info(f"[Capture] every={capture_every} max_frames={capture_max or 'unlimited'}")

    was_playing = False
    stop_requested = False
    try:
        while simulation_app.is_running():
            if timeline is not None:
                if timeline.is_stopped():
                    if was_playing:
                        carb.log_info("[Pipeline] Timeline stopped by user. Exiting simulation loop.")
                        stop_requested = True
                        break
                    timeline.play()
                    simulation_app.update()
                    continue
                if not timeline.is_playing():
                    timeline.play()
                    simulation_app.update()
                    continue

            was_playing = True
            world.step(render=True)
            frame_idx += 1
            if capture_enabled:
                if frame_idx % capture_every == 0:
                    rep.orchestrator.step()
                    captured += 1
                    if capture_max and captured >= capture_max:
                        carb.log_info("[Capture] Reached capture limit. Stopping simulation.")
                        capture_enabled = False
                        stop_requested = True
                        if timeline is not None and not timeline.is_stopped():
                            timeline.stop()
                        break
    finally:
        if teleop_proc is not None and teleop_proc.poll() is None:
            try:
                os.killpg(teleop_proc.pid, signal.SIGTERM)
            except Exception:
                pass
        simulation_app.close()

    if stop_requested:
        run_fixer = os.getenv("RUN_FIXER_TO_VIDEO", "0") == "1"
        if run_fixer:
            input_dir = args.capture_dir or os.getenv("FIXER_INPUT_DIR") or os.getenv("CAPTURE_DIR") or ""
            _run_fixer_to_video(input_dir, output_dir=fixer_dir or None, video_dir=trial_dir or None)
        else:
            carb.log_info("[Pipeline] RUN_FIXER_TO_VIDEO disabled. Skipping Fixer on stop.")


if __name__ == "__main__":
    main()
