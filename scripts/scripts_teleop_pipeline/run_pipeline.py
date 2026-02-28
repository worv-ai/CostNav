import argparse
import os
import signal
import sys
from typing import List

from dotenv import load_dotenv


SCRIPT_DIR = os.path.dirname(__file__)
ENV_PATH = os.path.join(SCRIPT_DIR, ".env")
load_dotenv(ENV_PATH)


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
        choices=["auto", "manual", "skip"],
        default=os.getenv("MESH_MODE", "auto"),
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
import omni.usd
import carb
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema

from mesh_utils import ensure_usd_mesh
from teleop_utils import (
    build_joystick_teleop_cmd,
    detect_joystick_device,
    launch_background_teleop,
    launch_terminal_teleop,
)


ROS2_BRIDGE_EXTENSIONS = ("isaacsim.ros2.bridge", "omni.isaac.ros2_bridge")


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

    def _is_ok(result):
        try:
            return result == omni.client.Result.OK
        except Exception:
            return bool(result)

    try:
        if hasattr(omni.client, "login"):
            result = omni.client.login(server, user, password)
        elif hasattr(omni.client, "authenticate"):
            result = omni.client.authenticate(server, user, password)
        elif hasattr(omni.client, "set_credential"):
            result = omni.client.set_credential(server, user, password)
        else:
            carb.log_error("[Pipeline] omni.client has no known login/auth API.")
            return False

        if _is_ok(result):
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


def _apply_collider(stage, root_path: str) -> None:
    for prim in stage.Traverse():
        if not prim.GetPath().pathString.startswith(root_path):
            continue
        if prim.GetTypeName() != "Mesh":
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        PhysxSchema.PhysxCollisionAPI.Apply(prim)


def _add_mesh(usd_path: str, prim_path: str, offset, rot, scale) -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        carb.log_error("[Pipeline] No USD stage available.")
        return

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


def _save_stage(path: str) -> None:
    if not path:
        return
    try:
        omni.usd.get_context().save_as_stage(path)
        carb.log_info(f"[Pipeline] Stage saved: {path}")
    except Exception as exc:
        carb.log_error(f"[Pipeline] Failed to save stage: {exc}")


def main():
    _enable_ros2_bridge_extension()

    # World 생성
    world = World(stage_units_in_meters=1.0)
    simulation_app.update()

    _nucleus_login_from_env()

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
            teleop_cmd = build_joystick_teleop_cmd(args.joystick_teleop_config, device=joystick_device)
        teleop_proc = launch_background_teleop(teleop_cmd, "joystick")
    elif teleop_mode == "keyboard":
        launch_terminal_teleop(args.keyboard_teleop_cmd, "keyboard")
        carb.log_info("[Pipeline] Keyboard teleop is handled by external terminal.")
    else:
        carb.log_info("[Pipeline] Teleop disabled.")

    # Map 로드
    if args.map:
        _load_map(args.map)

    # Mesh 처리
    mesh_mode = (args.mesh_mode or "auto").lower()
    if mesh_mode == "manual":
        carb.log_warn("[Pipeline] Mesh mode is manual. Import and align mesh in Isaac Sim UI.")
    elif mesh_mode == "auto" and args.mesh:
        mesh_usd = ensure_usd_mesh(args.mesh, args.mesh_cache, force=args.mesh_force_convert)
        if mesh_usd:
            offset = _parse_vec3(args.mesh_offset, [0.0, 0.0, 0.0])
            rot = _parse_vec3(args.mesh_rot, [0.0, 0.0, 0.0])
            scale = _parse_vec3(args.mesh_scale, [1.0, 1.0, 1.0])
            _add_mesh(mesh_usd, args.mesh_prim_path, offset, rot, scale)
            if args.save_map_on_start or args.save_map:
                save_path = args.save_map or os.getenv("SAVE_MAP_PATH", "")
                _save_stage(save_path)
        else:
            carb.log_warn("[Pipeline] Mesh conversion failed. Skipping mesh import.")
    elif mesh_mode == "skip":
        carb.log_info("[Pipeline] Mesh import skipped.")

    # 로봇 소환
    if args.robot and _path_exists(args.robot):
        import omni.isaac.core.utils.stage as stage_utils
        from omni.isaac.core.robots import Robot
        stage_utils.add_reference_to_stage(usd_path=args.robot, prim_path="/World/E1_Robot")
        world.scene.add(Robot(prim_path="/World/E1_Robot", name="e1_robot", position=_parse_vec3(os.getenv("ROBOT_SPAWN_POS", "0,0,0"), [0.0, 0.0, 0.0])))

    world.reset()
    carb.log_info(f"[Pipeline] Ready! Teleop mode: {teleop_mode}")

    # Capture setup
    capture_enabled = bool(args.capture_dir)
    capture_every = max(1, int(args.capture_every))
    capture_max = max(0, int(args.capture_max_frames))
    captured = 0
    frame_idx = 0

    if capture_enabled:
        import omni.replicator.core as rep

        os.makedirs(args.capture_dir, exist_ok=True)
        res = _parse_res(args.capture_res)
        render_product = rep.create.render_product(args.capture_camera, res)
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=args.capture_dir, rgb=True)
        writer.attach([render_product])
        carb.log_info(f"[Capture] Enabled: dir={args.capture_dir} cam={args.capture_camera} res={res}")
        carb.log_info(f"[Capture] every={capture_every} max_frames={capture_max or 'unlimited'}")

    try:
        while simulation_app.is_running():
            world.step(render=True)
            frame_idx += 1
            if capture_enabled:
                if frame_idx % capture_every == 0:
                    rep.orchestrator.step()
                    captured += 1
                    if capture_max and captured >= capture_max:
                        carb.log_info("[Capture] Reached capture limit. Stopping capture.")
                        capture_enabled = False
    finally:
        if teleop_proc is not None and teleop_proc.poll() is None:
            try:
                os.killpg(teleop_proc.pid, signal.SIGTERM)
            except Exception:
                pass
        simulation_app.close()


if __name__ == "__main__":
    main()
