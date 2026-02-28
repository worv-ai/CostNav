import os
from pathlib import Path

import carb


USD_EXTS = {".usd", ".usdc", ".usda", ".usdz"}
MESH_EXTS = {".ply", ".obj", ".fbx", ".stl"}


def ensure_usd_mesh(mesh_path: str, cache_dir: str | None = None, force: bool = False) -> str | None:
    if not mesh_path:
        return None

    mesh_path = os.path.abspath(os.path.expandvars(mesh_path))
    ext = Path(mesh_path).suffix.lower()
    if ext in USD_EXTS:
        return mesh_path

    if ext not in MESH_EXTS:
        carb.log_error(f"[Mesh] Unsupported mesh extension: {ext}")
        return None

    if not cache_dir:
        cache_dir = os.path.join(os.path.dirname(mesh_path), "_usd_cache")

    os.makedirs(cache_dir, exist_ok=True)
    usd_path = os.path.join(cache_dir, Path(mesh_path).stem + ".usd")

    if os.path.exists(usd_path) and not force:
        carb.log_info(f"[Mesh] Using cached USD: {usd_path}")
        return usd_path

    try:
        import omni.kit.asset_converter as asset_converter
    except Exception as exc:
        carb.log_error(f"[Mesh] Asset converter not available: {exc}")
        return None

    carb.log_info(f"[Mesh] Converting {mesh_path} → {usd_path}")
    try:
        settings = asset_converter.AssetConverterSettings()
        settings.use_meter_as_world_unit = True
        settings.merge_mtl = True
        settings.convert_materials = True
        converter = asset_converter.AssetConverter()
        task = converter.create_converter_task(mesh_path, usd_path, settings)

        if hasattr(task, "wait_until_finished"):
            task.wait_until_finished()
        elif hasattr(task, "wait"):
            task.wait()

        if not os.path.exists(usd_path):
            carb.log_error("[Mesh] Conversion finished but USD not found.")
            return None

        carb.log_info(f"[Mesh] Conversion complete: {usd_path}")
        return usd_path
    except Exception as exc:
        carb.log_error(f"[Mesh] Conversion failed: {exc}")
        return None
