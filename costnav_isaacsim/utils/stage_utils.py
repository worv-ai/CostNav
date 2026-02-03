"""Stage loading helpers."""

import logging

logger = logging.getLogger("costnav_launch")


def open_stage(simulation_app, usd_path: str) -> None:
    """Open a USD stage and wait until it finishes loading."""
    import omni.usd
    from isaacsim.core.utils.stage import is_stage_loading

    logger.info("Loading: %s", usd_path)

    usd_context = omni.usd.get_context()
    result = usd_context.open_stage(usd_path)

    if not result:
        raise RuntimeError(f"Failed to open USD file: {usd_path}")

    while is_stage_loading():
        simulation_app.update()

    logger.info("Stage loaded successfully")
