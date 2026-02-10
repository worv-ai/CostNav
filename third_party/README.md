# Third-Party References

The folders in this directory mirror the external projects we draw inspiration or tooling from. They are checked out as git submodules but are kept separate from the main CostNav code so we can inspect upstream implementations without mixing their code into our core packages.

<h2 align="center"><em>Huge thanks to the wider robotics and simulation communities whose work powers these references.</em></h2>

## Layout

- `IsaacLab/` — reference checkout of NVIDIA's IsaacLab. Useful for reading upstream tasks, assets, and training utilities while building CostNav-specific environments.
- `urban-sim/` — reference checkout of the Metadriverse urban simulator, used for the COCO robot configuration and related assets.
- `PeopleAPI/` — reference checkout of the worv-ai PeopleAPI Isaac Sim extension for quick development while waiting on community extension updates.
- `FoodAssets/` — reference checkout of the worv-ai FoodAssets Isaac Sim extension, used as a curated asset library for food-related scenes.

## Usage

1. Initialize the submodules when you need to browse the upstream sources:
   ```bash
   git submodule update --init --recursive
   ```
2. Treat these directories as read-only references. All CostNav-specific code should live under `costnav_isaaclab/` or the appropriate top-level package.
3. When updating to newer upstream commits, use standard submodule commands (e.g., `git submodule update --remote`) and capture the resulting commit hashes in your PR description for traceability.

These references are intentionally excluded from Docker build contexts and install hooks; they exist only to provide context while developing CostNav.
