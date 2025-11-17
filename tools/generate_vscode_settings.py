#!/usr/bin/env python3
"""Generate VS Code settings with Isaac Lab + CostNav search paths."""

from __future__ import annotations

import os
import textwrap
import time
from pathlib import Path
from typing import Iterable, List, Optional

import typer
from rich.console import Console
from rich.progress import (
    Progress,
    SpinnerColumn,
    TextColumn,
    BarColumn,
    TaskProgressColumn,
)

VSCODE_TEMPLATE = """
{
    "editor.rulers": [79],
    "python.languageServer": "Pylance",
    "python.jediEnabled": false,
    "python.defaultInterpreterPath": "PYTHON.DEFAULTINTERPRETERPATH",
    "python.analysis.extraPaths": [
PYTHON.ANALYSIS.EXTRAPATHS
    ],
    "python.autoComplete.extraPaths": [
PYTHON.AUTOCOMPLETE.EXTRAPATHS
    ],
    "python.envFile": "${workspaceFolder}/.vscode/.python.env",
    "terminal.integrated.env.linux": {
        "ISAAC_PATH": "ISAAC.PATH",
        "CARB_APP_PATH": "CARB.APP.PATH",
        "EXP_PATH": "EXP.PATH",
        "PYTHONPATH": "PYTHON.PATH.ENV"
    }
}
"""

PYTHON_ENV_TEMPLATE = """ISAAC_PATH=ISAAC.PATH
CARB_APP_PATH=CARB.APP.PATH
EXP_PATH=EXP.PATH

PYTHONPATH=${ISAAC_PATH}/kit/kernel/py
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/kit/exts
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/kit/extscore
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/exts
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/extscache
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/extsDeprecated
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/extsUser
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/kit/python/lib/python3.11/site-packages

LD_LIBRARY_PATH=${ISAAC_PATH}/kit/lib
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ISAAC_PATH}/kit/plugins
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ISAAC_PATH}/kit/exts
"""


def _find_isaac_sim_path() -> Path | None:
    """Find Isaac Sim installation path."""
    isaac_path = os.environ.get("ISAAC_PATH") or os.environ.get("ISAAC_SIM_PATH")
    if isaac_path and Path(isaac_path).exists():
        return Path(isaac_path)

    common_paths = [
        Path("/isaac-sim"),
        Path.home() / ".local/share/ov/pkg/isaac-sim-*",
    ]

    for path_pattern in common_paths:
        if "*" in str(path_pattern):
            import glob

            matches = glob.glob(str(path_pattern))
            if matches:
                return Path(sorted(matches)[-1])
        elif path_pattern.exists():
            return path_pattern

    return None


def _find_isaaclab_source_paths() -> List[str]:
    """Find isaaclab source paths from the installed package.

    Returns:
        List of paths to isaaclab source modules (isaaclab, isaaclab_assets, etc.)
    """
    paths: List[str] = []

    try:
        # Try to import isaaclab to find its installation location
        import isaaclab

        # Get the isaaclab package location
        isaaclab_init = Path(isaaclab.__file__)
        isaaclab_pkg_dir = isaaclab_init.parent

        # Check if there's a 'source' subdirectory (installed package structure)
        source_dir = isaaclab_pkg_dir / "source"
        if source_dir.is_dir():
            # Add all subdirectories in source/ (isaaclab, isaaclab_assets, etc.)
            for subdir in source_dir.iterdir():
                if subdir.is_dir() and not subdir.name.startswith("_"):
                    paths.append(str(subdir))
        else:
            # If no source directory, add the package directory itself
            paths.append(str(isaaclab_pkg_dir))

    except (ImportError, AttributeError):
        # isaaclab not installed or not importable
        pass

    return paths


def _gather_extra_paths(repo_root: Path, isaac_sim_path: Path) -> List[str]:
    """Gather all Python paths for Isaac Sim, Omni Kit, and CostNav."""
    paths: List[str] = []

    kit_path = isaac_sim_path / "kit"
    if kit_path.exists():
        kernel_py = kit_path / "kernel" / "py"
        if kernel_py.is_dir():
            paths.append(str(kernel_py))

        for folder_name in ["exts", "extscore"]:
            folder = kit_path / folder_name
            if folder.is_dir():
                paths.append(str(folder))

    for folder_name in ["exts", "extscache", "extsDeprecated", "extsUser"]:
        folder = isaac_sim_path / folder_name
        if folder.is_dir():
            paths.append(str(folder))

    site_packages = (
        isaac_sim_path / "kit" / "python" / "lib" / "python3.11" / "site-packages"
    )
    if site_packages.is_dir():
        paths.append(str(site_packages))

    # Dynamically find isaaclab source paths from installed package
    isaaclab_paths = _find_isaaclab_source_paths()
    paths.extend(isaaclab_paths)

    for local_folder in ["costnav_isaaclab", "costnav"]:
        candidate = repo_root / local_folder
        if candidate.is_dir():
            paths.append(str(candidate))

    return paths


def _render_settings(isaac_sim_path: Path, extra_paths: Iterable[str]) -> str:
    """Render VS Code settings.json content."""
    interpreter = str(isaac_sim_path / "python.sh")

    entries = [f'"{path}"' for path in extra_paths]
    joined = ",\n".join(entries)
    joined = textwrap.indent(joined, " " * 8)

    pythonpath_parts = [
        "${env:ISAAC_PATH}/kit/kernel/py",
        "${env:ISAAC_PATH}/kit/exts",
        "${env:ISAAC_PATH}/kit/extscore",
        "${env:ISAAC_PATH}/exts",
        "${env:ISAAC_PATH}/extscache",
        "${env:ISAAC_PATH}/extsDeprecated",
        "${env:ISAAC_PATH}/extsUser",
        "${env:ISAAC_PATH}/kit/python/lib/python3.11/site-packages",
        "${env:PYTHONPATH}",
    ]
    pythonpath_env = ":".join(pythonpath_parts)

    settings = VSCODE_TEMPLATE
    settings = settings.replace("PYTHON.DEFAULTINTERPRETERPATH", interpreter)
    settings = settings.replace("PYTHON.ANALYSIS.EXTRAPATHS", joined)
    settings = settings.replace("PYTHON.AUTOCOMPLETE.EXTRAPATHS", joined)
    settings = settings.replace("ISAAC.PATH", str(isaac_sim_path))
    settings = settings.replace("CARB.APP.PATH", str(isaac_sim_path / "kit"))
    settings = settings.replace("EXP.PATH", str(isaac_sim_path / "apps"))
    settings = settings.replace("PYTHON.PATH.ENV", pythonpath_env)

    return settings


def _render_python_env(isaac_sim_path: Path) -> str:
    """Render .python.env content."""
    env_content = PYTHON_ENV_TEMPLATE
    env_content = env_content.replace("ISAAC.PATH", str(isaac_sim_path))
    env_content = env_content.replace("CARB.APP.PATH", str(isaac_sim_path / "kit"))
    env_content = env_content.replace("EXP.PATH", str(isaac_sim_path / "apps"))
    return env_content


app = typer.Typer(help="Generate VS Code settings for CostNav + Isaac Lab")
console = Console()


def main(
    isaac_sim: Optional[str] = typer.Option(
        None,
        "--isaac-sim",
        help=(
            "Path to Isaac Sim installation "
            "(default: auto-detect from ISAAC_PATH or /isaac-sim)"
        ),
    ),
    output: str = typer.Option(
        ".vscode/settings.json",
        "--output",
        help="Path to write settings.json",
    ),
    force: bool = typer.Option(
        False,
        "--force",
        help="Overwrite existing settings file",
    ),
) -> None:
    """Generate VS Code settings with Isaac Lab + CostNav search paths."""
    repo_root = Path(__file__).resolve().parents[1]
    timings = {}

    start_time = time.perf_counter()
    with Progress(
        TextColumn("  "),
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console,
    ) as progress:
        task = progress.add_task(
            description="[cyan]Locating Isaac Sim installation...",
            total=None,
        )

        if isaac_sim:
            isaac_sim_path = Path(isaac_sim)
            if not isaac_sim_path.exists():
                progress.stop()
                console.print(
                    f"[bold red]✗[/bold red] Isaac Sim path does not "
                    f"exist: {isaac_sim_path}"
                )
                raise typer.Exit(1)
        else:
            isaac_sim_path = _find_isaac_sim_path()
            if not isaac_sim_path:
                progress.stop()
                console.print(
                    "[bold red]✗[/bold red] Unable to locate "
                    "Isaac Sim installation."
                )
                console.print(
                    "  [yellow]💡 Hint:[/yellow] Please specify "
                    "--isaac-sim /path/to/isaac-sim"
                )
                raise typer.Exit(1)

        progress.update(task, completed=True)

    elapsed = time.perf_counter() - start_time
    timings["locate_isaac"] = elapsed
    console.print(
        f"[bold green]✓[/bold green] Found Isaac Sim: "
        f"[cyan]{isaac_sim_path}[/cyan] [dim]({elapsed:.3f}s)[/dim]"
    )

    output_path = Path(output)
    if output_path.exists() and not force:
        console.print(
            f"\n[yellow]⚠[/yellow]  VS Code settings already exist at "
            f"[cyan]{output_path}[/cyan]"
        )
        console.print("[dim]Use --force to overwrite[/dim]")
        raise typer.Exit(0)

    start_time = time.perf_counter()
    with Progress(
        TextColumn("  "),
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(),
        TaskProgressColumn(),
        console=console,
    ) as progress:
        task = progress.add_task(
            description="[cyan]Gathering Python paths...",
            total=100,
        )

        progress.update(task, advance=30)
        extra_paths = _gather_extra_paths(repo_root, isaac_sim_path)
        progress.update(task, advance=70)

    elapsed = time.perf_counter() - start_time
    timings["gather_paths"] = elapsed

    if not extra_paths:
        console.print(
            f"[yellow]⚠[/yellow]  Warning: No extra paths found. "
            f"[dim]({elapsed:.3f}s)[/dim]"
        )
    else:
        console.print(
            f"[bold green]✓[/bold green] Gathered {len(extra_paths)} "
            f"Python paths [dim]({elapsed:.3f}s)[/dim]"
        )

    with Progress(
        TextColumn("  "),
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console,
    ) as progress:
        task1 = progress.add_task(
            description="[cyan]Generating settings.json...",
            total=None,
        )
        start_time = time.perf_counter()
        settings = _render_settings(isaac_sim_path, extra_paths)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(settings)
        elapsed = time.perf_counter() - start_time
        timings["generate_settings"] = elapsed
        progress.update(task1, completed=True)

        task2 = progress.add_task(
            description="[cyan]Generating .python.env...",
            total=None
        )
        start_time = time.perf_counter()
        python_env_path = output_path.parent / ".python.env"
        python_env = _render_python_env(isaac_sim_path)
        python_env_path.write_text(python_env)
        elapsed = time.perf_counter() - start_time
        timings["generate_env"] = elapsed
        progress.update(task2, completed=True)

    console.print(
        f"[bold green]✓[/bold green] VS Code settings written to "
        f"[cyan]{output_path}[/cyan] "
        f"[dim]({timings['generate_settings']:.3f}s)[/dim]"
    )
    console.print(
        f"[bold green]✓[/bold green] Python environment file written to "
        f"[cyan]{python_env_path}[/cyan] "
        f"[dim]({timings['generate_env']:.3f}s)[/dim]"
    )


if __name__ == "__main__":
    typer.run(main)
