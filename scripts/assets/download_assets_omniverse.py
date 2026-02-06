#!/usr/bin/env python3
"""
Download Omniverse assets from the internal server to local tmp folder.

This script downloads all assets referenced in the codebase from
omniverse://10.50.2.21/ to ./assets/ maintaining the same directory structure.

It also parses USD files to find and download all dependencies (references,
sublayers, payloads, and texture assets).

"""

import os
import re
import sys
from pathlib import Path

# Try to bootstrap Isaac Sim environment if not already done
def setup_isaac_sim():
    """Setup Isaac Sim environment for omni.client access."""
    isaac_sim_path = os.environ.get("ISAAC_PATH", "/isaac-sim")

    # Add Isaac Sim paths if not already present
    paths_to_add = [
        os.path.join(isaac_sim_path, "kit", "kernel", "py"),
        os.path.join(isaac_sim_path, "kit", "plugins"),
        os.path.join(isaac_sim_path, "kit", "exts"),
        os.path.join(isaac_sim_path, "kit", "extscore"),
        os.path.join(isaac_sim_path, "exts"),
        os.path.join(isaac_sim_path, "extscache"),
    ]

    for p in paths_to_add:
        if os.path.exists(p) and p not in sys.path:
            sys.path.insert(0, p)


setup_isaac_sim()

# List of all Omniverse assets used in the codebase
OMNIVERSE_ASSETS = [
    "omniverse://10.50.2.21/Users/worv/costnav/foods/popcorn/popcorn.usd",
    "omniverse://10.50.2.21/Users/worv/costnav/street_sidewalk_segwaye1_Corrected.usd",
    "omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd",
]

OMNIVERSE_SERVER = "omniverse://10.50.2.21"
LOCAL_SERVER = "omniverse://localhost"
# Get the repository root (parent of scripts directory)
REPO_ROOT = Path(__file__).parent.parent.parent
OUTPUT_DIR = REPO_ROOT / "assets"


def download_asset(omni_client, url: str, output_dir: Path) -> bool:
    """Download a single asset from Omniverse to local filesystem."""
    # Extract relative path from URL
    relative_path = url.replace(f"{OMNIVERSE_SERVER}/", "")
    local_path = output_dir / relative_path
    
    # Create parent directories
    local_path.parent.mkdir(parents=True, exist_ok=True)
    
    print(f"Downloading: {url}")
    print(f"        To: {local_path}")
    
    # Read from Omniverse
    result, _, content = omni_client.read_file(url)
    
    if result != omni_client.Result.OK:
        print(f"  ERROR: Failed to read {url} - {result}")
        return False
    
    # Write to local file
    with open(local_path, "wb") as f:
        f.write(memoryview(content))
    
    print(f"  SUCCESS: Downloaded {len(content)} bytes")
    return True


def rewrite_references_to_localhost(local_path: Path) -> bool:
    """Rewrite Omniverse server references in a USD file to point to localhost.

    This modifies the downloaded USD file to replace references like
    omniverse://10.50.2.21/... with omniverse://localhost/...

    Args:
        local_path: Path to the local USD file

    Returns:
        True if the file was modified, False otherwise
    """
    if not local_path.exists():
        return False

    # Only process text-based USD files (USDA) for now
    # Binary USD files would need pxr library to modify safely
    suffix = local_path.suffix.lower()

    try:
        content = local_path.read_bytes()

        # Check if this is a text-based USD file (starts with #usda or contains readable text)
        is_text_based = content.startswith(b'#usda') or suffix == '.usda'

        if is_text_based:
            # Text-based USDA file - simple string replacement
            text_content = content.decode('utf-8')
            original_content = text_content

            # Replace the server URL with localhost
            text_content = text_content.replace(OMNIVERSE_SERVER, LOCAL_SERVER)

            if text_content != original_content:
                local_path.write_text(text_content, encoding='utf-8')
                print(f"  REWRITTEN: References updated to {LOCAL_SERVER}")
                return True
        else:
            # Binary USD file - try using pxr library
            try:
                from pxr import Sdf

                # Open the layer for editing
                layer = Sdf.Layer.FindOrOpen(str(local_path))
                if not layer:
                    return False

                modified = False

                # Function to rewrite a path
                def rewrite_path(path: str) -> str:
                    if path and OMNIVERSE_SERVER in path:
                        return path.replace(OMNIVERSE_SERVER, LOCAL_SERVER)
                    return path

                # Rewrite sublayer paths
                new_sublayers = []
                for sublayer in layer.subLayerPaths:
                    new_path = rewrite_path(sublayer)
                    new_sublayers.append(new_path)
                    if new_path != sublayer:
                        modified = True
                if modified:
                    layer.subLayerPaths = new_sublayers

                # Traverse and rewrite all prim specs
                def process_prim_spec(prim_spec):
                    nonlocal modified

                    # Rewrite references
                    if prim_spec.hasReferences:
                        for ref_list in [prim_spec.referenceList.prependedItems,
                                         prim_spec.referenceList.appendedItems,
                                         prim_spec.referenceList.explicitItems]:
                            for i, ref in enumerate(ref_list):
                                if ref.assetPath and OMNIVERSE_SERVER in ref.assetPath:
                                    new_path = rewrite_path(ref.assetPath)
                                    new_ref = Sdf.Reference(new_path, ref.primPath, ref.layerOffset, ref.customData)
                                    ref_list[i] = new_ref
                                    modified = True

                    # Rewrite payloads
                    if prim_spec.hasPayloads:
                        for payload_list in [prim_spec.payloadList.prependedItems,
                                             prim_spec.payloadList.appendedItems,
                                             prim_spec.payloadList.explicitItems]:
                            for i, payload in enumerate(payload_list):
                                if payload.assetPath and OMNIVERSE_SERVER in payload.assetPath:
                                    new_path = rewrite_path(payload.assetPath)
                                    new_payload = Sdf.Payload(new_path, payload.primPath, payload.layerOffset)
                                    payload_list[i] = new_payload
                                    modified = True

                    # Rewrite asset attributes
                    for attr_spec in prim_spec.attributes:
                        if attr_spec.typeName == Sdf.ValueTypeNames.Asset:
                            default_val = attr_spec.default
                            if default_val and hasattr(default_val, 'path') and default_val.path:
                                if OMNIVERSE_SERVER in default_val.path:
                                    new_path = rewrite_path(default_val.path)
                                    attr_spec.default = Sdf.AssetPath(new_path)
                                    modified = True

                    # Recurse into children
                    for child_spec in prim_spec.nameChildren:
                        process_prim_spec(child_spec)

                for prim_spec in layer.rootPrims:
                    process_prim_spec(prim_spec)

                if modified:
                    layer.Save()
                    print(f"  REWRITTEN: References updated to {LOCAL_SERVER}")
                    return True

            except ImportError:
                # pxr not available - try binary string replacement as last resort
                # This is risky for binary files but may work for simple cases
                if OMNIVERSE_SERVER.encode() in content:
                    # Only do this if the server strings are the same length
                    # to avoid corrupting the binary file
                    if len(OMNIVERSE_SERVER) == len(LOCAL_SERVER):
                        new_content = content.replace(
                            OMNIVERSE_SERVER.encode(),
                            LOCAL_SERVER.encode()
                        )
                        local_path.write_bytes(new_content)
                        print(f"  REWRITTEN: References updated to {LOCAL_SERVER} (binary)")
                        return True
                    else:
                        print(f"  WARNING: Cannot rewrite binary USD - server URL lengths differ")

            except Exception as e:
                print(f"  WARNING: Error rewriting references with pxr: {e}")

    except Exception as e:
        print(f"  WARNING: Could not rewrite references in {local_path}: {e}")

    return False


# Built-in materials and assets that should not be downloaded
# These are part of the Omniverse/Isaac Sim installation
SKIP_PATTERNS = [
    "OmniPBR.mdl",
    "OmniGlass.mdl",
    "OmniSurface.mdl",
    "OmniHair.mdl",
    "OmniEmissive.mdl",
    "OmniVolume.mdl",
    "NVIDIA/Materials/",
    "NVIDIA/Assets/",
    "isaac-sim/",
    "Isaac/",
]


def resolve_asset_path(base_url: str, asset_path: str) -> str | None:
    """Resolve an asset path relative to a base URL.

    Args:
        base_url: The URL of the file containing the reference
        asset_path: The path found in the USD file (may be relative or absolute)

    Returns:
        The resolved absolute URL, or None if it cannot be resolved
    """
    if not asset_path:
        return None

    # Skip built-in NVIDIA/Omniverse materials and assets
    for pattern in SKIP_PATTERNS:
        if pattern in asset_path:
            return None

    # Skip built-in/omniverse standard paths
    if asset_path.startswith(("omniverse://localhost/", "http://", "https://")):
        # Only process paths from our server
        if not asset_path.startswith(OMNIVERSE_SERVER):
            return None
        return asset_path

    # Handle absolute omniverse paths
    if asset_path.startswith("omniverse://"):
        return asset_path

    # Handle relative paths
    if asset_path.startswith("./") or asset_path.startswith("../") or not asset_path.startswith("/"):
        # Get the directory of the base file
        base_dir = "/".join(base_url.split("/")[:-1])
        # Resolve relative path
        if asset_path.startswith("./"):
            asset_path = asset_path[2:]

        # Handle ../ by going up directories
        parts = base_dir.split("/")
        path_parts = asset_path.split("/")

        for part in path_parts:
            if part == "..":
                if len(parts) > 3:  # Keep at least omniverse://server/
                    parts.pop()
            elif part and part != ".":
                parts.append(part)

        return "/".join(parts)

    # Absolute path on the same server
    if asset_path.startswith("/"):
        return f"{OMNIVERSE_SERVER}{asset_path}"

    return None


def extract_dependencies_with_pxr(local_path: Path, base_url: str) -> list[str]:
    """Extract dependencies using the pxr (OpenUSD) library.

    This properly handles binary USD files.
    """
    dependencies = []

    try:
        from pxr import Sdf
    except ImportError:
        print("  WARNING: pxr library not available, falling back to regex parsing")
        return []

    try:
        # Open the layer directly (not as a stage) to get raw asset paths
        # This avoids path resolution issues
        layer = Sdf.Layer.FindOrOpen(str(local_path))
        if not layer:
            print(f"  WARNING: Could not open USD layer for {local_path}")
            return []

        found_paths = set()

        # Get sublayer paths directly from the layer
        for sublayer_path in layer.subLayerPaths:
            if sublayer_path:
                found_paths.add(sublayer_path)
                print(f"    [pxr] Found sublayer: {sublayer_path}")

        # Recursively traverse the layer to find all asset paths
        def traverse_layer(sdf_layer):
            """Traverse a layer and collect all external references."""

            def process_prim_spec(prim_spec):
                if not prim_spec:
                    return

                # Get references
                if prim_spec.hasReferences:
                    for ref in prim_spec.referenceList.prependedItems:
                        if ref.assetPath:
                            found_paths.add(ref.assetPath)
                            print(f"    [pxr] Found reference: {ref.assetPath}")
                    for ref in prim_spec.referenceList.appendedItems:
                        if ref.assetPath:
                            found_paths.add(ref.assetPath)
                            print(f"    [pxr] Found reference: {ref.assetPath}")
                    for ref in prim_spec.referenceList.explicitItems:
                        if ref.assetPath:
                            found_paths.add(ref.assetPath)
                            print(f"    [pxr] Found reference: {ref.assetPath}")

                # Get payloads
                if prim_spec.hasPayloads:
                    for payload in prim_spec.payloadList.prependedItems:
                        if payload.assetPath:
                            found_paths.add(payload.assetPath)
                            print(f"    [pxr] Found payload: {payload.assetPath}")
                    for payload in prim_spec.payloadList.appendedItems:
                        if payload.assetPath:
                            found_paths.add(payload.assetPath)
                            print(f"    [pxr] Found payload: {payload.assetPath}")
                    for payload in prim_spec.payloadList.explicitItems:
                        if payload.assetPath:
                            found_paths.add(payload.assetPath)
                            print(f"    [pxr] Found payload: {payload.assetPath}")

                # Check attributes for asset paths (textures, etc.)
                for attr_spec in prim_spec.attributes:
                    if attr_spec.typeName == Sdf.ValueTypeNames.Asset:
                        default_val = attr_spec.default
                        if default_val and hasattr(default_val, 'path') and default_val.path:
                            found_paths.add(default_val.path)
                            print(f"    [pxr] Found asset attr: {default_val.path}")
                    elif attr_spec.typeName == Sdf.ValueTypeNames.AssetArray:
                        default_val = attr_spec.default
                        if default_val:
                            for asset in default_val:
                                if hasattr(asset, 'path') and asset.path:
                                    found_paths.add(asset.path)
                                    print(f"    [pxr] Found asset array item: {asset.path}")

                # Recurse into children
                for child_spec in prim_spec.nameChildren:
                    process_prim_spec(child_spec)

            # Start from root prims
            for prim_spec in sdf_layer.rootPrims:
                process_prim_spec(prim_spec)

        traverse_layer(layer)

        # Resolve all found paths
        for asset_path in found_paths:
            resolved = resolve_asset_path(base_url, asset_path)
            if resolved:
                dependencies.append(resolved)

    except Exception as e:
        print(f"  WARNING: Error extracting dependencies with pxr: {e}")
        import traceback
        traceback.print_exc()

    return dependencies


def extract_dependencies_with_regex(local_path: Path, base_url: str) -> list[str]:
    """Parse a USD file using regex (fallback for when pxr is not available).

    This finds references, sublayers, payloads, and asset paths (textures, etc.)
    Works best with text-based USDA files.
    """
    dependencies = []

    try:
        content = local_path.read_bytes()
        # Try to decode as text, but also search in binary
        text_content = content.decode('utf-8', errors='ignore')
    except Exception as e:
        print(f"  WARNING: Could not read {local_path}: {e}")
        return dependencies

    # Patterns to find in USD files (works for both USDA text format and finding strings in binary)
    # These patterns capture various USD reference syntaxes
    patterns = [
        # References: @path/to/file.usd@
        r'@([^@\x00]+\.(?:usd|usda|usdc|png|jpg|jpeg|exr|hdr|tex|dds|tga|bmp|tif|tiff))@',
        # Asset paths in attributes: asset = @path@
        r'asset\s*=\s*@([^@\x00]+)@',
        # prepend references: prepend references = [@path@]
        r'prepend\s+references\s*=\s*@([^@\x00]+)@',
        # sublayerPaths
        r'subLayerPaths\s*=\s*\[\s*@([^@\x00]+)@',
        # Payload paths
        r'payload\s*=\s*@([^@\x00]+)@',
        # defaultPrim references
        r'references\s*=\s*@([^@\x00]+)@',
        # Generic file references with common extensions
        r'"([^"\x00]+\.(?:usd|usda|usdc|png|jpg|jpeg|exr|hdr|tex|dds|tga|bmp|tif|tiff))"',
    ]

    found_paths = set()
    for pattern in patterns:
        matches = re.findall(pattern, text_content, re.IGNORECASE)
        for match in matches:
            # Clean up the match (remove any trailing parameters like </Prim>)
            clean_path = match.split('<')[0].split('>')[0].strip()
            if clean_path:
                found_paths.add(clean_path)

    # Resolve all found paths
    for asset_path in found_paths:
        resolved = resolve_asset_path(base_url, asset_path)
        if resolved:
            dependencies.append(resolved)

    return dependencies


def extract_dependencies_from_usd(local_path: Path, base_url: str) -> list[str]:
    """Parse a USD file and extract all external dependencies.

    This finds references, sublayers, payloads, and asset paths (textures, etc.)
    Uses pxr library if available, falls back to regex parsing.

    Args:
        local_path: Local path to the downloaded USD file
        base_url: The original Omniverse URL of the file

    Returns:
        List of resolved Omniverse URLs for dependencies
    """
    if not local_path.exists():
        return []

    # Try pxr first (handles binary USD properly)
    dependencies = extract_dependencies_with_pxr(local_path, base_url)

    # If pxr didn't find anything, try regex as fallback
    if not dependencies:
        dependencies = extract_dependencies_with_regex(local_path, base_url)

    return dependencies


def extract_dependencies_from_mdl(local_path: Path, base_url: str) -> list[str]:
    """Parse an MDL file and extract all external dependencies.

    This finds module imports like:
    - using .::OmniUe4Function import *;
    - using .::OmniUe4Base import *;
    - import ::some_module::*;

    Args:
        local_path: Local path to the downloaded MDL file
        base_url: The original Omniverse URL of the file

    Returns:
        List of resolved Omniverse URLs for dependencies
    """
    if not local_path.exists():
        return []

    dependencies = []
    # Get parent directory URL by removing the filename
    base_dir = base_url.rsplit("/", 1)[0]  # Parent directory URL

    try:
        content = local_path.read_text(encoding='utf-8', errors='ignore')
    except Exception as e:
        print(f"  WARNING: Could not read MDL file {local_path}: {e}")
        return dependencies

    # Pattern to find local module imports: using .::ModuleName import *;
    # The .:: prefix means "same directory"
    local_import_pattern = r'using\s+\.::(\w+)\s+import'

    found_modules = set()
    for match in re.finditer(local_import_pattern, content):
        module_name = match.group(1)
        found_modules.add(module_name)

    # Resolve module names to URLs
    for module_name in found_modules:
        # Local modules are in the same directory as the MDL file
        module_url = f"{base_dir}/{module_name}.mdl"

        # Skip if it's a built-in module
        skip = False
        for pattern in SKIP_PATTERNS:
            if pattern in module_url:
                skip = True
                break
        if skip:
            continue

        # The module URL is already absolute, just add it
        dependencies.append(module_url)
        print(f"    [mdl] Found module import: {module_name}.mdl")

    return dependencies


def download_asset_with_dependencies(omni_client, url: str, output_dir: Path, downloaded: set, stats: dict, include_siblings: bool = False) -> bool:
    """Download an asset and all its referenced dependencies (recursively).

    Args:
        omni_client: The Omniverse client instance
        url: The Omniverse URL to download
        output_dir: Local directory to save files
        downloaded: Set of already downloaded URLs (to avoid duplicates)
        stats: Dictionary with 'success' and 'failed' counts
        include_siblings: Whether to download sibling files in the same directory

    Returns:
        True if the main asset was downloaded successfully
    """
    if url in downloaded:
        return True

    downloaded.add(url)

    if not download_asset(omni_client, url, output_dir):
        stats['failed'] += 1
        return False

    stats['success'] += 1

    # Parse USD files for dependencies
    if url.endswith((".usd", ".usda", ".usdc")):
        relative_path = url.replace(f"{OMNIVERSE_SERVER}/", "")
        local_path = output_dir / relative_path

        # Extract and download dependencies (before rewriting references)
        dependencies = extract_dependencies_from_usd(local_path, url)
        if dependencies:
            print(f"  Found {len(dependencies)} dependencies in {url.split('/')[-1]}")
            for dep_url in dependencies:
                if dep_url not in downloaded:
                    print(f"    -> Dependency: {dep_url.split('/')[-1]}")
                    download_asset_with_dependencies(omni_client, dep_url, output_dir, downloaded, stats, include_siblings=False)

        # Rewrite references to point to localhost after extracting dependencies
        rewrite_references_to_localhost(local_path)

    # Parse MDL files for module imports (OmniUe4Function, OmniUe4Base, etc.)
    if url.endswith(".mdl"):
        relative_path = url.replace(f"{OMNIVERSE_SERVER}/", "")
        local_path = output_dir / relative_path

        # Extract and download MDL module dependencies
        dependencies = extract_dependencies_from_mdl(local_path, url)
        if dependencies:
            print(f"  Found {len(dependencies)} MDL module dependencies in {url.split('/')[-1]}")
            for dep_url in dependencies:
                if dep_url not in downloaded:
                    print(f"    -> MDL Module: {dep_url.split('/')[-1]}")
                    download_asset_with_dependencies(omni_client, dep_url, output_dir, downloaded, stats, include_siblings=False)

    # Optionally download sibling files (disabled by default)
    if include_siblings and url.endswith((".usd", ".usda", ".usdc")):
        # List directory to find related assets (textures, materials, etc.)
        parent_url = "/".join(url.split("/")[:-1])
        result, entries = omni_client.list(parent_url)

        if result == omni_client.Result.OK:
            for entry in entries:
                entry_url = f"{parent_url}/{entry.relative_path}"
                if entry_url not in downloaded and not entry.flags & omni_client.ItemFlags.CAN_HAVE_CHILDREN:
                    # It's a file, download it
                    if download_asset(omni_client, entry_url, output_dir):
                        stats['success'] += 1
                    else:
                        stats['failed'] += 1
                    downloaded.add(entry_url)

    return True


def get_omni_client():
    """Try multiple methods to import omni.client."""
    # Method 1: Direct import (works if PYTHONPATH is set correctly)
    try:
        import omni.client as omni_client
        return omni_client
    except ImportError:
        pass

    # Method 2: Try importing via isaacsim bootstrap
    try:
        import isaacsim
        from omni.isaac.kit import SimulationApp
        # Create a minimal simulation app to initialize omni
        simulation_app = SimulationApp({"headless": True})
        import omni.client as omni_client
        return omni_client
    except ImportError:
        pass

    # Method 3: Try importing carb first (lower-level approach)
    try:
        import carb
        import omni.client as omni_client
        return omni_client
    except ImportError:
        pass

    return None


def main():
    omni_client = get_omni_client()

    if omni_client is None:
        print("ERROR: omni.client not available.")
        print("This script must be run inside Isaac Sim environment.")
        print("")

        sys.exit(1)

    # Initialize Omniverse client
    omni_client.initialize()
    
    print("=" * 60)
    print("Omniverse Asset Downloader")
    print("=" * 60)
    print(f"Server: {OMNIVERSE_SERVER}")
    print(f"Output: {OUTPUT_DIR.absolute()}")
    print(f"Assets: {len(OMNIVERSE_ASSETS)}")
    print("=" * 60)
    
    # Create output directory
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    downloaded = set()
    stats = {'success': 0, 'failed': 0}

    for url in OMNIVERSE_ASSETS:
        print("")
        download_asset_with_dependencies(omni_client, url, OUTPUT_DIR, downloaded, stats)

    print("")
    print("=" * 60)
    print(f"Download complete: {stats['success']} succeeded, {stats['failed']} failed")
    print(f"Total files processed: {len(downloaded)}")
    print("=" * 60)

    # Shutdown client
    omni_client.shutdown()

    return 0 if stats['failed'] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

