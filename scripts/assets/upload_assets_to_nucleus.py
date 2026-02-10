#!/usr/bin/env python3
"""Upload assets to local Nucleus server using omni.client API.

This script must be run from within an Omniverse environment (Isaac Sim, etc.)
where the omni.client library is available.

Usage from Isaac Sim container:
    PYTHONPATH=/isaac-sim/kit/extscore/omni.client.lib:$PYTHONPATH \\
    /isaac-sim/python.sh upload_assets_to_nucleus.py \\
        --local-path /workspace/assets/Users \\
        --nucleus-url omniverse://localhost/Users \\
        --username omniverse \\
        --password costnav123
"""

import os
import sys
import time
import argparse
from pathlib import Path

# Add omni.client path if running from Isaac Sim
ISAAC_SIM_PATH = os.environ.get("ISAAC_PATH", "/isaac-sim")
OMNI_CLIENT_PATH = os.path.join(ISAAC_SIM_PATH, "kit/extscore/omni.client.lib")
if os.path.exists(OMNI_CLIENT_PATH) and OMNI_CLIENT_PATH not in sys.path:
    sys.path.insert(0, OMNI_CLIENT_PATH)

# Try to import omni.client
try:
    import omni.client
except ImportError:
    print("ERROR: omni.client not available.")
    print("This script must be run from within an Omniverse environment (Isaac Sim, etc.)")
    print("")
    print("Alternative: Use the Nucleus Web UI at http://localhost:8080 to upload files manually.")
    sys.exit(1)

# Global credentials (set by main)
_username = None
_password = None

# Skip patterns for built-in NVIDIA materials that shouldn't be uploaded
# These are part of the Omniverse/Isaac Sim installation
SKIP_PATTERNS = [
    "NVIDIA/Materials/",
    "NVIDIA/Assets/",
    "NVIDIA/Environments/",
    ".cache/",
]


def should_skip_file(rel_path: str) -> bool:
    """Check if a file should be skipped (built-in NVIDIA materials)."""
    rel_path_str = str(rel_path).replace("\\", "/")
    for pattern in SKIP_PATTERNS:
        if pattern in rel_path_str:
            return True
    return False


def auth_callback(url: str):
    """Provide authentication credentials for Nucleus connection."""
    global _username, _password
    if _username and _password:
        return (_username, _password)
    return None


def wait_for_nucleus(server_url: str, timeout: int = 120) -> bool:
    """Wait for Nucleus server to be ready."""
    print(f"Waiting for Nucleus server at {server_url}...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        try:
            result, entry = omni.client.stat(server_url)
            if result == omni.client.Result.OK:
                print("Nucleus server is ready!")
                return True
            elif result == omni.client.Result.ERROR_NOT_FOUND:
                # Server is reachable but path doesn't exist yet - that's OK
                print("Nucleus server is ready (path will be created)!")
                return True
        except Exception as e:
            print(f"  Connection attempt failed: {e}")

        elapsed = int(time.time() - start_time)
        print(f"  Still waiting... ({elapsed}s)")
        time.sleep(3)

    print(f"Timeout waiting for Nucleus server after {timeout}s")
    return False


def upload_directory(local_path: Path, nucleus_url: str, timeout: int) -> bool:
    """Upload a directory recursively to Nucleus."""
    # Initialize omni.client
    omni.client.initialize()

    # Register authentication callback (subscription kept alive for duration)
    _auth_subscription = omni.client.register_authentication_callback(auth_callback)  # noqa: F841
    print(f"Authentication callback registered (credentials: {_username})")

    # Extract server URL for connection check
    server_url = "omniverse://localhost"

    # Wait for server
    if not wait_for_nucleus(server_url, timeout):
        omni.client.shutdown()
        return False

    # Collect all files to upload (skipping NVIDIA built-in materials)
    files_to_upload = []
    skipped = 0
    for root, dirs, files in os.walk(local_path):
        rel_root = Path(root).relative_to(local_path)
        for file in files:
            local_file = Path(root) / file
            rel_path = rel_root / file
            if should_skip_file(rel_path):
                skipped += 1
                continue
            remote_url = f"{nucleus_url}/{rel_path}".replace("\\", "/")
            files_to_upload.append((local_file, rel_path, remote_url))

    print(f"\nFound {len(files_to_upload)} files to upload ({skipped} NVIDIA built-in files skipped)")

    # Create necessary directories first
    created_dirs = set()
    for local_file, rel_path, remote_url in files_to_upload:
        # Get parent directory URL
        parent_url = "/".join(remote_url.rsplit("/", 1)[:-1])
        if parent_url and parent_url not in created_dirs:
            result = omni.client.create_folder(parent_url)
            if result == omni.client.Result.OK:
                print(f"  Created folder: {parent_url}")
            created_dirs.add(parent_url)

    # Upload files
    uploaded = 0
    failed = 0

    for local_file, rel_path, remote_url in files_to_upload:
        try:
            with open(local_file, "rb") as f:
                content = f.read()

            result = omni.client.write_file(remote_url, content)

            if result == omni.client.Result.OK:
                print(f"✓ {rel_path}")
                uploaded += 1
            else:
                print(f"✗ {rel_path}: {result}")
                failed += 1
        except Exception as e:
            print(f"✗ {rel_path}: {e}")
            failed += 1

    print(f"\nUpload complete: {uploaded} succeeded, {failed} failed")
    omni.client.shutdown()
    return failed == 0


def main():
    global _username, _password

    parser = argparse.ArgumentParser(
        description="Upload assets to Nucleus server using omni.client",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example usage from Isaac Sim container:
    PYTHONPATH=/isaac-sim/kit/extscore/omni.client.lib:$PYTHONPATH \\
    /isaac-sim/python.sh upload_assets_to_nucleus.py \\
        --local-path /workspace/assets/Users \\
        --nucleus-url omniverse://localhost/Users \\
        --username omniverse \\
        --password costnav123
        """,
    )
    parser.add_argument("--local-path", default="assets/Users", help="Local path to upload (default: assets/Users)")
    parser.add_argument(
        "--nucleus-url",
        default="omniverse://localhost/Users",
        help="Nucleus destination URL (default: omniverse://localhost/Users)",
    )
    parser.add_argument("--timeout", type=int, default=120, help="Timeout waiting for Nucleus (default: 120s)")
    parser.add_argument(
        "--username",
        default=os.environ.get("OMNI_USER", "omniverse"),
        help="Nucleus username (default: omniverse or OMNI_USER env var)",
    )
    parser.add_argument(
        "--password", default=os.environ.get("OMNI_PASS", ""), help="Nucleus password (default: OMNI_PASS env var)"
    )

    args = parser.parse_args()

    # Set global credentials for auth callback
    _username = args.username
    _password = args.password

    local_path = Path(args.local_path)
    if not local_path.exists():
        print(f"ERROR: Local path does not exist: {local_path}")
        sys.exit(1)

    print(f"Uploading {local_path} to {args.nucleus_url}")
    print("=" * 60)

    success = upload_directory(local_path, args.nucleus_url, args.timeout)

    print("\n" + "=" * 60)
    if success:
        print("SUCCESS: All assets uploaded to Nucleus!")
    else:
        print("COMPLETED: Assets uploaded to Nucleus (some files failed)")
    print(f"Assets available at: {args.nucleus_url}")
    print("=" * 60)

    # Always exit with success - partial uploads are acceptable
    sys.exit(0)


if __name__ == "__main__":
    main()
