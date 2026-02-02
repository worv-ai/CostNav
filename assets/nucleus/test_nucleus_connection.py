#!/usr/bin/env python3
import os
import socket
import sys
import time
from urllib.request import urlopen


def wait_for_tcp(host: str, port: int, timeout_s: int) -> bool:
    start = time.time()
    while time.time() - start < timeout_s:
        try:
            with socket.create_connection((host, port), timeout=2):
                return True
        except OSError:
            remaining = int(timeout_s - (time.time() - start))
            print(f"  waiting for TCP {host}:{port}... ({remaining}s left)")
            time.sleep(2)
    return False


def wait_for_http(url: str, timeout_s: int) -> bool:
    start = time.time()
    while time.time() - start < timeout_s:
        try:
            with urlopen(url, timeout=3) as resp:
                if 200 <= resp.status < 500:
                    return True
        except Exception:
            remaining = int(timeout_s - (time.time() - start))
            print(f"  waiting for HTTP {url}... ({remaining}s left)")
            time.sleep(2)
    return False


def main() -> int:
    host = os.getenv("NUCLEUS_HOST", "localhost")
    web_port = int(os.getenv("NUCLEUS_WEB_PORT", "8080"))
    nucleus_port = int(os.getenv("NUCLEUS_PORT", "3009"))
    timeout_s = int(os.getenv("NUCLEUS_TIMEOUT", "2"))

    web_url = f"http://{host}:{web_port}"

    print(f"Checking Web UI: {web_url}")
    if not wait_for_http(web_url, timeout_s):
        print(f"ERROR: Web UI not reachable within {timeout_s}s.")
        return 1

    print(f"Checking Nucleus port: {host}:{nucleus_port}")
    if not wait_for_tcp(host, nucleus_port, timeout_s):
        print(f"ERROR: Nucleus port not reachable within {timeout_s}s.")
        return 1

    print("OK: Nucleus appears reachable.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
