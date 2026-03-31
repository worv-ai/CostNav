#!/usr/bin/env python3
"""Quick probe to check if Nucleus server is reachable via omni.client."""

import os
import sys

ISAAC_SIM_PATH = os.environ.get("ISAAC_PATH", "/isaac-sim")
OMNI_CLIENT_PATH = os.path.join(ISAAC_SIM_PATH, "kit/extscore/omni.client.lib")
if os.path.exists(OMNI_CLIENT_PATH) and OMNI_CLIENT_PATH not in sys.path:
    sys.path.insert(0, OMNI_CLIENT_PATH)

import omni.client

omni.client.initialize()
omni.client.register_authentication_callback(
    lambda url: (os.environ.get("OMNI_USER", "omniverse"), os.environ.get("OMNI_PASS", ""))
)
result, _ = omni.client.stat("omniverse://localhost")
omni.client.shutdown()
sys.exit(0 if result in (omni.client.Result.OK, omni.client.Result.ERROR_NOT_FOUND) else 1)
