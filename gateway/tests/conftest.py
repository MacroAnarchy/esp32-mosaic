import os
import sys

# Make the gateway modules importable as top-level modules (they are scripts,
# not a package) and keep them away from the real ~/.orb deployment data.
GATEWAY_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if GATEWAY_DIR not in sys.path:
    sys.path.insert(0, GATEWAY_DIR)
