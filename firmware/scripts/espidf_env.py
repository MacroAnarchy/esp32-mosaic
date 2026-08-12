"""
Per-env ESP-IDF project source dir hook (env:ui).

PlatformIO 6.1.x resolves ``$PROJECT_SRC_DIR`` from the ``[platformio]``
section's ``src_dir`` only — the per-env ``src_dir`` option in
``[env:ui]`` is not applied to the ESP-IDF builder (``LoadProjectOptions``
only maps options that declare a ``buildenvvar``, and ``src_dir`` has
none). Without this hook the UI env would silently build
``firmware/src/`` (the Arduino sense engine) as its app component.

This pre-script runs before the platform builder script and points the
ESP-IDF app component at the env's declared ``src_dir`` (``ui/``), so
``pio run -e ui`` builds the face node: ``firmware/ui`` + the
``components/sense`` ESP-IDF sense engine in one firmware.
"""

import os

Import("env")

src_dir = env.GetProjectOption("src_dir")
if src_dir:
    abs_src_dir = os.path.join(env.subst("$PROJECT_DIR"), src_dir)
    env.Replace(PROJECT_SRC_DIR=abs_src_dir)
    print("espidf_env: PROJECT_SRC_DIR -> %s" % abs_src_dir)
