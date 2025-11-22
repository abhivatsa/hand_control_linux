# Merai Module

Runtime and configuration layer for the hand-control stack: parses JSON configs, exposes them via shared memory, provides a shared logger, and installs launcher/logger systemd services.

## Contents
- `include/merai/ParameterServer.h/.cpp` – parse EtherCAT/robot/startup JSON into POD structs (SHM-safe).
- `include/merai/RTMemoryLayout.h` – POD layout for RT data buffers.
- `include/merai/SharedLogger.h` + `src/LoggerProcess.cpp` – per-module SPSC rings for logs, logger consumer.
- `src/Launcher.cpp` – creates and seeds SHM (`/ParameterServerShm`, `/RTDataShm`, `/LoggerShm`).
- `services/` – systemd units (`merai_launcher`, `merai_logger`, `merai_stack.target`).
- `tests/` – basic CTest targets (parser and logger).

## Dependencies
- Build: `cmake`, `pkg-config`, `libsystemd-dev` (for journald logging), standard C++17 toolchain.
- Runtime: systemd/journald present on Debian/Ubuntu; EtherCAT master/dev headers if building fieldbus later.

## Build & Test (merai only)
From repo root:
```
cmake -S src/merai -B build/merai \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON \
  -DMERAI_CONFIG_INSTALL_DIR=/etc/hand_control_merai
cmake --build build/merai --target merai_foundation merai_launcher merai_logger
ctest --test-dir build/merai --output-on-failure   # optional
```
With presets (whole repo):
```
cmake --preset dev
cmake --build --preset dev --target merai_foundation merai_launcher merai_logger
ctest --preset dev --output-on-failure
```

## Packaging
From the build directory (`build/` for preset, or `build/merai` if configured directly):
```
cpack   # produces the .deb
sudo dpkg -i hand-control-merai_*.deb
```

## Config Files
- Installed defaults: `${MERAI_CONFIG_INSTALL_DIR}` (default `/etc/hand_control_merai`).
- Source defaults (for in-tree runs): `config/` at repo root.
- Launcher resolves config dir by: env `MERAI_CONFIG_DIR` override → installed dir if exists → source dir.

## Running
Dev (from build tree):
```
./build/src/merai/merai_launcher
./build/src/merai/merai_logger   # separate terminal
```
Installed binaries: `/usr/bin/merai_launcher`, `/usr/bin/merai_logger`.

Systemd (after install):
```
sudo systemctl start merai_stack.target
systemctl status launcher.service logger.service
```

## Shared Memory
- Names: `/ParameterServerShm`, `/RTDataShm`, `/LoggerShm`.
- POD structs with `magic`/`version`; trivially copyable.
- Ensure all SHM users run under the same user or set permissive modes (default 0666).

## Logger
- One SPSC ring per module (fieldbus/control/logic) in `multi_ring_logger_memory`.
- Messages: timestamp (steady_clock ns), level, code, text (128 bytes), drop counter per ring.
- Logger process validates magic/version, forwards to journald (requires `libsystemd`) and reports drops.

## Notes
- EtherCAT config uses enums/ints (no strings) for types/indices; SDOs store numeric indices/data types.
- If SHM exists from a different user (e.g., root), clean `/dev/shm/ParameterServerShm`, etc., or run all services as the same user.
