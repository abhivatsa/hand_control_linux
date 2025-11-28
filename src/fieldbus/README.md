# Fieldbus Module (fieldbus)

EtherCAT master and drive handling for the seven-axis robot. Attaches to shared memory produced by `merai_launcher`, configures drives from `ParameterServer`, and runs the cyclic EtherCAT loop.

## Components
- Library: `fieldbus` (EthercatMaster, ServoDrive, IoDrive).
- Executable: `fieldbus_exe`.
- Service: `services/fieldbus.service` (systemd, RT priority/affinity).
- Test: `servo_drive_pdo_tests` (headless PDO mapping sanity with stubbed ecrt).

## Dependencies
- Build: EtherCAT/IGH headers (`ecrt.h`), C++17 toolchain.
- Runtime: EtherCAT master device/driver available; shared memories from `merai_launcher` (`/ParameterServerShm`, `/RTDataShm`, `/LoggerShm`).

## Build & Test
```
cmake --preset dev
cmake --build --preset dev
ctest --preset dev --output-on-failure   # runs stubbed test
```

## Runtime
- SHM: Expects `ParameterServer` (config), `RTMemoryLayout` (RT data), and `multi_ring_logger` (logs) to be present.
- EtherCAT config: Parsed into `DriveConfig` by `merai`; EthercatMaster consumes that (PDOs, sync managers). Servo drives use torque-based PDOs; IoDrive is stubbed until a concrete IO drive is defined.
- Loop: Fixed 1 ms period; RT scheduling/affinity configured in `services/fieldbus.service`.
- Logging: Uses shared logger; no stdio in the RT loop.

## Packaging
- Targets/headers install under the `fieldbus` namespace; systemd service installs to `lib/systemd/system`.

## Notes
- IoDrive exists as a placeholder; implement PDO offsets/mapping when an IO drive is chosen.
- Consider enabling systemd watchdog and expanding tests (init failure paths, RT buffer IO, optional hardware smoke) for production.
