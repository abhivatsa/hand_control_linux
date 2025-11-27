# Merai Stack TODO / Roadmap

- Config validation: enforce JSON schema/range checks (limits, masses, axes, drive params), and reject/flag missing fields; validate SHM magic/version on readers.
- Health/watchdogs: add loop overrun and data-freshness checks with safe-stop path; wire optional systemd watchdog pings once loops are stable.
- Error handling/logging: standardize init failure codes/messages (SHM map, drive config, EtherCAT init), log state transitions and timing anomalies, expose basic status/heartbeat for GUI/ops.
- Testing: expand parser tests (invalid/edge configs), add SHM layout sanity checks, and smoke tests for control/fieldbus init where feasible.
- RT hygiene: pin priorities/affinity for real-time loops; keep allocations/logging out of hot paths; capture/report loop timing metrics.
- Network API hardening: input validation, timeouts, and rate limits on external-facing endpoints.
- Fieldbus: clamp/reject `driveCount > MAX_SERVO_DRIVES` on init to avoid overrunning `drives_`.
- Fieldbus: add lightweight error counters/flags for RT loop failures (ecrt receive/send, domain/master state) and report outside the RT loop.
- Fieldbus: add per-buffer sequence counters in servo Tx/Rx SHM to detect missed frames (no logging in RT).
- Fieldbus IO: decide on IO SHM integration; if needed, add IO buffers/indices and include them in the per-cycle context; otherwise document IO as local-only.
- Fieldbus: defend `cycleCtx_` use in drives (ensure set before use or assert) to prevent buffer 0 default misuse.
- Fieldbus tests: add basic PDO mapping/SHM index tests for ServoDrive and any IO drive, and CI hook for fieldbus lib/exe.
