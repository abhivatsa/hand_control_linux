# Merai Stack TODO / Roadmap

- Config validation: enforce JSON schema/range checks (limits, masses, axes, drive params), and reject/flag missing fields; validate SHM magic/version on readers.
- Health/watchdogs: add loop overrun and data-freshness checks with safe-stop path; wire optional systemd watchdog pings once loops are stable.
- Error handling/logging: standardize init failure codes/messages (SHM map, drive config, EtherCAT init), log state transitions and timing anomalies, expose basic status/heartbeat for GUI/ops.
- Testing: expand parser tests (invalid/edge configs), add SHM layout sanity checks, and smoke tests for control/fieldbus init where feasible.
- RT hygiene: pin priorities/affinity for real-time loops; keep allocations/logging out of hot paths; capture/report loop timing metrics.
- Network API hardening: input validation, timeouts, and rate limits on external-facing endpoints.
