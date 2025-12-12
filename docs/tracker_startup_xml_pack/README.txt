Matrix Optimization Tracker - Startup XML Pack

Files:
- system.xml                 Top-level wiring; selects active operating profile.
- profiles.xml               Operating points (50k@1Hz and 10k@5Hz) and budgets.
- sensors.xml                Sensor definitions and scan scheduling.
- tracker.xml                Canonical ECEF state, prediction/update, track management.
- gating_association.xml     Coarse + fine gating and association (GNN) settings.
- persistence.xml            Persistent state store + spatial index + snapshot config.
- recorder_playback.xml      Append-only recorder + playback config.
- telemetry_logging.xml      Logging, metrics, and compute-aware degradation.

Notes:
- Tracks remain in ECEF for storage/update. AER is only measurement-space and is computed for candidates.
- Spatial index uses voxel/cell boxes to reduce index churn at high update rates.
- Snapshot period is configurable; track deltas recording enables quick catch-up after restore.
