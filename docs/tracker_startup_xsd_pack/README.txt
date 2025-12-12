Tracker Startup XSD Pack

Validate each XML document against its matching XSD:
- system.xml                -> system.xsd
- profiles.xml              -> profiles.xsd
- sensors.xml               -> sensors.xsd
- tracker.xml               -> tracker.xsd
- gating_association.xml    -> gating_association.xsd
- persistence.xml           -> persistence.xsd
- recorder_playback.xml     -> recorder_playback.xsd
- telemetry_logging.xml     -> telemetry_logging.xsd

Convenience:
- tracker_config_master.xsd xs:includes all schemas (useful for IDE tooling).
  Note: validation still happens per XML document; XSD cannot automatically follow <Includes> across multiple files.

Forward-compatibility:
- Many complexTypes include xs:any (processContents="lax") so you can add new elements without breaking validation.
  Tighten this later once the config interface stabilizes.
