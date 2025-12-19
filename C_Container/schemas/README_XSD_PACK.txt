XSD Pack for minimal tracker configuration (backend chosen by container build)

This pack is designed to validate the minimal example XMLs we drafted, while also
showing optional settings those examples were “set from” (threads/env defaults,
extra store knobs, optional scan schedule modes, etc.).

Schemas:
- system.xsd
- runtime_profiles.xsd
- tracker_model.xsd
- performance.xsd
- sensors.xsd
- gating_association.xsd
- store.xsd
- persistence.xsd
- scenario.xsd
- ownship.xsd
- targets_gen.xsd
- common_types.xsd

Notes:
- Math backend (eigen/std/mkl) is compile-time (container). Config does not select it.
- Many fields are optional to allow gradual build-out; validation still enforces structure/enums.
