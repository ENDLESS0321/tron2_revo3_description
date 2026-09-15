# Upstream assets

This local integration does not grant additional rights to third-party assets.

- LimX: https://github.com/limxdynamics/tron2-robot-description at
  `9939c22e69d27653ec0ba8a505859a2903dd1a71`. Original Apache-2.0 LICENSE,
  NOTICE, ASSETS.md and THIRD_PARTY_NOTICES.md remain in
  `vendor/tron2-robot-description/`. The upstream mesh provenance includes
  qualifications which are retained verbatim.
- BrainCo: https://github.com/BrainCoTech/brainco-description at
  `f332a6f0dc944e26b82976b637074b03f7ee8a2c`. Its current README says license
  information will be provided with the public release; no LICENSE file was
  present in the inspected tree. Preserve this unresolved license status when
  considering redistribution. Original assets and README are in
  `vendor/brainco-revo3/`.
- The printed adapter is user-provided. Original file remains at
  `../../tron2-revo3转接件_第1版.3mf` relative to this project directory.
  Its hash and every geometric conversion are recorded in
  `meshes/adapter/adapter_report.json`.

Generated copies under `meshes/` retain the provenance of their sources. This
project has not been published or sent to any third party.
