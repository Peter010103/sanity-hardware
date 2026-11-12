# Licences and attribution

Copyright © 2026 Prorok Lab, University of Cambridge.

- Lab-authored Python/C code, tests, build and setup tools: [MIT](licenses/MIT.txt).
- README, documentation and historical reference/calibration data:
  [CC BY 4.0](licenses/CC-BY-4.0.txt). Credit **Sanity — Prorok Lab, University of
  Cambridge**, link to [sanity-hardware](https://github.com/Peter010103/sanity-hardware)
  and the licence, and indicate changes.
- `patches/tenma-72-13350.patch` modifies Jordi Castells's **tenma-serial** and is
  distributed under the upstream **GPL-3.0-or-later** terms. The complete GPLv3
  text is [included](licenses/tenma-GPL-3.0.txt). Upstream copyright notices are
  retained by the fetched checkout. The pinned upstream source is
  <https://github.com/kxtells/tenma-serial/tree/d1b73c25b61d308349acdfd8695041d0ea05ec09>;
  the setup script fetches that separate dependency and applies this patch.

The setup script does not relicense or vendor tenma-serial under MIT. Preserve
upstream terms when redistributing its modified source or software incorporating
it. PySerial, Jetson.GPIO, pytest, matplotlib, Python and system dependencies
remain under their respective licences. No Betaflight firmware, manufacturer
schematics, ROS workspace, aircraft CAD, credentials or experiment recordings
are included in this branch.

`SOURCE_MANIFEST.json` records the local campaign sources used to construct this
standalone package and their original hashes. Packaging adaptations remove
workspace/home-directory dependencies, add current operator declarations and
extend the experiment lock to cover supply setup and teardown. These source
paths are provenance, not runtime requirements.
