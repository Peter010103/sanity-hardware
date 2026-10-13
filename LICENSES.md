# Licences and attribution

Copyright © 2026 Prorok Lab, University of Cambridge. The licences apply by file type:

| Content | Licence |
|---|---|
| Lab-authored CAD in `models/sources/`, visual and collision meshes in `models/meshes/`, and their materials | [CC BY 4.0](licenses/CC-BY-4.0.txt) |
| Measurements, processed data and parameter files in `data/` and `parameters/` | [CC BY 4.0](licenses/CC-BY-4.0.txt) |
| README, documentation, lab photographs, videos and generated figures | [CC BY 4.0](licenses/CC-BY-4.0.txt) |
| `models/sanity.urdf`, `models/sanity.xml`, Python/JavaScript code, tools and tests | [MIT](licenses/MIT.txt), with existing third-party notices retained |

Both licences permit modification and commercial redistribution. For CC BY material, credit **Sanity — Prorok Lab, University of Cambridge**, link to [this repository](https://github.com/Peter010103/sanity-hardware) and [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), and indicate changes. For MIT files, retain the copyright and licence notice.

## CAD and measurements

The Sanity CAD models were created from scratch in Autodesk Fusion 360 using physical measurements of a BETAFPV Pavo Pico. No manufacturer CAD files were used. These files provide a geometric representation for robotics simulation; they are not official BETAFPV design files.

The mass, thrust and motor-response data were collected in-house. The inertia tensor comes from in-house bifilar pendulum measurements.

These licences cover rights the lab can grant in this simulation package. They do not grant separate rights in the manufacturer's underlying product design or trademarks.

## Paper

Heedo Woo, Kazi R. I. Sanim, Keisuke Okumura, Guang Yang, Ajay Shankar and Amanda Prorok. **Sanity: An Agile Brushless Quadrotor for Multi-Agent Experiments.** [Paper](paper/sanity.pdf) · [OpenReview](https://openreview.net/forum?id=i5dKOANPZZ) · [Citation](CITATION.cff).

The manuscript is separate from the package licence grant and retains its publication terms; no explicit redistribution licence was identified in the supplied PDF.

## Third-party material

| Material | Attribution and terms |
|---|---|
| `tools/geometry/vendor/USDCParser.mjs` | Three.js r183, MIT; retain [the original notice](licenses/THREE-MIT.txt) and the adjacent vendor copy |

Python, Node.js, PyTorch, Isaac Lab, MuJoCo and other dependencies are installed separately and retain their own licences. The build guide links to the manufacturer's pinout; that external material is not included in this repository or its licence grant.
