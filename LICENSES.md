# Licences and attribution

Lab-authored host Python, scripts and tests follow the Sanity hardware repository's
[MIT licence](licenses/MIT.txt). Lab-authored documentation, measurement/model
parameters and generated figures use [CC BY 4.0](licenses/CC-BY-4.0.txt): credit
**Sanity — Prorok Lab, University of Cambridge**, link to this repository and the
licence, and indicate changes. Existing source notices take precedence for any
third-party component.

`firmware/fc/source.patch` contains modifications to Betaflight 4.5.1 and the
Sanity native rate-controller integration, distributed under the retained
[GPL-3.0-or-later terms](licenses/Betaflight-GPL-3.0.txt). The INDIflight algorithm
reference is revision `72c0b7614b4a462b4fd1db43477a189ba13a32b1`; pinned
ActiveSetCtlAlloc source uses revision `f6d6ef55fcb756650ece1eb51accc6eb816caccd`
and its included [licence](licenses/ActiveSetCtlAlloc-LICENSE). The complete build
source delta and reproducible upstream checkout procedure are included.

The Zenoh-Pico patches retain upstream notices and terms. Zenoh-Pico, acados,
HPIPM, BLASFEO, CasADi, ROS, ESP-IDF, PySerial/other tools, NumPy, SciPy, matplotlib
and pytest remain separate dependencies under their own licences. They are not
relicensed by the host MIT notice. Firmware builds and generated solver code must
retain their dependencies' applicable notices when redistributed.

The CTBR and pitch MPC Python implementations are independent lab implementations
inspired by the Agilicious formulation; they are not a claim to reproduce its
published controller or flight results. The quadrotor model/mesh material retains
the original Sanity hardware attribution. No manufacturer firmware binary, private
Wi-Fi configuration or trained actor weights are included.
