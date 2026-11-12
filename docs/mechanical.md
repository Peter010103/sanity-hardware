# Thrust stand mechanical assembly

The complete stand assembly is available in two formats, exported from the
corrected Fusion design on **12 November 2026**:

| File | Use |
|---|---|
| [Fusion assembly (.f3d)](../cad/thrust-test-stand/assembly.f3d) | Native Fusion archive for editing the assembly and its joints |
| [STEP assembly (.step)](../cad/thrust-test-stand/assembly.step) | Neutral CAD exchange for other CAD tools; use the Fusion archive when editing assembly constraints |

Both files describe the same assembly, including the load cells, brackets and
fasteners. Dimensions in the design are in millimetres.

## Assembly arrangement

- The two horizontal load cells (`Load Cell v2:2` and `Load Cell v2:3`) attach
  to the **underside of `top`**.
- The **bottom face of the horizontal mounting plate of `mount` rests on the
  upper faces of the horizontal load cells** at their opposite ends. Thus both
  `top` and the mounting plate of `mount` sit above the cells; `mount` is not
  suspended below their lower faces. The front plate of `mount` points downward.
- The four M5 × 16 mm fasteners secure the cells to `top`; the four M4 × 16 mm
  fasteners secure `mount` to the cells. Their axes align with the mounting holes.
- Rigid joints retain the corrected placement of the two cells, `mount` and
  those eight fasteners. The base, vertical load cell, `top` and the other
  fasteners retain their previous positions.

The native assembly contains 11 rigid joints for these connections. These joints
represent nominal CAD placement, not load-cell compliance or a structural model.

## Export checks

Before export, Fusion reported zero interference among the main parts (base,
`top`, `mount` and the three load cells). The mating faces had zero nominal gap,
all eight relocated fastener axes matched the mounting holes, and all 11 added
rigid joints were healthy. The Fusion archive passed its archive-integrity check;
the STEP file contains the complete 22-occurrence assembly structure.

These are CAD checks, not a load-rating or physical test of the stand. Use the
[wiring guide](wiring.md) and [operating procedure](operation.md) for setup,
calibration and operation; the measurement limitations in the
[main README](../README.md#measurements-and-limitations) still apply.
