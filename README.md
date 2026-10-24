# NEox Compact 40M

**A portable, drill-driven winder for a single 40 m flat drip-irrigation tape — full CAD, drawings, calculations and BOM (Revision 2.0).**

> **Status: design / prototype stage.** The design is fully dimensioned, collision-checked in CAD and analytically calculated. It has **not been physically built or tested**, and it is **not released for series manufacturing**. See [Known limitations](#known-limitations-and-open-risks).

> **Language note:** the engineering documents (spec, drawings, workbook, project portal) are written in **German**. This README is in English.

---

## What is this?

The NEox Compact 40M is a small frame-mounted machine that winds one continuous 40 m length of flat drip tape (ALLWIN GOLD, 25 mm lay-flat) onto a tapered drum, using a 12 V cordless drill (INGCO CIDLI12206) as the drive. The wound roll is strapped and can be removed without cutting the tape, while the valve-side end of the tape stays connected and stationary.

This repository contains everything for Revision 2.0: parametric CAD scripts, STEP / STL / GLB exports, technical drawings, an Excel calculation workbook with BOM and fastener register, and two interactive HTML viewers.

## Key specifications

| Item | Value |
|---|---|
| Tape | ALLWIN GOLD flat drip tape, 25 mm lay-flat, 40 m, single continuous length |
| Drum | Conical (2° taper), R 52.0 → 48.15 mm, 110 mm long, 4 tape lanes (4 × 10 m) |
| Flanges | Ø220 mm, with 3 radial strapping slots (16 × 58 mm) |
| Calculated roll radius | R_coil ≈ 96.58 mm (13.4 mm clearance to flange) |
| Spindle / bearings | Ø17 k5 spindle, 2 × 6003-2RS; guide roller on Ø15 k5 axle, 2 × 6002-2RS |
| Drive | INGCO CIDLI12206, 12 V (400 / 1500 rpm no-load, 20 Nm catalog max) |
| Target winding time | 3–5 min (requires a speed-reducing gearbox, see open risks) |
| Mass target | ≤ 5 kg without drill and battery (workbook states 3.10 kg calculated) |
| Parts / fasteners | 23 numbered parts (P-01 … P-23), 3 reference parts (REF-01 … REF-03), 34 fasteners (F-01 … F-34) |
| Overall size | approx. 466 × 240 × 304 mm |

## Repository structure

```
NEox-Compact-40M/
├── CAD/
│   ├── GLB/                  Assembled and exploded 3D models (open in any glTF viewer)
│   ├── STEP/                 Assembly + per-part STEP files
│   ├── STL/                  Assembly + per-part STL files (3D printing / mesh tools)
│   └── Parametric_Scripts/   OpenSCAD, CadQuery and Python mesh/STEP generator
├── Documentation/
│   ├── Aenderungsprotokoll_und_Pruefbericht_v2_DE.md   Change log v1.0 → v2.0 (REV-01 … REV-07)
│   └── Navaneeth_Compact_40M_v2_Master_Spezifikation_DE.yaml   Original requirements brief
├── Drawings/
│   └── Navaneeth_Compact_40M_v2_Technical_Drawings_DE.pdf   6 sheets: assembly, exploded, parts
├── Navaneeth_Compact_40M_Calculations_BOM_DE.xlsx   12-sheet workbook: calculations, BOM, fasteners, joints, risks
├── Navaneeth_Compact_40M_Interactive_CAD.html       Interactive 3D viewer + kinematics simulation
├── Navaneeth_Compact_40M_Project.html               Project portal (data, BOM, drawings, simulator)
└── Navneeth_Compact_40M_Portfolio.pdf               50-page project portfolio
```

## How to use

**View the model**
- Open `Navaneeth_Compact_40M_Interactive_CAD.html` or `Navaneeth_Compact_40M_Project.html` in a browser (an internet connection is needed once, because they load Three.js and Tailwind from a CDN).
- Or drag the `.glb` files from `CAD/GLB/` into any glTF viewer (e.g. gltf-viewer.donmccurdy.com, Blender).

**Read the numbers**
- Open the `.xlsx` workbook in Excel or LibreOffice (formulas recalculate on open).

**Rebuild the CAD**
```bash
# OpenSCAD (simplified drum/flange/spindle preview)
openscad CAD/Parametric_Scripts/navaneeth_compact_40m_v2.scad

# CadQuery (rear flange, drum and spindle -> STEP)
pip install cadquery
python CAD/Parametric_Scripts/navaneeth_compact_40m_v2_cadquery.py

# Full mesh / STEP / STL / GLB generator
pip install numpy scipy shapely
python CAD/Parametric_Scripts/navaneeth_compact_40m_v2_mesh_step_kernel.py
```

## Revision 2.0 highlights

Changes compared with v1.0 (details in `Documentation/Aenderungsprotokoll_und_Pruefbericht_v2_DE.md`):

- **REV-01** Strapping slots in both flanges and strap channels in the drum, so the roll can be removed in +X.
- **REV-02** Stronger cantilever guide axle (Ø15 k5 seat) with 6002-2RS bearings.
- **REV-03** Spindle Ø17 k5 with correct 6003-2RS bearing seats and a cross-bolt torque connection.
- **REV-04** Frame-mounted torque support for the drill handle (20 Nm reaction torque).
- **REV-05** Ground clearance ≥ 12 mm; all frame bolts mounted from below with recessed heads.
- **REV-06** 4-lane tape guide carriage (lane centres X = 19, 45, 71, 97 mm).
- **REV-07** Valve stem gap closed and a strain-relief clamp added.

## Known limitations and open risks

- **Not physically tested.** All results are calculations and CAD checks; no prototype has been built.
- **RISK-01 – stationary valve end:** the drum turns about 87 revolutions while the valve-side tape end must stay fixed and untwisted. The field procedure still has to be defined and tested.
- **RISK-02 – speed reduction:** the drill's 400 rpm would wind the tape in about 13 s. A planetary/reduction gearbox (approx. 12:1 – 20:1) is required for the 3–5 min target and is **not yet part of the CAD model**.
- **Assumed values:** effective tape thickness (1.5 mm), packing factor (0.70) and tape pull force (35 N) are assumptions and must be measured on the real tape.
- **STEP files are mesh-derived** (generated from triangulated geometry), not parametric solid models with editable features. Use the Python/CadQuery/OpenSCAD scripts for parametric work, and verify STEP import in your CAD tool.
- **CadQuery and OpenSCAD scripts are partial** — they cover only the main rotating parts (flange, drum, spindle), not the full assembly.
- Component names, bearing sizes and drill data come from manufacturer catalogs and should be re-checked against the exact parts you buy.

## Safety

This is an unverified prototype design that involves a rotating drum, a power tool and tension in the tape. Do not build or operate it without your own engineering review, guards, and a safe stop procedure.

## License

Released under the **MIT License** for code and scripts, and **CC BY 4.0** for the design files and documentation. See [`LICENSE`](LICENSE).

*(ALLWIN, INGCO and SKF are trademarks of their respective owners and are used here only to identify compatible products. This project is not affiliated with or endorsed by them.)*

## Author

Designed and documented by **Navaneeth** — student builder (CAD, Linux, IoT).
GitHub: [@neoxey](https://github.com/neoxey)

Feedback, bug reports and suggestions are welcome via [GitHub Issues](../../issues).
