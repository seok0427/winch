# winch

Autodesk Fusion script that builds the rotating bodies of winch v8.8 (22/56).

- Script: [`winch_bodies_fusion_v8/winch_bodies_fusion_v8.py`](winch_bodies_fusion_v8/winch_bodies_fusion_v8.py)
- Original attachment: [winch_bodies_fusion_v8_8_22.py](https://github.com/user-attachments/files/34947236/winch_bodies_fusion_v8_8_22.py)

## Bodies

`S1_pinion`, `CG1` … `CG7`, `CG8a_gear`, `CG8b_pinion`, `OUT_G9_spool`

| Stage | Type | Teeth | Module | Width (mm) |
|---|---|---|---|---|
| S1 | helical 25°, shift ±0.4 | 17 / 56 | 1.0 | 6 |
| S2–S7 | spur | 23 / 55 | 1.0 | 5 |
| S8 | spur | 23 / 55 | 1.0 | 6 |
| S9 | herringbone 19.56°, shift ±0.3 | 14 / 35 | 1.5 | 10 |

Overall reduction ≈ 3680 : 1. See the script's docstring for the v8 change log.

## Run

1. Keep the folder `winch_bodies_fusion_v8` with the script of the same name inside it.
2. In Fusion, open an empty design → Utilities → Add-Ins → Scripts and Add-Ins → add the folder → Run.
3. Units are mm / deg; X is the shaft axis (0 = inner face of W2).
4. Afterwards check Inspect → Interference between meshing parts (should be none).
