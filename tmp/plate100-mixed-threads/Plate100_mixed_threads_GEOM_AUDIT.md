# Plate100 mixed-thread STEP geometric audit (read-only)

File: `/workspace/tmp/plate100-mixed-threads/Plate100_4x_mixed_threads.step`
Import: faces=118 shells=1 surface=false
No `validity()` call.

Surfaces: planes=6 bspline/unknown=104
Cylinder radii (count):
- r=2.100000 ×1
- r=2.125000 ×1
- r=2.500000 ×2
- r=2.550000 ×1
- r=2.600000 ×1
- r=3.000000 ×2

## M5x0.8 @ (25,25)

- expected major/2=2.500 tap_r=2.100 pitch=0.8
- bore empty at mid-height: true
- crest cylinder present: true
- root/major cylinder present: true
- local cylinder radii: [2.099999999999992, 2.4999999999999956]
- flank-like (non-analytic) faces near bore: 26
- crest-edge pitch samples: n=52 median=0.800000 (tol 0.05) ok=true

## M6x0.9 @ (75,25)

- expected major/2=3.000 tap_r=2.550 pitch=0.9
- bore empty at mid-height: true
- crest cylinder present: true
- root/major cylinder present: true
- local cylinder radii: [2.54999999999997, 3.000000000000003]
- flank-like (non-analytic) faces near bore: 24
- crest-edge pitch samples: n=48 median=0.900000 (tol 0.05) ok=true

## M5x0.75 @ (25,75)

- expected major/2=2.500 tap_r=2.125 pitch=0.75
- bore empty at mid-height: true
- crest cylinder present: true
- root/major cylinder present: true
- local cylinder radii: [2.1250000000000067, 2.5000000000000044]
- flank-like (non-analytic) faces near bore: 28
- crest-edge pitch samples: n=56 median=0.750000 (tol 0.05) ok=true

## M6x0.8 @ (75,75)

- expected major/2=3.000 tap_r=2.600 pitch=0.8
- bore empty at mid-height: true
- crest cylinder present: true
- root/major cylinder present: true
- local cylinder radii: [2.599999999999968, 2.999999999999974]
- flank-like (non-analytic) faces near bore: 26
- crest-edge pitch samples: n=52 median=0.800000 (tol 0.05) ok=true

## Verdict

All four holes match requested major, tap/crest radius, pitch (sampled), and have helical flank B-rep.
