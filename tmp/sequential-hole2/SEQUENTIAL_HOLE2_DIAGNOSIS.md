# Sequential modeled Hole2 diagnosis (2-hole minimal)

Fixture: 60×60×10 plate, M5×0.8 tap r=2.1, centers (15.0, 15.0) then (45.0, 15.0).

Path A = production modeled Hole: boolean Cut then `try_replace` per feature.
Path B = plain holes first, then `try_replace` (known-good Phase 4B order).

## Path A — after Hole1 modeled (before Hole2 boolean)

- boolean1=10.32165ms replace1=5.496455226s
- faces=34 shells=1 surface=false cond=Closed free_edges=0
- surfaces planes/cyl/other/unknown=(6, 2, 0, 26)
- Hole1 flank-like faces near axis: 26
- unmatched flank boundary edges (use-count 1): 56
- bore1 empty: true
- plane face wire counts: [(0, 1), (1, 1), (2, 1), (3, 1), (32, 2), (33, 2)]

  - face 5 BSplineSurface sample=Vec3 { x: 16.653085309279003, y: 16.295109632519946, z: 0.3846153846153848 } r=2.100
  - face 6 BSplineSurface sample=Vec3 { x: 16.91498967658338, y: 15.861866891450802, z: 1.1538461538461546 } r=2.100
  - face 7 BSplineSurface sample=Vec3 { x: 17.065601804588997, y: 15.378535579409212, z: 1.9230769230769234 } r=2.100
  - face 8 BSplineSurface sample=Vec3 { x: 17.096168663868966, y: 14.873205155413217, z: 2.6923076923076934 } r=2.100
  - face 9 BSplineSurface sample=Vec3 { x: 17.00491381966795, y: 14.375243586904102, z: 3.461538461538463 } r=2.100
  - face 10 BSplineSurface sample=Vec3 { x: 16.79714067183305, y: 13.913590590226942, z: 4.230769230769232 } r=2.100
  - face 11 BSplineSurface sample=Vec3 { x: 16.484924240491758, y: 13.515075759508264, z: 5.000000000000002 } r=2.100
  - face 12 BSplineSurface sample=Vec3 { x: 16.086409409773097, y: 13.202859328166976, z: 5.76923076923077 } r=2.100

## Path A — after Hole2 boolean Cut into threaded body

- boolean2 elapsed=1.71752572s (budget 90s)
- boolean2 OK faces=36 shells=1 surface=false cond=Closed free_edges=0
- surfaces after: (6, 4, 0, 26)
- Hole1 flank-like faces after: 26 (was 26)
- faces near Hole2 axis (unexpected flanks): 0
- edge ids lost/new vs Hole1 body: 96/102
- unmatched Hole1 flank free edges: 56 (was 56)
- bore1 empty=true bore2 empty=true
- Hole2 cylinder selectable: true
- plane wire counts: [(0, 1), (1, 1), (2, 1), (3, 1), (32, 3), (33, 3)]

- validity after boolean2: []
  (took 4.95467178s)

### Hole2 try_replace attempt 1 (11.16983006s)

- OK faces=62 shells=1 free=0 flanks_near_h1=26 flanks_near_h2=26
- bore1 empty=true bore2 empty=true

## Path B — plain Hole1+Hole2 boolean, then two try_replace (control)

- plain boolean @(15,15) 9.634129ms
- plain boolean @(45,15) 10.420892ms
- replace 0 @(15,15) OK 5.223951302s: faces=36 free=0 flanks_h1=26 flanks_h2=0
- replace 1 @(45,15) OK 11.17829481s: faces=62 free=0 flanks_h1=26 flanks_h2=26

Path B final: faces=62 cond=Closed bore1=true bore2=true

## Same-cause / stop list

(none recorded — Path A may have succeeded unexpectedly)

## Preliminary root-cause candidates (filled by human/agent after run)

See agent analysis in `SEQUENTIAL_HOLE2_ROOTCAUSE.md`.
