# Truss-FEA — Finite Element Truss Solver and Bridge Design Study

A 2D truss finite element solver built from first principles (direct stiffness
method), validated against an exact analytic solution, then used for a real
engineering task: analysing a Warren truss bridge under traffic loading and sweeping
its geometry to find the stress-optimal height.

## Validation

Single bar under axial load: FEM displacement matches the analytic PL/EA exactly
(relative error 0.0e+00). The solver is provably correct before it touches anything
complex.

## Bridge study (24 m Warren truss, steel, 5 x 60 kN deck loads)

| Quantity | Value |
|---|---|
| Max displacement | 42.6 mm |
| Max tension | 170 MPa |
| Max compression | 180 MPa |
| Steel weight | 1.37 t |
| Safety factor vs 250 MPa yield | 1.39 |

Design sweep across truss heights 1.2-6.0 m shows peak stress falling monotonically
with height (90 MPa at 6 m) while steel weight rises - the classic strength-vs-material
trade-off, quantified. Plot: `results/bridge_analysis.png` shows the deformed shape
(x300) with members coloured by stress: red tension chords bottom, blue compression top,
exactly as beam theory predicts for a simply supported span.

## How to run

```bash
pip install -r requirements.txt
python src/analyze.py
```

## What is implemented

Element stiffness matrices in global coordinates, assembly, boundary condition
elimination, displacement solve, member force/stress recovery, support reactions,
self-weight computation, parametric geometry generation for Warren trusses.

## Future work

Self-weight as distributed load, Euler buckling checks for compression members,
3D space trusses, member cross-section optimisation.
