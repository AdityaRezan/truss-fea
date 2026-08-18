# About Truss-FEA

## What this project is

Finite element analysis is how every bridge, tower and frame is designed, and
commercial FEA is a black box. This project implements the direct stiffness method
for 2D trusses from scratch and applies it to a genuine design question: how tall
should a Warren truss bridge be?

## The method

A truss member only carries axial force. Its stiffness along its own axis is EA/L;
rotating that into global x-y coordinates using the member's direction cosines
(c, s) gives a 4x4 element matrix built from the outer product of [-c -s c s].
Assembly adds each element matrix into the global K at the rows/columns of its two
nodes' degrees of freedom. Supports are enforced by deleting fixed DOF rows/columns,
the reduced system K_ff u_f = F_f is solved, and member forces are recovered from
end displacements projected onto the member axis. Reactions come from K u - F.

## What each file does

### src/truss.py
Truss2D class: geometry, element stiffness, assembly, solve, force/stress
recovery, weight. warren_bridge() generates parametric Warren truss geometry -
bottom chord nodes, top chord nodes offset by half a panel, alternating diagonals.

### src/analyze.py
1. Validation: a single bar under axial load, FEM vs the hand formula PL/EA.
   Exact agreement to machine precision.
2. Bridge analysis: 24 m span, 6 panels, pinned + roller supports, five 60 kN
   deck loads. Reports displacement, stresses, weight, safety factor.
3. Design sweep: 25 truss heights from 1.2 to 6.0 m, tracking peak stress and
   steel weight - a real structural optimisation trade-off curve.

## Results and what they mean

The deformed-shape plot is textbook mechanics made visible: bottom chord in tension
(red), top chord in compression (blue), diagonals alternating, max deflection at
midspan. Safety factor 1.39 at 3 m height is below the ~1.67 codes typically demand,
and the sweep shows why raising the truss helps: taller trusses give the chord
couple a longer lever arm, so chord forces drop roughly as 1/h.
