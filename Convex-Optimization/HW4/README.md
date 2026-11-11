# Convex Optimization: LP, SDP and quasi-convex reformulations

Rewriting norm problems as LPs, matrix-fractional constraints as LMIs, and ratio objectives as quasi-convex problems. Three small solvers: an LP that separates two point clouds, a QP for the distance between two polytopes, and a mixed-integer QP for the optimal set point of a CSTR with a piecewise cost.

## Files

- [`report_en.pdf`](report_en.pdf): English summary (LaTeX source next to it)
- `Q10.m`
- `Q8.py`
- `Q9.py`

## How to run

`Q8.py`, `Q9.py`: Python with CVXPY (see `requirements.txt` at the top level). `Q10.m`: MATLAB with [YALMIP](https://yalmip.github.io/) (uses its built-in branch-and-bound with `quadprog`).
