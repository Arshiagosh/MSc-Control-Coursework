# Convex Optimization: LMIs in control: Riccati, dissipativity and H-infinity

The Riccati inequality as an LMI, dissipativity and passivity checks for simple loops and an RLC circuit, the fastest decay rate as a generalised eigenvalue problem, and LMI-based state-feedback design (LQR-type and H-infinity).

## Files

- [`report_en.pdf`](report_en.pdf): English summary (LaTeX source next to it)
- [`report_fa.pdf`](report_fa.pdf): my original report in Persian
- `Q6.m`
- `Q8.m`
- `Q9.m`

## How to run

MATLAB with YALMIP (`lmilab` from the Robust Control Toolbox as the SDP solver). `Q6.m` runs. `Q8.m` has a missing parenthesis on its last line, and `Q9.m` uses strict inequalities that current YALMIP rejects; both are left as submitted.
