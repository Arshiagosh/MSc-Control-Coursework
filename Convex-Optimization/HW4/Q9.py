import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
# Question 9: QP for distance between two polytopes
# Example polytope 1: unit square centered at origin
A1 = np.array([[1, 0], [ -1, 0], [0, 1], [0, -1]])
b1 = np.array([1, 1, 1, 1])

# Example polytope 2: shifted square
A2 = A1.copy()
b2 = b1 + 3  # shift by +3 in all constraints

# Variables x in P1, y in P2
x = cp.Variable(2)
y = cp.Variable(2)

# Objective: minimize squared distance
objective = cp.Minimize(cp.sum_squares(x - y))

# Constraints
constraints_q9 = [A1 @ x <= b1,
                  A2 @ y <= b2]

prob_q9 = cp.Problem(objective, constraints_q9)
prob_q9.solve()

dist = np.linalg.norm(x.value - y.value)
print("Minimum distance between polytopes:", dist)
