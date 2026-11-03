import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt


np.random.seed(0)
A_points = np.random.randn(20,2) + np.array([2,2])
B_points = np.random.randn(20,2) + np.array([-2,-2])

# Define variables
w = cp.Variable(2)
b = cp.Variable()

# Constraints: A_points side >= 1, B_points side <= -1
constraints = [w.T @ A_points[i] + b >= 1 for i in range(A_points.shape[0])]
constraints += [w.T @ B_points[j] + b <= -1 for j in range(B_points.shape[0])]

# Objective: minimize L1 norm of w for uniqueness
obj = cp.Minimize(cp.norm1(w))

prob = cp.Problem(obj, constraints)
prob.solve()

print("Optimal w, b:", w.value, b.value)

# Plot
plt.figure(figsize=(6,6))
plt.scatter(A_points[:,0], A_points[:,1], color='blue', label='A')
plt.scatter(B_points[:,0], B_points[:,1], color='red', label='B')

# plot decision boundary w^T x + b = 0
xx = np.linspace(-5,5,100)
yy = -(w.value[0]*xx + b.value)/w.value[1]
plt.plot(xx, yy, 'k--', label='separating line')

plt.legend()
plt.title("Question 8: Separating Hyperplane via LP")
plt.xlabel("x1")
plt.ylabel("x2")
plt.grid(True)
plt.show()