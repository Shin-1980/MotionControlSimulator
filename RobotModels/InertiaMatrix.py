import numpy as np

# Define link masses and inertia tensors
masses = [9.0, 17.4, 4.8, 0.82, 0.34, 0.09]
inertias = [
    np.diag([0.035, 0.1, 0.09]),
    np.diag([0.13, 0.524, 0.539]),
    np.diag([0.066, 0.086, 0.0125]),
    np.diag([0.0018, 0.0013, 0.0018]),
    np.diag([0.0003, 0.0004, 0.0003]),
    np.diag([0.00015, 0.00015, 0.00004])
]

# Placeholder for Jacobians (Jv, Jw) for simplicity
# These would be calculated based on DH parameters and forward kinematics
def compute_jacobian(q):
    # Example: Jacobians for each link (to be replaced with actual computation)
    Jv = [np.random.rand(3, 6) for _ in range(6)]  # Linear Jacobians
    Jw = [np.random.rand(3, 6) for _ in range(6)]  # Angular Jacobians
    return Jv, Jw

# Joint configuration
q = [0.5, 0.3, -0.2, 0.1, -0.5, 0.2]

# Compute Jacobians
Jv, Jw = compute_jacobian(q)

# Compute inertia matrix
M = np.zeros((6, 6))
for i in range(6):
    M += masses[i] * Jv[i].T @ Jv[i] + Jw[i].T @ inertias[i] @ Jw[i]

# Print inertia matrix
print("Inertia Matrix (M):")
print(M)
