import numpy as np

# Constants
g = np.array([0, 0, -9.81])  # Gravitational acceleration (m/s^2)

# Link masses (kg) for PUMA 560
masses = [9.0, 17.4, 4.8, 0.82, 0.34, 0.09]

# Link centers of mass in local coordinates (example values, adjust for your setup)
com_positions = [
    np.array([0, 0, 0.1]),  # Link 1
    np.array([0.1, 0, 0]),  # Link 2
    np.array([0.05, 0, 0]), # Link 3
    np.array([0, 0, 0.05]), # Link 4
    np.array([0, 0, 0.02]), # Link 5
    np.array([0, 0, 0.01])  # Link 6
]

# DH parameters for PUMA 560
DH = [
    [0, np.pi / 2, 0.6718, 0],
    [0.4318, 0, 0, 0],
    [0.0203, -np.pi / 2, 0, 0],
    [0, np.pi / 2, 0.150, 0],
    [0, -np.pi / 2, 0, 0],
    [0, 0, 0.085, 0]
]

# Compute transformation matrices from DH parameters
def compute_transformation_matrix(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Forward kinematics to compute position of each link's center of mass
def forward_kinematics(q):
    T = np.eye(4)  # Initialize as identity matrix
    transformations = []
    for i, (a, alpha, d, theta_offset) in enumerate(DH):
        theta = q[i] + theta_offset
        T_next = compute_transformation_matrix(a, alpha, d, theta)
        T = T @ T_next
        transformations.append(T)
    return transformations

# Compute gravity vector
def compute_gravity_vector(q):
    transformations = forward_kinematics(q)
    G = np.zeros(6)  # Gravity vector
    for i in range(6):
        # Position of the center of mass in world coordinates
        com_world = transformations[i] @ np.append(com_positions[i], 1)  # Homogeneous coordinates
        com_world = com_world[:3]  # Extract (x, y, z)

        # Gravity force at the center of mass
        F_gravity = masses[i] * g

        # Torque due to gravity at joint
        r = com_world  # Vector from joint to center of mass
        torque = np.cross(r, F_gravity)

        # Contribution to gravity torque
        G[i] = np.dot(torque, transformations[i][:3, 2])  # Project onto the joint axis
    return G

# Example joint configuration (radians)
q = [0.5, 0.3, -0.2, 0.1, -0.5, 0.2]

# Compute the gravity vector
G = compute_gravity_vector(q)

# Output the gravity vector
print("Gravity Vector (G):")
print(G)
