import numpy as np

class PUMA560:
    
  def __init__ (self):
    # Define DH parameters for PUMA 560
    self.DH = [
        [0, np.pi / 2, 0.6718, 0],    # Link 1
        [0.4318, 0, 0, 0],            # Link 2
        [0.0203, -np.pi / 2, 0, 0],   # Link 3
        [0, np.pi / 2, 0.150, 0],     # Link 4
        [0, -np.pi / 2, 0, 0],        # Link 5
        [0, 0, 0.085, 0]              # Link 6
    ]

    # Link masses (kg) for PUMA 560
    self.masses = [9.0, 17.4, 4.8, 0.82, 0.34, 0.09]

    # Define link masses and inertia tensors
    self.inertias = [
      np.diag([0.035, 0.1, 0.09]),
      np.diag([0.13, 0.524, 0.539]),
      np.diag([0.066, 0.086, 0.0125]),
      np.diag([0.0018, 0.0013, 0.0018]),
      np.diag([0.0003, 0.0004, 0.0003]),
      np.diag([0.00015, 0.00015, 0.00004])
    ]

    # Link centers of mass in local coordinates (example values, adjust for your setup)
    self.com_positions = [
      np.array([0, 0, 0.1]),  # Link 1
      np.array([0.1, 0, 0]),  # Link 2
      np.array([0.05, 0, 0]), # Link 3
      np.array([0, 0, 0.05]), # Link 4
      np.array([0, 0, 0.02]), # Link 5
      np.array([0, 0, 0.01])  # Link 6
    ] 

  # Compute forward kinematics
  def compute_TCP(self, joint_positions):

    # Function to compute transformation matrix from DH parameters
    def dh_to_transformation(a, alpha, d, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),                d],
            [0,              0,                            0,                            1]
        ])

    T = np.eye(4)  # Start with identity matrix
    for i, (a, alpha, d, _) in enumerate(self.DH):
        theta = joint_positions[i]  # Add the joint angle
        T_i = dh_to_transformation(a, alpha, d, theta)
        T = T @ T_i  # Multiply transformations

    tcp_position = T[:3, 3]

    return tcp_position

  def compute_transformation_matrix(self, a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

  def forward_kinematics(self, q):
    T = np.eye(4)
    transformations = []
    for i, (a, alpha, d, theta_offset) in enumerate(self.DH):
        theta = q[i] + theta_offset
        T_next = self.compute_transformation_matrix(a, alpha, d, theta)
        T = T @ T_next
        transformations.append(T)
    return transformations

  def compute_jacobian(self, q):
    n = len(q)
    T = np.eye(4)
    transformations = self.forward_kinematics(q)

    Jv = []
    Jw = []
    z_prev = np.array([0, 0, 1])  # z-axis of the base frame
    p_prev = np.zeros(3)          # Origin of the base frame

    for i in range(n):
        T_i = transformations[i]
        z_i = T[:3, 2]             # z-axis of the current frame
        p_i = T[:3, 3]             # Origin of the current frame

        # Compute the linear velocity Jacobian (cross product)
        Jv_i = np.cross(z_prev, (p_i - p_prev))
        Jv.append(Jv_i)

        # Compute the angular velocity Jacobian
        Jw_i = z_prev
        Jw.append(Jw_i)

        # Update the previous frame origin and z-axis
        z_prev = z_i
        p_prev = p_i

    return np.array(Jv).T, np.array(Jw).T

  def compute_gravity_vector(self, q):
    g = np.array([0, 0, -9.81])
    transformations = self.forward_kinematics(q)
    G = np.zeros(len(self.masses))
    for i in range(len(self.masses)):
        com_world = transformations[i] @ np.append(self.com_positions[i], 1)
        com_world = com_world[:3]
        F_gravity = self.masses[i] * g
        r = com_world
        torque = np.cross(r, F_gravity)
        G[i] = np.dot(torque, transformations[i][:3, 2])
    return G

  def compute_inertia_matrix(self, q):
    Jv, Jw = self.compute_jacobian(q)
    n = len(q)  # Number of joints
    M = np.zeros((n, n))  # Initialize the inertia matrix (6x6 for PUMA560)

    # Compute the contribution of each link
    for i in range(len(self.masses)):
        # Linear velocity contribution to the inertia matrix
        M += self.masses[i] * (Jv[:, i].T @ Jv[:, i])

        # Angular velocity contribution to the inertia matrix
        Jw_i = Jw[:, i].reshape(3, 1)  # Extract the angular Jacobian for joint i
        M += Jw_i.T @ self.inertias[i] @ Jw_i

    return M


  def compute_coriolis(self, q, qd):
    n = len(q)
    M = self.compute_inertia_matrix(q)
    C = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            for k in range(n):
                dM_ij_dqk = (self.compute_inertia_matrix(q + np.eye(n)[k] * 1e-6)[i, j] -
                            self.compute_inertia_matrix(q - np.eye(n)[k] * 1e-6)[i, j]) / 2e-6
                dM_ik_dqj = (self.compute_inertia_matrix(q + np.eye(n)[j] * 1e-6)[i, k] -
                            self.compute_inertia_matrix(q - np.eye(n)[j] * 1e-6)[i, k]) / 2e-6
                dM_jk_dqi = (self.compute_inertia_matrix(q + np.eye(n)[i] * 1e-6)[j, k] -
                            self.compute_inertia_matrix(q - np.eye(n)[i] * 1e-6)[j, k]) / 2e-6
                C[i, j] += 0.5 * (dM_ij_dqk + dM_ik_dqj - dM_jk_dqi) * qd[k]
    return C @ qd

  def compute_torque(self, q, qd, qdd):
    G = self.compute_gravity_vector(q)
    M = self.compute_inertia_matrix(q)
    Cqd = self.compute_coriolis(q, qd)
    return M @ qdd + Cqd + G
