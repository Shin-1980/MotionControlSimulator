import numpy as np

class PUMA560:
  DH = []
  
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

  # Compute forward kinematics
  def forward_kinematics(self, joint_positions):

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

# Joint positions (in radians)
joint_positions = [0.5, 0.3, -0.2, 0.1, -0.5, 0.2]

puma560 = PUMA560()

# Compute TCP transformation matrix
tcp_position = puma560.forward_kinematics(joint_positions)

# Output the TCP position
print("TCP Position (x, y, z):", tcp_position)
