import numpy as np
import matplotlib.pyplot as plt

def inverse_kinematics(x, y, L1, L2):
    # Calculate distance from origin to end-effector
    d = np.sqrt(x**2 + y**2)
    
    # Check if the point is reachable
    if d > (L1 + L2):
        raise ValueError("Target is unreachable")
    
    # Calculate the angle theta2
    cos_theta2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)
    theta2 = np.arctan2(sin_theta2, cos_theta2)
    
    # Calculate the angles k1 and k2 for theta1
    k1 = L1 + L2 * cos_theta2
    k2 = L2 * sin_theta2
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    
    return theta1, theta2

def compute_end_effector(theta1, theta2, L1, L2):
    x0, y0 = 0, 0
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    return x2, y2

def plot_trajectory(points, L1, L2, title):
    # Compute the end-effector positions for each point
    end_effector_positions = []
    q1s = []
    q2s = []
    recalculated_positions = []
    for x, y in points:
        theta1, theta2 = inverse_kinematics(x, y, L1, L2)
        q1s.append(theta1)
        q2s.append(theta2)
        x_end, y_end = compute_end_effector(theta1, theta2, L1, L2)
        end_effector_positions.append((x, y))
        recalculated_positions.append((x_end, y_end))

    # Print q1 and q2 values
    print(f'{title}')
    for i, (q1, q2) in enumerate(zip(q1s, q2s)):
        print(f'Point {i+1}: q1 = {np.degrees(q1):.2f}°, q2 = {np.degrees(q2):.2f}°')
    print()

    # Plot the original and recalculated trajectories
    plt.figure(figsize=(10, 10))
    end_effector_positions = np.array(end_effector_positions)
    recalculated_positions = np.array(recalculated_positions)

    plt.plot(end_effector_positions[:, 0], end_effector_positions[:, 1], 'o-', linewidth=2, color='blue', label='Original')
    plt.plot(recalculated_positions[:, 0], recalculated_positions[:, 1], 'x-', linewidth=2, color='red', label='Recalculated')
    
    plt.scatter(end_effector_positions[:, 0], end_effector_positions[:, 1], color='blue')
    plt.scatter(recalculated_positions[:, 0], recalculated_positions[:, 1], color='red')

    plt.xlim(-L1-L2-1, L1+L2+1)
    plt.ylim(-L1-L2-1, L1+L2+1)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.show()

# Parameters
L1 = 5
L2 = 5

# Points to draw the letter "M"
points_M = [
    (0, 0), (0, 8), (3, 4), (6, 8), (6, 0)  # Vertical line, diagonal down, diagonal up, vertical line
]
plot_trajectory(points_M, L1, L2, 'Trajectory of the Letter "M" by 2R Manipulator')

# Points to draw a circle
circle_radius = 5
circle_points = []
for theta in np.linspace(0, 2 * np.pi, 100):
    x = circle_radius * np.cos(theta)
    y = circle_radius * np.sin(theta)
    circle_points.append((x, y))
plot_trajectory(circle_points, L1, L2, 'Trajectory of a Circle by 2R Manipulator')

# Points to draw the letter "N"
points_N = [
    (0, 0), (0, 8), (6, 0), (6, 8)  # Vertical line, diagonal down, vertical line
]
plot_trajectory(points_N, L1, L2, 'Trajectory of the Letter "N" by 2R Manipulator')
