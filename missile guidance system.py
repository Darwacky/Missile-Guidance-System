import numpy as np
import matplotlib.pyplot as plt

# Missile and Target Parameters
missile_pos = np.array([0.0, 0.0])  # Initial missile position (x,y)
missile_vel = np.array([50.0, 0.0])  # Initial missile velocity (vx, vy)
target_pos = np.array([1000.0, 500.0])  # Initial target position
target_vel = np.array([-10.0, 5.0])  # Target velocity

# Guidance Parameters
N = 3.0  # Proportionality constant (N)
dt = 0.1  # Time step for simulation

# Simulation History
missile_path = [missile_pos.copy()]
target_path = [target_pos.copy()]

# Simulation Loop
max_time = 100.0  # Maximum simulation   action time (seconds)
time = 0.0
while time < max_time:
    # Calculate Line-of-Sight (LOS) vector and distance
    los_vec = target_pos - missile_pos
    dist = np.linalg.norm(los_vec)
    if dist == 0:
        print("Collision at zero distance.")
        break

    # Calculate Unit LOS vector
    los_unit = los_vec / dist

    # Calculate Line-of-Sight Rate (LOS_dot) approximation using relative velocity
    relative_velocity = target_vel - missile_vel
    # Use 3D vectors for cross product to keep compatibility with NumPy >=2.0.
    # Append a zero z-component and take the z (index 2) of the 3D cross product.
    los_rate_approx = np.cross(np.append(los_unit, 0.0), np.append(relative_velocity, 0.0))[2] / dist

    # Calculate commanded acceleration (perpendicular to LOS)
    commanded_accel_mag = N * np.linalg.norm(missile_vel) * los_rate_approx

    # Rotate LOS unit vector by 90 degrees to get perpendicular direction
    perpendicular_los_unit = np.array([-los_unit[1], los_unit[0]])
    commanded_accel = commanded_accel_mag * perpendicular_los_unit

    # Update missile velocity and position
    missile_vel = missile_vel + commanded_accel * dt
    missile_pos = missile_pos + missile_vel * dt

    # Update target position (assuming constant velocity)
    target_pos = target_pos + target_vel * dt

    # Store path for visualization
    missile_path.append(missile_pos.copy())
    target_path.append(target_pos.copy())

    # Check for interception (simple distance check)
    if dist < 10.0:  # Close enough to consider as hit
        print("Target intercepted!")
        break

    time += dt

# Plotting the Results
missile_path = np.array(missile_path)
target_path = np.array(target_path)

plt.figure(figsize=(10, 6))
plt.plot(missile_path[:, 0], missile_path[:, 1], label='Missile Path', color='tab:blue')
plt.plot(target_path[:, 0], target_path[:, 1], label='Target Path', color='tab:red')
plt.scatter(missile_path[0, 0], missile_path[0, 1], color='green', marker='o', label='Missile Start', zorder=5)
plt.scatter(target_path[0, 0], target_path[0, 1], color='orange', marker='x', label='Target Start', zorder=5)
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Missile Guidance Simulation')
plt.grid(True)
plt.show()