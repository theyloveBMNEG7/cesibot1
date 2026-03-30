import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('tracker_log.csv')

x = df['x_m'].to_numpy()   # lateral
z = df['z_m'].to_numpy()   # forward

x = -x
# z = -z
# Raw 2D positions
P = np.column_stack((x, z))

# detect our first 50 cm from the dataset

P0 = P[0]

calib_idx = None
for i in range(len(P)):
    dist = np.linalg.norm(P[i] - P0)
    if dist >= 0.50:
        calib_idx = i
        break

if calib_idx is None:
    print("Robot did not reach 50 cm")
    exit()

P_50 = P[calib_idx]


# Compute ideal direction 

direction = P_50 - P0
direction = direction / np.linalg.norm(direction)


# Create ideal line starting from 50cm point

# Length of ideal line equal to remaining distance
remaining_length = np.linalg.norm(P[-1] - P_50)

t = np.linspace(0, remaining_length, 1000)

ideal_line = P_50 + np.outer(t, direction)


# Corridor +-5cm

corridor_width = 0.05

# Perpendicular vector
perp = np.array([-direction[1], direction[0]])

upper_corridor = ideal_line + corridor_width * perp
lower_corridor = ideal_line - corridor_width * perp


plt.figure(figsize=(10, 10))

# Real trajectory (untouched)
plt.plot(x, z, label="Real Trajectory", linewidth=2)

# Calibration segment
plt.plot(x[:calib_idx+1],
         z[:calib_idx+1],
         linewidth=2.5,
         label="First 50 cm (Calibration)")

# Ideal line (starting at 50cm point)
plt.plot(ideal_line[:,0],
         ideal_line[:,1],
         linestyle='--',
         label="Ideal Direction")

# Corridor
plt.plot(upper_corridor[:,0],
         upper_corridor[:,1],
         linestyle=':', color = 'green',
         label="+5 cm Corridor")

plt.plot(lower_corridor[:,0],
         lower_corridor[:,1],
         linestyle=':',color = 'green',
         label="-5 cm Corridor")


# Mark calibration endpoint
plt.scatter(P_50[0], P_50[1], s=50, label="50 cm Point")

plt.xlabel("Lateral X [m]")
plt.ylabel("Forward Z [m]")
plt.title("Robot Trajectory with 50cm Calibration Reference")
plt.legend()
plt.axis("equal")
plt.grid(True)

plt.show()