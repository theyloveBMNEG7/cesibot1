import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 1. Load your tracker data (relative coordinates – start at 0,0)
df = pd.read_csv("tracker_log.csv")
df = df.dropna(subset=['x_m', 'z_m'])

# Use x = lateral, z = forward (change if your forward is another axis)
x = df['x_m'].to_numpy()
z = df['z_m'].to_numpy()

# Normalize to start at (0,0)
x = x - x[0]
z = z - z[0]

points_real = np.array(list(zip(x, z)), dtype=np.float32)  # (N, 2)

# 2. Load terrain image
img = cv2.imread("terrain_2026.png")
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # matplotlib likes RGB

height, width = img.shape[:2]

# 3. Define 4 corresponding points (real meters → image pixels)
# You MUST measure/adjust these 4 points on YOUR image!
pts_src = np.array([
    [0.00, 0.00],      # real start (after normalization)
    [0.00, 2.00],      # real 2 m forward
    [0.50, 2.00],      # real 0.5 m right + 2 m forward
    [0.50, 0.00]       # real 0.5 m right
], dtype=np.float32)

# Corresponding pixels on the terrain image (measure them!)
pts_dst = np.array([
    [150, 1850],       # bottom-left on image
    [150, 350],        # top-left-ish (forward)
    [700, 350],        # top-right-ish
    [700, 1850]        # bottom-right-ish
], dtype=np.float32)

# 4. Compute homography matrix
H, _ = cv2.findHomography(pts_src, pts_dst, cv2.RANSAC)

# 5. Transform robot path to image coordinates
points_img = cv2.perspectiveTransform(points_real.reshape(-1, 1, 2), H)
points_img = points_img.reshape(-1, 2).astype(np.int32)

# 6. Plot everything
plt.figure(figsize=(12, 10))
plt.imshow(img)

# Robot path
plt.plot(points_img[:, 0], points_img[:, 1],
         color='blue', linewidth=3, label='Actual path')

# Start / end markers
plt.scatter(points_img[0, 0], points_img[0, 1],
            c='lime', s=200, edgecolors='black', label='Start')
plt.scatter(points_img[-1, 0], points_img[-1, 1],
            c='red', s=200, edgecolors='black', label='End')

plt.title("Robot path overlaid on Coupe de France 2026 terrain\n(±5 cm precision corridor not shown here)")
plt.legend()
plt.axis('off')
plt.tight_layout()
plt.show()