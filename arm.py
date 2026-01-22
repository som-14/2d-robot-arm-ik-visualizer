# 2D Robotic Arm Inverse Kinematics Visualizer


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#  Arm parameters 
L1 = 1.0          # Length of first link
L2 = 1.0          # Length of second link
target = np.array([1.2, 0.8])  # Initial target position [x, y]


fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-2.1, 2.1)
ax.set_ylim(-2.1, 2.1)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_title("2D 2-Link Robot Arm – Inverse Kinematics")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")

# Visual elements
arm_line, = ax.plot([], [], 'o-', lw=5, color='#1f77b4', markersize=10, zorder=2)
target_marker, = ax.plot(target[0], target[1], 'o', ms=14, color='red', zorder=3)
base_marker = ax.plot(0, 0, 'o', ms=12, color='black', zorder=3)[0]


def inverse_kinematics(x, y):
    """Geometric 2-link IK - returns theta1, theta2 (elbow down)"""
    r2 = x**2 + y**2
    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)  
    
    theta2 = np.arccos(cos_theta2)
   
    
    k1 = L1 + L2 * cos_theta2
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    
    return theta1, theta2


def get_arm_positions(theta1, theta2):
    """Calculate joint and end-effector positions"""
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    return [0, x1, x2], [0, y1, y2]


def update(frame):
    theta1, theta2 = inverse_kinematics(target[0], target[1])
    xs, ys = get_arm_positions(theta1, theta2)
    arm_line.set_data(xs, ys)
    target_marker.set_data([target[0]], [target[1]])
    return arm_line, target_marker



ani = FuncAnimation(fig, update, interval=40, blit=True)



def on_mouse_click(event):
    if event.inaxes != ax:
        return
    global target
    target = np.array([event.xdata, event.ydata])
   
    theta1, theta2 = inverse_kinematics(target[0], target[1])
    print(f"Target: ({target[0]:.2f}, {target[1]:.2f}) → θ₁ = {np.degrees(theta1):5.1f}°, θ₂ = {np.degrees(theta2):5.1f}°")


fig.canvas.mpl_connect('button_press_event', on_mouse_click)

plt.tight_layout()
plt.show()
