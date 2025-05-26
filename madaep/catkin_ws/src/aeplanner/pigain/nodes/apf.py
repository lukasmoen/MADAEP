import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define parameters
sigma = 0.5
grid_size = 20
num_points = 1000
update_plot_every = 50  # Update plot every N points

# Create 3D grid
x = np.linspace(-5, 5, grid_size)
y = np.linspace(-5, 5, grid_size)
z = np.linspace(-5, 5, grid_size)
X, Y, Z = np.meshgrid(x, y, z, indexing='ij')

# Initialize previous points and field
previous_points = []
field = np.zeros_like(X)

def update_field(X, Y, Z, previous_points, sigma=0.5):
    """Recalculate the field based on previous points."""
    field = np.zeros_like(X)
    for pt in previous_points:
        diff_x = X - pt[0]
        diff_y = Y - pt[1]
        diff_z = Z - pt[2]
        dist_squared = diff_x**2 + diff_y**2 + diff_z**2
        field += np.exp(-dist_squared / (2 * sigma**2))
    return field

def plot_field(X, Y, Z, field, threshold=0.1):
    """Plot the field where values are above a threshold."""
    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')

    mask = field > threshold
    ax.scatter(X[mask], Y[mask], Z[mask], c=field[mask], cmap='hot', marker='o', alpha=0.6, s=5)

    ax.set_xlim([x.min(), x.max()])
    ax.set_ylim([y.min(), y.max()])
    ax.set_zlim([z.min(), z.max()])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.title(f"Field after {len(previous_points)} points")
    plt.show()

def add_point_to_field(X, Y, Z, field, new_point, sigma=0.5):
    """Add the contribution of a new point directly to the existing field."""
    diff_x = X - new_point[0]
    diff_y = Y - new_point[1]
    diff_z = Z - new_point[2]
    dist_squared = diff_x**2 + diff_y**2 + diff_z**2
    field += np.exp(-dist_squared / (2 * sigma**2))
    return field

# Loop to add random points and update field
for i in range(num_points):
    # Add random point
    new_point = np.random.uniform(-5, 5, size=3)
    previous_points.append(new_point)

    # Update field by adding just the new point's effect
    field = add_point_to_field(X, Y, Z, field, new_point, sigma=sigma)

    # Plot every N points
    if (i+1) % update_plot_every == 0:
        print(f"Plotting at step {i+1}...")
        plot_field(X, Y, Z, field, threshold=0.1)

