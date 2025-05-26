import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import time

def gaussian_3d(X, Y, Z, p, sigma, amplitude=1.0):
    """
    Calculate 3D Gaussian function centered at point p with given sigma and amplitude.
    
    Parameters:
    - X, Y, Z: Grid coordinates
    - p: Center point [x, y, z]
    - sigma: Standard deviation (can be scalar or [sigma_x, sigma_y, sigma_z])
    - amplitude: Peak value of the Gaussian
    
    Returns:
    - 3D array of Gaussian values
    """
    # Support different sigmas for each dimension
    if np.isscalar(sigma):
        sigma_x = sigma_y = sigma_z = sigma
    else:
        sigma_x, sigma_y, sigma_z = sigma
        
    # Vectorized calculation (more efficient)
    exponent = -0.5 * (
        ((X - p[0])**2 / sigma_x**2) + 
        ((Y - p[1])**2 / sigma_y**2) + 
        ((Z - p[2])**2 / sigma_z**2)
    )
    
    return amplitude * np.exp(exponent)

def potential_field(X, Y, Z, points, sigmas, amplitudes=None):
    """
    Calculate combined potential field from multiple Gaussian sources.
    
    Parameters:
    - X, Y, Z: Grid coordinates
    - points: Array of center points, shape (n_points, 3)
    - sigmas: Array of sigmas, shape (n_points,) or (n_points, 3)
    - amplitudes: Array of amplitudes, shape (n_points,)
    
    Returns:
    - 3D array of potential field values
    """
    pot = np.zeros_like(X)
    
    # Default amplitudes if not provided
    if amplitudes is None:
        amplitudes = np.ones(len(points))
    
    for i, p in enumerate(points):
        # Check if sigma is scalar or array
        sigma = sigmas[i]
        if np.isscalar(sigma):
            sigma = [sigma, sigma, sigma]  # Convert scalar to [sigma_x, sigma_y, sigma_z]
        pot += gaussian_3d(X, Y, Z, p, sigma, amplitudes[i])
        
    return pot


def plot_3d_isosurface(X, Y, Z, Pot, points, level=0.5, alpha=0.4, cmap='viridis'):
    """Plot isosurface of the potential field and the source points."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Use a colormap with built-in normalization
    norm = plt.Normalize(np.min(points), np.max(points))
    
    # Plot isosurface
    verts, faces, _, _ = measure.marching_cubes(Pot, level)
    
    # Scale vertices to match the grid coordinates
    x_min, x_max = np.min(X), np.max(X)
    y_min, y_max = np.min(Y), np.max(Y)
    z_min, z_max = np.min(Z), np.max(Z)
    
    x_scale = (x_max - x_min) / (Pot.shape[0] - 1)
    y_scale = (y_max - y_min) / (Pot.shape[1] - 1)
    z_scale = (z_max - z_min) / (Pot.shape[2] - 1)
    
    scaled_verts = np.zeros_like(verts)
    scaled_verts[:, 0] = verts[:, 0] * x_scale + x_min
    scaled_verts[:, 1] = verts[:, 1] * y_scale + y_min
    scaled_verts[:, 2] = verts[:, 2] * z_scale + z_min
    
    mesh = ax.plot_trisurf(
        scaled_verts[:, 0], scaled_verts[:, 1], faces, scaled_verts[:, 2],
        cmap=cmap, alpha=alpha, edgecolor='none'
    )
    
    # Plot the source points
    ax.scatter(
        points[:, 0], points[:, 1], points[:, 2], 
        color='red', s=80, label='Gaussian Centers', 
        edgecolor='black', linewidth=1
    )
    
    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'3D Gaussian Potential Field (Isosurface at {level})')
    
    plt.legend()
    return fig, ax

def plot_3d_slices(X, Y, Z, Pot, points, num_slices=3, cmap='viridis'):
    """Plot slices of the potential field at different z-levels."""
    fig = plt.figure(figsize=(16, 5))
    z_indices = np.linspace(0, Z.shape[2]-1, num_slices+2, dtype=int)[1:-1]
    
    for i, z_idx in enumerate(z_indices):
        ax = fig.add_subplot(1, num_slices, i+1)
        z_val = Z[0, 0, z_idx]
        
        # Plot 2D contour of the potential field
        contour = ax.contourf(
            X[:, :, z_idx], Y[:, :, z_idx], Pot[:, :, z_idx],
            levels=20, cmap=cmap, alpha=0.8
        )
        
        # Plot the points that are near this slice
        nearby_points = []
        for p in points:
            if abs(p[2] - z_val) < 0.5:  # Points close to the current slice
                nearby_points.append(p)
        
        if nearby_points:
            nearby_points = np.array(nearby_points)
            ax.scatter(
                nearby_points[:, 0], nearby_points[:, 1],
                color='red', s=50, edgecolor='white'
            )
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title(f'Z = {z_val:.2f}')
        
        plt.colorbar(contour, ax=ax, shrink=0.8, label='Potential')
    
    plt.tight_layout()
    return fig

def interactive_potential_field(grid_size=30, default_sigma=0.5):
    """Generate and visualize a 3D Gaussian potential field with interactive features."""
    try:
        from skimage import measure
    except ImportError:
        print("Please install scikit-image for isosurface plotting: pip install scikit-image")
        return
        
    # Create Gaussian centers
    points = np.array([
        [1.0, 1.0, 1.0],
        [3.0, 2.0, 2.0],
        [2.0, 4.0, 3.0],
        [4.0, 4.0, 4.0],
        [1.0, 3.0, 4.0],
    ])
    
    # Different sigmas for each point (can be scalar or [sigma_x, sigma_y, sigma_z])
    sigmas = np.array([
        [default_sigma, default_sigma, default_sigma],  # Isotropic
        [default_sigma*1.5, default_sigma*0.8, default_sigma],  # Anisotropic
        default_sigma*1.2,  # Scalar
        [default_sigma, default_sigma*2, default_sigma*0.7],  # Anisotropic
        default_sigma*0.8,  # Scalar
    ])
    
    # Different amplitudes for each Gaussian
    amplitudes = np.array([1.0, 0.8, 1.2, 0.7, 1.5])
    
    # Create a 3D grid
    x = np.linspace(0, 5, grid_size)
    y = np.linspace(0, 5, grid_size)
    z = np.linspace(0, 5, grid_size)
    X, Y, Z = np.meshgrid(x, y, z)
    
    print(f"Computing potential field on {grid_size}x{grid_size}x{grid_size} grid...")
    start_time = time.time()
    
    # Compute the potential field
    Pot = potential_field(X, Y, Z, points, sigmas, amplitudes)
    
    end_time = time.time()
    print(f"Computation completed in {end_time - start_time:.2f} seconds")
    
    # Plot 3D isosurface
    print("Plotting isosurface...")
    isosurface_fig, _ = plot_3d_isosurface(X, Y, Z, Pot, points, level=0.3)
    
    # Plot 2D slices
    print("Plotting 2D slices...")
    slices_fig = plot_3d_slices(X, Y, Z, Pot, points, num_slices=3)
    
    # Show plots
    plt.show()
    
    return X, Y, Z, Pot, points, sigmas, amplitudes

if __name__ == "__main__":
    # Make sure scikit-image is installed
    try:
        from skimage import measure
        interactive_potential_field(grid_size=50)
    except ImportError:
        print("Please install scikit-image for isosurface plotting:")
        print("pip install scikit-image")
        
        # Fall back to basic plotting without isosurface
        print("Falling back to basic visualization...")
        
        # Define parameters
        points = np.array([
            [1.0, 1.0, 1.0],
            [3.0, 2.0, 2.0],
            [2.0, 4.0, 3.0],
        ])
        sigma = 0.5
        
        # Create a 3D grid
        grid_size = 30
        x = np.linspace(0, 5, grid_size)
        y = np.linspace(0, 5, grid_size)
        z = np.linspace(0, 5, grid_size)
        X, Y, Z = np.meshgrid(x, y, z)
        
        # Compute potential field
        Pot = potential_field(X, Y, Z, points, sigma)
        
        # Create 3D plot with multiple slices
        fig = plt.figure(figsize=(15, 5))
        
        # Z-slice
        ax1 = fig.add_subplot(131, projection='3d')
        z_idx = grid_size // 2
        z_val = z[z_idx]
        ax1.plot_surface(X[:,:,z_idx], Y[:,:,z_idx], Pot[:,:,z_idx], cmap='viridis', alpha=0.8)
        ax1.set_title(f'Z-slice at z={z_val:.2f}')
        
        # X-slice
        ax2 = fig.add_subplot(132, projection='3d')
        x_idx = grid_size // 2
        x_val = x[x_idx]
        ax2.plot_surface(Y[x_idx,:,:], Z[x_idx,:,:], Pot[x_idx,:,:], cmap='plasma', alpha=0.8)
        ax2.set_title(f'X-slice at x={x_val:.2f}')
        
        # Y-slice
        ax3 = fig.add_subplot(133, projection='3d')
        y_idx = grid_size // 2
        y_val = y[y_idx]
        ax3.plot_surface(X[:,y_idx,:], Z[:,y_idx,:], Pot[:,y_idx,:], cmap='inferno', alpha=0.8)
        ax3.set_title(f'Y-slice at y={y_val:.2f}')
        
        # Add the points to all subplots
        for ax in [ax1, ax2, ax3]:
            ax.scatter(points[:,0], points[:,1], points[:,2], color='r', s=50)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z/Potential')
        
        plt.tight_layout()
        plt.show()