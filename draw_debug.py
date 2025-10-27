import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def load_obj(filename):
    """
    Load vertices and faces from a Wavefront OBJ file.
    Returns:
        vertices: Nx3 numpy array
        faces: Mx3 numpy array (0-based indices)
    """
    vertices = []
    faces = []

    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()
                x, y, z = map(float, parts[1:4])
                vertices.append([x, y, z])
            elif line.startswith('f '):
                parts = line.strip().split()
                # OBJ is 1-indexed, convert to 0-indexed
                face = [int(p.split('/')[0]) - 1 for p in parts[1:4]]
                faces.append(face)

    return np.array(vertices), np.array(faces)


# Load OBJ
vertices, faces = load_obj("triangles.obj")

# Prepare 3D plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

triangles = [vertices[face] for face in faces]
poly3d = Poly3DCollection(triangles, alpha=0.5,
                          facecolor='cyan', edgecolor='black')
ax.add_collection3d(poly3d)

# Auto scale
ax.set_xlim(vertices[:, 0].min()-0.1, vertices[:, 0].max()+0.1)
ax.set_ylim(vertices[:, 1].min()-0.1, vertices[:, 1].max()+0.1)
ax.set_zlim(vertices[:, 2].min()-0.1, vertices[:, 2].max()+0.1)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.view_init(elev=30, azim=45)

# Save image locally
# plt.savefig("triangles_visualization.png", dpi=300)
plt.show()
plt.close()  # Close figure to free memory
