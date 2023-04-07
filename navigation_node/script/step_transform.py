import pyvista as pv
import meshio
import pyassimp
import mesh
import open3d as o3d
   
# Load the STEP file
mesh = o3d.io.read_triangle_mesh("00_rmuc2023.STEP")
