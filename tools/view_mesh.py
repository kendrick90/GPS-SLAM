"""Simple mesh viewer using PyVista"""
import pyvista as pv
import sys

mesh_path = sys.argv[1] if len(sys.argv) > 1 else "output/release/replica/office0/tsdf_mesh.ply"
print(f"Loading mesh: {mesh_path}")

mesh = pv.read(mesh_path)
print(f"Loaded: {mesh.n_points:,} points, {mesh.n_cells:,} cells")

plotter = pv.Plotter()
plotter.add_mesh(mesh, rgb=True if mesh.point_data.get('RGB') is not None else False)
plotter.enable_fly_to_right_click()
plotter.show()
