"""
FPS-style Mesh Viewer for GPS-SLAM outputs

Controls:
  WASD - Move forward/left/back/right
  Space/C - Move up/down
  Mouse - Look around (hold right-click)
  Scroll - Adjust move speed
  ESC - Exit
"""
import sys
import numpy as np
import argparse
import os

# Check for required packages
try:
    import trimesh
except ImportError:
    print("Installing trimesh...")
    os.system("pip install trimesh")
    import trimesh

try:
    import pyglet
    from pyglet.gl import *
    from pyglet.math import Vec3
except ImportError:
    print("Installing pyglet...")
    os.system("pip install pyglet")
    import pyglet
    from pyglet.gl import *
    from pyglet.math import Vec3


class FPSCamera:
    def __init__(self):
        self.position = Vec3(0, 0, 3)
        self.yaw = -90.0
        self.pitch = 0.0
        self.move_speed = 2.0
        self.mouse_sensitivity = 0.2
        self.front = Vec3(0, 0, -1)
        self.up = Vec3(0, 1, 0)
        self.right = Vec3(1, 0, 0)
        self._update_vectors()

    def _update_vectors(self):
        rad_yaw = np.radians(self.yaw)
        rad_pitch = np.radians(self.pitch)
        front_x = np.cos(rad_yaw) * np.cos(rad_pitch)
        front_y = np.sin(rad_pitch)
        front_z = np.sin(rad_yaw) * np.cos(rad_pitch)
        self.front = Vec3(front_x, front_y, front_z).normalize()
        self.right = self.front.cross(Vec3(0, 1, 0)).normalize()
        self.up = self.right.cross(self.front).normalize()

    def process_mouse(self, dx, dy):
        self.yaw += dx * self.mouse_sensitivity
        self.pitch -= dy * self.mouse_sensitivity
        self.pitch = max(-89.0, min(89.0, self.pitch))
        self._update_vectors()

    def move_forward(self, delta):
        self.position = Vec3(
            self.position.x + self.front.x * delta * self.move_speed,
            self.position.y + self.front.y * delta * self.move_speed,
            self.position.z + self.front.z * delta * self.move_speed
        )

    def move_right(self, delta):
        self.position = Vec3(
            self.position.x + self.right.x * delta * self.move_speed,
            self.position.y + self.right.y * delta * self.move_speed,
            self.position.z + self.right.z * delta * self.move_speed
        )

    def move_up(self, delta):
        self.position = Vec3(
            self.position.x,
            self.position.y + delta * self.move_speed,
            self.position.z
        )

    def apply_view(self):
        target = Vec3(
            self.position.x + self.front.x,
            self.position.y + self.front.y,
            self.position.z + self.front.z
        )
        gluLookAt(
            self.position.x, self.position.y, self.position.z,
            target.x, target.y, target.z,
            self.up.x, self.up.y, self.up.z
        )


class MeshViewer(pyglet.window.Window):
    def __init__(self, mesh_path, max_faces=500000, width=1280, height=720):
        super().__init__(width, height, caption=f"GPS-SLAM Viewer", resizable=True)

        self.camera = FPSCamera()
        self.keys = set()
        self.mouse_captured = False
        self.mesh_path = mesh_path
        self.max_faces = max_faces
        self.loading = True
        self.load_progress = 0
        self.vertices = None
        self.colors = None
        self.normals = None
        self.indices = None
        self.vbo_vertex = None
        self.vbo_color = None
        self.vbo_normal = None
        self.vbo_index = None
        self.num_indices = 0

        # Setup basic GL
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.1, 0.1, 0.15, 1.0)

        # Start loading in background
        pyglet.clock.schedule_once(self._load_mesh, 0.1)
        pyglet.clock.schedule_interval(self.update, 1/60)

        print(f"\nLoading mesh: {mesh_path}")
        print("Please wait...")

    def _load_mesh(self, dt):
        try:
            print("Reading PLY file...")
            mesh = trimesh.load(self.mesh_path)

            original_faces = len(mesh.faces)
            print(f"Original: {len(mesh.vertices):,} vertices, {original_faces:,} faces")

            # Decimate if too large
            if original_faces > self.max_faces:
                print(f"Decimating to {self.max_faces:,} faces...")
                ratio = self.max_faces / original_faces
                mesh = mesh.simplify_quadric_decimation(self.max_faces)
                print(f"Decimated: {len(mesh.vertices):,} vertices, {len(mesh.faces):,} faces")

            # Center and scale
            center = mesh.centroid
            extent = max(mesh.extents)
            scale = 2.0 / extent if extent > 0 else 1.0
            mesh.apply_translation(-center)
            mesh.apply_scale(scale)

            # Extract data
            self.vertices = np.array(mesh.vertices, dtype=np.float32).flatten()

            # Colors
            if hasattr(mesh.visual, 'vertex_colors') and mesh.visual.vertex_colors is not None:
                self.colors = np.array(mesh.visual.vertex_colors[:, :3], dtype=np.float32).flatten() / 255.0
            else:
                self.colors = np.ones(len(mesh.vertices) * 3, dtype=np.float32) * 0.7

            # Normals
            if hasattr(mesh, 'vertex_normals') and mesh.vertex_normals is not None:
                self.normals = np.array(mesh.vertex_normals, dtype=np.float32).flatten()
            else:
                self.normals = np.zeros_like(self.vertices)
                self.normals[2::3] = 1.0  # Default Z-up normals

            # Indices
            self.indices = np.array(mesh.faces, dtype=np.uint32).flatten()
            self.num_indices = len(self.indices)

            # Create VBOs
            self._create_vbos()

            # Position camera based on mesh
            self.camera.position = Vec3(0, 0, 2.5)

            self.loading = False
            print(f"\nMesh loaded! {self.num_indices // 3:,} triangles")
            print("\nControls:")
            print("  WASD - Move")
            print("  Space/C - Up/Down")
            print("  Right-click + drag - Look around")
            print("  Scroll - Adjust speed")
            print("  ESC - Exit")

        except Exception as e:
            print(f"Error loading mesh: {e}")
            import traceback
            traceback.print_exc()
            self.loading = False

    def _create_vbos(self):
        # Create VBOs
        self.vbo_vertex = GLuint()
        self.vbo_color = GLuint()
        self.vbo_normal = GLuint()
        self.vbo_index = GLuint()

        glGenBuffers(1, self.vbo_vertex)
        glGenBuffers(1, self.vbo_color)
        glGenBuffers(1, self.vbo_normal)
        glGenBuffers(1, self.vbo_index)

        # Upload vertex data
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_vertex)
        glBufferData(GL_ARRAY_BUFFER, self.vertices.nbytes,
                     (GLfloat * len(self.vertices))(*self.vertices), GL_STATIC_DRAW)

        # Upload color data
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_color)
        glBufferData(GL_ARRAY_BUFFER, self.colors.nbytes,
                     (GLfloat * len(self.colors))(*self.colors), GL_STATIC_DRAW)

        # Upload normal data
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_normal)
        glBufferData(GL_ARRAY_BUFFER, self.normals.nbytes,
                     (GLfloat * len(self.normals))(*self.normals), GL_STATIC_DRAW)

        # Upload index data
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.vbo_index)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, self.indices.nbytes,
                     (GLuint * len(self.indices))(*self.indices), GL_STATIC_DRAW)

        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)

        print("VBOs created successfully")

    def on_draw(self):
        self.clear()

        if self.loading:
            self._draw_loading()
            return

        if self.vbo_vertex is None:
            return

        # Set projection
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60, self.width / self.height, 0.01, 100.0)

        # Set view
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.camera.apply_view()

        # Setup lighting
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        light_pos = (GLfloat * 4)(
            self.camera.position.x,
            self.camera.position.y + 1,
            self.camera.position.z,
            1.0
        )
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos)

        ambient = (GLfloat * 4)(0.3, 0.3, 0.3, 1.0)
        diffuse = (GLfloat * 4)(0.8, 0.8, 0.8, 1.0)
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambient)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse)

        # Draw mesh with VBOs
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)

        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_vertex)
        glVertexPointer(3, GL_FLOAT, 0, None)

        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_color)
        glColorPointer(3, GL_FLOAT, 0, None)

        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_normal)
        glNormalPointer(GL_FLOAT, 0, None)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.vbo_index)
        glDrawElements(GL_TRIANGLES, self.num_indices, GL_UNSIGNED_INT, None)

        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
        glDisableClientState(GL_NORMAL_ARRAY)

        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)

        glDisable(GL_LIGHTING)

        # Draw HUD
        self._draw_hud()

    def _draw_loading(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, self.width, 0, self.height, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        label = pyglet.text.Label(
            "Loading mesh... Please wait",
            font_name='Arial', font_size=24,
            x=self.width//2, y=self.height//2,
            anchor_x='center', anchor_y='center',
            color=(255, 255, 255, 255)
        )
        label.draw()

    def _draw_hud(self):
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.width, 0, self.height, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        glDisable(GL_DEPTH_TEST)

        pos = self.camera.position
        label = pyglet.text.Label(
            f"Pos: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) | Speed: {self.camera.move_speed:.1f} | Triangles: {self.num_indices//3:,}",
            font_name='Arial', font_size=12,
            x=10, y=self.height - 20,
            color=(255, 255, 255, 200)
        )
        label.draw()

        controls = pyglet.text.Label(
            "WASD: Move | Space/C: Up/Down | Right-click+drag: Look | Scroll: Speed | ESC: Exit",
            font_name='Arial', font_size=10,
            x=10, y=10,
            color=(200, 200, 200, 180)
        )
        controls.draw()

        glEnable(GL_DEPTH_TEST)

        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

    def update(self, dt):
        if self.loading:
            return

        # Handle movement
        if 'w' in self.keys or 'W' in self.keys:
            self.camera.move_forward(dt)
        if 's' in self.keys or 'S' in self.keys:
            self.camera.move_forward(-dt)
        if 'a' in self.keys or 'A' in self.keys:
            self.camera.move_right(-dt)
        if 'd' in self.keys or 'D' in self.keys:
            self.camera.move_right(dt)
        if 'space' in self.keys:
            self.camera.move_up(dt)
        if 'c' in self.keys or 'C' in self.keys:
            self.camera.move_up(-dt)

    def on_key_press(self, symbol, modifiers):
        if symbol == pyglet.window.key.ESCAPE:
            self.close()
        elif symbol == pyglet.window.key.SPACE:
            self.keys.add('space')
        else:
            try:
                self.keys.add(chr(symbol).lower())
            except:
                pass

    def on_key_release(self, symbol, modifiers):
        if symbol == pyglet.window.key.SPACE:
            self.keys.discard('space')
        else:
            try:
                self.keys.discard(chr(symbol).lower())
            except:
                pass

    def on_mouse_press(self, x, y, button, modifiers):
        if button == pyglet.window.mouse.RIGHT:
            self.mouse_captured = True
            self.set_exclusive_mouse(True)

    def on_mouse_release(self, x, y, button, modifiers):
        if button == pyglet.window.mouse.RIGHT:
            self.mouse_captured = False
            self.set_exclusive_mouse(False)

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if self.mouse_captured:
            self.camera.process_mouse(dx, dy)

    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        self.camera.move_speed *= (1.1 ** scroll_y)
        self.camera.move_speed = max(0.1, min(50.0, self.camera.move_speed))

    def on_resize(self, width, height):
        glViewport(0, 0, width, height)
        return pyglet.event.EVENT_HANDLED


def main():
    parser = argparse.ArgumentParser(description='FPS-style Mesh Viewer')
    parser.add_argument('mesh_path', nargs='?',
                        default='output/release/replica/office0/tsdf_mesh.ply',
                        help='Path to PLY mesh file')
    parser.add_argument('--max-faces', type=int, default=500000,
                        help='Maximum faces to render (will decimate if larger)')
    args = parser.parse_args()

    if not os.path.exists(args.mesh_path):
        print(f"Error: File not found: {args.mesh_path}")
        sys.exit(1)

    viewer = MeshViewer(args.mesh_path, max_faces=args.max_faces)
    pyglet.app.run()


if __name__ == '__main__':
    main()
