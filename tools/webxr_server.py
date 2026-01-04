#!/usr/bin/env python3
"""
WebXR SLAM Viewer - Standalone Python Server
Streams point cloud and trajectory data to Quest 3 browser via WebSocket

Usage:
    python webxr_server.py                    # Demo mode with random data
    python webxr_server.py --ply model.ply    # Load PLY point cloud
    python webxr_server.py --port 6689        # Custom port
"""

import asyncio
import json
import struct
import argparse
import math
import numpy as np
from pathlib import Path
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import webbrowser

try:
    import websockets
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False
    print("Install websockets: pip install websockets")

# WebXR viewer HTML directory
WEBXR_DIR = Path(__file__).parent.parent / "plugins" / "viewers" / "webxr"


class WebXRServer:
    def __init__(self, port=6689, http_port=8080):
        self.port = port
        self.http_port = http_port
        self.clients = set()
        self.points = None
        self.colors = None
        self.trajectory = []
        self.frame_count = 0

    async def handler(self, websocket, path=None):
        """Handle WebSocket connections"""
        self.clients.add(websocket)
        client_addr = websocket.remote_address
        print(f"[WebXR] Client connected: {client_addr}")

        try:
            # Send initial scene data
            if self.points is not None:
                await self.send_point_cloud(websocket)

            if self.trajectory:
                await self.send_trajectory(websocket)

            # Keep connection alive and handle messages
            async for message in websocket:
                data = json.loads(message)
                print(f"[WebXR] Received: {data}")

        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            print(f"[WebXR] Client disconnected: {client_addr}")

    async def send_point_cloud(self, websocket=None):
        """Send point cloud to client(s)"""
        if self.points is None:
            return

        targets = [websocket] if websocket else self.clients

        # Binary protocol: [type:4][count:4][positions][colors]
        count = len(self.points)
        header = struct.pack('<II', 1, count)  # type=1 for point cloud

        positions = self.points.astype(np.float32).tobytes()
        colors = self.colors.astype(np.float32).tobytes() if self.colors is not None else \
                 (np.ones((count, 3), dtype=np.float32) * 0.7).tobytes()

        data = header + positions + colors

        for client in targets:
            try:
                await client.send(data)
            except:
                pass

    async def send_trajectory(self, websocket=None):
        """Send trajectory to client(s)"""
        if not self.trajectory:
            return

        targets = [websocket] if websocket else self.clients

        # Extract positions from trajectory poses
        positions = []
        for pose in self.trajectory:
            if isinstance(pose, np.ndarray) and pose.shape == (4, 4):
                positions.extend([pose[0, 3], pose[1, 3], pose[2, 3]])
            elif isinstance(pose, (list, tuple)) and len(pose) == 3:
                positions.extend(pose)

        count = len(positions) // 3
        header = struct.pack('<II', 3, count)  # type=3 for trajectory
        positions_bytes = np.array(positions, dtype=np.float32).tobytes()

        data = header + positions_bytes

        for client in targets:
            try:
                await client.send(data)
            except:
                pass

    async def send_pose(self, pose, frame_id):
        """Send camera pose to all clients"""
        # Binary: [type:4][count:4][matrix:64][frame_id:8]
        header = struct.pack('<II', 4, 1)  # type=4 for pose
        matrix_bytes = pose.astype(np.float32).tobytes()
        frame_bytes = struct.pack('<q', frame_id)

        data = header + matrix_bytes + frame_bytes

        for client in self.clients:
            try:
                await client.send(data)
            except:
                pass

    async def broadcast_json(self, msg):
        """Broadcast JSON message"""
        data = json.dumps(msg)
        for client in self.clients:
            try:
                await client.send(data)
            except:
                pass

    def set_point_cloud(self, points, colors=None):
        """Set the point cloud data"""
        self.points = np.array(points, dtype=np.float32)
        if colors is not None:
            self.colors = np.array(colors, dtype=np.float32)
        else:
            self.colors = None

    def add_trajectory_pose(self, pose):
        """Add a pose to the trajectory"""
        self.trajectory.append(pose)

    def clear_trajectory(self):
        """Clear the trajectory"""
        self.trajectory = []

    async def run_websocket_server(self):
        """Run the WebSocket server"""
        print(f"[WebXR] WebSocket server starting on ws://0.0.0.0:{self.port}")
        async with websockets.serve(self.handler, "0.0.0.0", self.port):
            await asyncio.Future()  # Run forever

    def run_http_server(self):
        """Run HTTP server for static files"""
        class Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, directory=None, **kwargs):
                super().__init__(*args, directory=str(WEBXR_DIR), **kwargs)

            def log_message(self, format, *args):
                pass  # Suppress logging

        server = HTTPServer(("0.0.0.0", self.http_port), Handler)
        print(f"[WebXR] HTTP server starting on http://0.0.0.0:{self.http_port}")
        server.serve_forever()


def load_ply(path):
    """Load PLY point cloud file"""
    from pathlib import Path

    points = []
    colors = []

    with open(path, 'rb') as f:
        # Read header
        line = f.readline().decode().strip()
        if line != 'ply':
            raise ValueError("Not a PLY file")

        vertex_count = 0
        properties = []
        in_header = True
        is_binary = False

        while in_header:
            line = f.readline().decode().strip()
            if line.startswith('format'):
                is_binary = 'binary' in line
            elif line.startswith('element vertex'):
                vertex_count = int(line.split()[-1])
            elif line.startswith('property'):
                properties.append(line.split()[2])
            elif line == 'end_header':
                in_header = False

        # Find property indices
        x_idx = properties.index('x') if 'x' in properties else 0
        y_idx = properties.index('y') if 'y' in properties else 1
        z_idx = properties.index('z') if 'z' in properties else 2

        has_color = 'red' in properties or 'r' in properties
        if has_color:
            r_idx = properties.index('red') if 'red' in properties else properties.index('r')
            g_idx = properties.index('green') if 'green' in properties else properties.index('g')
            b_idx = properties.index('blue') if 'blue' in properties else properties.index('b')

        # Read vertices
        if is_binary:
            # Simple binary reader (assumes float32 for positions)
            dtype = np.float32
            for _ in range(vertex_count):
                data = struct.unpack(f'<{len(properties)}f', f.read(4 * len(properties)))
                points.append([data[x_idx], data[y_idx], data[z_idx]])
                if has_color:
                    colors.append([data[r_idx]/255, data[g_idx]/255, data[b_idx]/255])
        else:
            # ASCII
            for _ in range(vertex_count):
                line = f.readline().decode().strip()
                values = [float(v) for v in line.split()]
                points.append([values[x_idx], values[y_idx], values[z_idx]])
                if has_color:
                    colors.append([values[r_idx]/255, values[g_idx]/255, values[b_idx]/255])

    return np.array(points), np.array(colors) if colors else None


def generate_demo_scene():
    """Generate a demo point cloud scene"""
    points = []
    colors = []

    # Create a room-like structure
    # Floor
    for x in np.linspace(-2, 2, 50):
        for z in np.linspace(-2, 2, 50):
            points.append([x, 0, z])
            colors.append([0.3, 0.3, 0.3])

    # Walls
    for y in np.linspace(0, 2, 20):
        for x in np.linspace(-2, 2, 30):
            points.append([x, y, -2])
            colors.append([0.6, 0.5, 0.4])
            points.append([x, y, 2])
            colors.append([0.6, 0.5, 0.4])
        for z in np.linspace(-2, 2, 30):
            points.append([-2, y, z])
            colors.append([0.5, 0.6, 0.4])
            points.append([2, y, z])
            colors.append([0.5, 0.6, 0.4])

    # Add some random furniture-like objects
    np.random.seed(42)

    # Table
    for x in np.linspace(-0.5, 0.5, 20):
        for z in np.linspace(-0.3, 0.3, 15):
            points.append([x, 0.8, z])
            colors.append([0.5, 0.3, 0.2])

    # Chair
    for x in np.linspace(0.7, 1.0, 10):
        for z in np.linspace(-0.2, 0.2, 10):
            for y in np.linspace(0, 0.5, 10):
                points.append([x, y, z])
                colors.append([0.3, 0.4, 0.6])

    # Some random objects
    for _ in range(500):
        x = np.random.uniform(-1.5, 1.5)
        y = np.random.uniform(0.1, 1.5)
        z = np.random.uniform(-1.5, 1.5)
        points.append([x, y, z])
        colors.append([np.random.uniform(0.3, 0.8) for _ in range(3)])

    return np.array(points), np.array(colors)


def generate_demo_trajectory():
    """Generate a circular camera trajectory"""
    trajectory = []

    for i in range(100):
        t = i / 100 * 2 * math.pi
        x = 1.5 * math.cos(t)
        z = 1.5 * math.sin(t)
        y = 1.2 + 0.3 * math.sin(t * 2)

        # Create 4x4 pose matrix (looking at center)
        pose = np.eye(4, dtype=np.float32)
        pose[0, 3] = x
        pose[1, 3] = y
        pose[2, 3] = z

        trajectory.append(pose)

    return trajectory


async def demo_animation(server):
    """Animate the demo scene"""
    trajectory = generate_demo_trajectory()

    frame_id = 0
    while True:
        # Update pose along trajectory
        pose_idx = frame_id % len(trajectory)
        pose = trajectory[pose_idx]

        if server.clients:
            await server.send_pose(pose, frame_id)

            # Periodically resend trajectory
            if frame_id % 100 == 0:
                server.trajectory = trajectory[:pose_idx + 1]
                await server.send_trajectory()

        frame_id += 1
        await asyncio.sleep(1/30)  # 30 FPS


async def main():
    parser = argparse.ArgumentParser(description='WebXR SLAM Viewer Server')
    parser.add_argument('--port', type=int, default=6689, help='WebSocket port')
    parser.add_argument('--http-port', type=int, default=8080, help='HTTP port for viewer')
    parser.add_argument('--ply', type=str, help='Load PLY point cloud file')
    parser.add_argument('--no-browser', action='store_true', help='Do not open browser')
    args = parser.parse_args()

    if not HAS_WEBSOCKETS:
        print("Error: websockets library required. Install with: pip install websockets")
        return

    server = WebXRServer(port=args.port, http_port=args.http_port)

    # Load or generate scene
    if args.ply:
        print(f"[WebXR] Loading PLY file: {args.ply}")
        points, colors = load_ply(args.ply)
        server.set_point_cloud(points, colors)
        print(f"[WebXR] Loaded {len(points)} points")
    else:
        print("[WebXR] Generating demo scene...")
        points, colors = generate_demo_scene()
        server.set_point_cloud(points, colors)
        server.trajectory = generate_demo_trajectory()
        print(f"[WebXR] Generated {len(points)} points")

    # Start HTTP server in background thread
    http_thread = threading.Thread(target=server.run_http_server, daemon=True)
    http_thread.start()

    # Print connection info
    print()
    print("=" * 60)
    print("WebXR SLAM Viewer")
    print("=" * 60)
    print(f"HTTP Viewer:  http://localhost:{args.http_port}/")
    print(f"WebSocket:    ws://localhost:{args.port}")
    print()
    print("On Quest 3:")
    print(f"  1. Open browser and go to: http://<your-pc-ip>:{args.http_port}/")
    print(f"  2. Enter WebSocket URL: ws://<your-pc-ip>:{args.port}")
    print(f"  3. Click Connect, then Enter VR")
    print("=" * 60)
    print()

    # Open browser
    if not args.no_browser:
        url = f"http://localhost:{args.http_port}/?server=ws://localhost:{args.port}"
        print(f"[WebXR] Opening browser: {url}")
        webbrowser.open(url)

    # Run WebSocket server with demo animation
    if not args.ply:
        await asyncio.gather(
            server.run_websocket_server(),
            demo_animation(server)
        )
    else:
        await server.run_websocket_server()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[WebXR] Shutting down...")
