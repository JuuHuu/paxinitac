import csv
import math
from pathlib import Path
from typing import List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class TactileForceMarkerNode(Node):
    """Converts tactile PointCloud2 force vectors into RViz MarkerArray arrows."""

    def __init__(self) -> None:
        super().__init__('tactile_force_marker_array')

        default_csv = self._default_csv_path()

        self.declare_parameter('pointcloud_topic', '/tactile/sensor_1/raw_point_data')
        self.declare_parameter('marker_topic', '/tactile/sensor_1/force_markers')
        self.declare_parameter('base_points_csv', str(default_csv))
        self.declare_parameter('base_point_scale', 0.001)
        self.declare_parameter('vector_scale', 0.0002)
        self.declare_parameter('max_magnitude', 255.0)
        self.declare_parameter('shaft_diameter', 0.0005)
        self.declare_parameter('head_diameter', 0.0008)
        self.declare_parameter('head_length', 0.0008)
        self.declare_parameter('default_frame', 'sensor_1_link')
        self.declare_parameter('use_surface_normals', True)
        self.declare_parameter('normal_k_neighbors', 8)

        self._vector_scale = float(self.get_parameter('vector_scale').value)
        self._max_magnitude = max(1e-6, float(self.get_parameter('max_magnitude').value))
        self._shaft_diameter = float(self.get_parameter('shaft_diameter').value)
        self._head_diameter = float(self.get_parameter('head_diameter').value)
        self._head_length = float(self.get_parameter('head_length').value)
        self._default_frame = str(self.get_parameter('default_frame').value)
        self._base_point_scale = float(self.get_parameter('base_point_scale').value)
        self._use_surface_normals = bool(self.get_parameter('use_surface_normals').value)
        self._normal_k_neighbors = max(3, int(self.get_parameter('normal_k_neighbors').value))

        self._base_points = self._load_base_points(Path(self.get_parameter('base_points_csv').value))
        self._base_normals = self._compute_normals(self._base_points) if self._use_surface_normals else None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pointcloud_topic = str(self.get_parameter('pointcloud_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)

        self._subscription = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self._on_pointcloud,
            qos,
        )
        self._publisher = self.create_publisher(MarkerArray, marker_topic, qos)

        self.get_logger().info(
            'Tactile force markers ready:\n'
            f'  base CSV: {Path(self.get_parameter("base_points_csv").value)} ({len(self._base_points)} points, scale={self._base_point_scale})\n'
            f'  subscribing to {pointcloud_topic}\n'
            f'  publishing markers on {marker_topic}'
        )

    def _default_csv_path(self) -> Path:
        try:
            share_dir = Path(get_package_share_directory('tactile_raw_point_publisher'))
            return share_dir / 'data' / 'PX6AX-GEN3-DP-S2716-Core.csv'
        except PackageNotFoundError:
            return Path(__file__).resolve().parents[4] / 'PX6AX-GEN3-DP-S2716-Core.csv'

    def _load_base_points(self, csv_path: Path) -> List[Tuple[float, float, float]]:
        if not csv_path.exists():
            raise FileNotFoundError(f'CSV file not found: {csv_path}')

        points: List[Tuple[float, float, float]] = []
        with csv_path.open('r', newline='') as csvfile:
            reader = csv.reader(csvfile)
            next(reader, None)  # drop header
            for row in reader:
                if len(row) < 4:
                    continue
                try:
                    x, y, z = float(row[1]), float(row[2]), float(row[3])
                except ValueError:
                    self.get_logger().warn(f'Unable to parse base row: {row}', throttle_duration_sec=2.0)
                    continue
                scale = self._base_point_scale
                points.append((x * scale, y * scale, z * scale))

        if not points:
            raise ValueError(f'No base points could be read from {csv_path}')
        return points

    def _compute_normals(self, points: List[Tuple[float, float, float]]) -> List[np.ndarray]:
        arr = np.asarray(points, dtype=np.float64)
        centroid = arr.mean(axis=0)
        normals: List[np.ndarray] = []
        for idx, point in enumerate(arr):
            diffs = arr - point
            distances = np.linalg.norm(diffs, axis=1)
            neighbor_idx = np.argsort(distances)[1:self._normal_k_neighbors + 1]
            neighbors = arr[neighbor_idx] if neighbor_idx.size else arr
            neighbor_vectors = neighbors - point
            if neighbor_vectors.shape[0] < 3:
                normals.append(np.array([0.0, 0.0, 1.0], dtype=np.float64))
                continue
            cov = np.cov(neighbor_vectors, rowvar=False)
            eigvals, eigvecs = np.linalg.eigh(cov)
            normal = eigvecs[:, np.argmin(eigvals)]
            if np.linalg.norm(normal) < 1e-9:
                normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            if np.dot(normal, point - centroid) < 0:
                normal = -normal
            normals.append(normal / np.linalg.norm(normal))
        return normals

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        vectors = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not vectors:
            self.get_logger().warn('Received empty PointCloud2; skipping markers.', throttle_duration_sec=1.0)
            return

        if len(vectors) != len(self._base_points):
            self.get_logger().warn(
                f'PointCloud2 has {len(vectors)} points, but base CSV has {len(self._base_points)}. '
                'Markers will use the overlapping subset.',
                throttle_duration_sec=5.0,
            )

        marker_array = MarkerArray()
        frame_id = msg.header.frame_id or self._default_frame
        marker_count = min(len(vectors), len(self._base_points))

        for idx in range(marker_count):
            base = self._base_points[idx]
            vec = vectors[idx]
            marker = self._make_arrow_marker(idx, base, vec, msg.header.stamp, frame_id)
            marker_array.markers.append(marker)

        self._publisher.publish(marker_array)

    def _make_arrow_marker(
        self,
        idx: int,
        base: Sequence[float],
        vec: Sequence[float],
        stamp,
        frame_id: str,
    ) -> Marker:
        force_vec = self._resolve_force_vector(idx, vec)
        start = Point(x=float(base[0]), y=float(base[1]), z=float(base[2]))
        end = Point(
            x=start.x + force_vec[0] * self._vector_scale,
            y=start.y + force_vec[1] * self._vector_scale,
            z=start.z + force_vec[2] * self._vector_scale,
        )

        magnitude = math.sqrt(force_vec[0] ** 2 + force_vec[1] ** 2 + force_vec[2] ** 2)
        color = self._color_from_magnitude(magnitude)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'tactile_force'
        marker.id = idx
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [start, end]
        marker.scale.x = self._shaft_diameter
        marker.scale.y = self._head_diameter
        marker.scale.z = self._head_length
        marker.color = color
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0  # persist until replaced
        return marker

    def _resolve_force_vector(self, idx: int, vec: Sequence[float]) -> Tuple[float, float, float]:
        vx = float(vec[0])
        vy = float(vec[1])
        vz = float(vec[2])
        if not self._base_normals:
            return (vx, vy, vz)
        magnitude = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
        normal = self._base_normals[min(idx, len(self._base_normals) - 1)]
        return (
            float(normal[0] * magnitude),
            float(normal[1] * magnitude),
            float(normal[2] * magnitude),
        )

    def _color_from_magnitude(self, magnitude: float):
        norm = max(0.0, min(1.0, magnitude / self._max_magnitude))
        return ColorRGBA(r=norm, g=1.0 - norm, b=0.2, a=0.95)


def main(args=None):
    rclpy.init(args=args)
    node = TactileForceMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
