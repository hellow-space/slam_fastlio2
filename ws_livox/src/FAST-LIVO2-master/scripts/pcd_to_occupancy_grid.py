#!/usr/bin/env python3
import numpy as np
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import os


class PcdToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('pcd_to_occupancy_grid')

        # 参数
        self.declare_parameter('pcd_path', '')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('occ_threshold', 0.05)
        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('z_min', -0.36)
        self.declare_parameter('z_max', 0.43)

        pcd_path   = self.get_parameter('pcd_path').value
        resolution = self.get_parameter('resolution').value
        unknown_val= self.get_parameter('unknown_value').value
        z_min      = self.get_parameter('z_min').value
        z_max      = self.get_parameter('z_max').value

        if not pcd_path or not os.path.isfile(pcd_path):
            self.get_logger().error(f'PCD file not found: {pcd_path}')
            exit(1)

        # ---- 纯 numpy 读 PCD ----
        pts = self.read_pcd_xyz(pcd_path)
        if pts.size == 0:
            self.get_logger().error('Empty PCD!')
            exit(1)

        # Z 过滤
        mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
        pts  = pts[mask]
        if pts.size == 0:
            self.get_logger().error('No points within Z range!')
            exit(1)

        # 投影到 2D
        xy = pts[:, :2]
        x_min, y_min = xy.min(axis=0)
        x_max, y_max = xy.max(axis=0)
        width  = int(np.ceil((x_max - x_min) / resolution))
        height = int(np.ceil((y_max - y_min) / resolution))

        grid = np.zeros((height, width), dtype=np.int8)
        ix = ((xy[:, 0] - x_min) / resolution).astype(int)
        iy = ((xy[:, 1] - y_min) / resolution).astype(int)
        ix = np.clip(ix, 0, width - 1)
        iy = np.clip(iy, 0, height - 1)
        grid[iy, ix] = 100

        if unknown_val == -1:
            grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = -1

        # 组装 OccupancyGrid
        occ_grid = OccupancyGrid()
        occ_grid.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        occ_grid.info = MapMetaData(
            width=width,
            height=height,
            resolution=resolution,
            origin=Pose(position=Point(x=float(x_min), y=float(y_min), z=0.),
            orientation=Quaternion(x=0., y=0., z=0., w=1.))
        )
        occ_grid.data = grid.ravel().tolist()

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        pub = self.create_publisher(OccupancyGrid, '/map', qos)
        pub.publish(occ_grid)
        self.get_logger().info(
            f'Published /map  {width}x{height}  resolution={resolution} m'
        )

    # 只读 ASCII /binary 通用 XYZ 字段
    def read_pcd_xyz(self, path):
        with open(path, 'rb') as f:
            header = []
            while True:
                line = f.readline().decode().strip()
                header.append(line)
                if line.startswith('DATA'):
                    data_type = line.split()[1]  # ASCII or binary
                    break
            fields = None
            for h in header:
                if h.startswith('FIELDS'):
                    fields = h.split()[1:]
                if h.startswith('SIZE'):
                    sizes = list(map(int, h.split()[1:]))
                if h.startswith('TYPE'):
                    types = h.split()[1:]
                if h.startswith('COUNT'):
                    counts = list(map(int, h.split()[1:]))
            x_idx = fields.index('x')
            y_idx = fields.index('y')
            z_idx = fields.index('z')

            pts = []
            if data_type == 'ascii':
                for line in f:
                    xyz = line.decode().split()[x_idx:z_idx+1]
                    pts.append([float(xyz[0]), float(xyz[1]), float(xyz[2])])
            elif data_type == 'binary':
                row_fmt = ''.join(['f' if t == 'F' else 'd' if t == 'D' else 'f'
                                   for t, c in zip(types, counts)])
                row_size = struct.calcsize(row_fmt)
                while True:
                    row = f.read(row_size)
                    if len(row) != row_size:
                        break
                    vals = struct.unpack(row_fmt, row)
                    pts.append([vals[x_idx], vals[y_idx], vals[z_idx]])
            return np.array(pts, dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = PcdToOccupancyGrid()
    try:
        rclpy.spin(node)          # 持续驻留，而不是 spin_once
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()