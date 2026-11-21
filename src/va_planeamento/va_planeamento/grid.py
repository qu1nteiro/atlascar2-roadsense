"""Local occupancy grid in the car frame (base_link).

ROS-free library (numpy only), shared by all groups:
- each group's perception node builds a LocalGrid and marks on it
  what it detects (points, polygons, free rays);
- the local planner (A*) reads the same grid back from the
  nav_msgs/OccupancyGrid message.

Value convention (same as nav_msgs/OccupancyGrid):
    -1  unknown      0  free      100  occupied
Indices: data[iy, ix], with ix along x (forward of the car)
and iy along y (to the left). Cell (0, 0) has its lower-left
corner at (x_min, y_min).
"""
import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


class LocalGrid:

    def __init__(self, resolution=0.2, x_min=-5.0, x_max=35.0,
                 y_min=-10.0, y_max=10.0, initial_value=UNKNOWN):
        self.res = float(resolution)
        self.x_min = float(x_min)
        self.y_min = float(y_min)
        self.nx = int(round((x_max - x_min) / self.res))
        self.ny = int(round((y_max - y_min) / self.res))
        self.data = np.full((self.ny, self.nx), initial_value, dtype=np.int8)

    # ------------------------------------------------------------ conversions
    @property
    def x_max(self):
        return self.x_min + self.nx * self.res

    @property
    def y_max(self):
        return self.y_min + self.ny * self.res

    def to_cell(self, x, y):
        """Coordinates (m) -> indices (ix, iy) and mask of those inside the grid."""
        ix = np.floor((np.asarray(x, dtype=float) - self.x_min) / self.res).astype(int)
        iy = np.floor((np.asarray(y, dtype=float) - self.y_min) / self.res).astype(int)
        inside = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return ix, iy, inside

    def to_meters(self, ix, iy):
        """Indices -> coordinates (m) of the cell center."""
        x = self.x_min + (np.asarray(ix) + 0.5) * self.res
        y = self.y_min + (np.asarray(iy) + 0.5) * self.res
        return x, y

    # --------------------------------------------------------------- marking
    def mark_points(self, x, y, value=OCCUPIED):
        """Mark the cells containing the points (x, y), e.g. LiDAR hits."""
        ix, iy, ok = self.to_cell(x, y)
        self.data[iy[ok], ix[ok]] = value

    def mark_free_rays(self, x0, y0, x, y):
        """Mark as free the cells crossed by the rays (x0,y0)->(x,y).

        Useful for ranging sensors: between the sensor and the hit there is nothing.
        The hit points themselves are not marked (use mark_points afterwards).
        """
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        length = np.hypot(x - x0, y - y0)
        n = int(np.ceil(length.max() / (0.5 * self.res))) if length.size else 0
        if n == 0:
            return
        s = np.linspace(0.0, 1.0, n, endpoint=False)[:, None]  # (n, 1)
        xs = x0 + s * (x - x0)                                  # (n, rays)
        ys = y0 + s * (y - y0)
        ix, iy, ok = self.to_cell(xs.ravel(), ys.ravel())
        free = self.data[iy[ok], ix[ok]] != OCCUPIED
        self.data[iy[ok][free], ix[ok][free]] = FREE

    def mark_polygon(self, vertices, value=OCCUPIED):
        """Mark the interior of a polygon [(x, y), ...] in base_link.

        Useful for cameras (off-lane area, a vehicle's box projected on the
        ground) and for 3D LiDAR (cluster outline).
        """
        v = np.asarray(vertices, dtype=float)
        ix0, iy0, _ = self.to_cell(v[:, 0].min(), v[:, 1].min())
        ix1, iy1, _ = self.to_cell(v[:, 0].max(), v[:, 1].max())
        ix0, iy0 = max(int(ix0), 0), max(int(iy0), 0)
        ix1, iy1 = min(int(ix1), self.nx - 1), min(int(iy1), self.ny - 1)
        if ix1 < ix0 or iy1 < iy0:
            return
        gx, gy = np.meshgrid(np.arange(ix0, ix1 + 1), np.arange(iy0, iy1 + 1))
        px, py = self.to_meters(gx, gy)
        inside = np.zeros(px.shape, dtype=bool)
        j = len(v) - 1
        for i in range(len(v)):            # crossing-number test
            xi, yi = v[i]
            xj, yj = v[j]
            crosses = ((yi > py) != (yj > py)) & \
                (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi)
            inside ^= crosses
            j = i
        self.data[gy[inside], gx[inside]] = value

    # ------------------------------------------------------------- messages
    def to_msg(self, header):
        """Convert to nav_msgs/OccupancyGrid (the ROS import stays in here)."""
        from nav_msgs.msg import OccupancyGrid
        msg = OccupancyGrid()
        msg.header = header
        msg.info.resolution = self.res
        msg.info.width = self.nx
        msg.info.height = self.ny
        msg.info.origin.position.x = self.x_min
        msg.info.origin.position.y = self.y_min
        msg.info.origin.orientation.w = 1.0
        msg.data = self.data.ravel().tolist()
        return msg

    @classmethod
    def from_msg(cls, msg):
        """Build a LocalGrid from a nav_msgs/OccupancyGrid (no rotation)."""
        g = cls.__new__(cls)
        g.res = float(msg.info.resolution)
        g.x_min = float(msg.info.origin.position.x)
        g.y_min = float(msg.info.origin.position.y)
        g.nx = int(msg.info.width)
        g.ny = int(msg.info.height)
        g.data = np.asarray(msg.data, dtype=np.int8).reshape(g.ny, g.nx)
        return g
