#!/usr/bin/env python3
"""
Scan Context: Egocentric Spatial Descriptor for Place Recognition
Simplified Python implementation based on:
  Kim et al., "Scan Context: Egocentric Spatial Descriptor for Place Recognition
  within 3D Point Cloud Map", IROS 2018

Core API:
  - make_scancontext(pts_xyz): generate descriptor from point cloud
  - make_ringkey(desc): generate ring key from descriptor
  - distance(desc1, desc2): compute scan context distance with yaw alignment
"""

import numpy as np
from sklearn.neighbors import KDTree


class ScanContextManager:
    """Manages a database of scan contexts for global place recognition."""

    def __init__(self, num_rings=20, num_sectors=60, max_len=80.0, z_min=-5.0, z_max=15.0):
        self.num_rings = num_rings
        self.num_sectors = num_sectors
        self.max_len = max_len
        self.z_min = z_min
        self.z_max = z_max
        self.ring_gap = max_len / num_rings
        self.sector_gap = 2 * np.pi / num_sectors

        # Database
        self.descriptors = []      # list of (num_rings, num_sectors) arrays
        self.ringkeys = []         # list of (num_rings,) arrays
        self.poses = []            # list of (x, y, yaw) tuples
        self.kdtree = None         # KDTree on ringkeys

    # ------------------------------------------------------------------
    # Descriptor generation
    # ------------------------------------------------------------------
    def make_scancontext(self, pts_xyz):
        """
        pts_xyz: Nx3 numpy array (x, y, z)
        Returns: (num_rings, num_sectors) descriptor, filled with max height per cell
        """
        desc = np.zeros((self.num_rings, self.num_sectors), dtype=np.float32)

        x = pts_xyz[:, 0]
        y = pts_xyz[:, 1]
        z = pts_xyz[:, 2]

        # Polar coordinates
        r = np.sqrt(x ** 2 + y ** 2)
        theta = np.arctan2(y, x)  # [-pi, pi]

        # Filter by max range and z bounds
        valid = (r < self.max_len) & (z > self.z_min) & (z < self.z_max)
        r = r[valid]
        theta = theta[valid]
        z = z[valid]

        if len(r) == 0:
            return desc

        # Bin indices
        ring_idx = np.floor(r / self.ring_gap).astype(np.int32)
        ring_idx = np.clip(ring_idx, 0, self.num_rings - 1)

        # theta in [0, 2pi)
        theta_shifted = theta + np.pi
        sector_idx = np.floor(theta_shifted / self.sector_gap).astype(np.int32)
        sector_idx = np.clip(sector_idx, 0, self.num_sectors - 1)

        # Fill descriptor with max height per cell
        for i in range(len(r)):
            ri, si = ring_idx[i], sector_idx[i]
            if z[i] > desc[ri, si]:
                desc[ri, si] = z[i]

        return desc

    @staticmethod
    def make_ringkey(desc):
        """Compress descriptor into ring key (mean of each ring)."""
        return np.mean(desc, axis=1)

    # ------------------------------------------------------------------
    # Distance / matching
    # ------------------------------------------------------------------
    @staticmethod
    def distance(desc1, desc2):
        """
        Compute scan context distance with yaw alignment via FFT-based
        cross-correlation (inspired by SC-LeGO-LOAM circulant shift search).
        Uses FFT to evaluate all shifts simultaneously: O(N log N) vs O(N^2).
        Returns: (min_distance, best_shift_sectors)
        """
        if desc1.shape != desc2.shape:
            raise ValueError("Descriptor shapes must match")

        # FFT-based cross-correlation: for each ring, find shift that maximizes
        # correlation with desc1. Since ||a-b||^2 = ||a||^2 + ||b||^2 - 2*a·b,
        # maximizing correlation is equivalent to minimizing L2 distance.
        f1 = np.fft.fft(desc1, axis=1)
        f2 = np.fft.fft(desc2, axis=1)
        corr = np.fft.ifft(f1 * f2.conj(), axis=1).real  # (rings, sectors)
        total_corr = corr.sum(axis=0)  # sum across rings
        best_shift = int(np.argmax(total_corr))

        # Compute exact L2 distance at best shift
        desc2_shifted = np.roll(desc2, best_shift, axis=1)
        min_dist = np.linalg.norm(desc1 - desc2_shifted)

        return min_dist, best_shift

    @staticmethod
    def distance_fast(desc1, desc2, num_shifts_to_try=None):
        """
        Fallback: brute-force search over a subset of shifts.
        Kept for compatibility; distance() is now preferred (faster via FFT).
        """
        num_sectors = desc1.shape[1]
        shifts = range(num_sectors) if num_shifts_to_try is None else range(num_shifts_to_try)

        min_dist = float('inf')
        best_shift = 0
        for shift in shifts:
            desc2_shifted = np.roll(desc2, shift, axis=1)
            dist = np.sum(np.abs(desc1 - desc2_shifted))
            if dist < min_dist:
                min_dist = dist
                best_shift = shift
        return min_dist, best_shift

    # ------------------------------------------------------------------
    # Database management
    # ------------------------------------------------------------------
    def add_place(self, pts_xyz, pose_xyyaw):
        """
        Add a place to the database.
        pts_xyz: Nx3 point cloud for this place
        pose_xyyaw: (x, y, yaw) tuple
        """
        desc = self.make_scancontext(pts_xyz)
        rk = self.make_ringkey(desc)
        self.descriptors.append(desc)
        self.ringkeys.append(rk)
        self.poses.append(pose_xyyaw)

    def build_kdtree(self):
        """Build KD-tree on ringkeys for fast candidate retrieval."""
        if len(self.ringkeys) == 0:
            raise ValueError("Database is empty")
        X = np.stack(self.ringkeys, axis=0)
        self.kdtree = KDTree(X, leaf_size=10)

    def query(self, pts_xyz, k=5):
        """
        Query the database with a point cloud.
        Returns: list of (distance, shift_sectors, pose) for top-k matches,
                 sorted by distance (ascending).
        """
        if self.kdtree is None:
            raise ValueError("KD-tree not built. Call build_kdtree() first.")

        q_desc = self.make_scancontext(pts_xyz)
        q_rk = self.make_ringkey(q_desc).reshape(1, -1)

        # Stage 1: KD-tree candidate proposal
        dists, indices = self.kdtree.query(q_rk, k=min(k, len(self.ringkeys)))
        dists = dists.flatten()
        indices = indices.flatten()

        # Stage 2: Pairwise scan context comparison (FFT-optimized circulant shift)
        results = []
        for idx in indices:
            c_desc = self.descriptors[idx]
            dist, shift = self.distance(q_desc, c_desc)
            results.append((dist, shift, self.poses[idx]))

        results.sort(key=lambda x: x[0])
        return results

    def save_database(self, path):
        """Save database to npz file."""
        np.savez(path,
                 descriptors=np.stack(self.descriptors, axis=0),
                 ringkeys=np.stack(self.ringkeys, axis=0),
                 poses=np.array(self.poses))

    def load_database(self, path):
        """Load database from npz file."""
        data = np.load(path)
        self.descriptors = list(data['descriptors'])
        self.ringkeys = list(data['ringkeys'])
        self.poses = list(data['poses'])
        self.build_kdtree()


# ------------------------------------------------------------------
# Helpers for map pre-processing
# ------------------------------------------------------------------

def extract_local_cloud(global_pts, center_xy, radius):
    """Extract points within radius of center."""
    dx = global_pts[:, 0] - center_xy[0]
    dy = global_pts[:, 1] - center_xy[1]
    d2 = dx ** 2 + dy ** 2
    mask = d2 < radius ** 2
    return global_pts[mask]


def build_database_from_pcd(pcd_path, grid_res=5.0, local_radius=30.0,
                            sc_manager=None, min_points=500):
    """
    Build Scan Context database from a merged PCD map.
    Splits the map into a regular XY grid and generates a descriptor for each cell.
    Vectorized for speed.
    """
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)
    # NOTE: we build SC database in ORIGINAL coordinate system
    # because the original z-axis has the richest structural information
    # (buildings, terrain height variation).
    # Coordinate swap (x,y,z)->(z,x,y) is applied only when publishing the pose.
    print(f"[SC-DB] Loaded map: {len(pts)} points (original coords)")

    if sc_manager is None:
        sc_manager = ScanContextManager()

    # Determine grid bounds
    x_min, x_max = pts[:, 0].min(), pts[:, 0].max()
    y_min, y_max = pts[:, 1].min(), pts[:, 1].max()
    print(f"[SC-DB] Map bounds: X[{x_min:.1f}, {x_max:.1f}] Y[{y_min:.1f}, {y_max:.1f}]")

    x_centers = np.arange(x_min + grid_res / 2, x_max, grid_res)
    y_centers = np.arange(y_min + grid_res / 2, y_max, grid_res)
    print(f"[SC-DB] Grid: {len(x_centers)} x {len(y_centers)} = {len(x_centers)*len(y_centers)} cells")

    added = 0
    r2 = local_radius ** 2
    for xc in x_centers:
        dx = pts[:, 0] - xc
        dx2 = dx * dx
        for yc in y_centers:
            dy = pts[:, 1] - yc
            mask = dx2 + dy * dy < r2
            local_pts = pts[mask]
            if len(local_pts) < min_points:
                continue
            sc_manager.add_place(local_pts, (xc, yc, 0.0))
            added += 1

    print(f"[SC-DB] Added {added} places to database")
    sc_manager.build_kdtree()
    return sc_manager


if __name__ == "__main__":
    # Simple test
    print("Scan Context module loaded. Run as ROS node via scan_context_initializer.py")
