"""PCD geometry, nearest-depth angular raster and surfel ray intersections."""
import numpy as np
from scipy.spatial import cKDTree

def read_pcd(path):
    with open(path, "rb") as f:
        h = {}
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError("Missing PCD DATA header")
            line = raw.decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(); h[parts[0]] = parts[1:]
            if parts[0] == "DATA":
                break
        names = h["FIELDS"]; counts = list(map(int, h.get("COUNT", ["1"]*len(names))))
        types = {"F": "f", "I": "i", "U": "u"}
        fields = [(n, "<"+types[t]+s, (c,)) if c != 1 else (n, "<"+types[t]+s)
                  for n, s, t, c in zip(names, h["SIZE"], h["TYPE"], counts)]
        if h["DATA"][0] == "binary":
            a = np.fromfile(f, dtype=np.dtype(fields), count=int(h["POINTS"][0]))
            xyz = np.column_stack([a[k] for k in ("x", "y", "z")])
        elif h["DATA"][0] == "ascii":
            a = np.loadtxt(f, ndmin=2)
            starts = np.r_[0, np.cumsum(counts)]
            xyz = np.column_stack([a[:, starts[names.index(k)]] for k in ("x", "y", "z")])
        else:
            raise ValueError("Use binary/ascii PCD; binary_compressed is not supported")
    return xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64)

def normals_for_map(xyz, tree):
    normals = np.empty_like(xyz)
    for start in range(0, len(xyz), 20000):
        p = xyz[start:start+20000]
        _, neighbors = tree.query(p, k=min(16, len(xyz)))
        v = xyz[neighbors]
        v = v-v.mean(axis=1, keepdims=True)
        covariance = np.einsum("nki,nkj->nij", v, v)
        _, vec = np.linalg.eigh(covariance)
        normals[start:start+len(p)] = vec[:, :, 0]
    return normals

class PointRenderer:
    """Nearest-depth angular raster plus bounded local surfel intersections."""
    def __init__(self, xyz, resolution, radius, max_range):
        self.xyz = xyz; self.tree = cKDTree(xyz)
        self.normals = normals_for_map(xyz, self.tree)
        self.az_count = int(np.ceil(2*np.pi/np.deg2rad(resolution)))
        self.el_count = int(np.ceil(np.pi/np.deg2rad(resolution)))
        self.radius = radius; self.max_range = max_range
    def bins(self, direction):
        az = np.arctan2(direction[:, 1], direction[:, 0])
        el = np.arcsin(np.clip(direction[:, 2], -1, 1))
        ai = np.floor((az+np.pi)/(2*np.pi)*self.az_count).astype(int) % self.az_count
        ei = np.clip(np.floor((el+np.pi/2)/np.pi*self.el_count).astype(int), 0, self.el_count-1)
        return ai, ei
    def prepare(self, origin, rotation):
        ids = np.asarray(self.tree.query_ball_point(origin, self.max_range+self.radius), dtype=int)
        if not len(ids):
            return None
        p = (self.xyz[ids]-origin)@rotation
        n = self.normals[ids]@rotation
        ranges = np.linalg.norm(p, axis=1)
        nonzero = ranges > 1e-5
        p, n, ranges = p[nonzero], n[nonzero], ranges[nonzero]
        ai, ei = self.bins(p/ranges[:, None])
        keys = ei*self.az_count+ai
        order = np.lexsort((ranges, keys))
        _, first = np.unique(keys[order], return_index=True)
        chosen = order[first]
        grid = np.full(self.az_count*self.el_count, -1, dtype=np.int32)
        grid[keys[chosen]] = chosen
        return p, n, grid, np.asarray(origin), np.asarray(rotation)

    def cast(self, rays, scene, origin, rotation, blind):
        if scene is None:
            return np.zeros(len(rays))
        p, n, grid, reference_origin, reference_rotation = scene
        ray_origin = (origin-reference_origin)@reference_rotation
        rays = rays@rotation.T@reference_rotation
        ra, re = self.bins(rays)
        best = np.full(len(rays), np.inf)
        # Adjacent angular cells provide local surface support, never global hole filling.
        for de in (-1, 0, 1):
            for da in (-1, 0, 1):
                cell = np.clip(re+de, 0, self.el_count-1)*self.az_count+(ra+da)%self.az_count
                cand = grid[cell]
                good = cand >= 0
                idx = np.flatnonzero(good); ci = cand[good]
                if not len(idx):
                    continue
                denominator = np.einsum("ij,ij->i", rays[idx], n[ci])
                dist = np.full(len(idx), np.inf)
                valid = np.abs(denominator) > 1e-5
                dist[valid] = np.einsum("ij,ij->i", p[ci][valid]-ray_origin, n[ci][valid])/denominator[valid]
                valid &= (dist >= blind) & (dist <= self.max_range)
                surface_distance = np.full(len(idx), np.inf)
                surface_distance[valid] = np.linalg.norm(
                    ray_origin+rays[idx[valid]]*dist[valid, None]-p[ci[valid]], axis=1)
                valid &= surface_distance <= self.radius
                best[idx[valid]] = np.minimum(best[idx[valid]], dist[valid])
        best[~np.isfinite(best)] = 0
        return best

    def scan(self, rays, origin, rotation, blind):
        return self.cast(rays, self.prepare(origin, rotation), origin, rotation, blind)

