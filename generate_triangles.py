#!/usr/bin/env python3
"""
Generate triangles.txt (100000 triangles) and intersections.txt (total intersecting pairs).
Produces triangles in clustered regions to create many intersections.
"""

import random
import math
import sys
from collections import defaultdict
from itertools import combinations

# === Parameters ===
NUM_TRIANGLES = 100
OUT_TRI_FILE = "triangles.txt"
OUT_INTER_FILE = "intersections.txt"
# SEED = random.randint(0, 1000000)
SEED = 43384
print(SEED)

# Space layout: create several clusters where triangles overlap
NUM_CLUSTERS = 50
CLUSTER_RADIUS = 10.0    # radius of cluster where triangle vertices lie
CLUSTER_SPREAD = 2.0  # distance between cluster centers
TRI_SIZE = 1.0          # typical triangle size (scale of edge length)

# Spatial hashing grid resolution: number of cells on each axis
# Increase to reduce comparisons; decrease to ensure nearby triangles fall into same cell.
GRID_RES = 1

random.seed(SEED)

# === Geometry utilities ===

def vec_sub(a, b):
    return (a[0]-b[0], a[1]-b[1], a[2]-b[2])

def vec_add(a, b):
    return (a[0]+b[0], a[1]+b[1], a[2]+b[2])

def vec_dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def vec_cross(a, b):
    return (a[1]*b[2]-a[2]*b[1],
            a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0])

def vec_scale(a, s):
    return (a[0]*s, a[1]*s, a[2]*s)

def bbox_of_triangle(tri):
    xs = [tri[0][0], tri[1][0], tri[2][0]]
    ys = [tri[0][1], tri[1][1], tri[2][1]]
    zs = [tri[0][2], tri[1][2], tri[2][2]]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))

# Möller triangle-triangle intersection test (translated)
# Reference: "A Fast Triangle-Triangle Overlap Test", Tomas Möller, 1997.
# This version checks if two triangles intersect (including edge touches).
# For brevity we implement a reliable function using separating axis tests.
# Source algorithm adapted; for production use, consider vetted geometry libraries.

EPS = 1e-9

def plane_from_triangle(tri):
    v0, v1, v2 = tri
    e1 = vec_sub(v1, v0)
    e2 = vec_sub(v2, v0)
    n = vec_cross(e1, e2)
    d = -vec_dot(n, v0)
    return n, d

def coplanar_tri_tri(n, tri1, tri2):
    # project onto largest component of normal and do 2D tri-tri overlap test
    absn = (abs(n[0]), abs(n[1]), abs(n[2]))
    if absn[0] > absn[1]:
        if absn[0] > absn[2]:
            i0, i1 = 1, 2
        else:
            i0, i1 = 0, 1
    else:
        if absn[2] > absn[1]:
            i0, i1 = 0, 1
        else:
            i0, i1 = 0, 2
    def proj(tri):
        return [(p[i0], p[i1]) for p in tri]
    t1 = proj(tri1)
    t2 = proj(tri2)
    # Use separating axis in 2D: check edge normals and triangle vertices
    def tri_overlap_2d(a, b):
        # edge axes from a
        for i in range(3):
            x1, y1 = a[i]
            x2, y2 = a[(i+1)%3]
            axis = (y1 - y2, x2 - x1)  # perpendicular
            minA = maxA = axis[0]*a[0][0] + axis[1]*a[0][1]
            for p in a[1:]:
                val = axis[0]*p[0] + axis[1]*p[1]
                if val < minA: minA = val
                if val > maxA: maxA = val
            minB = maxB = axis[0]*b[0][0] + axis[1]*b[0][1]
            for p in b[1:]:
                val = axis[0]*p[0] + axis[1]*p[1]
                if val < minB: minB = val
                if val > maxB: maxB = val
            if maxA < minB - EPS or maxB < minA - EPS:
                return False
        # repeat for b's edges
        for i in range(3):
            x1, y1 = b[i]
            x2, y2 = b[(i+1)%3]
            axis = (y1 - y2, x2 - x1)
            minA = maxA = axis[0]*a[0][0] + axis[1]*a[0][1]
            for p in a[1:]:
                val = axis[0]*p[0] + axis[1]*p[1]
                if val < minA: minA = val
                if val > maxA: maxA = val
            minB = maxB = axis[0]*b[0][0] + axis[1]*b[0][1]
            for p in b[1:]:
                val = axis[0]*p[0] + axis[1]*p[1]
                if val < minB: minB = val
                if val > maxB: maxB = val
            if maxA < minB - EPS or maxB < minA - EPS:
                return False
        return True
    return tri_overlap_2d(t1, t2)

def tri_tri_intersect(tri1, tri2):
    # Compute plane equations
    n1, d1 = plane_from_triangle(tri1)
    n2, d2 = plane_from_triangle(tri2)
    # Evaluate signed distances of tri2 vertices to plane1
    du = [vec_dot(n1, p) + d1 for p in tri2]
    dv = [vec_dot(n2, p) + d2 for p in tri1]
    # Coplanarity check
    if all(abs(x) < EPS for x in du) and all(abs(x) < EPS for x in dv):
        return coplanar_tri_tri(n1, tri1, tri2)
    # If all points on same side for one plane -> no intersection
    if (du[0] > 0 and du[1] > 0 and du[2] > 0) or (du[0] < 0 and du[1] < 0 and du[2] < 0):
        return False
    if (dv[0] > 0 and dv[1] > 0 and dv[2] > 0) or (dv[0] < 0 and dv[1] < 0 and dv[2] < 0):
        return False
    # For robustness and brevity, fall back to a simple approach:
    # Check segment-triangle intersections for all edges of tri1 vs tri2 and vice versa.
    # This is slower but acceptable for our use.
    def seg_tri_intersect(p0, p1, tri):
        u = vec_sub(tri[1], tri[0])
        v = vec_sub(tri[2], tri[0])
        n = vec_cross(u, v)
        dir = vec_sub(p1, p0)
        w0 = vec_sub(p0, tri[0])
        a = -vec_dot(n, w0)
        b = vec_dot(n, dir)
        if abs(b) < EPS:
            return False  # segment parallel to triangle plane
        r = a / b
        if r < -EPS or r > 1+EPS:
            return False
        I = vec_add(p0, vec_scale(dir, r))
        # check inside triangle via barycentric coords
        uu = vec_dot(u, u)
        uv = vec_dot(u, v)
        vv = vec_dot(v, v)
        w = vec_sub(I, tri[0])
        wu = vec_dot(w, u)
        wv = vec_dot(w, v)
        D = uv*uv - uu*vv
        if abs(D) < EPS:
            return False
        s = (uv*wv - vv*wu) / D
        if s < -EPS or s > 1+EPS:
            return False
        t = (uv*wu - uu*wv) / D
        if t < -EPS or (s + t) > 1+EPS:
            return False
        return True

    # edges of tri1 against tri2
    for i in range(3):
        if seg_tri_intersect(tri1[i], tri1[(i+1)%3], tri2):
            return True
    for i in range(3):
        if seg_tri_intersect(tri2[i], tri2[(i+1)%3], tri1):
            return True
    # no edge crosses; possible overlap when one tri is fully inside other coplanar - handled earlier
    return False

# === Triangle generator ===

def random_point_in_sphere(radius):
    while True:
        x = random.uniform(-radius, radius)
        y = random.uniform(-radius, radius)
        z = random.uniform(-radius, radius)
        if x*x + y*y + z*z <= radius*radius:
            return (x, y, z)

def make_triangle(center, scale):
    # pick random base point near center, then two offset vectors
    a = random_point_in_sphere(scale)
    b = random_point_in_sphere(scale)
    c = random_point_in_sphere(scale)
    return (vec_add(center, a), vec_add(center, b), vec_add(center, c))

# Create cluster centers
cluster_centers = []
for i in range(NUM_CLUSTERS):
    cx = (i % int(math.ceil(NUM_CLUSTERS**(1/3)))) * CLUSTER_SPREAD
    cy = ((i // int(math.ceil(NUM_CLUSTERS**(1/3)))) % int(math.ceil(NUM_CLUSTERS**(1/3)))) * CLUSTER_SPREAD
    cz = (i // (int(math.ceil(NUM_CLUSTERS**(2/3))))) * CLUSTER_SPREAD
    # add jitter
    cluster_centers.append((cx + random.uniform(-20,20),
                            cy + random.uniform(-20,20),
                            cz + random.uniform(-20,20)))

# generate triangles
triangles = []
for i in range(NUM_TRIANGLES):
    center = random.choice(cluster_centers)
    scale = TRI_SIZE * (0.5 + random.random()*1.5)
    # jiggle center slightly
    jitter = (random.uniform(-CLUSTER_RADIUS, CLUSTER_RADIUS),
              random.uniform(-CLUSTER_RADIUS, CLUSTER_RADIUS),
              random.uniform(-CLUSTER_RADIUS, CLUSTER_RADIUS))
    c = vec_add(center, jitter)
    tri = make_triangle(c, scale)
    triangles.append(tri)
    if (i+1) % 10000 == 0:
        print(f"Generated {i+1} triangles", file=sys.stderr)

# write triangles to file
with open(OUT_TRI_FILE, "w") as f:
    f.write(str(len(triangles) * 9) + '\n')
    for tri in triangles:
        coords = []
        for p in tri:
            coords.extend([f"{p[0]}\n", f"{p[1]}\n", f"{p[2]}\n"])
        f.write("".join(coords))

# === Spatial hashing to limit pair tests ===

# compute global bbox
minx = min(p[0] for tri in triangles for p in tri)
miny = min(p[1] for tri in triangles for p in tri)
minz = min(p[2] for tri in triangles for p in tri)
maxx = max(p[0] for tri in triangles for p in tri)
maxy = max(p[1] for tri in triangles for p in tri)
maxz = max(p[2] for tri in triangles for p in tri)

# Expand a tiny bit
pad = 1e-6
minx -= pad; miny -= pad; minz -= pad
maxx += pad; maxy += pad; maxz += pad

def cell_coords_for_bbox(bmin, bmax):
    def idx(x, minv, maxv):
        if maxv == minv:
            return 0
        t = (x - minv) / (maxv - minv)
        return int(max(0, min(GRID_RES-1, math.floor(t * GRID_RES))))
    ix0 = idx(bmin[0], minx, maxx)
    iy0 = idx(bmin[1], miny, maxy)
    iz0 = idx(bmin[2], minz, maxz)
    ix1 = idx(bmax[0], minx, maxx)
    iy1 = idx(bmax[1], miny, maxy)
    iz1 = idx(bmax[2], minz, maxz)
    cells = []
    for ix in range(ix0, ix1+1):
        for iy in range(iy0, iy1+1):
            for iz in range(iz0, iz1+1):
                cells.append((ix,iy,iz))
    return cells

grid = defaultdict(list)
for idx_tri, tri in enumerate(triangles):
    bmin, bmax = bbox_of_triangle(tri)
    cells = cell_coords_for_bbox(bmin, bmax)
    for c in cells:
        grid[c].append(idx_tri)
    if (idx_tri+1) % 20000 == 0:
        print(f"Hashed {idx_tri+1} triangles into grid", file=sys.stderr)

# collect candidate pairs (only within same cell)
pairs_checked = set()
intersections = 0
count_candidates = 0

cell_items = list(grid.items())
for cell, idxs in cell_items:
    if len(idxs) < 2:
        continue
    # test all combinations within cell
    for i, j in combinations(idxs, 2):
        if i == j: continue
        a, b = (i,j) if i < j else (j,i)
        if (a,b) in pairs_checked: continue
        pairs_checked.add((a,b))
        count_candidates += 1
        if tri_tri_intersect(triangles[a], triangles[b]):
            print(triangles[a], triangles[b])
            intersections += 1
    # progress log
    if len(pairs_checked) % 1000000 < 50:
        print(f"Pairs checked: {len(pairs_checked)}", file=sys.stderr)

print(f"Candidate pairs checked: {count_candidates}", file=sys.stderr)
print(f"Intersections found: {intersections}", file=sys.stderr)

with open(OUT_INTER_FILE, "w") as f:
    f.write(str(intersections) + "\n")

print("Done.")
