#!/usr/bin/env python3
"""Lay a GRID of AprilTags over the free floor of an occupancy map -> tag graph (nodes + edges) + Gazebo world with the tags.

  make_tag_grid.py --map maps/warehouse.yaml --world-in ../my_bot_description/worlds/tugbot_warehouse.sdf --name warehouse_grid

Nodes : grid points (spacing --spacing) whose clearance to any obstacle/unknown cell >= --node-clear (robot half-diagonal + margin)
Edges : 4-neighbours (+ diagonals with --diagonals) when the whole segment keeps >= --edge-clear (half robot width + margin)
        -> a lane exists only where the robot really fits; tags on two sides of a shelf row are NOT joined.
Each tag id is a key into the table: the yaml stores id -> (x, y, yaw) in the world frame (+ its grid cell), so detecting a tag
gives the robot its absolute position. 36h11 has ids 0..586, so at most 587 tags.
Out   : config/tag_map_<name>.yaml, worlds/<name>.sdf (tags inline), models/agv_tags/materials/textures/*.png, maps/<name>_preview.png
"""
import argparse, math, os, re, sys, heapq
import numpy as np
import yaml

# collision footprints of the Fuel props used in the warehouse world: (x offset in model frame, size x, size y)
PROPS = {'shelf': (-0.5, 3.6, 0.6), 'shelf_big': (-0.5, 2.1, 18.0), 'pallet_box_mobile': (0.0, 1.22, 0.8),
         'cart_model_2': (0.0, 0.9, 0.6), 'Tugbot-charging-station': (0.0, 0.55, 0.1), 'Tugbot': (0.0, 0.6, 0.5)}


def load_map(path):
    m = yaml.safe_load(open(path))
    img = open(os.path.join(os.path.dirname(path), m['image']), 'rb').read()
    toks, pos = [], 0
    while len(toks) < 4:                                           # P5 / w h / maxval
        while img[pos:pos + 1].isspace(): pos += 1
        if img[pos:pos + 1] == b'#':
            while img[pos:pos + 1] != b'\n': pos += 1
            continue
        s = pos
        while not img[pos:pos + 1].isspace(): pos += 1
        toks.append(img[s:pos])
    pos += 1
    w, h = int(toks[1]), int(toks[2])
    a = np.frombuffer(img, np.uint8, offset=pos).reshape(h, w)
    return a, m['resolution'], m['origin'][0], m['origin'][1]


def world_obstacles(sdf_path, res, ox, oy, shape, pad=0.1):
    """rasterise the prop footprints from the world file (extra safety next to the map: the map can be ~0.3 m off)"""
    H, W = shape
    blocked = np.zeros(shape, bool)
    s = open(sdf_path).read()
    for kind, pose in re.findall(r'/models/([A-Za-z_0-9-]+)\s*</uri>\s*<name>[^<]+</name>\s*<pose>([^<]+)</pose>', s):
        if kind not in PROPS: continue
        x, y = map(float, pose.split()[:2]); off, sx, sy = PROPS[kind]
        x0, x1, y0, y1 = x + off - sx / 2 - pad, x + off + sx / 2 + pad, y - sy / 2 - pad, y + sy / 2 + pad
        c0, c1 = int((x0 - ox) / res), int(math.ceil((x1 - ox) / res))
        r1, r0 = H - 1 - int((y0 - oy) / res), H - 1 - int(math.ceil((y1 - oy) / res))
        blocked[max(r0, 0):max(r1 + 1, 0), max(c0, 0):max(c1 + 1, 0)] = True
    return blocked


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True); ap.add_argument('--world-in', required=True); ap.add_argument('--name', required=True)
    ap.add_argument('--spacing', type=float, default=2.0)
    ap.add_argument('--node-clear', type=float, default=0.55); ap.add_argument('--edge-clear', type=float, default=0.42)
    ap.add_argument('--bounds', type=float, nargs=4, default=[-14.3, 14.3, -24.3, 24.3], metavar=('X0', 'X1', 'Y0', 'Y1'))
    ap.add_argument('--diagonals', action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument('--tag-z', type=float, default=0.011, help='floor top height [m]')
    ap.add_argument('--pkg', default=os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')))
    ap.add_argument('--sim-pkg', default=os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'my_bot_description')))
    a = ap.parse_args()
    from scipy import ndimage as ndi

    img, res, ox, oy = load_map(a.map)
    H, W = img.shape
    blocked = (img != 254) | world_obstacles(a.world_in, res, ox, oy, img.shape)
    dt = ndi.distance_transform_edt(~blocked) * res                      # clearance to obstacle/unknown [m]

    def clear(x, y):
        c, r = int((x - ox) / res), H - 1 - int((y - oy) / res)
        return dt[r, c] if 0 <= c < W and 0 <= r < H else 0.0

    # ---- nodes on a regular grid (centred in the bounds)
    x0, x1, y0, y1 = a.bounds
    nx, ny = int((x1 - x0) / a.spacing) + 1, int((y1 - y0) / a.spacing) + 1
    gx = x0 + ((x1 - x0) - (nx - 1) * a.spacing) / 2 + np.arange(nx) * a.spacing
    gy = y0 + ((y1 - y0) - (ny - 1) * a.spacing) / 2 + np.arange(ny) * a.spacing
    cand = {(i, j): (float(gx[i]), float(gy[j])) for i in range(nx) for j in range(ny) if clear(gx[i], gy[j]) >= a.node_clear}

    def seg_ok(p, q):
        n = max(2, int(math.dist(p, q) / 0.1))
        return all(clear(p[0] + (q[0] - p[0]) * t / n, p[1] + (q[1] - p[1]) * t / n) >= a.edge_clear for t in range(n + 1))

    nbrs = [(1, 0), (0, 1)] + ([(1, 1), (1, -1)] if a.diagonals else [])
    edges = [((i, j), (i + di, j + dj)) for (i, j) in cand for di, dj in nbrs
             if (i + di, j + dj) in cand and seg_ok(cand[(i, j)], cand[(i + di, j + dj)])]
    # ---- keep the largest connected part
    adj = {k: [] for k in cand}
    for u, v in edges: adj[u].append(v); adj[v].append(u)
    seen, best = set(), []
    for k in cand:
        if k in seen: continue
        comp, st = [], [k]; seen.add(k)
        while st:
            u = st.pop(); comp.append(u)
            for v in adj[u]:
                if v not in seen: seen.add(v); st.append(v)
        if len(comp) > len(best): best = comp
    keep = set(best)
    dropped = len(cand) - len(keep)
    order = sorted(keep, key=lambda k: (k[1], k[0]))                     # ids row by row, south -> north, west -> east
    if len(order) > 587: sys.exit(f'{len(order)} tags > 587 ids of tag36h11: raise --spacing')
    tid = {k: n for n, k in enumerate(order)}
    edges = [(tid[u], tid[v]) for u, v in edges if u in keep and v in keep]

    # ---- tag map yaml (id -> world coordinates)
    out = {'family': 'tag36h11', 'tag_size': 0.2, 'frame': 'map',
           'grid': {'spacing': a.spacing, 'diagonals': a.diagonals},
           'nodes': {tid[k]: {'x': round(cand[k][0], 3), 'y': round(cand[k][1], 3), 'yaw': 0.0, 'grid': [k[0], k[1]]} for k in order},
           'edges': [{'from': u, 'to': v, 'is_bidirectional': True} for u, v in edges]}
    ypath = f'{a.pkg}/config/tag_map_{a.name}.yaml'
    with open(ypath, 'w') as f:
        f.write(f'# Tag grid for the warehouse: node = AprilTag id -> world (x, y); edge = lane the robot fits through.\n'
                f'# Generated by scripts/make_tag_grid.py (spacing {a.spacing} m, node clearance {a.node_clear} m, edge clearance {a.edge_clear} m)\n')
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=None)

    # ---- textures (same orientation convention as the first 20 tag models: OpenCV marker rotated 180 deg)
    import cv2
    tex = f'{a.sim_pkg}/models/agv_tags/materials/textures'; os.makedirs(tex, exist_ok=True)
    d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    for n in range(len(order)):
        m = cv2.aruco.drawMarker(d, n, 320, borderBits=1)
        m = cv2.copyMakeBorder(m, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(f'{tex}/tag36_11_{n:05d}.png', np.rot90(m, 2))
    mdir = f'{a.sim_pkg}/models/agv_tags'
    open(f'{mdir}/model.config', 'w').write('<?xml version="1.0"?>\n<model><name>agv_tags</name><version>1.0</version>'
        '<sdf version="1.8">model.sdf</sdf><description>AprilTag 36h11 floor textures for the tag grid</description></model>\n')
    open(f'{mdir}/model.sdf', 'w').write('<?xml version="1.0"?>\n<sdf version="1.8"><model name="agv_tags"><static>true</static>'
        '<link name="link"/></model></sdf>\n')

    # ---- world: original + the tags (inline models)
    src = open(a.world_in).read()
    # drop the Tugbot robot (unused: its lidar/cameras/physics cost CPU); the charging station prop stays
    src = re.sub(r'\s*<include>\s*<uri>\s*[^<]*models/Tugbot\s*</uri>\s*<name>tugbot</name>.*?</include>', '', src, flags=re.S)
    assert 'models/Tugbot\n' not in src and '<name>tugbot</name>' not in src
    assert '<topic>/model/tugbot/cmd_vel</topic>' in src
    src = src.replace('<topic>/model/tugbot/cmd_vel</topic>', '<topic>/cmd_vel_teleop</topic>')     # Gazebo keyboard panel -> agv_01

    src = src.replace('<start_paused>true</start_paused>', '<start_paused>false</start_paused>')    # GUI must not pause the sim
    block = f'\n    <!-- ===== {len(order)} AprilTag floor markers (make_tag_grid.py) ===== -->\n'
    for k in order:
        n = tid[k]; x, y = cand[k]
        block += (f'    <model name="tag_{n}"><static>true</static><pose>{x:.3f} {y:.3f} {a.tag_z} 0 0 0</pose><link name="link">'
                  f'<visual name="v"><pose>0 0 0.0015 0 0 3.14159265</pose><cast_shadows>false</cast_shadows>'
                  f'<geometry><plane><normal>0 0 1</normal><size>0.25 0.25</size></plane></geometry><material>'
                  f'<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular><pbr><metal>'
                  f'<albedo_map>model://agv_tags/materials/textures/tag36_11_{n:05d}.png</albedo_map>'
                  f'<roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material></visual></link></model>\n')
    wpath = f'{a.sim_pkg}/worlds/{a.name}.sdf'
    open(wpath, 'w').write(src.replace('  </world>', block + '  </world>'))

    # ---- preview image
    from PIL import Image, ImageDraw
    sc = 2                                                                # preview px per map cell... drawn at 0.5 scale
    pv = Image.fromarray(np.where(blocked, np.where(img == 254, 120, 70), 235).astype(np.uint8)).convert('RGB')
    pv = pv.resize((W // sc, H // sc)); dr = ImageDraw.Draw(pv)
    P = lambda x, y: ((x - ox) / res / sc, (H - 1 - (y - oy) / res) / sc)
    for u, v in edges:
        pu, pw = [(out['nodes'][z]['x'], out['nodes'][z]['y']) for z in (u, v)]
        dr.line([P(*pu), P(*pw)], fill=(60, 140, 255), width=1)
    for n, nd in out['nodes'].items():
        px, py = P(nd['x'], nd['y']); dr.ellipse([px - 3, py - 3, px + 3, py + 3], fill=(255, 80, 0))
    pv.save(f'{a.pkg}/maps/{a.name}_preview.png')
    deg = np.mean([len(adj[k]) for k in keep])
    print(f'{len(order)} tags (dropped {dropped} unreachable), {len(edges)} lanes, mean {deg:.1f} lanes/tag, grid {nx}x{ny}')
    print(f'wrote {ypath}\n      {wpath}\n      {len(order)} textures in {tex}\n      {a.pkg}/maps/{a.name}_preview.png')


if __name__ == '__main__':
    main()
