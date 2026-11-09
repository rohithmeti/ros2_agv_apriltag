#!/usr/bin/env python3
"""ORTHOGONAL tag grid: tags on the crossings of vertical and horizontal lines, lanes only along those lines (no diagonals).

  make_tag_ortho.py --map maps/warehouse.yaml --world-in ../my_bot_description/worlds/tugbot_warehouse.sdf --name warehouse_ortho

Lines   : (1) aisle centre-lines found in the map: where a corridor (free run between obstacles, 0.8-5 m wide) keeps the same
              centre for >= --min-aisle metres, so every aisle between racks gets its own line;
          (2) a regular lattice (--spacing) for open areas; lattice lines closer than --min-sep to an aisle line are dropped.
Nodes   : line crossings with clearance >= --node-clear to obstacles/unknown.
Lanes   : only between ADJACENT crossings, both ends nodes, whole segment clear (>= --edge-clear)  ->  every lane is axis-aligned,
          parallel or perpendicular to the others, and two lanes only meet at a tag (no crossings in mid-lane).
Out     : config/tag_map_<name>.yaml (id -> world x,y), worlds/<name>.sdf (tags + Tugbot removed), worlds/<name>.spawn.yaml
          (agv start pose = on tag 1), models/agv_tags textures, maps/<name>_preview.png
"""
import argparse, math, os, re, sys
import numpy as np, yaml
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_tag_grid import load_map, world_obstacles


def aisle_lines(cap, res, axis, lo, hi, min_len, max_w=5.0, min_w=0.8, step=2):
    """positions (m, along the axis perpendicular to the corridors) of corridor centre-lines.
    axis 'x': vertical corridors -> scan rows, returns x positions;  axis 'y': horizontal corridors -> scan columns."""
    H, W = cap.shape
    votes = {}
    n = H if axis == 'x' else W
    for i in range(0, n, step):
        line = cap[i, :] if axis == 'x' else cap[:, i]
        d = np.diff(np.concatenate([[0], line.astype(np.int8), [0]]))
        for s, e in zip(np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]):
            w = (e - s) * res
            if min_w <= w <= max_w:
                key = int(round((s + e - 1) / 2 / 2.0))                 # 2-cell bins of the run centre
                votes[key] = votes.get(key, 0) + 1
    if not votes: return []
    # smooth over +-1 bin, then non-max suppression
    ks = sorted(votes); score = {k: votes.get(k - 1, 0) + votes[k] + votes.get(k + 1, 0) for k in ks}
    out = []
    for k in sorted(score, key=score.get, reverse=True):
        length = score[k] * step * res / 1.0 / 1.0                      # metres of corridor with this centre (3 bins)
        if length < min_len: break
        pos = k * 2 + 0.5
        out.append(pos)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True); ap.add_argument('--world-in', required=True); ap.add_argument('--name', required=True)
    ap.add_argument('--spacing', type=float, default=2.0)
    ap.add_argument('--min-sep', type=float, default=1.0, help='min distance between parallel lines [m]')
    ap.add_argument('--min-aisle', type=float, default=2.5, help='shortest aisle that gets its own line [m]')
    ap.add_argument('--node-clear', type=float, default=0.55); ap.add_argument('--edge-clear', type=float, default=0.42)
    ap.add_argument('--max-edge', type=float, default=4.5)
    ap.add_argument('--bounds', type=float, nargs=4, default=[-14.3, 14.3, -24.3, 24.3], metavar=('X0', 'X1', 'Y0', 'Y1'))
    ap.add_argument('--tag-z', type=float, default=0.011)
    ap.add_argument('--spawn-tag', type=int, default=1, help='AGV start pose: on this tag')
    ap.add_argument('--center', type=float, nargs=2, default=[0.0, 0.0], metavar=('X', 'Y'),
                    help='the tag nearest to this point gets the id --spawn-tag (default: middle of the warehouse)')
    ap.add_argument('--pkg', default=os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')))
    ap.add_argument('--sim-pkg', default=os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'my_bot_description')))
    a = ap.parse_args()
    from scipy import ndimage as ndi

    img, res, ox, oy = load_map(a.map)
    H, W = img.shape
    blocked = (img != 254) | world_obstacles(a.world_in, res, ox, oy, img.shape)
    dt = ndi.distance_transform_edt(~blocked) * res

    def clear(x, y):
        c, r = int((x - ox) / res), H - 1 - int((y - oy) / res)
        return dt[r, c] if 0 <= c < W and 0 <= r < H else 0.0

    # ---- lines
    x0, x1, y0, y1 = a.bounds
    cap = dt >= a.edge_clear
    cx = lambda c: ox + (c + 0.5) * res                                  # column -> x
    cy = lambda r: oy + (H - 1 - r + 0.5) * res                          # row -> y
    ax_pos = aisle_lines(cap, res, 'x', 0, W, a.min_aisle)               # in column units
    ay_pos = aisle_lines(cap, res, 'y', 0, H, a.min_aisle)               # in row units
    ax = [cx(p) for p in ax_pos if x0 <= cx(p) <= x1]
    ay = [cy(p) for p in ay_pos if y0 <= cy(p) <= y1]

    def merge(aisle, lo, hi):
        aisle = sorted(aisle); kept = []
        for p in aisle:                                                   # aisle lines themselves keep >= min_sep
            if all(abs(p - q) >= a.min_sep for q in kept): kept.append(p)
        n = int((hi - lo) / a.spacing) + 1
        lat = lo + ((hi - lo) - (n - 1) * a.spacing) / 2 + np.arange(n) * a.spacing
        kept += [float(v) for v in lat if all(abs(v - q) >= a.min_sep for q in kept)]
        return sorted(kept)
    X = merge(ax, x0, x1); Y = merge(ay, y0, y1)

    # ---- nodes at crossings, lanes between adjacent crossings
    node = {(i, j): (X[i], Y[j]) for i in range(len(X)) for j in range(len(Y)) if clear(X[i], Y[j]) >= a.node_clear}

    def seg_ok(p, q):
        n = max(2, int(math.dist(p, q) / 0.1))
        return all(clear(p[0] + (q[0] - p[0]) * t / n, p[1] + (q[1] - p[1]) * t / n) >= a.edge_clear for t in range(n + 1))
    edges = []
    for (i, j) in node:
        for di, dj in ((1, 0), (0, 1)):
            k = (i + di, j + dj)
            if k in node and math.dist(node[(i, j)], node[k]) <= a.max_edge and seg_ok(node[(i, j)], node[k]):
                edges.append(((i, j), k))
    adj = {k: [] for k in node}
    for u, v in edges: adj[u].append(v); adj[v].append(u)
    seen, best = set(), []
    for k in node:
        if k in seen: continue
        comp, st = [], [k]; seen.add(k)
        while st:
            u = st.pop(); comp.append(u)
            for v in adj[u]:
                if v not in seen: seen.add(v); st.append(v)
        if len(comp) > len(best): best = comp
    keep = set(best); dropped = len(node) - len(keep)
    order = sorted(keep, key=lambda k: (k[1], k[0]))                      # ids row by row, south -> north, west -> east
    if len(order) > 587: sys.exit(f'{len(order)} tags > 587 ids of tag36h11: raise --spacing')
    tid = {k: n for n, k in enumerate(order)}
    kc = min(order, key=lambda k: math.hypot(node[k][0] - a.center[0], node[k][1] - a.center[1]))      # tag nearest the centre ...
    k1 = next(k for k, n in tid.items() if n == a.spawn_tag)                                            # ... swaps ids with the spawn tag
    tid[kc], tid[k1] = tid[k1], tid[kc]
    order = sorted(keep, key=lambda k: tid[k])
    edges = [(tid[u], tid[v]) for u, v in edges if u in keep and v in keep]

    out = {'family': 'tag36h11', 'tag_size': 0.2, 'frame': 'map',
           'grid': {'type': 'orthogonal', 'x_lines': [round(v, 3) for v in X], 'y_lines': [round(v, 3) for v in Y]},
           'nodes': {tid[k]: {'x': round(node[k][0], 3), 'y': round(node[k][1], 3), 'yaw': 0.0} for k in order},
           'edges': [{'from': u, 'to': v, 'is_bidirectional': True} for u, v in edges]}
    ypath = f'{a.pkg}/config/tag_map_{a.name}.yaml'
    with open(ypath, 'w') as f:
        f.write('# ORTHOGONAL tag grid for the warehouse: node = AprilTag id -> world (x, y); lane = axis-aligned, only between adjacent crossings.\n'
                f'# Generated by scripts/make_tag_ortho.py (lattice {a.spacing} m + aisle centre-lines, node clearance {a.node_clear} m, lane clearance {a.edge_clear} m)\n')
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=None)

    # ---- textures (id n -> same image for every layout), world, spawn
    import cv2
    tex = f'{a.sim_pkg}/models/agv_tags/materials/textures'; os.makedirs(tex, exist_ok=True)
    d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    for n in range(len(order)):
        m = cv2.aruco.drawMarker(d, n, 320, borderBits=1)
        m = cv2.copyMakeBorder(m, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(f'{tex}/tag36_11_{n:05d}.png', np.rot90(m, 2))
    mdir = f'{a.sim_pkg}/models/agv_tags'; os.makedirs(mdir, exist_ok=True)
    if not os.path.exists(f'{mdir}/model.config'):
        open(f'{mdir}/model.config', 'w').write('<?xml version="1.0"?>\n<model><name>agv_tags</name><version>1.0</version>'
            '<sdf version="1.8">model.sdf</sdf><description>AprilTag 36h11 floor textures</description></model>\n')
        open(f'{mdir}/model.sdf', 'w').write('<?xml version="1.0"?>\n<sdf version="1.8"><model name="agv_tags"><static>true</static><link name="link"/></model></sdf>\n')
    src = open(a.world_in).read()
    src = re.sub(r'\s*<include>\s*<uri>\s*[^<]*models/Tugbot\s*</uri>\s*<name>tugbot</name>.*?</include>', '', src, flags=re.S)   # unused robot
    assert '<name>tugbot</name>' not in src and '<topic>/model/tugbot/cmd_vel</topic>' in src
    src = src.replace('<topic>/model/tugbot/cmd_vel</topic>', '<topic>/cmd_vel_teleop</topic>')        # Gazebo keyboard panel -> agv_01
    src = src.replace('<start_paused>true</start_paused>', '<start_paused>false</start_paused>')       # do not pause at start
    block = f'\n    <!-- ===== {len(order)} AprilTag floor markers (make_tag_ortho.py) ===== -->\n'
    for k in order:
        n = tid[k]; x, y = node[k]
        block += (f'    <model name="tag_{n}"><static>true</static><pose>{x:.3f} {y:.3f} {a.tag_z} 0 0 0</pose><link name="link">'
                  f'<visual name="v"><pose>0 0 0.0015 0 0 3.14159265</pose><cast_shadows>false</cast_shadows>'
                  f'<geometry><plane><normal>0 0 1</normal><size>0.25 0.25</size></plane></geometry><material>'
                  f'<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular><pbr><metal>'
                  f'<albedo_map>model://agv_tags/materials/textures/tag36_11_{n:05d}.png</albedo_map>'
                  f'<roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material></visual></link></model>\n')
    wpath = f'{a.sim_pkg}/worlds/{a.name}.sdf'
    open(wpath, 'w').write(src.replace('  </world>', block + '  </world>'))
    st = out['nodes'][a.spawn_tag]
    nb = [(e['to'] if e['from'] == a.spawn_tag else e['from']) for e in out['edges'] if a.spawn_tag in (e['from'], e['to'])]
    yaw = 0.0
    for n2 in nb:                                                          # face along a lane: north if possible, else east
        if abs(out['nodes'][n2]['x'] - st['x']) < 1e-6 and out['nodes'][n2]['y'] > st['y']: yaw = math.pi / 2; break
    open(f'{a.sim_pkg}/worlds/{a.name}.spawn.yaml', 'w').write(
        f'# AGV start pose for {a.name}.sdf: on tag {a.spawn_tag}, facing along a lane\nx: {st["x"]}\ny: {st["y"]}\nyaw: {yaw:.4f}\ntag: {a.spawn_tag}\n')

    # ---- preview
    from PIL import Image, ImageDraw
    sc = 2
    pv = Image.fromarray(np.where(blocked, np.where(img == 254, 120, 70), 235).astype(np.uint8)).convert('RGB').resize((W // sc, H // sc))
    dr = ImageDraw.Draw(pv); P = lambda x, y: ((x - ox) / res / sc, (H - 1 - (y - oy) / res) / sc)
    for u, v in edges:
        dr.line([P(out['nodes'][u]['x'], out['nodes'][u]['y']), P(out['nodes'][v]['x'], out['nodes'][v]['y'])], fill=(60, 140, 255), width=1)
    for n, nd in out['nodes'].items():
        px, py = P(nd['x'], nd['y']); r = 4 if n == a.spawn_tag else 3
        dr.ellipse([px - r, py - r, px + r, py + r], fill=(0, 200, 0) if n == a.spawn_tag else (255, 80, 0))
    pv.save(f'{a.pkg}/maps/{a.name}_preview.png')

    # ---- report
    nodes = out['nodes']; diag = sum(1 for e in edges if abs(nodes[e[0]]['x'] - nodes[e[1]]['x']) > 1e-6 and abs(nodes[e[0]]['y'] - nodes[e[1]]['y']) > 1e-6)
    mask = np.ones((H, W), bool)
    for n in nodes.values(): mask[H - 1 - int((n['y'] - oy) / res), int((n['x'] - ox) / res)] = False
    dn = ndi.distance_transform_edt(mask) * res
    xs = ox + (np.arange(W)[None, :] + 0.5) * res; ys = oy + (H - 1 - np.arange(H)[:, None] + 0.5) * res
    usable = (dt >= a.node_clear) & (xs > x0) & (xs < x1) & (ys > y0) & (ys < y1)
    gap = usable & (dn > 2.0); lab, ng = ndi.label(gap)
    print(f'{len(order)} tags (dropped {dropped} unreachable), {len(edges)} lanes, {diag} diagonal, {len(X)} vertical x {len(Y)} horizontal lines '
          f'({len(ax)} + {len(ay)} aisle lines)')
    print('coverage of usable floor: ' + ', '.join(f'{100 * (usable & (dn <= D)).sum() / usable.sum():.1f}% within {D} m' for D in (1.0, 1.5, 2.0))
          + f'; pockets > 2 m from a tag: {ng}')
    print(f'spawn: tag {a.spawn_tag} at ({st["x"]}, {st["y"]}) yaw {yaw:.2f}\nwrote {ypath}\n      {wpath}\n      {a.pkg}/maps/{a.name}_preview.png')


if __name__ == '__main__':
    main()
