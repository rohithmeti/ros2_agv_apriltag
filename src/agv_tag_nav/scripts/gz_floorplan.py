#!/usr/bin/env python3
"""Occupancy-grid floor plan of a Gazebo world, built from ground truth - no SLAM, no drift, works for any world.

The robot (its lidar) is teleported over a grid of positions with Gazebo's set_pose service; at each position one /scan is
rasterised into the grid at the exact known pose. Positions inside/too close to obstacles are skipped.
What ends up in the map is exactly what the robot's lidar sees (plane at its mount height).

Needs: the simulator running with the robot (sim_agv01.launch.py) and NO other SLAM / tag stack running (they would move the robot).
  gz_floorplan.py --out warehouse --world world_demo --model agv_01 --bounds -14.5 14.5 -24.5 24.5 [--step 1.5]
Out : <maps>/<out>.pgm + <out>.yaml (ROS map_server format) + <out>_preview.png
"""
import argparse, math, os, subprocess, sys, time
import numpy as np


class Grid:
    def __init__(self, x0, x1, y0, y1, res):
        self.res, self.ox, self.oy = res, x0, y0
        self.w, self.h = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
        self.free = np.zeros((self.h, self.w), np.uint16)
        self.occ = np.zeros((self.h, self.w), np.uint16)

    def _idx(self, x, y):
        ix = np.floor((x - self.ox) / self.res).astype(int)
        iy = np.floor((y - self.oy) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        return ix[ok], iy[ok]

    def add_scan(self, px, py, yaw, ranges, angle_min, inc, range_min, max_range):
        r = np.asarray(ranges, np.float64)
        a = yaw + angle_min + np.arange(len(r)) * inc
        valid = ~np.isnan(r) & (r >= range_min)
        r, a = r[valid], a[valid]
        hit = np.isfinite(r) & (r < max_range - 0.02)
        rr = np.where(np.isfinite(r), np.minimum(r, max_range), max_range)
        step = self.res * 0.5
        n = int(max_range / step)
        t = (np.arange(n) * step)[None, :]
        keep = t < (rr[:, None] - self.res * 0.75)                  # free space stops just before the hit cell
        xs = px + t * np.cos(a)[:, None]
        ys = py + t * np.sin(a)[:, None]
        ix, iy = self._idx(xs[keep], ys[keep])
        cells = np.unique(iy * self.w + ix)                           # a cell counts once per scan
        np.add.at(self.free.ravel(), cells, 1)
        ix, iy = self._idx(px + rr[hit] * np.cos(a[hit]), py + rr[hit] * np.sin(a[hit]))
        cells = np.unique(iy * self.w + ix)
        np.add.at(self.occ.ravel(), cells, 1)

    def image(self):
        """uint8 image, row 0 = max y. 0 occupied, 254 free, 205 unknown (map_server 'trinary')"""
        img = np.full((self.h, self.w), 205, np.uint8)
        img[self.free > 0] = 254
        img[(self.occ >= 1) & (self.occ >= 0.2 * (self.free + self.occ))] = 0
        return img[::-1]


def save(grid, name, folder):
    os.makedirs(folder, exist_ok=True)
    img = grid.image()
    base = os.path.join(folder, name)
    with open(base + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (grid.w, grid.h)); f.write(img.tobytes())
    with open(base + '.yaml', 'w') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {grid.res}\norigin: [{grid.ox}, {grid.oy}, 0.0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    try:
        from PIL import Image
        Image.fromarray(img).save(base + '_preview.png')
    except Exception:
        pass
    return base


def set_pose(world, model, x, y, yaw=0.0):
    req = (f'name: "{model}" position {{x: {x:.3f} y: {y:.3f} z: 0.02}} '
           f'orientation {{z: {math.sin(yaw / 2):.5f} w: {math.cos(yaw / 2):.5f}}}')
    r = subprocess.run(['ign', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '2000', '--req', req],
                       capture_output=True, text=True)
    return 'true' in r.stdout


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True, help='map name')
    ap.add_argument('--world', default='world_demo')
    ap.add_argument('--model', default='agv_01')
    ap.add_argument('--bounds', type=float, nargs=4, metavar=('X0', 'X1', 'Y0', 'Y1'), required=True)
    ap.add_argument('--step', type=float, default=1.5, help='spacing of the scan positions [m]')
    ap.add_argument('--res', type=float, default=0.05, help='cell size [m]')
    ap.add_argument('--max-range', type=float, default=10.0)
    ap.add_argument('--dir', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps'))
    ap.add_argument('--margin', type=float, default=0.5)
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    rclpy.init()
    node = Node('gz_floorplan', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    last = {}
    node.create_subscription(LaserScan, '/scan', lambda m: last.__setitem__('m', m), qos_profile_sensor_data)

    def fresh_scan(after, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            m = last.get('m')
            if m is not None and m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 > after:
                return m
        return None

    x0, x1, y0, y1 = a.bounds
    grid = Grid(x0 - a.margin, x1 + a.margin, y0 - a.margin, y1 + a.margin, a.res)
    xs = np.arange(x0, x1 + 1e-6, a.step)
    ys = np.arange(y0, y1 + 1e-6, a.step)
    poses = [(x, y) for i, x in enumerate(xs) for y in (ys if i % 2 == 0 else ys[::-1])]   # lawn-mower order
    print(f'{len(poses)} positions, grid {grid.w}x{grid.h} cells ({grid.w * a.res:.0f} x {grid.h * a.res:.0f} m)', flush=True)

    while last.get('m') is None:
        rclpy.spin_once(node, timeout_sec=0.5)
        print('waiting for /scan ...', flush=True)
    used = skipped = 0
    t_start = time.time()
    for k, (x, y) in enumerate(poses):
        m = last['m']
        now = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not set_pose(a.world, a.model, x, y):
            print('set_pose failed - is the world/model name right?  (ign model --list)', file=sys.stderr); break
        m = fresh_scan(now + 0.5)
        if m is None:
            skipped += 1; continue
        r = np.asarray(m.ranges, np.float64)
        fin = r[np.isfinite(r) & (r >= m.range_min)]
        if len(fin) < 0.2 * len(r) or fin.min() < 0.45:          # inside / touching an obstacle: robot may have been pushed
            skipped += 1; continue
        grid.add_scan(x, y, 0.0, r, m.angle_min, m.angle_increment, m.range_min, min(a.max_range, m.range_max))
        used += 1
        if (k + 1) % 50 == 0:
            el = time.time() - t_start
            print(f'  {k + 1}/{len(poses)}  used {used} skipped {skipped}  ~{el / (k + 1) * (len(poses) - k - 1) / 60:.1f} min left', flush=True)
    set_pose(a.world, a.model, *poses[0][:2])
    base = save(grid, a.out, a.dir)
    unknown = float((grid.image() == 205).mean()) * 100
    print(f'done: {used} scans used, {skipped} skipped -> {base}.pgm/.yaml   (unknown cells {unknown:.0f}%)')


if __name__ == '__main__':
    main()
