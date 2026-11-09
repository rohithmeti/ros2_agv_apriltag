#!/usr/bin/env python3
"""Drift / delocalisation check: drive a list of goal tags and compare the stack's pose estimate (TF map->base_footprint)
with Gazebo's ground truth the whole way.

  drift_check.py 100 200 1            # goals = tag ids, visited in order (needs the sim + agv_tag_nav stack running, no one else clicking)
  options: --yaml <tag map>  --world world_demo  --model agv_01  --timeout 400

Reports per leg: time, max / mean position error and max heading error while driving, and the stop error against the
goal tag's coordinates. Position error > 0.3 m is flagged as DELOCALISED. Stops early if another goal replaces ours.
"""
import argparse, json, math, re, subprocess, sys, threading, time
import numpy as np, yaml

ap = argparse.ArgumentParser()
ap.add_argument('goals', type=int, nargs='*', help='goal tag ids; none = WATCH mode (send no goals, just record while someone else drives)')
ap.add_argument('--watch-time', type=float, default=600.0)
import os
ap.add_argument('--yaml', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'tag_map_warehouse_ortho.yaml'))
ap.add_argument('--world', default='world_demo'); ap.add_argument('--model', default='agv_01')
ap.add_argument('--timeout', type=float, default=400.0)
a = ap.parse_args()

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformListener

nodes = yaml.safe_load(open(a.yaml))['nodes']
gt = {'p': None}


def gt_reader():                                   # streams Gazebo's moving-entity poses, keeps the robot's
    pr = subprocess.Popen(['ign', 'topic', '-e', '-t', f'/world/{a.world}/dynamic_pose/info'], stdout=subprocess.PIPE, text=True)
    buf = []
    for line in pr.stdout:
        buf.append(line)
        if line.startswith('}') and len(buf) > 3:  # one pose {...} block ends
            blk = ''.join(buf); buf = []
            if f'name: "{a.model}"' not in blk or 'position' not in blk: continue
            def field(section, key):                      # protobuf text omits zero values
                m = re.search(rf'{section} \{{(.*?)\}}', blk, re.S)
                f = re.search(rf'\b{key}: ([-\d.e]+)', m.group(1)) if m else None
                return float(f.group(1)) if f else 0.0
            qz, qw = field('orientation', 'z'), field('orientation', 'w')
            gt['p'] = (field('position', 'x'), field('position', 'y'), 2 * math.atan2(qz, qw or 1.0), time.time())
threading.Thread(target=gt_reader, daemon=True).start()

rclpy.init(); node = Node('drift_check', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
buf = Buffer(); TransformListener(buf, node, spin_thread=True)
pub = node.create_publisher(Int32, '/agv/goal_node', 1); st = {}
node.create_subscription(String, '/agv/status', lambda m: st.update(d=json.loads(m.data)), 10)
wrap = lambda v: (v + math.pi) % (2 * math.pi) - math.pi


def est():
    try:
        t = buf.lookup_transform('map', 'base_footprint', Time()).transform
        return t.translation.x, t.translation.y, 2 * math.atan2(t.rotation.z, t.rotation.w)
    except Exception:
        return None


t0 = time.time()
while gt['p'] is None or est() is None:
    time.sleep(0.2)
    if time.time() - t0 > 30: sys.exit('no ground truth / no map->base_footprint TF: is the sim + stack running?')
def watch():                                       # passive: log error while goals come from elsewhere (RViz clicks)
    print(f'WATCH mode for {a.watch_time:.0f} s: click goals in RViz, nothing is sent from here', flush=True)
    t_end = time.time() + a.watch_time; last_goal = None; last_state = None; errs = []; worst = 0.0; n_arr = 0; t_print = time.time()
    while time.time() < t_end:
        time.sleep(0.1); d = st.get('d'); e, g = est(), gt['p']
        if e and g and time.time() - g[3] < 0.5:
            r = math.hypot(e[0] - g[0], e[1] - g[1]); errs.append(r); worst = max(worst, r)
        if d and d['state'] == 'ARRIVED' and last_state != 'ARRIVED':
            time.sleep(1.0); g = gt['p']; tx, ty = nodes[d['goal']]['x'], nodes[d['goal']]['y']; n_arr += 1
            print(f'  arrived at tag {d["goal"]:3d} ({tx:6.2f},{ty:6.2f}): stop error {math.hypot(g[0]-tx, g[1]-ty)*100:4.1f} cm, '
                  f'estimate-vs-truth max so far {worst*100:5.1f} cm' + ('  <-- DELOCALISED' if worst > 0.3 else ''), flush=True)
        if d: last_state = d['state']
        if time.time() - t_print > 30:
            t_print = time.time(); print(f'  t+{t_end - time.time() + a.watch_time:4.0f}s state {last_state} mean err {np.mean(errs or [0])*100:4.1f} cm max {worst*100:5.1f} cm', flush=True)
    print(f'WATCH SUMMARY: {n_arr} arrivals, estimate-vs-truth error mean {np.mean(errs or [0])*100:.1f} cm, max {worst*100:.1f} cm')
    sys.exit(0)


e0, g0 = est(), gt['p']
print(f'start: estimate ({e0[0]:.2f}, {e0[1]:.2f}, {math.degrees(e0[2]):.0f} deg)  truth ({g0[0]:.2f}, {g0[1]:.2f}, {math.degrees(g0[2]):.0f} deg)  error {math.hypot(e0[0]-g0[0], e0[1]-g0[1])*100:.1f} cm', flush=True)
if not a.goals: watch()
worst = 0.0; rows = []
for goal in a.goals:
    time.sleep(0.5); pub.publish(Int32(data=goal)); t1 = time.time(); errs = []; hd = []; state = None; route = None
    while time.time() - t1 < a.timeout:
        time.sleep(0.1)
        d = st.get('d')
        if d and d['goal'] == goal:
            state = d['state']; route = route or d['route']
            if state == 'ARRIVED': break
        elif d and d['goal'] not in (goal, None) and time.time() - t1 > 3:
            print(f'goal {goal}: INTERRUPTED, the stack now has goal {d["goal"]} (someone clicked another goal)'); sys.exit(1)
        e, g = est(), gt['p']
        if e and g and time.time() - g[3] < 0.5:
            errs.append(math.hypot(e[0] - g[0], e[1] - g[1])); hd.append(abs(wrap(e[2] - g[2])))
    time.sleep(1.0)
    g = gt['p']; tx, ty = nodes[goal]['x'], nodes[goal]['y']
    stop = math.hypot(g[0] - tx, g[1] - ty) * 100
    length = sum(math.dist((nodes[p]['x'], nodes[p]['y']), (nodes[q]['x'], nodes[q]['y'])) for p, q in zip(route, route[1:])) if route else 0
    ok = state == 'ARRIVED'
    worst = max(worst, max(errs or [0]))
    flag = ' <-- DELOCALISED' if max(errs or [0]) > 0.3 else ''
    print(f'goal {goal:3d} ({tx:6.2f},{ty:6.2f}) {"ARRIVED" if ok else "NOT ARRIVED (" + str(state) + ")"} {time.time()-t1:5.0f}s route {len(route or []):2d} tags {length:5.1f} m | '
          f'est-vs-truth while driving: max {max(errs or [0])*100:5.1f} cm mean {np.mean(errs or [0])*100:5.1f} cm, heading max {math.degrees(max(hd or [0])):4.1f} deg | stop error {stop:4.1f} cm{flag}', flush=True)
    rows.append((ok, stop, max(errs or [0])))
print(f'SUMMARY: {sum(r[0] for r in rows)}/{len(rows)} arrived, stop error max {max(r[1] for r in rows):.1f} cm, worst estimate-vs-truth error {worst*100:.1f} cm')
