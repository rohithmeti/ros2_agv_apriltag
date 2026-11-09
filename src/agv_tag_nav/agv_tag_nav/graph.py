"""The AGV's internal map: AprilTag ids are nodes, lanes are edges. Dijkstra over it."""
import heapq
import math
import yaml


class TagGraph:
    def __init__(self, path):
        d = yaml.safe_load(open(path))
        self.tag_size = float(d['tag_size'])
        self.frame = d.get('frame', 'map')
        self.nodes = {int(i): (float(n['x']), float(n['y']), float(n.get('yaw', 0.0)))
                      for i, n in d['nodes'].items()}
        self.adj = {i: [] for i in self.nodes}
        for e in d['edges']:
            a, b = int(e['from']), int(e['to'])
            c = self.dist(a, b)
            self.adj[a].append((b, c))
            if e.get('is_bidirectional', True):
                self.adj[b].append((a, c))

    def dist(self, a, b):
        return math.hypot(self.nodes[a][0] - self.nodes[b][0], self.nodes[a][1] - self.nodes[b][1])

    def nearest(self, x, y):
        return min(self.nodes, key=lambda i: math.hypot(self.nodes[i][0] - x, self.nodes[i][1] - y))

    def shortest_path(self, start, goal):
        """-> (list of node ids start..goal, total length m) or (None, inf)."""
        best, prev, heap = {start: 0.0}, {}, [(0.0, start)]
        while heap:
            c, u = heapq.heappop(heap)
            if u == goal:
                path = [u]
                while u in prev:
                    u = prev[u]
                    path.append(u)
                return path[::-1], c
            if c > best.get(u, math.inf):
                continue
            for v, w in self.adj[u]:
                if c + w < best.get(v, math.inf):
                    best[v], prev[v] = c + w, u
                    heapq.heappush(heap, (c + w, v))
        return None, math.inf
