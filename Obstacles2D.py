import Lib as lib


class Obstacles2D:
    def __init__(self, spawn_x, spawn_y, corners: list, color):
        # PROPERTIES
        self.id = 99    # TODO: obstacle id
        self.color = color
        self.spawn = [spawn_x, spawn_y]
        self.position = []  # in m - later: instantiation of 2d space with dates in metres
        self.edges = corners
        self.centroid = 0
        self.min_dist = self.make_min_distance()

    def make_min_distance(self):
        edges = []
        for i in range(int(len(self.edges)/2)):
            edges.append([self.edges[2*i], self.edges[2*i+1]])
        self.make_centroid()
        out = lib.dist(self.centroid, edges[0])
        for e in edges:
            dist = lib.dist(self.centroid, e)
            if dist > out:
                out = dist
        return out

    def make_centroid(self):
        edges = []
        for i in range(int(len(self.edges)/2)):
            edges.append([self.edges[2*i], self.edges[2*i+1]])
        a = 0
        x = 0
        y = 0
        for i in range(len(edges) - 1):
            a += edges[i][0] * edges[i+1][1] - edges[i+1][0] * edges[i][1]
            x += (edges[i][0] + edges[i+1][0]) * (edges[i][0]*edges[i+1][1] - edges[i+1][0]*edges[i][1])
            y += (edges[i][1] + edges[i+1][1]) * (edges[i][0]*edges[i+1][1] - edges[i+1][0]*edges[i][1])
        a = 0.5 * a
        x = x / (3*a)
        y = y / (3*a)
        self.centroid = [x, y]
