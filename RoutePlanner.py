from Obstacles2D import Obstacles2D
from Point import Point
import numpy as np


class RoutePlanner:
    def __init__(self, god):
        self.god = god
        self.spacing = None

    def make_layout(self, width, height, obs_x: int, obs_y: int):
        ob_width = width / (2 * obs_x + 1)
        ob_height = height / (2 * obs_y + 1)

        self.spacing = np.array([0.75 * ob_width, 0.75 * ob_height])

        for x in range(obs_x):
            for y in range(obs_y):
                spawn_x = (2 * x + 1) * ob_width
                spawn_y = (2 * y + 1) * ob_height
                edges = [spawn_x, spawn_y, spawn_x + ob_width, spawn_y, spawn_x + ob_width, spawn_y + ob_height,
                         spawn_x, spawn_y + ob_height]

                obstacle = Obstacles2D(spawn_x, spawn_y, edges, 'gray')
                self.god.obstacles.append(obstacle)

    def make_route(self, points):
        start = points[0]
        end = points[-1]
        route_points = []

        # make tuples of consecutive points
        pairs = []
        for p in range(len(points)-1):
            pairs.append((points[p], points[p+1]))

        # Add "Edge"
        route_points.append(start)
        for p in pairs:
            p1 = Point(p[0].x, p[1].y)
            p2 = p[1]
            if not (p1.x == route_points[-1].x) & (p1.y == route_points[-1].y):
                route_points.append(p1)
            if not (p2.x == route_points[-1].x) & (p2.y == route_points[-1].y):
                route_points.append(p2)

        # OR
        # direct connection between two points

        # make path "discrete"

        # make curves
        new_points = [start]
        for p in range(1, len(route_points)-1):
            p1, p2 = self.make_curve(route_points[p], route_points[p-1], route_points[p+1])
            p1 = Point(p1[0], p1[1])
            p2 = Point(p2[0], p2[1])
            new_points.append(p1)
            new_points.append(p2)

        new_points.append(end)
        for p in points:
            print(p.x, p.y)
        return new_points

    def make_curve(self, point, prev_point, next_point):
        point = np.array([point.x, point.y])
        prev_point = np.array([prev_point.x, prev_point.y])
        next_point = np.array([next_point.x, next_point.y])

        start = point + np.sign(prev_point - point) * 0.25 * self.spacing
        end = point + np.sign(next_point - point) * 0.25 * self.spacing

        return start, end
