from Obstacles2D import Obstacles2D
from Point import Point
import numpy as np


class RoutePlanner:
    def __init__(self, god):
        self.god = god
        self.spacing = None
        self.ob_width = 0
        self.ob_height = 0

    def make_layout(self, width, height, obs_x: int, obs_y: int):
        self.ob_width = width / (2 * obs_x + 1)
        self.ob_height = height / (2 * obs_y + 1)

        self.spacing = np.array([self.ob_width, self.ob_height])

        for x in range(obs_x):
            for y in range(obs_y):
                spawn_x = (2 * x + 1) * self.ob_width
                spawn_y = (2 * y + 1) * self.ob_height
                edges = [spawn_x, spawn_y, spawn_x + self.ob_width, spawn_y, spawn_x + self.ob_width, spawn_y + self.ob_height,
                         spawn_x, spawn_y + self.ob_height]

                obstacle = Obstacles2D(spawn_x, spawn_y, edges, 'gray')
                self.god.obstacles.append(obstacle)

    def make_route(self, car):
        points = self.make_exit_and_entry(car)
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
            test = (p1.x == route_points[-1].x) & (p1.y == route_points[-1].y)
            test2 = (p2.x == route_points[-1].x) & (p2.y == route_points[-1].y)
            if not (p1.x == route_points[-1].x) & (p1.y == route_points[-1].y):
                route_points.append(p1)
            if not (p2.x == route_points[-1].x) & (p2.y == route_points[-1].y):
                route_points.append(p2)

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

    def make_exit_and_entry(self, car):
        points = car.path.points
        start = points[0]
        end = points[1]
        if car.start_dir == "north":
            ext = Point(start.x, start.y + self.spacing[1])
            start_cross = Point(ext.x + np.sign(end.x - start.x) * self.spacing[0], ext.y)
        else:
            if car.start_dir == "east":
                ext = Point(start.x + self.spacing[0], start.y)
                start_cross = Point(ext.x, ext.y + np.sign(end.y - start.y) * self.spacing[1])
            else:
                if car.start_dir == "south":
                    ext = Point(start.x, start.y - self.spacing[1])
                    start_cross = Point(ext.x + np.sign(end.x - start.x) * self.spacing[0], ext.y)
                else:
                    ext = Point(start.x - self.spacing[0], start.y)
                    start_cross = Point(ext.x, ext.y + np.sign(end.y - start.y) * self.spacing[1])

        if car.end_dir == "north":
            entry = Point(end.x, end.y + self.spacing[1])
            end_cross = Point(entry.x - np.sign(start.x - end.x) * self.spacing[0], entry.y)
        else:
            if car.end_dir == "east":
                entry = Point(end.x + self.spacing[0], end.y)
                end_cross = Point(entry.x, entry.y - np.sign(start.y - end.y) * self.spacing[1])
            else:
                if car.end_dir == "south":
                    entry = Point(end.x, end.y - self.spacing[1])
                    end_cross = Point(entry.x - np.sign(start.x - end.x) * self.spacing[0], entry.y)
                else:
                    entry = Point(end.x - self.spacing[0], end.y)
                    end_cross = Point(entry.x, entry.y - np.sign(start.y - end.y) * self.spacing[1])

        points = [start, ext, start_cross, end_cross, entry, end]
        return points

    def make_curve(self, point, prev_point, next_point):
        point = np.array([point.x, point.y])
        prev_point = np.array([prev_point.x, prev_point.y])
        next_point = np.array([next_point.x, next_point.y])

        start = point + np.sign(prev_point - point) * 0.25 * self.spacing
        end = point + np.sign(next_point - point) * 0.25 * self.spacing

        return start, end
