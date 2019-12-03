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
            if not (p1.x == route_points[-1].x) & (p1.y == route_points[-1].y):
                route_points.append(p1)
            if not (p2.x == route_points[-1].x) & (p2.y == route_points[-1].y):
                route_points.append(p2)

        # make curves
        new_points = [start]
        for p in range(1, len(route_points)-1):
            p, p2, p3 = self.make_curve(route_points[p], route_points[p-1], route_points[p+1], car)
            p1 = Point(p[0], p[1])
            try:
                if not (new_points[-1].x == p1.x) & (new_points[-1].y == p1.y):
                    new_points.append(p1)
            except IndexError:
                new_points.append(p1)

            if not np.all(p == p3):
                p2 = Point(p2[0], p2[1])
                new_points.append(p2)
                p3 = Point(p3[0], p3[1])
                new_points.append(p3)

        if car.end_dir == "north":
            end.x = end.x - 0.25 * self.spacing[0]
        else:
            if car.end_dir == "east":
                end.y = end.y + 0.25 * self.spacing[1]
            else:
                if car.end_dir == "south":
                    end.x = end.x + 0.25 * self.spacing[0]
                else:
                    end.y = end.y - 0.25 * self.spacing[1]
        new_points.append(end)

        spawn_x = new_points[0].x
        spawn_y = new_points[0].y
        if car.start_dir == "north":
            spawn_x = spawn_x + 0.25 * self.spacing[0]
        else:
            if car.start_dir == "east":
                spawn_y = spawn_y - 0.25 * self.spacing[1]
            else:
                if car.start_dir == "south":
                    spawn_x = spawn_x - 0.25 * self.spacing[0]
                else:
                    spawn_y = spawn_y + 0.25 * self.spacing[1]
        new_points[0] = Point(spawn_x, spawn_y)
        car.spawn = [spawn_x, spawn_y]
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
                end_cross = Point(entry.x, entry.y + np.sign(start.y - end.y) * self.spacing[1])
            else:
                if car.end_dir == "south":
                    entry = Point(end.x, end.y - self.spacing[1])
                    end_cross = Point(entry.x + np.sign(start.x - end.x) * self.spacing[0], entry.y)
                else:
                    entry = Point(end.x - self.spacing[0], end.y)
                    end_cross = Point(entry.x, entry.y + np.sign(start.y - end.y) * self.spacing[1])
        print("points", [start.x, start.y], [ext.x, ext.y], [start_cross.x, start_cross.y], [end_cross.x, end_cross.y], [entry.x, entry.y], [end.x, end.y])
        return [start, ext, start_cross, end_cross, entry, end]

    def make_curve(self, point, prev_point, next_point, car):
        point = np.array([point.x, point.y])
        prev_point = np.array([prev_point.x, prev_point.y])
        prev_vec = point - prev_point
        next_point = np.array([next_point.x, next_point.y])
        next_vec = next_point - point

        # start = point + np.sign(prev_point - point) * 0.25 * self.spacing
        # end = point + np.sign(next_point - point) * 0.25 * self.spacing

        angle = np.sign(np.cross(prev_vec, next_vec))
        if angle == -1:
            pass
            start, mid, end = self.make_right_turn(prev_point, point, next_point, car)
        else:
            if angle == 1:
                pass
                start, mid, end = self.make_left_turn(prev_point, point, next_point, car)
            else:
                start = mid = end = self.go_straight(prev_point, point)

        return start, mid, end

    def make_right_turn(self, prev_point, point, next_point, car):
        if np.all(prev_point == car.spawn) or np.all(next_point == car.end):
            start_fin = 0
        else:
            start_fin = 1
        start_fin = 1
        prev_vec = point-prev_point
        vec_norm = prev_vec / np.linalg.norm(prev_vec)
        dist = np.flip(prev_vec, 0)
        dist_norm = dist / np.linalg.norm(dist)
        dist_norm = dist_norm * (-np.cross(vec_norm, dist_norm))
        #print(vec_norm, dist_norm)
        start = point + np.sign(prev_point - point) * 0.5 * self.spacing + 0.25 * dist_norm * self.spacing * start_fin
        end = point + np.sign(next_point - point) * 0.5 * self.spacing - 0.25 * vec_norm * self.spacing * start_fin

        center = start + np.cos(0.25 * np.pi) * np.linalg.norm(end-start) * dist_norm

        end_c = end-center
        start_c = start - center
        r = np.sqrt(end_c[0]**2+end_c[1]**2)
        #print(r)
        phi1 = np.arctan2(end_c[1], end_c[0])
        phi2 = np.arctan2(start_c[1], start_c[0])
        phi = 0.5*(phi1 + phi2)
        #print(phi)
        mid = np.array([r * np.cos(phi), r * np.sin(phi)]) + center
        return start, mid, end

    def make_left_turn(self, prev_point, point, next_point, car):
        vec = point-prev_point
        vec_norm = vec / np.linalg.norm(vec)
        dist = np.flip(vec, 0)
        dist_norm = dist / np.linalg.norm(dist)
        dist_norm = dist_norm * (-np.cross(vec_norm, dist_norm))
        #print(vec_norm, dist_norm)
        start = point + np.sign(prev_point - point) * 0.5 * self.spacing + 0.25 * dist_norm * self.spacing
        end = point + np.sign(next_point - point) * 0.5 * self.spacing + 0.25 * vec_norm * self.spacing

        center = start - 0.75 * dist_norm * self.spacing

        end_c = end-center
        start_c = start - center
        r = np.sqrt(end_c[0]**2+end_c[1]**2)
        #print(r)
        phi1 = np.arctan2(end_c[1], end_c[0])
        phi2 = np.arctan2(start_c[1], start_c[0])
        phi = 0.5*(phi1 + phi2)
        #print(phi)
        mid = np.array([r * np.cos(phi), r * np.sin(phi)]) + center
        return start, mid, end

    def go_straight(self, prev_point, point):
        vec = point-prev_point
        vec_norm = vec / np.linalg.norm(vec)
        dist = np.flip(vec, 0)
        dist_norm = dist / np.linalg.norm(dist)
        dist_norm = dist_norm * (-np.cross(vec_norm, dist_norm))
        new = point + 0.25 * dist_norm * self.spacing
        return new
