from __future__ import annotations

import math
from typing import List

from src.models import CarPose, Cone, Path2D


class PathPlanning:
    """Simple center-line path planner for the cone track task."""

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        # A few simple assumptions make the problem manageable:
        # - blue cones are on the left and yellow cones are on the right
        # - the track is roughly 3 m wide
        # - nearby cones are part of the same local track
        track_width = 3.0
        step = 0.5
        path_length = 6.0

        car = (self.car_pose.x, self.car_pose.y)
        heading = (
            math.cos(self.car_pose.yaw),
            math.sin(self.car_pose.yaw),
        )

        def distance(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        def midpoint(a, b):
            return (
                (a[0] + b[0]) / 2.0,
                (a[1] + b[1]) / 2.0,
            )

        def direction_at(index, side):
            # With only one cone, the car heading is the best simple guess.
            if len(side) == 1:
                return heading

            if index == 0:
                a, b = side[0], side[1]
            elif index == len(side) - 1:
                a, b = side[-2], side[-1]
            else:
                a, b = side[index - 1], side[index + 1]

            dx = b[0] - a[0]
            dy = b[1] - a[1]
            length = math.hypot(dx, dy)

            if length < 1e-6:
                return heading

            return dx / length, dy / length

        # Split cones by side and put the closest ones first.
        blue = [(c.x, c.y) for c in self.cones if c.color == 1]
        yellow = [(c.x, c.y) for c in self.cones if c.color == 0]

        blue.sort(key=lambda p: distance(car, p))
        yellow.sort(key=lambda p: distance(car, p))

        centers = []

        # If both sides are visible, use the midpoint.
        pair_count = min(len(blue), len(yellow))

        for i in range(pair_count):
            centers.append(midpoint(blue[i], yellow[i]))

        # If only one side is available for a cone, estimate the
        # other side using the track width and take the midpoint.
        for side, is_blue in ((blue, True), (yellow, False)):
            for i in range(pair_count, len(side)):
                cone = side[i]
                dx, dy = direction_at(i, side)

                # Left normal of the local track direction.
                nx, ny = -dy, dx

                # Blue is on the left -> center is to the right.
                # Yellow is on the right -> center is to the left.
                sign = -1.0 if is_blue else 1.0

                other_side = (
                    cone[0] + sign * nx * track_width,
                    cone[1] + sign * ny * track_width,
                )

                centers.append(midpoint(cone, other_side))

        # If there are no cones, drive straight ahead.
        if not centers:
            return [
                (
                    car[0] + heading[0] * step * i,
                    car[1] + heading[1] * step * i,
                )
                for i in range(1, int(path_length / step) + 1)
            ]

        # Keep center points in a simple near-to-far order.
        centers.sort(key=lambda p: distance(car, p))

        # Start from the car.
        waypoints = [car] + centers

        # Add one forward point if the first waypoint is not ahead enough.
        if len(waypoints) == 1:
            waypoints.append(
                (
                    car[0] + heading[0],
                    car[1] + heading[1],
                )
            )

        # Remove duplicate points.
        clean = [waypoints[0]]
        for point in waypoints[1:]:
            if distance(point, clean[-1]) > 1e-6:
                clean.append(point)

        # Build a path with points every 0.5 m.
        path = []
        target = step
        travelled = 0.0

        for i in range(len(clean) - 1):
            a = clean[i]
            b = clean[i + 1]
            segment = distance(a, b)

            if segment < 1e-6:
                continue

            while target <= travelled + segment and target <= path_length:
                t = (target - travelled) / segment
                path.append(
                    (
                        a[0] + t * (b[0] - a[0]),
                        a[1] + t * (b[1] - a[1]),
                    )
                )
                target += step

            travelled += segment

            if travelled >= path_length:
                break

        # If the visible cones do not give us 6 m of path,
        # continue straight from the last point.
        if path:
            last = path[-1]
        else:
            last = car

        if len(clean) >= 2:
            a, b = clean[-2], clean[-1]
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            length = math.hypot(dx, dy)
            if length > 1e-6:
                last_direction = (dx / length, dy / length)
            else:
                last_direction = heading
        else:
            last_direction = heading

        while target <= path_length:
            path.append(
                (
                    last[0] + last_direction[0] * (target - travelled),
                    last[1] + last_direction[1] * (target - travelled),
                )
            )
            target += step

        return path
