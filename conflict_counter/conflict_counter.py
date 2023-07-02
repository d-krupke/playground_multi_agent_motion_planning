"""
This file implements some logic on checking efficiently with which robots
a movement would collide, given the paths of the robots.
"""

import typing

from .common import Vertex


class ConflictCounter:
    def __init__(self):
        self.paths = {}
        self.crossings = {}

    def add_path(self, robot: typing.Hashable, path: typing.Iterable[Vertex]):
        path = list(path)
        self.paths[robot] = path
        for t, v in enumerate(path):
            if v not in self.crossings:
                self.crossings[v] = []
            self.crossings[v].append((robot, t, t == len(path) - 1))

    def query(self, v: Vertex, w: Vertex, t: int) -> typing.List[int]:
        """
        Moving from v to w starting at time t, arriving at t+1.
        :param v: Origin vertex
        :param w: Target vertex, adjacent to v.
        :param t: Leaving v at t, arriving at w at t+1.
        :return: List of robot ids that will have a collision.
        """
        conflicts = 0
        conflict_robots = set()
        for robot, time, final in self.crossings.get(v, []):
            if time <= t and final:
                # collision with robot that reached its target
                conflicts += 1
                conflict_robots.add(robot)
            elif time == t:
                # collision at vertex
                conflicts += 1
                conflict_robots.add(robot)
            elif time == t + 1 and self.paths[robot][t] == w:
                # collision on edge
                conflicts += 1
                conflict_robots.add(robot)
        for robot, time, final in self.crossings.get(w, []):
            if time <= t + 1 and final:
                # collision with robot that reached its target
                conflicts += 1
                conflict_robots.add(robot)
            elif time == t:
                # collision at vertex
                conflicts += 1
                conflict_robots.add(robot)
        return list(conflict_robots)
