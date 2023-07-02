"""
This graph should be a flow graph that allows some maximum delay in the
makespan. For small max delay values (like 5), it should remain reasonably small.
"""
import typing

import networkx as nx

from .common import Vertex
from .conflict_counter import ConflictCounter

TimeVertex = typing.Tuple[Vertex, int]


class SingleRobotFlowGraph:
    def __init__(
        self,
        robot_id: typing.Hashable,
        origin: Vertex,
        target: Vertex,
        max_delay: int,
        graph: nx.Graph,
        all_pairs_shortest_path: typing.Dict,
        conflict_counter: ConflictCounter,
    ):
        """
        The SingleRobotFlowGraph will create a directed graph consisting of
        position&time-vertices. It will only contain directed edges (movements)
        that can potentially be a feasible movement for the robot from its
        `origin` to its `target` with a maximum delay (extra moves or waiting)
        of `max_delay`.
        :param robot_id:
        :param origin:
        :param target:
        :param max_delay:
        :param graph:
        :param all_pairs_shortest_path: Needed to determine if a vertex/edge
        is within reach.
        :param conflict_counter: Needed to remove colliding movements.
        """
        self.robot_id = robot_id
        self.origin = origin
        self.target = target
        self.max_delay = max_delay
        self.graph = graph
        self.all_pairs_shortest_path = all_pairs_shortest_path
        self.min_dist = all_pairs_shortest_path[origin][target]
        self.dgraph = nx.DiGraph()
        self._fill_graph(conflict_counter)

    def _fill_graph(self, conflict_counter):
        new_vertices = {(self.origin, 0)}
        while new_vertices:
            pre_vertices = new_vertices
            new_vertices = set()
            for v, t in pre_vertices:
                if self._is_in_range(v, t + 1) and not conflict_counter.query(v, v, t):
                    self.dgraph.add_edge((v, t), (v, t + 1))
                    new_vertices.add((v, t + 1))
                for w in self.graph.neighbors(v):
                    if self._is_in_range(w, t + 1) and not conflict_counter.query(
                        v, w, t
                    ):
                        self.dgraph.add_edge((v, t), (w, t + 1))
                        new_vertices.add((w, t + 1))

    def is_feasible(self):
        target_timevertex = (self.target, self.min_dist + self.max_delay)
        return self.dgraph.has_node(target_timevertex)

    def _is_in_range(self, w: Vertex, t: int):
        return (
            t + self.all_pairs_shortest_path[w][self.target]
            <= self.min_dist + self.max_delay
        )

    def is_entry(self, v: TimeVertex):
        return v[1] == 0

    def is_exit(self, v: TimeVertex):
        return v[1] == self.min_dist + self.max_delay

    def normalize_exit(self, v: TimeVertex):
        assert v[0] == self.target
        if v[1] > self.min_dist + self.max_delay:
            return (v[0], self.min_dist + self.max_delay)
        return v

    def edges(self):
        return self.dgraph.edges()

    def nodes(self):
        return self.dgraph.nodes

    def incoming(self, v: TimeVertex):
        return self.dgraph.in_edges(v)

    def outgoing(self, v: TimeVertex):
        return self.dgraph.out_edges(v)
