import typing

import gurobipy as gp
from gurobipy import GRB

from .common import Vertex
from .conflict_counter import ConflictCounter
from .flow_graph import SingleRobotFlowGraph


class SingleRobotFlowModel:
    """
    A MIP-model (or part of it) that describes a coherent movement of a robot
    within a time-graph from its origin to its source. Can be combined for
    multiple robots.

    This part only focuses on  a single robot, not on collisions etc.
    Use multiple instances of this model and add additional constraints that
    prevent collisions for multi robot motion planning.
    """

    def __init__(self, model: gp.Model, flow_graph: SingleRobotFlowGraph):
        self.model = model
        self.flow_graph = flow_graph
        self.vars = model.addVars(self.flow_graph.edges(), vtype=GRB.BINARY)
        self._add_flow_constraints()

    def _add_flow_constraints(self):
        for v in self.flow_graph.nodes():
            if self.flow_graph.is_entry(v):
                out_flow = gp.quicksum(
                    self.vars[e] for e in self.flow_graph.outgoing(v)
                )
                self.model.addConstr(out_flow == 1)
                assert not self.flow_graph.incoming(
                    v
                ), "Entry should not have incoming edges"
            elif self.flow_graph.is_exit(v):
                in_flow = gp.quicksum(self.vars[e] for e in self.flow_graph.incoming(v))
                self.model.addConstr(in_flow == 1)
                assert not self.flow_graph.outgoing(
                    v
                ), "Exit should not have outgoing edges"
            else:
                in_flow = gp.quicksum(self.vars[e] for e in self.flow_graph.incoming(v))
                out_flow = gp.quicksum(
                    self.vars[e] for e in self.flow_graph.outgoing(v)
                )
                self.model.addConstr(in_flow == out_flow)

    def in_sum(self, v: Vertex, t: int):
        if v == self.flow_graph.target:
            v, t = self.flow_graph.normalize_exit((v, t))
        return gp.quicksum(self.vars[e] for e in self.flow_graph.incoming((v, t)))

    def out_sum(self, v: Vertex, t: int):
        return gp.quicksum(self.vars[e] for e in self.flow_graph.outgoing((v, t)))

    def edge_sum(self, v: Vertex, w: Vertex, t: int):
        edges = (
            [((v, t), (w, t + 1)), ((w, t), (v, t + 1))]
            if v != w
            else [((v, t), (w, t + 1))]
        )
        edges = [(v, w) for v, w in edges if self.flow_graph.dgraph.has_edge(v, w)]
        return gp.quicksum(self.vars[e] for e in edges)

    def dist_sum(self):
        return gp.quicksum(
            self.vars[(v, w)] for v, w in self.flow_graph.edges() if v[0] != w[0]
        )

    def get_solution(self) -> typing.List[Vertex]:
        solution = {}
        for (v, w), x in self.vars.items():
            if x.X > 0.5:
                solution[v[1]] = v[0]
                if self.flow_graph.is_exit(w):
                    solution[w[1]] = w[0]
        solution_ = [None for _ in range(len(solution))]
        for i, x in solution.items():
            solution_[i] = x
        assert all(type(t) == tuple for t in solution_), "All entries filled"
        return solution_


class MultiRobotFlowModel:
    def __init__(self, sub_flows: typing.List[SingleRobotFlowGraph]):
        """

        :param graph: The environment to move within.
        :param targets: A list of tuples (s_i, t_i). Robot i must move from s_i to t_i.
        :param dist_map: A map that defines the distance to the target.
        :param time_horizon: The number of time steps to consider. E.g. 8.
        """
        self.model = gp.Model()
        self.sub_flows = sub_flows
        self.robot_models = [
            SingleRobotFlowModel(self.model, sub_flow) for sub_flow in self.sub_flows
        ]
        self._add_collision_constraints()
        self._set_dist_obj()

    def _set_dist_obj(self):
        self.model.setObjective(
            (gp.quicksum(rm.dist_sum() for rm in self.robot_models)), sense=GRB.MINIMIZE
        )

    def _add_collision_constraints(self):
        vertices = set()
        edges = set()
        for rm in self.robot_models:
            vertices.update(rm.flow_graph.nodes())
            for v, w in rm.flow_graph.edges():
                edges.add((min(v[0], w[0]), max(v[0], w[0]), v[1]))

        # prevent collisions on vertices
        for v, t in vertices:
            if t == 0:  # start vertices
                continue
            incoming = gp.quicksum(rm.in_sum(v, t) for rm in self.robot_models)
            self.model.addConstr(incoming <= 1)

        # prevent collisions on edges
        for v, w, t in edges:
            self.model.addConstr(
                gp.quicksum(rm.edge_sum(v, w, t) for rm in self.robot_models) <= 1
            )

    def optimize(self, timelimit=900) -> typing.List[typing.List[Vertex]]:
        self.model.Params.TimeLimit = timelimit
        self.model.optimize()
        return [rm.get_solution() for rm in self.robot_models]
