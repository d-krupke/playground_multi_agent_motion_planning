import typing
import gurobipy as gp
from gurobipy import GRB

from .common import Vertex
from .conflict_counter import ConflictCounter
from .flow_graph import SingleRobotFlowGraph
from .single_robot_model import SingleRobotFlowModel


class MultiRobotFlowModel:
    def __init__(
            self,
            sub_flows: typing.List[SingleRobotFlowGraph],
    ):
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
        if len(sub_flows) > 1:
            self._add_collision_constraints()
        self.conflict_counter = ConflictCounter()

    def add_path(self, robot, path):
        path = [path[i] if i < len(path) else path[-1] for i in range(self.sub_flows[0].n)]
        self.conflict_counter.add_path(robot, path)

    def set_objective(self, conflict_weight):
        edges = set()
        for rm in self.robot_models:
            for v, w in rm.flow_graph.edges():
                edges.add((v, w))
        objective = 0
        for e in edges:
            c = conflict_weight * len(self.conflict_counter.query(e[0][0], e[1][0], e[0][1]))
            if e[0][0] != e[1][0]:
                c += 1  # distance
            for rm in self.robot_models:
                if e in rm.vars:
                    objective += c * rm.vars[e]
        self.model.setObjective(objective, sense=GRB.MINIMIZE)

    def add_conflict_limit(self, max_conflicts):
        """
        Allows setting a limit on the conflicts.
        :param max_conflicts:
        :return:
        """
        edges = set()
        for rm in self.robot_models:
            for v, w in rm.flow_graph.edges():
                edges.add((v, w))
        conflict_sum = 0
        for e in edges:
            c = len(self.conflict_counter.query(e[0][0], e[1][0], e[0][1]))
            for rm in self.robot_models:
                if e in rm.vars and c > 0:
                    conflict_sum += c * rm.vars[e]
        self.model.addConstr(conflict_sum <= max_conflicts)

    def distance_limit(self, max_distance):
        """
        Allows settings a limit on the distance.
        :param max_distance:
        :return:
        """
        edges = set()
        for rm in self.robot_models:
            for v, w in rm.flow_graph.edges():
                edges.add((v, w))
        distance = 0
        for e in edges:
            for rm in self.robot_models:
                if e in rm.vars and e[0][0] != e[1][0]:
                    distance += rm.vars[e]
        self.model.addConstr(distance <= max_distance)

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
