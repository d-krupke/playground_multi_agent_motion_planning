import itertools
import typing
from collections import defaultdict

import networkx as nx
import gurobipy as gp
from gurobipy import GRB

Vertex = typing.Tuple[int, int]


def optimize(
    graph: nx.Graph, targets: typing.Dict[Vertex, Vertex]
) -> typing.Dict[typing.Tuple[int, int], typing.List]:
    pass


class _NaiveMipVars:
    """
    Just a variable container for the MIP-model.
    """

    def __init__(self, model: gp.Model, graph, n_robots, t):
        self.model = model
        self.t = t
        self.graph = graph
        self.n_robots = n_robots
        self.directed_edges = (
            [(v, w) for v, w in graph.edges]
            + [(w, v) for v, w in graph.edges]
            + [(v, v) for v in graph.nodes]
        )
        self._vars = model.addVars(
            itertools.product(self.directed_edges, range(n_robots), range(t)),
            vtype=GRB.BINARY,
        )

    def get_var(self, edge: typing.Tuple[Vertex, Vertex], robot: int, layer: int):
        return self._vars[(edge, robot, layer)]

    def get_vars(
        self,
        edge: typing.Optional[typing.Tuple[Vertex, Vertex]] = None,
        robot: typing.Optional[int] = None,
        layer: typing.Optional[int] = None,
    ):
        edges = [edge] if edge is not None else self.directed_edges
        robots = [robot] if robot is not None else list(range(self.n_robots))
        layers = [layer] if layer is not None else list(range(self.t))
        for e in edges:
            for r in robots:
                for t in layers:
                    yield self.get_var(e, r, t)

    def get_incoming(
        self,
        vertex,
        robot: typing.Optional[int] = None,
        layer: typing.Optional[int] = None,
    ):
        robots = [robot] if robot is not None else list(range(self.n_robots))
        layers = [layer] if layer is not None else list(range(self.t))
        for r in robots:
            for t in layers:
                yield self.get_var((vertex, vertex), r, t)
                for n in self.graph.neighbors(vertex):
                    yield self.get_var((n, vertex), r, t)

    def get_outgoing(
        self,
        vertex,
        robot: typing.Optional[int] = None,
        layer: typing.Optional[int] = None,
    ):
        robots = [robot] if robot is not None else list(range(self.n_robots))
        layers = [layer] if layer is not None else list(range(self.t))
        for r in robots:
            for t in layers:
                yield self.get_var((vertex, vertex), r, t)
                for n in self.graph.neighbors(vertex):
                    yield self.get_var((vertex, n), r, t)

    def get_solution(self):
        return [key for key, var in self._vars.items() if var.X > 0.5]


class NaiveMip:
    def __init__(
        self,
        graph: nx.Graph,
        targets: typing.List[typing.Tuple[Vertex, Vertex]],
        t: int,
    ):
        self.t = t
        self.graph = graph
        self.targets = targets
        self.model = gp.Model()
        self.vars = _NaiveMipVars(self.model, graph, n_robots=len(targets), t=t)
        # Minimize the sum of moves.
        self._minimize_distance()
        # prohibit robots from teleporting: if they leave a vertex, they have
        # to enter it before.
        self._enforce_flow()
        #  prohibit collisions
        self._prevent_collisions()
        # enforce that each robot starts at its origin
        self._enforce_sources()
        # enforce that each robot reaches its target at the end
        self._enforce_targets()
        # enforce that each robot has decides for one action at each timestep
        # TODO: Not sure if necessary. The flow constraints should suffice.
        self._enforce_on_edge_per_robot()

    def _minimize_distance(self):
        # reduce sum of used edges that go between different vertices
        self.model.setObjective(
            gp.quicksum(
                sum(self.vars.get_vars((v, w))) + sum(self.vars.get_vars((w, v)))
                for v, w in self.graph.edges()
            ),
            GRB.MINIMIZE,
        )

    def _enforce_flow(self):
        # when we visit a time-vertex, we have to leave it again.
        # exceptions are sources and targets
        for layer in range(self.t - 1):
            for vertex in self.graph.nodes:
                for robot in range(len(self.targets)):
                    self.model.addConstr(
                        sum(self.vars.get_outgoing(vertex, layer=layer, robot=robot))
                        == sum(
                            self.vars.get_incoming(vertex, layer=layer + 1, robot=robot)
                        )
                    )

    def _prevent_collisions(self):
        # only one robot can enter a vertex
        for layer in range(self.t):
            for vertex in self.graph.nodes:
                sum_incoming = 0
                for nbr in self.graph.neighbors(vertex):
                    sum_incoming += sum(self.vars.get_vars((nbr, vertex), layer=layer))
                sum_incoming += sum(self.vars.get_vars((vertex, vertex), layer=layer))
                self.model.addConstr(sum_incoming <= 1)
            for v, w in self.graph.edges:
                for layer in range(self.t):
                    self.model.addConstr(
                        sum(self.vars.get_vars(edge=(v, w), layer=layer))
                        + sum(self.vars.get_vars(edge=(w, v), layer=layer))
                        <= 1
                    )

    def _enforce_sources(self):
        # The robots leave their sources
        for i, (source, target) in enumerate(self.targets):
            self.model.addConstr(
                gp.quicksum(self.vars.get_outgoing(source, robot=i, layer=0)) == 1
            )

    def _enforce_targets(self):
        # the robots enter their targets
        for i, (source, target) in enumerate(self.targets):
            self.model.addConstr(
                gp.quicksum(self.vars.get_incoming(target, robot=i, layer=self.t - 1))
                == 1
            )

    def optimize(self, timelimit=90):
        self.model.Params.TimeLimit = timelimit
        self.model.optimize()

        solution = self.vars.get_solution()
        solution.sort(key=lambda k: k[2])
        paths = defaultdict(list)
        for entry in solution:
            if paths[entry[1]]:
                paths[entry[1]].pop()
            paths[entry[1]].append(entry[0][0])
            paths[entry[1]].append(entry[0][1])
        return paths

    def _enforce_on_edge_per_robot(self):
        # each robot has to use one edge in each time layer
        for robot in range(len(self.targets)):
            for layer in range(self.t):
                self.model.addConstr(
                    sum(self.vars.get_vars(robot=robot, layer=layer)) == 1
                )
