import typing
import networkx as nx
import gurobipy as gp
from gurobipy import GRB

Vertex = typing.Tuple[int, int]


class DistMap:
    def __init__(self, graph: nx.Graph, targets):
        self._dists = dict(nx.all_pairs_shortest_path_length(graph))
        self._targets = targets

    def get_cost(self, robot: int, position: Vertex) -> float:
        return self.dist(self._targets[robot][1], position) ** 2

    def dist(self, a: Vertex, b: Vertex) -> int:
        """
        Used to estimate if the vertex is reachable with the time horizon.
        :param a: Source
        :param b: Target
        :return: The distance in time steps between a and b
        """
        return self._dists[a][b]


class SubTimeGraph:
    def __init__(self, graph: nx.Graph, source: Vertex, time_horizon: int):
        dgraph = nx.DiGraph()
        positions = {source}
        for t in range(time_horizon):
            positions_ = set()
            for v in positions:
                dgraph.add_edge((v, t), (v, t + 1))
                positions_.add(v)
                for n in graph.neighbors(v):
                    dgraph.add_edge((v, t), (n, t + 1))
                    positions_.add(n)
            positions = positions_
        self.source_vertex = (source, 0)
        self.last_layer = [(p, time_horizon) for p in positions]
        assert all(v[1] <= time_horizon for v in dgraph.nodes)
        assert all(v[1] + 1 == w[1] for v, w in dgraph.edges)
        self.dgraph = dgraph
        self.time_horizon = time_horizon

    def is_entry(self, v):
        return v == self.source_vertex

    def is_exit(self, v):
        return v[1] == self.time_horizon

    def edges(self):
        return self.dgraph.edges()

    def nodes(self):
        return self.dgraph.nodes

    def incoming(self, v):
        return self.dgraph.in_edges(v)

    def outgoing(self, v):
        return self.dgraph.out_edges(v)


class SubTimeGraphModel:
    def __init__(
        self, model: gp.Model, graph: nx.Graph, source: Vertex, time_horizon: int
    ):
        self.model = model
        self.stgraph = SubTimeGraph(graph, source, time_horizon)
        self.vars = model.addVars(self.stgraph.edges(), vtype=GRB.BINARY)
        self.time_horizon = time_horizon
        self._add_flow_constraints()

    def enforce_target(self, v):
        self.model.addConstr(self.in_sum(v, self.time_horizon) == 1)

    def _add_flow_constraints(self):
        for v in self.stgraph.nodes():
            if self.stgraph.is_entry(v):
                out_flow = gp.quicksum(self.vars[e] for e in self.stgraph.outgoing(v))
                self.model.addConstr(out_flow == 1)
                assert not self.stgraph.incoming(v)
            elif self.stgraph.is_exit(v):
                in_flow = gp.quicksum(self.vars[e] for e in self.stgraph.incoming(v))
                self.model.addConstr(in_flow <= 1)
            else:
                in_flow = gp.quicksum(self.vars[e] for e in self.stgraph.incoming(v))
                out_flow = gp.quicksum(self.vars[e] for e in self.stgraph.outgoing(v))
                self.model.addConstr(in_flow == out_flow)

    def in_sum(self, v: Vertex, t: int):
        return gp.quicksum(self.vars[e] for e in self.stgraph.incoming((v, t)))

    def out_sum(self, v: Vertex, t: int):
        return gp.quicksum(self.vars[e] for e in self.stgraph.outgoing((v, t)))

    def edge_sum(self, v: Vertex, w: Vertex, t: int):
        edges = (
            [((v, t), (w, t + 1)), ((w, t), (v, t + 1))]
            if v != w
            else [((v, t), (w, t + 1))]
        )
        edges = [(v, w) for v, w in edges if self.stgraph.dgraph.has_edge(v, w)]
        return gp.quicksum(self.vars[e] for e in edges)

    def dist_sum(self):
        return gp.quicksum(
            self.vars[(v, w)] for v, w in self.stgraph.edges() if v[0] != w[0]
        )

    def cost_sum(self, i, dist_map: DistMap):
        return gp.quicksum(
            dist_map.get_cost(i, v) * self.in_sum(v, t)
            for v, t in self.stgraph.last_layer
        )

    def get_solution(self) -> typing.List[Vertex]:
        solution = [None for _ in range(self.time_horizon + 1)]
        for (v, w), x in self.vars.items():
            if x.X > 0.5:
                solution[v[1]] = v[0]
                if self.stgraph.is_exit(w):
                    solution[w[1]] = w[0]
        return solution


class Model:
    def __init__(
        self,
        graph: nx.Graph,
        targets: typing.List[typing.Tuple[Vertex, Vertex]],
        dist_map: DistMap,
        time_horizon: int,
    ):
        """

        :param graph: The environment to move within.
        :param targets: A list of tuples (s_i, t_i). Robot i must move from
                    s_i to t_i.
        :param dist_map: A map that defines the distance to the target.
        :param time_horizon: The number of time steps to consider. E.g. 8.
        """
        self.model = gp.Model()
        self.robot_models = [
            SubTimeGraphModel(self.model, graph, source, time_horizon)
            for (source, target) in targets
        ]
        self._add_collision_constraints()
        self.dist_map = dist_map
        self._set_cost_obj()

    def _set_dist_obj(self):
        self.model.setObjective(
            (gp.quicksum(rm.dist_sum() for rm in self.robot_models)), sense=GRB.MINIMIZE
        )

    def _set_cost_obj(self):
        self.model.setObjective(
            (
                gp.quicksum(
                    rm.cost_sum(i, self.dist_map)
                    for i, rm in enumerate(self.robot_models)
                )
            ),
            sense=GRB.MINIMIZE,
        )

    def _add_collision_constraints(self):
        vertices = set()
        edges = set()
        for rm in self.robot_models:
            vertices.update(rm.stgraph.nodes())
            for v, w in rm.stgraph.edges():
                edges.add((min(v[0], w[0]), max(v[0], w[0]), v[1]))

        for v in vertices:
            if v[1] == 0:  # start vertices
                continue
            self.model.addConstr(
                gp.quicksum(rm.in_sum(v[0], v[1]) for rm in self.robot_models) <= 1
            )

        for v, w, t in edges:
            self.model.addConstr(
                gp.quicksum(rm.edge_sum(v, w, t) for rm in self.robot_models) <= 1
            )

    def optimize(self, timelimit=900) -> typing.List[typing.List[Vertex]]:
        self.model.Params.TimeLimit = timelimit
        self.model.optimize()
        return [rm.get_solution() for rm in self.robot_models]

    def fix_targets(self):
        for rm in self.robot_models:
            rm.enforce_target(rm.get_solution()[-1])
        self._set_dist_obj()


def iterative_solve(graph: nx.Graph, targets, step_size=8, time_limit=30):
    print("Computing distance map")
    dist_map = DistMap(graph, targets)
    paths = [[target[0]] for target in targets]
    while any(paths[i][-1] != targets[i][1] for i in range(len(targets))):
        print("Building model")
        targets_ = [(paths[i][-1], t[1]) for i, t in enumerate(targets)]
        model = Model(graph, targets_, dist_map, step_size)
        print("Finding intermediate goals....")
        model.optimize(time_limit)
        model.fix_targets()
        print("Reach targets")
        solution = model.optimize(time_limit)
        for i, path in enumerate(solution):
            paths[i] += path[1:]
    return paths
