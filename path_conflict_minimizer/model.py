import gc
import typing
from collections import defaultdict

import networkx as nx

import gurobipy as gp
from gurobipy import GRB

from .utils import Vertex

TimeVertex = typing.Tuple[Vertex, int]


class Variables:
    def __init__(self, model: gp.Model):
        self._vars = defaultdict(lambda: model.addVar(vtype=GRB.BINARY))
        self._vconf_vars = defaultdict(lambda: model.addVar(vtype=GRB.CONTINUOUS))
        self._econf_vars = defaultdict(lambda: model.addVar(vtype=GRB.CONTINUOUS))

    def __call__(self, r_i: int, e):
        key = (r_i, e)
        return self._vars[key]

    def get_solution(self):
        movements = [key for key, var in self._vars.items() if var.X > 0.5]
        edge_conflicts = [key for key, var in self._econf_vars.items() if var.X >= 0.5]
        vertex_conflicts = [
            key for key, var in self._vconf_vars.items() if var.X >= 0.5
        ]
        return {
            "movements": movements,
            "edge_conflicts": edge_conflicts,
            "vertex_conflicts": vertex_conflicts,
        }

    def v_conflict(self, v: TimeVertex):
        # return self._vconf_vars[v]
        return self._vconf_vars[v[0]]

    def e_conflict(self, v: Vertex, w: Vertex, t: int):
        if v < w:
            return self.e_conflict(w, v, t)
        # return self._econf_vars[(v, w, t)]
        return self._econf_vars[(v, w)]

    def conflict_vars(self):
        yield from self._vconf_vars.values()
        yield from self._econf_vars.values()


class PathGraph:
    def __init__(self, path: typing.List[Vertex], t: int):
        self.dgraph = nx.DiGraph()
        self.source = (path[0], 0)
        self.target = (path[-1], len(path) + t - 2)
        self.vertices = defaultdict(list)
        for i, p in enumerate(path):
            for t_ in range(i, i + t):
                if t_ < i + t - 1:
                    self.dgraph.add_edge((p, t_), (p, t_ + 1))
                if i < len(path) - 1:
                    p_next = path[i + 1]
                    self.dgraph.add_edge((p, t_), (p_next, t_ + 1))
        assert self.dgraph.out_edges(self.source)
        assert self.dgraph.in_edges(self.target)
        assert not self.dgraph.out_edges(self.target)
        assert all(self.dgraph.out_degree(v) <= 2 for v in self.dgraph.nodes)
        for v, t in self.dgraph.nodes:
            self.vertices[v].append((v, t))

    def incoming(self, v: TimeVertex):
        if v not in self.dgraph.nodes:
            return []
        return self.dgraph.in_edges(v)

    def time_vertices(self, v: Vertex):
        return self.vertices[v]

    def blocks_indirectly(self, v: TimeVertex):
        return v[0] == self.target[0] and self.target[1] < v[1]

    def conflicting_edges(self, u: Vertex, v: Vertex, start_time: int):
        if u == v:
            if self.dgraph.has_edge((v, start_time), (v, start_time + 1)):
                yield ((v, start_time), (v, start_time + 1))
            return
        if self.dgraph.has_edge((u, start_time), (v, start_time + 1)):
            yield ((u, start_time), (v, start_time + 1))
        if self.dgraph.has_edge((v, start_time), (u, start_time + 1)):
            yield ((v, start_time), (u, start_time + 1))
        if self.dgraph.has_edge((v, start_time), (v, start_time + 1)):
            yield ((v, start_time), (v, start_time + 1))

    def time_edges(self):
        for v, w in self.dgraph.out_edges():
            yield v[0], w[0], v[1]


qsum = gp.quicksum


class Model:
    def __init__(self, paths: typing.List[typing.List[Vertex]], t: int):
        self.model = gp.Model()
        self.vars = Variables(self.model)
        self.paths = paths
        self.t = t
        self.path_graphs = [PathGraph(path, t) for path in paths]
        self._ensure_flow()
        self._prevent_edge_collisions()
        self._prevent_vertex_collisions()
        self._set_objective()

    def _set_objective(self):
        self.model.setObjective(sum(self.vars.conflict_vars()), GRB.MINIMIZE)

    def _ensure_flow(self):
        for r_i, pgraph in enumerate(self.path_graphs):
            for v in pgraph.dgraph.nodes:
                if v == pgraph.source:
                    self.model.addConstr(
                        qsum(self.vars(r_i, e) for e in pgraph.dgraph.out_edges(v)) == 1
                    )
                elif v == pgraph.target:
                    self.model.addConstr(
                        qsum(self.vars(r_i, e) for e in pgraph.dgraph.in_edges(v)) == 1
                    )
                else:
                    self.model.addConstr(
                        qsum(self.vars(r_i, e) for e in pgraph.dgraph.out_edges(v))
                        == qsum(self.vars(r_i, e) for e in pgraph.dgraph.in_edges(v))
                    )

    def _prevent_vertex_collisions(self):
        vertices = set(sum((list(g.dgraph.nodes) for g in self.path_graphs), start=[]))
        for v in vertices:
            is_blocked = any(pg.blocks_indirectly(v) for pg in self.path_graphs)
            if is_blocked:
                self.model.addConstr(
                    qsum(
                        self.vars(r_i, e)
                        for r_i, g in enumerate(self.path_graphs)
                        for e in g.incoming(v)
                    )
                    == 0 + self.vars.v_conflict(v)
                )
            else:
                self.model.addConstr(
                    qsum(
                        self.vars(r_i, e)
                        for r_i, g in enumerate(self.path_graphs)
                        for e in g.incoming(v)
                    )
                    <= 1 + self.vars.v_conflict(v)
                )

    def _prevent_edge_collisions(self):
        time_edges = set(
            sum((list(g.time_edges()) for g in self.path_graphs), start=[])
        )
        for v, w, t in time_edges:
            self.model.addConstr(
                qsum(
                    self.vars(r_i, e)
                    for r_i, pg in enumerate(self.path_graphs)
                    for e in pg.conflicting_edges(v, w, t)
                )
                <= 1 + self.vars.e_conflict(v, w, t)
            )

    def optimize(self, timelimit=90):
        self.model.Params.TimeLimit = timelimit
        self.model.optimize()
        return self.vars.get_solution()
