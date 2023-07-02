import matplotlib.pyplot as plt
import networkx as nx
import typing
from matplotlib import cm


class Robot:
    def __init__(self, ax, position, color):
        self._path = [position]
        self._path_line = ax.plot(
            [p[0] for p in self._path],
            [p[1] for p in self._path],
            zorder=2,
            color=color,
            alpha=0.7,
            lw=4,
        )
        self._outer_circle = plt.Circle(position, 0.32, color="black", zorder=4)
        self._inner_circle = plt.Circle(position, 0.3, color=color, zorder=5)
        ax.add_patch(self._outer_circle)
        ax.add_patch(self._inner_circle)

    def move(self, position):
        self._path.append(position)
        self._path_line.set_data([p[0] for p in self._path], [p[1] for p in self._path])
        self._outer_circle.update(position)
        self._inner_circle.update(position)


def plot_graph(ax: plt.Axes, graph: nx.Graph):
    ax.plot(
        [p[0] for p in graph.nodes],
        [p[1] for p in graph.nodes],
        "o",
        c="grey",
        zorder=1,
    )
    for e in graph.edges:
        ax.plot([e[0][0], e[1][0]], [e[0][1], e[1][1]], c="grey", zorder=1, lw=1)


class AnimationFunction:
    def __init__(self, graph: nx.Graph, solution: typing.Dict):
        self.solution = solution
        self.fig = plt.figure()
        self.ax = plt.gca()
        plot_graph(self.ax, graph)
        colors = cm.get_cmap("tab20c", len(solution))
        self.robots = [
            Robot(self.ax, path[0], colors(i)) for i, path in solution.items()
        ]

    def __len__(self):
        return len(self.solution[0])

    def __call__(self, *args, **kwargs):
        i = args[0]
        for r, path in self.solution.items():
            self.robots[r].move(path[i])
