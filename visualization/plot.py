import typing
import matplotlib.pyplot as plt
import networkx as nx
from matplotlib import cm


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


def plot_path(ax: plt.Axes, path, color):
    ax.plot(
        [p[0] for p in path],
        [p[1] for p in path],
        zorder=2,
        color=color,
        alpha=0.7,
        lw=4,
    )


def plot_robot(ax: plt.Axes, path, i, color):
    p = path[i]
    ax.add_patch(plt.Circle(p, 0.32, color="black", zorder=4))
    ax.add_patch(plt.Circle(p, 0.3, color=color, zorder=5))
    if i > 0:
        pp = path[i - 1]
        for t in [0.6, 0.7, 0.8, 0.9]:
            x = t * p[0] + (1 - t) * pp[0]
            y = t * p[1] + (1 - t) * pp[1]
            ax.add_patch(
                plt.Circle(
                    (x, y), 0.3, color=color, zorder=3, alpha=t**3, linewidth=0
                )
            )


def show_solution(graph: nx.Graph, solution: typing.Dict, **fig_params):
    length = len(solution[0])
    colors = cm.get_cmap("tab20c", len(solution))
    for i in range(length):
        fig = plt.figure(**fig_params)
        ax = fig.gca()
        ax.set_aspect("equal", adjustable="box", anchor="C")
        plot_graph(ax, graph)
        for r, path in solution.items():
            plot_robot(ax, path, i, colors(r))
            plot_path(ax, path[: i + 1], colors(r))
        plt.show()
