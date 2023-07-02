"""
Split the problem into subproblems of limited time horizon.
Of course, you won't be able to move all agents from their source to the target
within a (very) limited time horizon (of maybe 8 steps). Thus, you need to find
intermediate targets. Implementation of Yu and LaValle uses fixed targets for
every robot. However, we can also use dynamic intermediate targets and just
maximize the sum of intermediate target ratings. Having then a set of feasible
intermediate targets, we can optimize for any objective function we like.

Challenges: Rating intermediate agent positions. Trivial idea could be the
remaining distance to the target.
"""
# flake8: noqa F401
from .model import Model, DistMap, iterative_solve
