"""Contains the basic RRT algorithm

This is only the most basic RRT algorithm (no star) but contains many of the
important methods that are used for RRT planning

"""

from __future__ import division, print_function
from tree import *
from space import EmptySpace
import numpy as np

__all__ = ["RRT"]

class RRT(object):

    def __init__(self, space, initial_point, step_size=None):
        """Initialize the RRT

        Args:
            space (rrt.EmptySpace): The space to search over
                                    (must inherit from EmptySpace)
            initial_point (np.array): An (x,y) point to start from
            step_size (float): The maximum distance any step can go

        """
        self._space = space
        self._dims = space.dimensions
        self._dimensionality = len(self._dims)
        # If not given a step size, make it as big as possible
        if step_size is None:
            step_size = np.sqrt(sum(self._dims * self._dims))
        self._ssize = step_size
        self._tree = DirectedTree(initial_point)
        self._tol = min(self._dims) * 1e-6 # Distance to make a point "new"

    def steer(self, point_from, point_to):
        """Returns a point that gets moved to when steering towards a desired
            point

        Args:
            point_from (np.array): The (x, y) point to start at
            point_to (np.array): The (x, y) point to direct towards

        Returns:
            The new point to end at -- either the point moved to or a point
            in that direction of step_size set for the RRT
        """
        vdiff = point_to - point_from
        # Are these points within the max step size? Then we're cool
        vlen = np.linalg.norm(vdiff)
        if vlen < self._ssize:
            return point_to
        # Otherwise, step with max step size in the direction of point_to
        voffset = vdiff * self._ssize / vlen
        return point_from + voffset

    def step(self):
        """Runs one step of the RRT algorithm to create a single new node

        If a point is placed that cannot create a new path, just skip

        Returns:
            The new node added (or None if nothing is there)
        """
        # Pick a random point from the space
        q_rand = self._space.get_random_free_point()
        return self._step_to(q_rand)

    def _step_to(self, q_rand):
        """Helper for step function (used in inheritance)
        """
        node_nearest = self._tree.nearest(q_rand)
        q_nearest = node_nearest.point
        # Steers towards the new point
        q_path = self.steer(q_nearest, q_rand)
        # Check whether this is a legitimate path
        cost_path = self._space.get_path_cost(q_nearest, q_path)
        if np.isinf(cost_path):
            # Here there is a blocker... find the nearest point
            q_path = self._space.get_nearest_free_point(q_nearest, q_path)
            # Is this actually a new point? If not, skip
            if np.linalg.norm(q_nearest - q_path) < self._tol:
                return None
            cost_path = self._space.get_path_cost(q_nearest, q_path)
        # Hook this new point up to the tree
        node_path = self._tree.add_node(node_nearest, q_path, cost_path)
        # Check whether we have found a goal
        if self._space.check_point_in_goals(q_path):
            node_path.mark_goal()
        return node_path

    def get_shortest_path(self):
        """Returns the shortest path found so framework

        Returns:
            Two items:
                (1) The total cost of the path, and
                (2) A list of all points traversed on that path
            (Note this will be inf, None if no path has been found)
        """
        # Make sure we have found one
        goal_nodes = self._tree.goal_nodes
        if len(goal_nodes) == 0:
            return np.inf, None

        # Find the node with the shortest path
        costs = np.array([n.cost for n in goal_nodes])
        end_node = goal_nodes[np.argmin(costs)]
        # Return cost, path
        return end_node.cost, [p.point for p in self._tree.path_down(end_node)]

    def _expose_tree(self):
        return self._tree
    tree = property(_expose_tree)
    """Returns the tree of paths found so far"""

    def _expose_space(self):
        return self._space
    space = property(_expose_space)
    """Returns the space associated with the RRT"""
