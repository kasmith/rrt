"""Contains the RRT star algorithm

This method snips and reconnects the tree to find the shortest path through
the nodes
"""

from __future__ import division, print_function
from tree import *
from rrt_base import RRT
import numpy as np

__all__ = ['RRTstar']

class RRTstar(RRT):
    """RRT* algorithm

    Like RRT but snips and reconnects the tree to minimize the path

    Methods inhereted unchanged from RRT:
        steer(point_from, point_to)
        get_shortest_path()
        tree
        space
    """

    def __init__(self, space, initial_point, step_size=None, close_size=None):
        """Initialize the RRTstar algorithm

        Args:
            space (rrt.EmptySpace): The space to search over
                                    (must inherit from EmptySpace)
            initial_point (np.array): An (x,y) point to start from
            step_size (float): The maximum distance any step can go
            close_size(float): The distance to search for nearby points;
                               defaults to 3x step size

        """
        RRT.__init__(self, space, initial_point, step_size)
        if close_size is None:
            close_size = self._ssize * 3.
        self._close = close_size

    def step(self):
        """Runs one step of the RRT algorithm to create a single new node

        If a point is placed that cannot create a new path, just skip

        Returns:
            The new node added (or None if nothing is there)
        """
        # Pick a random point from the space
        q_rand = self._space.get_random_free_point()
        # Find the neighbors in the tree
        Q_near = self._tree.near(q_rand, self._close)
        # Check -- if everything too far, just use the nearest
        if len(Q_near) == 0:
            # Replicate the RRT basic step
            return RRT._step_to(self, q_rand)
        else:
            # Find the minimum cost in the neighborhood
            cost_min = np.inf
            new_cost_min = np.inf
            node_min = None
            steer_min = None
            for node_near in Q_near:
                # Figure out the point to move to
                q_near = node_near.point
                q_path = self.steer(q_near, q_rand)
                cost_path = self._space.get_path_cost(q_near, q_path)
                if np.isinf(cost_path):
                    q_path = self._space.get_nearest_free_point(q_near, q_path)
                    # If it's the same point, treat as inf cost
                    if np.linalg.norm(q_near - q_path) < self._tol:
                        cost_path = np.inf
                    else:
                        cost_path = self._space.get_path_cost(q_near, q_path)
                tot_cost = node_near.cost + cost_path
                # Check whether this is a new minimum
                if tot_cost < cost_min:
                    cost_min = tot_cost
                    new_cost_min = cost_path
                    node_min = node_near
                    steer_min = q_path
            # Degenerate case: can't move to a new node
            if node_min is None:
                return None
            # If it's good, we add this to the tree
            new_node = self._tree.add_node(node_min, steer_min, new_cost_min)
            q_path = new_node.point
            if self._space.check_point_in_goals(q_path):
                new_node.mark_goal()
            # Then we rewire the tree
            # Look within either the step size or closeness (the min of those)
            for node_near in self._tree.near(q_path,
                                             min(self._ssize, self._close)):
                q_near = node_near.point
                cost_new = self._space.get_path_cost(q_path, q_near)
                # If there's a path
                if not np.isinf(cost_new):
                    # Figure out the total cost
                    total_cost_new = cost_new + new_node.cost
                    # If it's less than the existing cost, reparent
                    if total_cost_new < node_near.cost:
                        node_near.reparent(new_node, cost_new)
            return new_node
