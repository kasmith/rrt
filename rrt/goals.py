"""Ways of defining goal regions for general RRT

Uses a framework GoalBase object and extends it to have similar usage
regardless of the goal shape
"""

from __future__ import division, print_function
import numpy as np

class GoalBase(object):
    """Framework for other goal objects -- DO NOT IMPLEMENT DIRECTLY"""

    def __init__(self):
        self._goal_val = True
        raise NotImplementedError("Should not make a GoalBase directly")

    def point_in(self, point):
        """Determines whether a point is inside the goal

        Args:
            point (np.array): the point to check

        Return:
            Boolean indicating whether the point is in the goal
        """
        raise NotImplementedError("point_in has not been extended!")

    def sphere_touch(self, point, radius):
        """Determines whether a sphere overlaps with the goal

        Args:
            point (np.array): the center of the sphere
            radius (float): the radius of the sphere

        Return:
            Boolean indicating whether the sphere overlaps the goal
        """
        raise NotImplementedError("sphere_touch has not been extended!")

    def box_touch(self, min_verts, max_verts):
        """Determines whether a sphere overlaps with the goal

        Args:
            min_verts (np.array): the minimum vertices of the box
            max_verts (np.array): the maximum vertices of the box

        Return:
            Boolean indicating whether the box overlaps the goal
        """
        raise NotImplementedError("box_touch has not been extended!")

    def _get_goal_val(self):
        return self._goal_val
    goal_value = property(_get_goal_val)
    """Returns the goal value that is attached to nodes that hit this spot"""

    def _get_extent(self):
        return self._extent
    extent = property(_get_extent)
    """Returns the spatial extent of the goal (differs by goal)"""

class BoxGoal2D(GoalBase):
    """A goal defined by a box in two dimensional space"""

    def __init__(self, lower_left, upper_right, goal_val=True):
        self._goal_val = goal_val
        self._ll = lower_left
        self._ur = upper_right
        self._extent = {'lower_left': self._ll,
                        'upper_right': self._ur}

    def point_in(self, point):
        return (np.all(point >= self._ll) and
                np.all(point <= self._ur))

    def sphere_touch(self, point, radius):
        return NotImplementedError("To be implemented for later more complex objects")

    def box_touch(self, min_verts, max_verts):
        return NotImplementedError("To be implemented for later more complex objects")
