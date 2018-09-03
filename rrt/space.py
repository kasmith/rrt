"""Defines a space that RRT traverses over

Two types: EmptySpace (which is a empty, rectangular table), and WallSpace
    (which is like EmptySpace but can have rectangular walls to stop motion)

"""

from __future__ import division, print_function
from .goals import *
import numpy as np
from geometry import find_intersection_point, lines_intersect
from .helpers import point_in_box

class EmptySpace(object):
    """The most basic space with no obstacles"""

    def __init__(self, dimensions, goals=[]):
        """Initialize an EmptySpace object

        Args:
            dimensions (np.array): The (x, y) dimensions of the space
            goals ([rrt.GoalBase]): A list of goals within the space

        """
        self._dims = dimensions
        self._goals = goals

    def _get_dimensions(self):
        return self._dims
    dimensions = property(_get_dimensions)
    """Returns the spatial extent of the space"""

    def _get_goals(self):
        return self._goals
    goals = property(_get_goals)
    """Expose the goals attached to the space"""

    def invert(self, point):
        """Inverts a point -- used for drawing since the convention is y->up

        Args:
            point (np.array): (x, y) point to invert

        Returns:
            Another point flipped in the y dimension
        """
        return np.array([point[0], self._dims[1]-point[1]])

    def collision_free(self, point_from, point_to):
        """Determines if there is a collision moving between two points

        Since the space is empty, the points just have to be in bounds

        Args:
            point_from (np.array): point in the space to move from
            point_to (np.array): point in the space to move to

        Returns:
            Boolean indicating whether there is a clear linear path between the
            two points
        """
        return (np.all(point_from > 0) and np.all(point_from < self._dims) and
                np.all(point_to > 0) and np.all(point_to < self._dims))

    def get_path_cost(self, point_from, point_to):
        """Determines the cost for traversing from point_from to point_to

        Since this is the basic space, this cost is just the distance

        Args:
            point_from (np.array): point in the space to move from
            point_to (np.array): point in the space to move to

        Returns:
            Inf if there is a collision, the distance between the two points
            otherwise
        """

        if not self.collision_free(point_from, point_to):
            return np.inf

        return np.linalg.norm(point_from - point_to)

    def get_nearest_free_point(self, point_from, point_to):
        """Returns the nearest legal point on a path

        Since this is clear board, this will always just be an in-bound point

        Returns:
            An np.array (x, y) point in bounds
        """
        # Go through all of the outer wall segments to check for intersection
        ilines = [
            [np.array([0, 0]), np.array([self._dims[0], 0])],
            [np.array([0, 0]), np.array([0, self._dims[1]])],
            [np.array([self._dims[0], 0]), self._dims],
            [np.array([0, self._dims[1]]), self._dims]
        ]

        for il in ilines:
            intersect = find_intersection_point(point_from, point_to,
                                                il[0], il[1])
            if intersect:
                return intersect
        return point_to # There is no blockage


    def get_random_free_point(self):
        """Returns a random point within the space

        Returns:
            A random (x, y) point within the space
        """
        return np.random.rand(2) * self._dims

    def check_point_in_goals(self, point):
        """Determines whether a new point is in one of the goals

        Returns:
            Boolean indicating whether the new point is in one of the goals
        """
        #return np.any(g.point_in(point) for g in self._goals)
        for g in self._goals:
            if g.point_in(point):
                return True
        return False

class WallSpace(EmptySpace):
    """A more complex space with rectangular walls that cannot be passed"""
    def __init__(self, dimensions, goals=[], walls=[]):
        """Initialize a WallSpace object

        Inherits from EmptySpace without change:
            dimensions
            goals
            invert(point)
            get_path_cost(point_from, point_to)
            check_point_in_goals(point)

        Args:
            dimensions (np.array): The (x, y) dimensions of the space
            goals ([rrt.GoalBase]): A list of goals within the space
            walls ([[ll, ur]]): A list of upper-left & lower-right wall pairs
        """
        self._dims = dimensions
        self._goals = goals
        self._walls = walls

    def _get_walls(self):
        return self._walls
    walls = property(_get_walls)
    """Expose the walls attached to the space"""

    def _point_in_walls(self, point):
        """Check if a point is inside any of the walls"""
        for w in self._walls:
            if point_in_box(w[0], w[1], point):
                return True
        return False

    def collision_free(self, point_from, point_to):
        """Determines if there is a collision moving between two points

        Points must be in bounds and not intersect any walls

        Args:
            point_from (np.array): point in the space to move from
            point_to (np.array): point in the space to move to

        Returns:
            Boolean indicating whether there is a clear linear path between the
            two points
        """

        # Check that the points are in bounds
        if not ((np.all(point_from > 0) and np.all(point_from < self._dims) and
                np.all(point_to > 0) and np.all(point_to < self._dims))):
            return False

        # Cache some stats for easy use
        min_x = min(point_from[0], point_to[0])
        max_x = max(point_from[0], point_to[0])
        min_y = min(point_from[1], point_to[1])
        max_y = max(point_from[1], point_to[1])

        for w in self._walls:
            # Only need to check if the points are close enough to the walls
            if not (min_x > w[1][0] or          # starts to the right of wall
                    min_y > w[1][1] or          # starts above the wall
                    max_x < w[0][0] or          # ends to the left of the wall
                    max_y < w[0][1]):           # ends below the wall
                # Quick check: are either of the points inside?
                if (point_in_box(w[0], w[1], point_from) or
                    point_in_box(w[0], w[1], point_to)):
                    return False
                # Otherwise, check that the line doesn't intersect borders
                ilines = [
                    [w[0], [w[0][0], w[1][1]]],
                    [w[0], [w[1][0], w[0][1]]],
                    [[w[0][0], w[1][1]], w[1]],
                    [[w[1][0], w[0][1]], w[1]]
                ]
                for il in ilines:
                    if lines_intersect(point_from, point_to,
                                       il[0], il[1]):
                        return False
        return True

    def get_nearest_free_point(self, point_from, point_to):
        """Returns the nearest legal point on a path

        First ensures point_to is inbounds, them looks for the intersecting wall

        This is slow and assumes a collision was found (does lots of checking
        otherwise, so often better to call collision_free first)

        Returns:
            An np.array (x, y) point in bounds
        """
        # Go through all of the outer wall segments to check for intersection
        ilines = [
            [np.array([0, 0]), np.array([self._dims[0], 0])],
            [np.array([0, 0]), np.array([0, self._dims[1]])],
            [np.array([self._dims[0], 0]), self._dims],
            [np.array([0, self._dims[1]]), self._dims]
        ]

        for il in ilines:
            intersect = find_intersection_point(point_from, point_to,
                                                il[0], il[1])
            if intersect:
                return intersect

        # Cache some stats for easy use
        min_x = min(point_from[0], point_to[0])
        max_x = max(point_from[0], point_to[0])
        min_y = min(point_from[1], point_to[1])
        max_y = max(point_from[1], point_to[1])

        # Go through each of the walls and find the intersections
        intersect_points = []
        for w in self._walls:
            # Only need to check if the points are close enough to the walls
            if not (min_x > w[1][0] or          # starts to the right of wall
                    min_y > w[1][1] or          # starts above the wall
                    max_x < w[0][0] or          # ends to the left of the wall
                    max_y < w[0][1]):           # ends below the wall
                # Find and store the intersection points
                ilines = [
                    [w[0], [w[0][0], w[1][1]]],
                    [w[0], [w[1][0], w[0][1]]],
                    [[w[0][0], w[1][1]], w[1]],
                    [[w[1][0], w[0][1]], w[1]]
                ]
                for il in ilines:
                    p = find_intersection_point(point_from, point_to,
                                                il[0], il[1])
                    if p is not None:
                        intersect_points.append(p)
        # If there are any intersection points, find the nearest to
        # point_from
        if len(intersect_points) > 0:
            # Easy case... only one intersection
            if len(intersect_points) == 1:
                return intersect_points[0]
            # Otherwise, find the point closest to point_from
            best_point = intersect_points[0]
            min_dist = np.linalg.norm(point_from - best_point)
            for ip in intersect_points[1:]:
                this_dist = np.linalg.norm(point_from - ip)
                if this_dist < min_dist:
                    best_point = ip
                    min_dist = this_dist
            return best_point

        # If we are here, there is no intersection
        return point_to

    def get_random_free_point(self):
        """Returns a random in-bound point

        Implemented relatively inefficiently via rejection sampling

        Returns:
            An (x, y) pair of a random good point
        """
        pt = np.random.rand(2) * self._dims
        if self._point_in_walls(pt):
            return self.get_random_free_point()
        else:
            return pt
