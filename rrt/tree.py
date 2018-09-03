"""Contains object definitions to help build and modify trees for RRT*

Objects here are mostly for private use but can be exported

"""

from __future__ import division, print_function
import numpy as np
import warnings

__all__ = ['Node', 'DirectedTree']


class Node(object):
    """Represents a node in a DirectedTree"""
    def __init__(self, pt, parent, add_cost=1):
        """Initialize the nodes

        Args:
            pt (list/np.array): a list of floats for the coordinate of the node
            parent (Node): the parent node
            add_cost (float): additional cost for traversing this node
        """
        self._pt = np.array(pt)
        self._parent = parent
        self._children = []
        self._newcost = add_cost
        if self._parent is None:
            self._cost = 0
        else:
            self._cost = parent.cost + add_cost
        self._goal = False  # Marks whether this node is on a goal

    def _get_children(self):
        return self._children
    children = property(_get_children)
    """Returns a list of child nodes"""

    def _get_parent(self):
        return self._parent
    parent = property(_get_parent)
    """Returns the parent node"""

    def _get_point(self):
        return self._pt
    point = property(_get_point)
    """Returns the location of this Node"""

    def _get_cost(self):
        return self._cost
    cost = property(_get_cost)
    """Returns the total cost of this node (with all children included)"""

    def _is_goal(self):
        return self._goal
    is_goal = property(_is_goal)
    """Exposes whether this is a goal node"""

    def add_child(self, child_node):
        """Appends a child node

        Args:
            child_node (Node): the child to add
        """
        self._children.append(child_node)

    def distance(self, newpt):
        """Returns the distance from this nodes point to another

        Args:
            newpt (np.array): a vector with the same dimensions as pt
        """
        return np.linalg.norm(self._pt - newpt)

    def get_terminals(self):
        """Returns a list of terminal nodes that descend from this node"""
        r = []
        for c in self._children:
            r = r + c.get_terminals()
        return r
    terminals = property(get_terminals)
    """Returns a list of terminal nodes that descend from this node"""

    def recost(self):
        """Recalculates travel costs of this and child nodes"""
        self._cost = self._parent._cost + self._newcost
        for c in self._children:
            c.recost()

    def reparent(self, newparent, newcost=1):
        """Makes a new parent connection and recalculates costs

        Args:
            newparent (Node): the new parent node
            newcost (float): the new cost of traversal
        """
        self._parent._children.remove(self)
        self._parent = newparent
        newparent._children.append(self)
        self._newcost = newcost
        self.recost()

    def mark_goal(self, goal=True):
        """Marks whether this Node touches one of the goal regions

        Args:
            goal: A way of marking particular goals (defaults to True)
        """
        self._goal = goal


class DirectedTree(object):
    """A directed tree that can be reshaped (for RRT*)"""

    def __init__(self, init_pt):
        """Initializes the DirectedTree

        Args:
            init_pt (list/np.array): the starting location of the tree
        """
        headnode = Node(init_pt, None)
        self._head = headnode
        self._nodes = [headnode]

    def _get_head(self):
        return self._head
    head = property(_get_head)
    """Returns the head node of the tree"""

    def _get_nodes(self):
        return self._nodes
    nodes = property(_get_nodes)
    """Returns a list of all nodes belonging to this tree"""

    def add_node(self, parent, childpt, add_cost=1):
        """Adds a node to the tree

        Args:
            parent (Node): the parent Node to attach it to
            childpt (list/np.array): the location of the new node
            add_cost (float): the cost of traversing from parent to child

        Returns:
            The Node object added to the tree
        """
        assert parent in self._nodes, "parent not found in this tree"
        child = Node(childpt, parent, add_cost)
        parent.add_child(child)
        self._nodes.append(child)
        return child

    def get_node(self, position):
        """Locates a node by its position

        Args:
            position (list/np.array): the position to find the node at

        Returns:
            The Node at that position (or a warning and None if not found)
        """
        for n in self._nodes:
            if np.all(n.pt == np.array(position)):
                return n
        warnings.warn('Node with that position not found: ' + str(position),
                      RuntimeWarning)
        return None

    def _get_free_nodes(self):
        return [n for n in self._nodes if n.is_goal is False]
    free_nodes = property(_get_free_nodes)
    """Returns only the nodes that are not touching a goal region"""

    def _get_goal_nodes(self):
        return [n for n in self._nodes if n.is_goal is True]
    goal_nodes = property(_get_goal_nodes)
    """Returns only the nodes that are touching a goal region"""

    def nearest(self, newpt):
        """Returns the nearest Node to a given point

        Args:
            newpt (np.array): the point to find the closest node to

        Returns:
            A Node from the tree closest to that point
        """
        frees = self.free_nodes
        dists = np.array(list(map(lambda n: n.distance(newpt), frees)))
        return frees[np.argmin(dists)]

    def near(self, newpt, radius):
        """Returns all nodes within a given distance of a point

        Args:
            newpt (np.array): the point to find the closest node to
            radius (float): the maximum distance away to include nodes

        Returns:
            A list of Nodes within radius of newpt
        """
        frees = self.free_nodes
        dists = list(map(lambda n: n.distance(newpt) <= radius, frees))
        rets = []
        for i in range(len(dists)):
            if dists[i]:
                rets.append(frees[i])
        return rets

    def path_down(self, end_node):
        """Finds the path from the head down to a given node

        Args:
            end_node (Node): a node in the tree that is the desired endpoint

        Returns:
            A list of Nodes tracing a path through the tree
        """
        assert end_node in self._nodes, "end_node not found in this tree"
        revpath = [end_node]
        curnode = end_node
        while curnode.parent is not None:
            revpath.append(curnode.parent)
            curnode = curnode.parent
        revpath.reverse()
        return revpath

    def _get_terminals(self):
        return self._head.get_terminals()
    terminals = property(_get_terminals)
    """Returns all of the terminal nodes of the tree"""

    def _get_n_vertices(self): return len(self._nodes)
    n_vertices = property(_get_n_vertices)
    """The number of Nodes in the tree"""

    def _get_all_points(self):
        return [np.array(n._pt) for n in self._nodes]
    all_points = property(_get_all_points)
    """Returns all positions of nodes in the tree"""
