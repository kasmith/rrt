from rrt import *
from rrt.viz import *
import numpy as np
import pygame as pg

"""Simple setup (screen size, number of steps)"""
DIMS = np.array([500, 500])
STEPS = 600

"""Define a couple spaces to test on:
1) An empty space with a single goal
2) A space with a wall and two goals
"""
simple_space = EmptySpace(DIMS,
                    [BoxGoal2D(np.array([375, 375]),
                              np.array([425, 425]))])

wall_space = WallSpace(DIMS,
                       [BoxGoal2D(np.array([375, 375]),
                                 np.array([425, 425]))],
                       [
                           [[200, 200], [300, 300]]
                       ])

"""Test the basic RRT algorithm on a space"""
def test_basic(space):
    pg.init()
    d = pg.display.set_mode(space.dimensions)
    rrt = RRT(space, np.array([50, 50]), 50)
    draw_space(d, space)
    pg.draw.circle(d, pg.Color('blue'),
                   intify(space.invert(rrt.tree.head.point)),
                   5)
    pg.event.pump()
    pg.display.flip()
    clk = pg.time.Clock()
    for _ in xrange(STEPS):
        new_node = rrt.step()
        parent_node = new_node.parent
        pg.draw.line(d, pg.Color('lightgrey'),
                     intify(space.invert(parent_node.point)),
                     intify(space.invert(new_node.point)))
        pg.draw.circle(d, pg.Color('darkgrey'),
                       intify(space.invert(new_node.point)), 3)

        pg.display.set_caption("FPS: " + str(int(clk.get_fps())))
        pg.event.pump()
        pg.display.flip()
        clk.tick(60)
    cost, path = rrt.get_shortest_path()
    print cost
    pt_path = [intify(space.invert(p)) for p in path]
    pg.draw.lines(d, pg.Color('blue'), False, pt_path, 2)
    pg.event.pump()
    pg.display.flip()
    running = True
    while running:
        for e in pg.event.get():
            if e.type == pg.constants.QUIT:
                running = False

"""Test RRTstar on a space"""
def test_star(space):
    pg.init()
    d = pg.display.set_mode(space.dimensions)
    rrt = RRTstar(space, np.array([50, 50]), 200, 50)
    draw_space(d, space)
    draw_tree(d, rrt.tree, space)
    pg.event.pump()
    pg.display.flip()
    clk = pg.time.Clock()
    for _ in xrange(STEPS):
        rrt.step()
        draw_rrt(d, rrt)
        pg.display.set_caption("FPS: " + str(int(clk.get_fps())))
        pg.event.pump()
        pg.display.flip()
        clk.tick(60)
    cost, path = rrt.get_shortest_path()
    print cost
    pt_path = [intify(space.invert(p)) for p in path]
    pg.draw.lines(d, pg.Color('blue'), False, pt_path, 2)
    pg.event.pump()
    pg.display.flip()
    running = True
    while running:
        for e in pg.event.get():
            if e.type == pg.constants.QUIT:
                running = False



if __name__ == '__main__':
    #test_basic(simple_space)
    test_star(wall_space)
