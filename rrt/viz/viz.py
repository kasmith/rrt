"""Simple functions to draw RRT in pygame

"""

from ..space import EmptySpace, WallSpace
import pygame as pg


def intify(vect):
    return list(map(int, vect))


"""Helper functions to draw the space and tree"""


def draw_space(surface, space, goal_color=pg.Color('green'),
               wall_color=pg.Color('black')):
    surface.fill(pg.Color('white'))
    # Draw the goals
    for g in space.goals:
        ext = g.extent
        ul = [ext['lower_left'][0], ext['upper_right'][1]]
        wh = ext['upper_right'] - ext['lower_left']
        r = pg.Rect(ul, wh)
        pg.draw.rect(surface, goal_color,
                     pg.Rect(space.invert(ul), wh))
    # Draw the walls
    if isinstance(space, WallSpace):
        for w in space.walls:
            ul = [w[0][0], w[1][1]]
            wh = [w[1][0] - w[0][0], w[1][1] - w[0][1]]
            r = pg.Rect(ul, wh)
            pg.draw.rect(surface, wall_color,
                         pg.Rect(space.invert(ul), wh))


def _draw_down_tree(surface, node, space, node_color=pg.Color('darkgrey'),
                    path_color=pg.Color('lightgrey')):
    pt = intify(space.invert(node.point))
    pg.draw.circle(surface, node_color, pt, 3)
    for c in node.children:
        pg.draw.line(surface, path_color,
                     pt, intify(space.invert(c.point)))
        _draw_down_tree(surface, c, space)


def draw_tree(surface, tree, space, node_color=pg.Color('darkgrey'),
              path_color=pg.Color('lightgrey'),
              start_color=pg.Color('blue')):
    pg.draw.circle(surface, start_color,
                   intify(space.invert(tree.head.point)), 5)
    _draw_down_tree(surface, tree.head, space, node_color, path_color)


def draw_rrt(surface, rrt, goal_color=pg.Color('green'),
             wall_color=pg.Color('black'), node_color=pg.Color('darkgrey'),
             path_color=pg.Color('lightgrey'), start_color=pg.Color('blue')):
    draw_space(surface, rrt.space, goal_color, wall_color)
    draw_tree(surface, rrt.tree, rrt.space,
              node_color, path_color, start_color)
    _, shortest = rrt.get_shortest_path()
    if shortest is not None:
        pt_path = [intify(rrt.space.invert(p)) for p in shortest]
        pg.draw.lines(surface, start_color, False, pt_path, 2)


def animate_rrt(surface, rrt, steps=1000, goal_color=pg.Color('green'),
             wall_color=pg.Color('black'), node_color=pg.Color('darkgrey'),
             path_color=pg.Color('lightgrey'), start_color=pg.Color('blue')):
    draw_space(surface, rrt.space, goal_color, wall_color)
    draw_tree(surface, rrt.tree, rrt.space,
              node_color, path_color, start_color)
    pg.display.flip()
    for _ in range(steps):
        pg.event.pump()
        rrt.step()
        draw_space(surface, rrt.space, goal_color, wall_color)
        draw_tree(surface, rrt.tree, rrt.space,
                  node_color, path_color, start_color)
        _, shortest = rrt.get_shortest_path()
        if shortest is not None:
            pt_path = [intify(rrt.space.invert(p)) for p in shortest]
            pg.draw.lines(surface, start_color, False, pt_path, 2)
        pg.display.flip()
    pause_pg()


def pause_pg():
    while True:
        for e in pg.event.get():
            if (e.type == pg.constants.QUIT or
                    e.type == pg.constants.KEYDOWN):
                return
