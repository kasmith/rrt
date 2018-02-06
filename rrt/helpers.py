"""Private functions that are useful throughout the RRT package"""

from __future__ import division, print_function
import numpy as np
import pygame as pg


def point_in_box(min_verts, max_verts, point):
    return (np.all(point >= min_verts) and np.all(point <= max_verts))


def _line_intersect_circle(center, rad, p1, p2):
    # Make sure p1 is to the left of p2
    if p2[0] < p1[0]:
        ptmp = p2
        p2 = p1
        p1 = ptmp
    # Get Ax + By + C = 0 equation from points
    a = p1[1] - p2[1]
    b = p2[0] - p1[0]
    c = p1[0] * p2[1] - p2[0] * p1[1]
    # Find the foot of the perpendicular to the center point
    foot_term = -(a * center[0] + b * center[1] + c) / (a * a + b * b)
    foot_x = foot_term * a + center[0]
    foot_y = foot_term * b + center[1]
    # If the distance between the foot and circle is more than the radius,
    #  there is no intersection
    if np.linalg.norm(center - np.array([foot_x, foot_y])) > rad:
        return False
    # If the foot isn't on the segment, there is no intersection
    if foot_x < p1[0] or foot_x > p2[0]:
        return False
    return True


def _ball_intersect_rect(bcenter, brad, rll, rur):
    rverts = [[rll[0], rll[1]], [rll[0], rur[1]],
              [rur[0], rur[1]], [rur[0], rll[1]]]
    return (point_in_box(rll, rur, bcenter) or
            _line_intersect_circle(bcenter, brad, rverts[0], rverts[1]) or
            _line_intersect_circle(bcenter, brad, rverts[1], rverts[2]) or
            _line_intersect_circle(bcenter, brad, rverts[2], rverts[3]) or
            _line_intersect_circle(bcenter, brad, rverts[3], rverts[0]))


def ball_intersect(bcent, rad, wall):
    bleft = bcent[0] - rad
    btop = bcent[1] - rad
    brect = pg.Rect(bleft, btop, rad * 2, rad * 2)

    wl, wu = wall[0]
    wr, wb = wall[1]
    rect = pg.Rect(wl, wu, wr - wl, wb - wu)

    if rect.colliderect(brect):
        if rect.contains(brect):
            return True
        if brect.bottom > rect.top or brect.top < rect.bottom:
            if bcent[0] > rect.left and bcent[0] < rect.right:
                return True
        if brect.right > rect.left or brect.left < rect.right:
            if bcent[1] > rect.top and bcent[1] < rect.bottom:
                return True

        return any(map(lambda pt:
                       np.linalg.norm(np.array(pt) - np.array(bcent)) <= rad,
                       [rect.topleft, rect.topright,
                        rect.bottomleft, rect.bottomright]))
    else:
        return False
