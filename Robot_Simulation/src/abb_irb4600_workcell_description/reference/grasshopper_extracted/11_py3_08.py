"""Provides a scripting component.
    Inputs:
        x: The x script variable
        y: The y script variable
    Output:
        a: The a output variable"""

__author__ = "aa9663"
__version__ = "2023.09.11"

import Rhino.Geometry as rg

collision_bricks = []


if check_collisions:
    for course in brick_wall_courses:
        c_num_bricks = len(course)
        next_brick_collision = False
        for brick_index, brick in enumerate(course):
            # Collision Check
            if brick_index < c_num_bricks - 1:
                next_brick = course[brick_index + 1]
                bb_intersect = rg.Intersect.Intersection.BrepBrep(
                brick.ToBrep(), next_brick.ToBrep(), 0.001)
                if len(bb_intersect[1]) != 0:
                    if next_brick_collision == False:
                        collision_bricks.append(brick)
                    collision_bricks.append(next_brick)
                    next_brick_collision = True
                else:
                     next_brick_collision = False
