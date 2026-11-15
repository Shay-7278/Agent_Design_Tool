"""Provides a scripting component.
    Inputs:
        x: The x script variable
        y: The y script variable
    Output:
        a: The a output variable"""

__author__ = "aa9663"
__version__ = "2023.09.11"

import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import math

courses = []
brick_wall_courses = []

sorted_bricks = sorted(bricks, key = lambda brick: (math.floor(brick.Center.Z), math.floor(brick.Center.X)))

current_z = round(bricks[0].Center.Z, 0)
current_course_bricks = []

if sort_bricks:
    for brick in sorted_bricks:
        #brick = rg.Box(brick)
        #print(brick)
        if round(brick.Center.Z, 0) > current_z:
            current_z = round(brick.Center.Z, 0)
            brick_wall_courses.append(current_course_bricks)
            current_course_bricks = []
            current_course_bricks.append(brick)
        else:
            current_course_bricks.append(brick)
    
    #Append the last course
    brick_wall_courses.append(current_course_bricks)
    
    course_bricks = brick_wall_courses[course_index]