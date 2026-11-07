Assumptions: Horizontal bed joints. Stability check under self-weight only. Number of bricks per course.
Input: surface, brick dimensions (brickL, brickW, brickH), mortar bed joint thickness mortar_t, running bond offset_distance, number of bricks per course N, material properties (brick density, friction coefficient mu, mortar/glue flexural tensile strength f_t), safety factor SF, nominal head joint head_joint (used to fit the courses in step 2), head joint range [min_head = 10 mm, max_head] for the moves in step 4 (keep head_joint above min_head, or step 4 has no space to redistribute), max overhang per course, wall thickness cap, max iterations max_iter
Note: confirm brick density (scripts default to 700 kg/m3; fired clay brick is ~1800-2000 kg/m3). Tension demand scales directly with it.

1. Receive the surface from Rhino.
2. Scale the surface to fit a whole number of bricks, then fit the bricks:
    - Height: scale the surface down in Z (about its base) to the nearest whole number of courses below its current height h:
        n_courses = floor((h + mortar_t) / (brickH + mortar_t))
        target height = n_courses * (brickH + mortar_t) - mortar_t
        Z scale factor = target height / h (<= 1). If n_courses < 2, stop: the surface is too short.
      Do this first, because it moves the heights where the courses are cut.
    - Length (per course, because the plan changes with height): cut the scaled surface at each course's mid-height, z = k * (brickH + mortar_t) + brickH / 2. Measure the length along the brick centreline (offset brickW / 2 from the surface if the surface is the wall face, not the centreline). Stretch that section along the wall-run direction (Y) only, about the wall's centre, until
        centreline length = N * (brickL + head_joint)
      Solve the stretch factor by bisection, as curve_generate.py does for the sine curve. Keep X (the overhang direction) unchanged so the offsets between courses, i.e. the corbel overhang the physics checks evaluate, are not altered by the fit. Each course gets its own stretch factor, so the plan proportions of the surface change. That is the price of an exact fit.
      Minimum length: as the stretch factor goes to 0 the section flattens onto the X axis, so its length cannot drop below the total distance it travels in X. If N * (brickL + head_joint) is below that, no stretch fits: stop and report the course.
      Overhang change: the overhang is perpendicular to the wall, which is X only where the wall runs along Y. Where the wall turns towards X, the Y stretch moves it in its overhang direction by a different amount in each course, so the fit changes the overhang there. This is accepted: steps 3-6 check the fitted wall, not the original surface.
    - Place N bricks per course along each fitted section in a running bond, with offset_distance as a variable. With an exact fit every head joint equals head_joint. On an open wall the odd courses stick out past the wall ends by offset_distance unless half bricks are used at the ends.
    - The bed face of course k sits exactly at z = k * (brickH + mortar_t) (brick centre at + brickH / 2), so the bed faces are coplanar (CRA only detects interfaces within tmax = 1e-5 m). Each brick is a frame (centre + rotation following the course tangent). This fitted position is the reference for the per-brick deviation reported in the output. The generated wall is the subject of analysis.
3. (L0) Geometry check, per brick:
    - Contact: each brick bears on the brick(s) below with at least a minimum overlap area. In running bond each brick sits on two bricks below, so check each pair.
    - Collision: bricks in the same course keep a head joint of at least min_head. Check every pair in the course, not only neighbours (non-adjacent bricks can collide on tight concave curves). Bricks in different courses cannot collide because of the horizontal bed joints.
    - Design limits: each brick's overhang relative to the brick(s) below stays within the max overhang, and the total wall thickness stays within the thickness cap. These are checked on the fitted wall, because the step 2 fit can change the overhang.
4. If L0 fails, modify the bricks (not the surface):
    - Contact failure: move the brick in its course's X-Y plane to the closest position that restores the minimum overlap.
    - Collision: move the bricks apart along the course until the head joint is >= min_head. Take the space from the other head joints in the course, keeping every head joint within [min_head, max_head]. If the course cannot fit N bricks within that range, stop and report the course (N must change).
    - Design limit failure: move the brick in its course's X-Y plane, perpendicular to the wall, to the closest position within the max overhang and the thickness cap.
    - Every move must stay within the max overhang and the thickness cap, and is the smallest move that passes (so the wall only loses lean where stability needs it). If no position within the limits passes, flag the brick instead of moving it further.
    - After any move, go back to step 3.
5. (L1) Export brick frames to JSON (export_frames_json.py) and run CRA_penalty with course 0 as fixed supports (cra_check.py --penalty, with mu and density from the inputs). Bridge Rhino <-> CPython with Hops, rhino.compute or a subprocess call so the loop runs without manual steps.
6. Read the CRA result:
    - No tension needed -> the wall is stable as a dry stack. Done.
    - Tension needed -> highlight the bricks/interfaces that need tension. For each tensioned interface, compute the net tensile stress from the CRA corner forces:
        - resultant normal force P = sum of the corner normal forces
        - moment M about the interface centroid from the corner forces
        - sigma_t = M/Z - P/A, using the actual contact polygon's area A and section modulus Z
        - if sigma_t > f_t / SF -> move the brick above the failing interface in its X-Y plane to the closest position that passes, within the max overhang and the thickness cap. Work top-down. If no position within the limits passes, flag the brick.
        - Note: the penalty solve finds the least tension needed for equilibrium, not the elastic stress distribution, so sigma_t is a lower-bound estimate; SF covers this.
    - Non-convergence -> diagnose before moving anything:
        - check the interface count and look for isolated/floating blocks (geometry problem -> back to step 3)
        - otherwise it is likely sliding, which tension cannot resist -> report the bricks involved; fix by reducing the overhang of the sliding bricks within the limits, then re-check tension.
7. Outer loop: after any move in step 6, go back to step 3 (re-run L0, then L1). Stop when L0 and L1 both pass, or after max_iter iterations. If max_iter is reached, report the remaining failing bricks/interfaces and the flagged bricks.

Output: a frame (centre + rotation) for each brick, plus per-brick status (passed / moved / flagged), each brick's distance from its fitted position (how far the checks changed the design; reported, not limited), and the per-interface sigma_t / (f_t / SF). This is fed into Rhino to generate the updated structure for visualization.
