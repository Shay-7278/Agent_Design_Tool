"""Brick wall fit + L0/L1 physics check (physics_check_workflow.md), pure CPython.

Steps of the workflow and the module that implements each:
    1-2  surface -> courses                Rhino side, ``physics_check_gh.py``
         (scale Z to a whole number of courses, cut one section per course)
    2    offset/align each section; brick   ``fit.py``
         count per course from its length
         place the bricks in running bond,  ``wall.py``
         fewer on tight curves
    3-4  L0 geometry check + repairs        ``wall.Wall.check``, ``l0.py``
    5    L1 compas_cra penalty solve,       ``cra.py``
         course 0 fixed
    6    sigma_t = M/Z - P/A per interface  ``cra.interface_stress``
         moves above failing interfaces     ``l1.py``
    7    outer loop                         ``loop.py``

Shared pieces: ``params.py`` (defaults, constants), ``geom2d.py`` (polygon
clipping, plan polylines).  Dependencies run one way:
    params -> geom2d -> fit, wall -> l0, cra -> l1 -> loop
Only ``cra.run_cra`` imports compas_cra, and only when it is called, so every
module imports in plain CPython without it.

Axes: frame Y = along the course (brickL), frame X = overhang / wall thickness
(brickW), Z up.  The wall run is world Y.  Lengths in mm; f_t in Pa; density
in kg/m3.  The CRA solve works in m (``params.MM``).
"""
