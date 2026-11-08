"""Constants, parameter defaults and ``resolve_params``."""

from __future__ import division

MM = 0.001          # mm -> m
G = 9.81            # compas_cra loads are volume * density [kg]; * G -> N
TENSION_TOL = 1e-3  # corner tension below this [N] counts as none
EPS = 1e-9

DEFAULTS = {
    "brickL": 190.0,
    "brickW": 90.0,
    "brickH": 57.0,
    "mortar_t": 10.0,
    "head_joint": 15.0,
    "min_head": 10.0,
    "max_head": 25.0,
    "offset_distance": None,   # -> (brickL + head_joint) / 2
    "face_offset": 0.0,
    "density": 700.0,
    "mu": 0.84,
    "f_t": 0.2e6,
    "SF": 2.0,
    "max_overhang": None,      # -> brickW / 2
    "thickness_cap": None,     # -> 3 * brickW
    "min_contact": 0.25,
    "max_iter": 10,
    "l0_max_iter": 20,         # inner L0 repair passes per outer iteration
    "move_step": 2.0,          # search step for the smallest-move search [mm]
    "max_move": None,          # -> brickL / 2
    "min_step": 1.0,           # smallest L1 move tried before flagging [mm]
    "slide_step": 5.0,         # overhang reduction for suspected sliding [mm]
    "nmax": 10,                # CRA neighbours per block
    "tmax": 1e-5,              # CRA interface flatness tolerance [m]
    "amin": 1e-4,              # CRA minimum interface area [m2]
    "cra_timeout": 300.0,      # time limit per CRA solve [s]; 0 = no limit
}


def resolve_params(given):
    """Fill defaults and the derived defaults."""
    p = dict(DEFAULTS)
    for k, v in (given or {}).items():
        if v is not None:
            p[k] = v
    for k in ("brickL", "brickW", "brickH", "mortar_t", "head_joint", "min_head",
              "max_head", "face_offset", "density", "mu", "f_t", "SF", "min_contact",
              "move_step", "min_step", "slide_step", "tmax", "amin", "cra_timeout"):
        p[k] = float(p[k])
    for k in ("max_iter", "l0_max_iter", "nmax"):
        p[k] = int(p[k])
    if p["offset_distance"] is None:
        p["offset_distance"] = 0.5 * (p["brickL"] + p["head_joint"])
    if p["max_overhang"] is None:
        p["max_overhang"] = 0.5 * p["brickW"]
    if p["thickness_cap"] is None:
        p["thickness_cap"] = 3.0 * p["brickW"]
    if p["max_move"] is None:
        p["max_move"] = 0.5 * p["brickL"]
    for k in ("offset_distance", "max_overhang", "thickness_cap", "max_move"):
        p[k] = float(p[k])
    return p
