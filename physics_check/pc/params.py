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
    "min_head": 5.0,
    "max_head": 25.0,
    "offset_distance": None,   # -> (brickL + head_joint) / 2
    "face_offset": 0.0,
    "density": 700.0,
    "mu": 0.84,
    "f_t": 0.2e6,
    "SF": 2.0,
    "max_overhang": None,      # -> brickW / 2
    "min_contact": 0.25,
    "min_balance": 10.0,       # brick centre at least this far inside its contact region [mm]
    "max_iter": 10,
    "l0_max_iter": 20,         # inner L0 repair passes per outer iteration
    "move_step": 2.0,          # search step for the smallest-move search [mm]
    "max_move": 0.0,           # largest deviation from the fitted position [mm]; 0 = no limit
                               # (greedy L0 and L1 then use brickL / 2 per move, see move_cap)
    "min_step": 1.0,           # smallest L1 move tried before flagging [mm]
    "slide_step": 5.0,         # overhang reduction for suspected sliding [mm]
    "nmax": 10,                # CRA neighbours per block
    "tmax": 1e-5,              # CRA interface flatness tolerance [m]
    "amin": 1e-4,              # CRA minimum interface area [m2]
    "cra_timeout": 300.0,      # time limit per CRA solve [s]; 0 = no limit
    "solver": "cra_penalty",   # L1 solver: "cra_penalty" (Eq. 14) or "rbe" (Eq. 6, faster)
    "l0_method": "qp",         # L0 repair: "qp" (all bricks together, l0qp.py) or "greedy" (l0.py)
    "qp_trust": 20.0,          # QP L0 repair: starting largest move per round [mm]
                               # (grows up to 4x after good rounds, halves after bad ones)
    "qp_max_iter": 50,         # QP L0 repair: rounds per call
}

SOLVERS = ("cra_penalty", "rbe")
L0_METHODS = ("qp", "greedy")


def move_cap(p):
    """Largest single move for the greedy L0 search and the L1 moves [mm]:
    max_move, or brickL / 2 when max_move is 0 (no limit)."""
    return p["max_move"] if p["max_move"] > 0 else 0.5 * p["brickL"]


def resolve_params(given):
    """Fill defaults and the derived defaults."""
    p = dict(DEFAULTS)
    for k, v in (given or {}).items():
        if v is not None:
            p[k] = v
    for k in ("brickL", "brickW", "brickH", "mortar_t", "head_joint", "min_head",
              "max_head", "face_offset", "density", "mu", "f_t", "SF", "min_contact", "min_balance",
              "move_step", "min_step", "slide_step", "tmax", "amin", "cra_timeout", "qp_trust"):
        p[k] = float(p[k])
    for k in ("max_iter", "l0_max_iter", "nmax", "qp_max_iter"):
        p[k] = int(p[k])
    if p["offset_distance"] is None:
        p["offset_distance"] = 0.5 * (p["brickL"] + p["head_joint"])
    if p["max_overhang"] is None:
        p["max_overhang"] = 0.5 * p["brickW"]
    if p["max_move"] is None or p["max_move"] < 0:
        p["max_move"] = 0.0
    for k in ("offset_distance", "max_overhang", "max_move"):
        p[k] = float(p[k])
    p["solver"] = str(p["solver"]).strip().lower()
    if p["solver"] not in SOLVERS:
        raise ValueError("solver must be one of {}, got {!r}".format(", ".join(SOLVERS), p["solver"]))
    p["l0_method"] = str(p["l0_method"]).strip().lower()
    if p["l0_method"] not in L0_METHODS:
        raise ValueError("l0_method must be one of {}, got {!r}".format(
            ", ".join(L0_METHODS), p["l0_method"]))
    return p
