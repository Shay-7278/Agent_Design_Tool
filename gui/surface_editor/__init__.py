"""NURBS surface editor: drag control points and scale a Rhino `.3dm` surface.

Modules
-------
model   Geometry core (load / evaluate / scale / save via rhino3dm).
editor  PyVista GUI (draggable CP handles, scale sliders, undo).
"""

from .model import NurbsModel

__all__ = ["NurbsModel"]
