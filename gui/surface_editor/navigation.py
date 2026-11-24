"""Upright CAD-style right-drag navigation, independent of surface editing."""
import math
import numpy as np
from PySide6 import QtCore


def orbit(camera, dx, dy):
    focal = np.asarray(camera.focal_point)
    offset = np.asarray(camera.position)-focal
    radius = max(float(np.linalg.norm(offset)), 1e-6)
    azimuth = math.atan2(offset[1], offset[0])-math.radians(dx*.35)
    elevation = math.asin(np.clip(offset[2]/radius, -1, 1))+math.radians(dy*.35)
    elevation = np.clip(elevation, math.radians(-89), math.radians(89))
    camera.position = focal + radius*np.array([math.cos(elevation)*math.cos(azimuth),
                                              math.cos(elevation)*math.sin(azimuth), math.sin(elevation)])
    camera.up = (0, 0, 1)


def pan(camera, dx, dy, height):
    offset = np.asarray(camera.focal_point)-np.asarray(camera.position)
    distance = max(np.linalg.norm(offset), 1e-6)
    direction = offset/distance
    right = np.cross(direction, [0, 0, 1])
    right /= max(np.linalg.norm(right), 1e-6)
    up = np.cross(right, direction)
    extent = camera.parallel_scale if camera.parallel_projection else distance*math.tan(math.radians(camera.view_angle/2))
    delta = (-dx*right+dy*up)*(2*extent/max(height, 1))
    camera.position = np.asarray(camera.position)+delta
    camera.focal_point = np.asarray(camera.focal_point)+delta


class UprightNavigation(QtCore.QObject):
    def __init__(self, plotter):
        super().__init__(plotter)
        self.plotter = plotter
        self.last = None
        plotter.interactor.installEventFilter(self)
        plotter.enable_terrain_style(mouse_wheel_zooms=True, shift_pans=True)
        plotter.camera.up = (0, 0, 1)

    def eventFilter(self, obj, event):
        kind = event.type()
        if kind == QtCore.QEvent.Type.MouseButtonPress and event.button() == QtCore.Qt.MouseButton.RightButton:
            self.last = event.position()
            obj.setFocus()
            return True
        if kind == QtCore.QEvent.Type.MouseMove and self.last is not None:
            pos = event.position()
            dx, dy = pos.x()-self.last.x(), pos.y()-self.last.y()
            self.last = pos
            if event.modifiers() & QtCore.Qt.KeyboardModifier.ShiftModifier:
                pan(self.plotter.camera, dx, dy, obj.height())
            else:
                orbit(self.plotter.camera, dx, dy)
            self.plotter.reset_camera_clipping_range()
            self.plotter.render()
            return True
        if kind == QtCore.QEvent.Type.MouseButtonRelease and event.button() == QtCore.Qt.MouseButton.RightButton:
            self.last = None
            return True
        if kind in (QtCore.QEvent.Type.FocusOut, QtCore.QEvent.Type.Hide):
            self.last = None
        return False
