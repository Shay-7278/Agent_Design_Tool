# Numeric brick placement format

The placement file is a JSON array. Each row describes one brick in this fixed order:

```text
[x, y, z, yaw]
```

Meaning:

```text
x   brick centre X position, metres
y   brick centre Y position, metres
z   brick centre Z position, metres
yaw rotation around vertical Z axis, radians
```

Example:

```json
[
  [0.6741378411, 0.2217890810, 0.0285, -0.7699470212],
  [0.8654993566, 0.1298630763, 0.0285, -0.1684007054],
  [1.0894382519, 0.1453941818, 0.0285,  0.2811147980],
  [1.2907735412, 0.2464283941, 0.0285,  0.5911400066]
]
```

The first row is brick 1, the second row is brick 2, and so on. The runtime converts
`yaw` to the quaternion required by ROS/Gazebo.

Positions are relative to `MT_base`, at the milling table top. Yaw is measured
about that frame's positive Z axis. These are placement centres; the fixed
pickup layout is supplied by the workcell and is not part of this JSON.
