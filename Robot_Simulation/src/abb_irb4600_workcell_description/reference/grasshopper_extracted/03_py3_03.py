import Rhino.Geometry as rg
import math

placement_frames = []
offset_placement_frames = []

for brick in bricks:

    # Original brick frame
    brick_frame = rg.Plane(brick.Plane)
    brick_frame.Origin = brick.Center

    # Brick to TCP transformation
    placement_frame = rg.Plane(brick_frame)
    placement_frame.Rotate(math.pi, placement_frame.XAxis)

    placement_frames.append(placement_frame)

    # Retract along world +Z
    offset_placement_frame = rg.Plane(placement_frame)
    offset_placement_frame.Translate(
        rg.Vector3d.ZAxis * z_retract
    )

    offset_placement_frames.append(offset_placement_frame)