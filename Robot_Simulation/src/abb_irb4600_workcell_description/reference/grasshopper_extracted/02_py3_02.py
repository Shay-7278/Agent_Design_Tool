import Rhino.Geometry as rg
import math

pickup_frames = []
offset_pickup_frames = []

for brick in pickup_bricks:

    # Original brick frame
    brick_frame = rg.Plane(brick.Plane)
    brick_frame.Origin = brick.Center

    # Brick to TCP transformation
    pickup_frame = rg.Plane(brick_frame)
    pickup_frame.Rotate(math.pi, pickup_frame.XAxis)

    pickup_frames.append(pickup_frame)

    # Retract along world +Z
    offset_pickup_frame = rg.Plane(pickup_frame)
    offset_pickup_frame.Translate(
        rg.Vector3d.ZAxis * z_retract
    )

    offset_pickup_frames.append(offset_pickup_frame)