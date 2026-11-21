"""Select brick geometry from confirmed sizes bound to an exact placement sequence."""
from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from .motion import Pose


def pickup_pose_on_platform(pose: Pose, model_sdf, platform_top=None) -> Pose:
    """Keep the exported slot XY/yaw; rest the selected box on the platform."""
    if platform_top is None:
        from ament_index_python.packages import get_package_share_directory
        fixtures = Path(get_package_share_directory('abb_irb4600_workcell_description')) / 'config/fixtures.yaml'
        config = yaml.safe_load(fixtures.read_text(encoding='utf-8'))
        platform_top = config['fixtures']['platform']['pose']['xyz'][2]
    dimensions = ET.parse(model_sdf).getroot().findtext('./model/link/collision/geometry/box/size')
    if dimensions is None:
        raise ValueError(f'pickup model must have box collision geometry: {model_sdf}')
    size = tuple(float(value) for value in dimensions.split())
    if len(size) != 3 or any(not math.isfinite(value) or value <= 0 for value in size):
        raise ValueError(f'invalid pickup collision dimensions: {model_sdf}')
    height = float(platform_top) + size[2] / 2
    return Pose((pose.position[0], pose.position[1], height), pose.orientation_xyzw)


def _placement_fingerprint(rows) -> str:
    if not isinstance(rows, list) or not rows:
        raise ValueError('placement file must contain [x,y,z,yaw] rows')
    normalized = []
    for row in rows:
        if (not isinstance(row, list) or len(row) != 4 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) for v in row)):
            raise ValueError('placement rows must contain four finite numbers [x,y,z,yaw]')
        normalized.append([float(value) for value in row])
    return hashlib.sha256(json.dumps(normalized, separators=(',', ':'), allow_nan=False).encode('utf-8')).hexdigest()


def load_brick_models(placement_file, model_dir, profile_path=None) -> tuple[Path, ...]:
    """Keep XYZ/yaw input unchanged; apply half sizes only to the confirmed poses.

    A different sequence keeps the existing full-brick default. Formatting or
    filename changes do not detach a sequence from its confirmed size profile.
    """
    rows = json.loads(Path(placement_file).read_text(encoding='utf-8'))
    fingerprint = _placement_fingerprint(rows)
    if profile_path is None:
        from ament_index_python.packages import get_package_share_directory
        profile_path = Path(get_package_share_directory('abb_irb4600_gazebo')) / 'config/brick_sizes.json'
    profile = json.loads(Path(profile_path).read_text(encoding='utf-8'))
    count, half_rows = profile['placement_count'], profile['half_rows']
    if (isinstance(count, bool) or not isinstance(count, int) or count < 1 or
            not isinstance(half_rows, list) or
            any(isinstance(row, bool) or not isinstance(row, int) or not 1 <= row <= count
                for row in half_rows) or len(set(half_rows)) != len(half_rows)):
        raise ValueError('brick size profile requires unique 1-based half rows within placement_count')
    matched = count == len(rows) and profile['placement_sha256'] == fingerprint
    half_indices = {row - 1 for row in half_rows} if matched else set()
    model_dir = Path(model_dir).resolve()
    full = model_dir / 'brick/model.sdf'
    half = model_dir / 'brick_half/model.sdf'
    for path in ([full, half] if half_indices else [full]):
        if not path.is_file():
            raise FileNotFoundError(f'brick model not installed: {path}')
    return tuple(half if index in half_indices else full for index in range(len(rows)))
