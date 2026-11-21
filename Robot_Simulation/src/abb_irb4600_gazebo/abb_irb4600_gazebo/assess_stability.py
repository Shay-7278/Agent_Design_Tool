"""Independent, bounded dynamic observation of an exported assembly.

This command never contacts the assembly world. Warnings and incomplete
observations are reports, not an assembly acceptance gate or a certificate.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET

import yaml


def _rpy(quaternion):
    x, y, z, w = (float(v) for v in quaternion)
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm == 0:
        raise ValueError('invalid assembly quaternion')
    x, y, z, w = (v/norm for v in (x, y, z, w))
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def _pose_text(position, orientation):
    values = [*position, *_rpy(orientation)]
    if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError('invalid assembly pose')
    return ' '.join(str(float(v)) for v in values)


def build_assessment_world(manifest, gazebo_share, description_share,
                           world_name='assembly_stability'):
    """Reuse environment physics/supports and copy individual dynamic models."""
    if manifest.get('frame') != 'workcell_base':
        raise ValueError('assessment manifest must use workcell_base poses')
    bricks = manifest.get('bricks')
    if not isinstance(bricks, list) or not bricks:
        raise ValueError('assessment manifest contains no bricks')
    gazebo_share, description_share = Path(gazebo_share), Path(description_share)
    root = ET.parse(gazebo_share/'worlds/irb4600_workcell.sdf').getroot()
    world = root.find('world')
    world.set('name', world_name)
    fixtures = yaml.safe_load((description_share/'config/fixtures.yaml').read_text())['fixtures']
    table_pose = fixtures['milling_table']['pose']
    table = ET.SubElement(world, 'model', name='assessment_MT_support')
    ET.SubElement(table, 'static').text = 'true'
    ET.SubElement(table, 'pose').text = ' '.join(str(v) for v in [*table_pose['xyz'], *table_pose['rpy']])
    table_link = ET.SubElement(table, 'link', name='table')
    collision_mesh = description_share/'meshes/collision/fixtures/MT_base.stl'
    for tag in ('collision', 'visual'):
        geometry = ET.SubElement(ET.SubElement(table_link, tag, name=tag), 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'mesh'), 'uri').text = collision_mesh.resolve().as_uri()

    # Read the same literal dimensions/centre as the platform macro, not a
    # guessed replacement for the calibrated support surface.
    macro = ET.parse(description_share/'urdf/fixtures/platform_macro.xacro').getroot()
    properties = {p.get('name'): p.get('value') for p in macro
                  if p.tag.rsplit('}', 1)[-1] == 'property'}
    length, width, thickness = [float(properties['platform_'+key])
                                for key in ('length', 'width', 'thickness')]
    px, py = [float(v) for v in properties['platform_pos_xy'].split()]
    pz = float(fixtures['platform']['pose']['xyz'][2]) - thickness/2
    platform = ET.SubElement(world, 'model', name='assessment_platform')
    ET.SubElement(platform, 'static').text = 'true'
    ET.SubElement(platform, 'pose').text = f'{px} {py} {pz} 0 0 0'
    link = ET.SubElement(platform, 'link', name='platform')
    for tag in ('collision', 'visual'):
        geometry = ET.SubElement(ET.SubElement(link, tag, name=tag), 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = f'{length} {width} {thickness}'

    names = set()
    for entry in bricks:
        name = entry['name']
        if name in names:
            raise ValueError(f'duplicate assembly brick: {name}')
        names.add(name)
        model = copy.deepcopy(ET.parse(entry['model_sdf']).getroot().find('model'))
        if model is None:
            raise ValueError(f'brick SDF has no model: {name}')
        model.set('name', name)
        static = model.find('static')
        if static is None:
            static = ET.SubElement(model, 'static')
        static.text = 'false'
        pose = model.find('pose')
        if pose is None:
            pose = ET.SubElement(model, 'pose')
        pose.text = _pose_text(entry['position'], entry['orientation_xyzw'])
        world.append(model)
    return ET.tostring(root, encoding='unicode')


def summarize_assessment(manifest, poses, elapsed_sim, requested_sim,
                         incomplete_reason=None, displacement_warning_m=.01,
                         peak_displacements=None):
    """Report movement without judging or stopping the assembly executor."""
    observations, warnings, missing = [], [], []
    for entry in manifest.get('bricks', []):
        actual = poses.get(entry['name'])
        if actual is None:
            missing.append(entry['name'])
            continue
        position = list(actual.position)
        displacement = math.dist(position, entry['position'])
        peak = max(displacement, (peak_displacements or {}).get(entry['name'], 0.))
        downward = float(entry['position'][2])-position[2]
        record = {'name': entry['name'], 'initial_position': entry['position'],
                  'actual_position': position, 'orientation_xyzw': list(actual.orientation_xyzw),
                  'displacement_m': displacement, 'peak_displacement_m': peak, 'downward_m': downward}
        observations.append(record)
        if peak > displacement_warning_m:
            warnings.append(f"{entry['name']} moved up to {peak:.6f} m from its exported pose")
    if missing and incomplete_reason is None:
        incomplete_reason = 'missing brick observations'
    if elapsed_sim < requested_sim-1e-6 and incomplete_reason is None:
        incomplete_reason = 'requested simulation interval was not completed'
    return {'status': 'INCOMPLETE' if incomplete_reason else ('WARNING' if warnings else 'OBSERVED'),
            'assembly_blocked': False, 'elapsed_sim_s': elapsed_sim,
            'requested_sim_s': requested_sim, 'incomplete_reason': incomplete_reason,
            'displacement_warning_m': displacement_warning_m, 'warnings': warnings,
            'missing_bricks': missing, 'observations': observations,
            'max_displacement_m': max((o['displacement_m'] for o in observations), default=None),
            'max_observed_displacement_m': max((o['peak_displacement_m'] for o in observations), default=None),
            'scope': 'Exported final geometry only; no construction-stage or stability certificate.'}


def run_assessment(manifest, gazebo_share, description_share, duration=10.,
                   max_real_seconds=60.):
    partition = 'assembly_assessment_'+uuid.uuid4().hex
    world_name = partition
    xml = build_assessment_world(manifest, gazebo_share, description_share, world_name)
    previous_partition = os.environ.get('GZ_PARTITION')
    process = None
    world = None
    elapsed_sim = 0.
    poses = {}
    peaks = {}
    incomplete = None
    started = time.monotonic()
    try:
        os.environ['GZ_PARTITION'] = partition
        # Import the transport adapter before protobuf messages (same order as
        # the production executor); this also keeps pure unit tests ROS-free.
        from .gz_brick_world import GazeboBrickWorld
        with tempfile.TemporaryDirectory(prefix='assembly_stability_') as directory:
            path = Path(directory)/'assessment.sdf'
            path.write_text(xml, encoding='utf-8')
            with (Path(directory)/'server.log').open('w') as log:
                process = subprocess.Popen(['gz', 'sim', '-s', '-r', '-v', '2', str(path)],
                                           stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                world = GazeboBrickWorld(world_name, manifest['bricks'][0]['model_sdf'])
                start_sim = last_sim = None
                last_progress = time.monotonic()
                while True:
                    now = time.monotonic()
                    with world._condition:
                        sim_now = world._sim_time
                        poses = {entry['name']: world._poses[entry['name']]
                                 for entry in manifest['bricks'] if entry['name'] in world._poses}
                    for entry in manifest['bricks']:
                        if entry['name'] in poses:
                            peaks[entry['name']] = max(peaks.get(entry['name'], 0.),
                                math.dist(poses[entry['name']].position, entry['position']))
                    if sim_now is not None:
                        if start_sim is None:
                            start_sim = sim_now
                        if last_sim is not None and sim_now < last_sim-.001:
                            incomplete = 'assessment clock reset'
                            break
                        if last_sim is None or sim_now > last_sim:
                            last_progress = now
                        last_sim = sim_now
                        elapsed_sim = sim_now-start_sim
                    if elapsed_sim >= duration:
                        break
                    if process.poll() is not None:
                        incomplete = f'assessment server exited with {process.returncode}'
                        break
                    if now-started >= max_real_seconds:
                        incomplete = 'real-time assessment budget reached'
                        break
                    if now-last_progress > 15:
                        incomplete = 'assessment clock stopped or unavailable for 15 real seconds'
                        break
                    # Movement/collapse warnings do not terminate observation.
                    time.sleep(.1)
                return summarize_assessment(manifest, poses, elapsed_sim, duration, incomplete,
                                            peak_displacements=peaks)
    finally:
        if process is not None and process.poll() is None:
            if os.name == 'posix':
                os.killpg(process.pid, signal.SIGTERM)
            else:
                process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                if os.name == 'posix':
                    os.killpg(process.pid, signal.SIGKILL)
                else:
                    process.kill()
                process.wait(timeout=5)
        if previous_partition is None:
            os.environ.pop('GZ_PARTITION', None)
        else:
            os.environ['GZ_PARTITION'] = previous_partition


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=Path('/ros2_ws/outputs/assembly_manifest.json'))
    parser.add_argument('--report', type=Path, default=Path('/ros2_ws/outputs/stability_report.json'))
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--max-real-seconds', type=float, default=60.)
    args = parser.parse_args(argv)
    try:
        if not all(math.isfinite(v) and v > 0 for v in (args.duration, args.max_real_seconds)):
            raise ValueError('assessment time budgets must be finite and positive')
        from ament_index_python.packages import get_package_share_directory
        manifest = json.loads(args.manifest.read_text(encoding='utf-8'))
        report = run_assessment(manifest, get_package_share_directory('abb_irb4600_gazebo'),
                                get_package_share_directory('abb_irb4600_workcell_description'),
                                args.duration, args.max_real_seconds)
    except Exception as exc:
        report = {'status': 'INCOMPLETE', 'assembly_blocked': False,
                  'incomplete_reason': str(exc), 'warnings': [str(exc)]}
    try:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    except OSError as exc:
        print(f'WARNING: could not write assessment report: {exc}')
    print(f"Assessment {report['status']}; assembly is not blocked. Report: {args.report}")
    # Assessment outcomes are deliberately non-blocking, including warnings.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
