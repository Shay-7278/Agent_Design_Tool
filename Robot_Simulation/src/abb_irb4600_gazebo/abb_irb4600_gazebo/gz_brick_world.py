"""Blocking entity operations and feedback-confirmed Gazebo detachable joints."""

from __future__ import annotations

import threading
import time
import copy
import math
import xml.etree.ElementTree as ET
import os
import tempfile
from pathlib import Path
from functools import wraps

from gz.transport13 import Node, SubscribeOptions
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.pose_pb2 import Pose as GazeboPose
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.geometry_pb2 import Geometry
from gz.msgs10.serialized_map_pb2 import SerializedStepMap

from .motion import Pose
from .brick_models import pickup_pose_on_platform


class GazeboCommandError(RuntimeError):
    pass


class GazeboRequestTimeout(GazeboCommandError):
    """The request outcome is unknown because its reply was not received."""


def bounded_operation(seconds=60.0):
    """One budget includes mutation submission and every verification read."""
    def decorate(method):
        @wraps(method)
        def bounded(self, *args, **kwargs):
            previous = getattr(self, '_operation_deadline', None)
            previous_clock = getattr(self, '_operation_start_clock', None)
            self._operation_deadline = min(previous or float('inf'), time.monotonic() + seconds)
            self._operation_start_clock = (previous_clock if previous is not None
                                           else getattr(self, '_sim_time', None))
            try:
                return method(self, *args, **kwargs)
            finally:
                self._operation_deadline = previous
                self._operation_start_clock = previous_clock
        return bounded
    return decorate


def _component_id(name: str) -> int:
    """Gazebo Common's FNV-1a hash64, used by the component factory."""
    value = 0xcbf29ce484222325
    for byte in name.encode('utf-8'):
        value = ((value ^ byte) * 0x100000001b3) & 0xffffffffffffffff
    return value


DETACHABLE_JOINT_TYPE = _component_id('gz_sim_components.DetachableJoint')
STATIC_COMPONENT_TYPE = _component_id('gz_sim_components.Static')
GEOMETRY_COMPONENT_TYPE = _component_id('gz_sim_components.Geometry')
COLLISION_COMPONENT_TYPE = _component_id('gz_sim_components.Collision')
PARENT_ENTITY_COMPONENT_TYPE = _component_id('gz_sim_components.ParentEntity')


def detachable_joints(snapshot):
    """Read actual ECM components, not one-shot transport notifications."""
    joints = {}
    for entity_id, entity in snapshot.state.entities.items():
        for component in entity.components.values():
            if component.type != DETACHABLE_JOINT_TYPE:
                continue
            try:
                parent, child, kind = component.component.decode('utf-8').split()
                joints[entity_id] = (int(parent), int(child), kind)
            except (ValueError, UnicodeError) as exc:
                raise GazeboCommandError('invalid Gazebo detachable-joint component') from exc
    return joints


class GazeboBrickWorld:
    def __init__(
        self,
        world_name: str,
        brick_sdf: str | Path,
        parent_model: str = "abb_irb4600_workcell",
        parent_link: str = "LL_link_6",
        brick_models: dict[str, str | Path] | None = None,
        static_supply: bool = False,
    ) -> None:
        self.world_name = world_name
        self.brick_sdf = str(Path(brick_sdf).resolve())
        self.brick_models = dict(brick_models or {})
        self.static_supply = static_supply
        self.parent_model = parent_model
        self.parent_link = parent_link
        self.node = Node()
        self._poses = {}
        self._pose_sequence = {}
        self._pose_time = {}
        self._pose_ids = {}
        self._states = {}
        self._notifications = {}
        self._plugins = set()
        self._publishers = {}
        self._link_ids = {}
        self.attachment_checks = {}
        self.placed = {}
        self.static_model_name = None
        self._static_generation = 0
        self._brick_geometry_cache = {}
        self._sim_time = None
        self._clock_progress_wall = time.monotonic()
        self._condition = threading.Condition()
        # Static wall links enlarge Pose_V considerably. Bound callback traffic
        # without changing the physics timestep or the controller update rate.
        self._pose_options = SubscribeOptions()
        self._pose_options.msgs_per_sec = 60
        self._clock_options = SubscribeOptions()
        self._clock_options.msgs_per_sec = 100
        self.node.subscribe(Pose_V, f"/world/{world_name}/pose/info", self._on_poses,
                            self._pose_options)
        self.node.subscribe(Clock, f"/world/{world_name}/clock", self._on_clock,
                            self._clock_options)

    def close(self):
        """Stop native callbacks before Python/ROS interpreter teardown."""
        client = getattr(self, '_service_client', None)
        if client is not None:
            client.close()
            self._service_client = None
        node = getattr(self, 'node', None)
        if node is None:
            return
        for topic in node.subscribed_topics():
            node.unsubscribe(topic)
        self._publishers.clear()
        self.node = None
        del node
        # Native transport callbacks already in flight must leave Python while
        # the interpreter is still alive, rather than during finalization.
        time.sleep(0.25)

    def _on_clock(self, message):
        with self._condition:
            stamp = message.sim.sec + message.sim.nsec * 1e-9
            if self._sim_time is None or stamp != self._sim_time:
                self._clock_progress_wall = time.monotonic()
            self._sim_time = stamp
            self._condition.notify_all()

    def _on_poses(self, message):
        with self._condition:
            for p in message.pose:
                self._poses[p.name] = Pose((p.position.x, p.position.y, p.position.z),
                                          (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                self._pose_sequence[p.name] = self._pose_sequence.get(p.name, 0) + 1
                self._pose_time[p.name] = (message.header.stamp.sec + message.header.stamp.nsec * 1e-9,
                                           time.monotonic())
                self._pose_ids[p.name] = p.id
            self._condition.notify_all()

    def _wait(self, predicate, description, timeout=15.0):
        deadline = min(time.monotonic() + timeout, getattr(self, '_operation_deadline', None) or float('inf'))
        with self._condition:
            while not predicate():
                self._check_operation()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise GazeboCommandError(f"timed out waiting for {description}")
                self._condition.wait(min(remaining, 0.2))
            self._check_operation()

    def _request(self, service, request, response_type=Boolean):
        # Readbacks are idempotent. A short-lived CLI client's reply can be
        # lost during scene updates; retry a read, never repeat a mutation.
        read_only = service in ('scene/info', 'state')
        attempt = 0
        while True:
            attempt += 1
            try:
                return self._request_once(service, request, response_type)
            except GazeboRequestTimeout:
                if not read_only or (getattr(self, '_operation_deadline', None) is None and attempt >= 2):
                    raise
                self._check_operation()
                time.sleep(.02)

    def _request_once(self, service, request, response_type=Boolean):
        from .persistent_service import PersistentServiceClient, ServiceTimeout, ServiceRejected, ServiceClientError
        self._check_operation()
        client = getattr(self, '_service_client', None)
        if client is not None and not client.healthy:
            client.close()
            client = None
        if client is None:
            client = self._service_client = PersistentServiceClient()
        remaining = (getattr(self, '_operation_deadline', None) or time.monotonic() + 10) - time.monotonic()
        try:
            reply = client.request(f'/world/{self.world_name}/{service}', request, response_type,
                                   timeout_ms=max(1, min(10000, int(remaining * 1000))))
        except ServiceTimeout as exc:
            raise GazeboRequestTimeout(f'Gazebo {service}: {exc}') from exc
        except ServiceRejected as exc:
            raise GazeboCommandError(f'Gazebo {service}: {exc}') from exc
        except ServiceClientError as exc:
            # Local framing/connection failure can occur after a mutation was
            # sent. Reconcile state; never infer that it did not execute.
            raise GazeboRequestTimeout(f'Gazebo {service}: {exc}') from exc
        if response_type is Boolean and not reply.data:
            raise GazeboCommandError(f"Gazebo {service} returned false")
        self._check_operation()
        return reply

    def _check_operation(self):
        deadline = getattr(self, '_operation_deadline', None)
        if deadline is not None and time.monotonic() >= deadline:
            raise GazeboCommandError('Gazebo operation deadline expired; state remains unconfirmed')
        initial = getattr(self, '_operation_start_clock', None)
        current = getattr(self, '_sim_time', None)
        if initial is not None and current is not None and current < initial - .001:
            raise GazeboCommandError('Gazebo world/clock reset during operation')

    def _confirm(self, predicate, description, timeout=30.0):
        deadline = min(time.monotonic() + timeout,
                       getattr(self, '_operation_deadline', None) or float('inf'))
        last_error = None
        while time.monotonic() < deadline:
            self._check_operation()
            try:
                result = predicate()
                self._check_operation()
                if result:
                    return result
            except GazeboRequestTimeout as exc:
                last_error = str(exc)
            time.sleep(.02)
        raise GazeboCommandError(f'timed out confirming {description}; last readback={last_error}')

    def _send_once(self, service, request):
        """Never repeat an uncertain mutation; caller must confirm its result."""
        try:
            self._request(service, request)
        except GazeboRequestTimeout as exc:
            return exc
        return None

    @bounded_operation()
    def _remove_confirmed(self, name, entity_id=None):
        if entity_id is None:
            entity_id = self.models().get(name)
        if entity_id is None:
            return
        self._send_once('remove/blocking', Entity(id=entity_id, type=Entity.MODEL))
        self._confirm(lambda: entity_id not in self.models().values(), f'removal of {name} entity {entity_id}')

    def models(self):
        return {model.name: model.id for model in self._request("scene/info", Empty(), Scene).model}

    def _static_flag(self, entity_id, snapshot=None):
        """Return the explicit physics flag, or None when it is unavailable."""
        if snapshot is None:
            snapshot = self._request('state', Empty(), SerializedStepMap)
        if entity_id not in snapshot.state.entities:
            return None
        for component in snapshot.state.entities[entity_id].components.values():
            if component.type == STATIC_COMPONENT_TYPE:
                value = component.component.strip().lower()
                if value in (b'1', b'true'):
                    return True
                if value in (b'0', b'false'):
                    return False
        return None

    def _is_static(self, entity_id):
        """Read the physics entity flag; Scene's is_static field may be unset."""
        return self._static_flag(entity_id) is True

    def _create_once(self, request):
        """Send once; an uncertain reply must be resolved by caller readback."""
        return self._send_once('create', request)

    def _resolve_links(self, name):
        if name not in self._link_ids:
            scene = self._request('scene/info', Empty(), Scene)
            models = {m.name: m for m in scene.model}
            try:
                parent = next(l.id for l in models[self.parent_model].link if l.name == self.parent_link)
                child = next(l.id for l in models[name].link if l.name == 'brick_link')
            except (KeyError, StopIteration) as exc:
                raise GazeboCommandError(f'cannot resolve attachment links for {name}') from exc
            self._link_ids[name] = (models[self.parent_model].id, parent, child)
        return self._link_ids[name]

    def _joint_readback(self, name):
        _, parent, child = self._resolve_links(name)
        snapshot = self._request('state', Empty(), SerializedStepMap)
        if parent not in snapshot.state.entities or child not in snapshot.state.entities:
            raise GazeboCommandError(f'attachment link disappeared for {name}')
        matching = {eid: data for eid, data in detachable_joints(snapshot).items() if data[1] == child}
        if any(data != (parent, child, 'fixed') for data in matching.values()) or len(matching) > 1:
            raise GazeboCommandError(f'unexpected physical joint for {name}: {matching}')
        stamp = snapshot.stats.sim_time.sec + snapshot.stats.sim_time.nsec * 1e-9
        return next(iter(matching), None), stamp

    def _wait_for_joint(self, name, attached, timeout_sim=10.0, stalled_wall=30.0):
        """Confirm ECM state; retry idempotent requests and bound a stalled clock."""
        start_wall = time.monotonic()
        start_sim = previous_sim = self._sim_time
        action = 'attach' if attached else 'detach'
        last_error = None
        while True:
            self._check_operation()
            now = time.monotonic()
            sim_now = self._sim_time
            if sim_now is not None:
                if start_sim is None:
                    start_sim = sim_now
                if previous_sim is not None and sim_now < previous_sim - 0.001:
                    raise GazeboCommandError(f'Gazebo clock reset during {action} of {name}')
                previous_sim = sim_now
                if sim_now - start_sim > timeout_sim:
                    raise GazeboCommandError(f'{action} of {name} unconfirmed after {timeout_sim} simulation seconds; last readback={last_error}')
            if now - max(start_wall, self._clock_progress_wall) > stalled_wall:
                raise GazeboCommandError(f'Gazebo clock unavailable or stopped for {stalled_wall} real seconds during {action} of {name}; last readback={last_error}')
            try:
                entity_id, stamp = self._joint_readback(name)
                self._check_operation()
                if (entity_id is not None) == attached:
                    state = 'attached' if attached else 'detached'
                    self._states[name] = state
                    self.attachment_checks[name] = {
                        'state': state, 'joint_entity': entity_id,
                        'parent_link': self._link_ids[name][1], 'child_link': self._link_ids[name][2],
                        'simulation_stamp': stamp, 'verification': 'Gazebo ECM readback',
                        'notification': self._notifications.get(name),
                    }
                    return
                last_error = f'joint_entity={entity_id}'
            except GazeboCommandError as exc:
                last_error = str(exc)
            # Native requests are idempotent. Do not reload a second plugin.
            self._publishers[name, action].publish(Empty())
            with self._condition:
                self._condition.wait(0.1)

    def pose(self, name):
        return self.observation(name)[0]

    def observation(self, name, after_sequence=None):
        # Consolidated members are fixed, verified model geometry, not a fresh
        # measurement of an independently simulated brick. Callers label these
        # records separately from dynamic observations.
        if name in self.placed:
            return self.placed[name], self._static_generation, self._sim_time
        self._wait(lambda: name in self._poses
                   and time.monotonic() - self._pose_time[name][1] < 5.0
                   and (after_sequence is None or self._pose_sequence[name] > after_sequence),
                   f"fresh pose of {name}")
        with self._condition:
            return self._poses[name], self._pose_sequence[name], self._pose_time[name][0]

    @bounded_operation()
    def spawn(self, name: str, pose: Pose) -> None:
        if name in self.models():
            raise GazeboCommandError(f'cannot spawn duplicate brick {name}')
        model_sdf = str(Path(self.brick_models.get(name, self.brick_sdf)).resolve())
        if getattr(self, 'static_supply', False):
            pose = pickup_pose_on_platform(pose, model_sdf)
        x, y, z = pose.position
        qx, qy, qz, qw = pose.orientation_xyzw
        if getattr(self, 'static_supply', False):
            root = ET.parse(model_sdf).getroot()
            model = root.find('model')
            static = model.find('static')
            if static is None:
                static = ET.SubElement(model, 'static')
            static.text = 'true'
            request = EntityFactory(sdf=ET.tostring(root, encoding='unicode'), name=name, allow_renaming=False)
        else:
            request = EntityFactory(sdf_filename=model_sdf, name=name, allow_renaming=False)
        request.pose.position.x, request.pose.position.y, request.pose.position.z = x, y, z
        request.pose.orientation.x, request.pose.orientation.y = qx, qy
        request.pose.orientation.z, request.pose.orientation.w = qz, qw
        self._create_once(request)
        self._confirm(lambda: self._supply_readback(name, pose, bool(getattr(self, 'static_supply', False))),
                      f'new supply body {name}')

    def _supply_readback(self, name, pose, static, old_id=None):
        scene = self._request('scene/info', Empty(), Scene)
        model = next((m for m in scene.model if m.name == name and m.id != old_id), None)
        if model is None or len(model.link) == 0 or self._pose_ids.get(name) != model.id:
            return False
        self._verify_supply(model, name, pose, static)
        return model.id

    @bounded_operation()
    def activate(self, name: str) -> None:
        """Replace one fixed supply box with its dynamic full/half model."""
        if not self.static_supply:
            return
        if name in self.placed or self._states.get(name) == 'attached':
            raise GazeboCommandError(f'cannot activate placed or attached brick {name}')
        old_id = self.models().get(name)
        if old_id is None:
            raise GazeboCommandError(f'cannot activate missing supply brick {name}')
        pose = self.pose(name)
        self._remove_confirmed(name, old_id)
        with self._condition:
            for cache in (self._poses, self._pose_sequence, self._pose_time, self._pose_ids, self._link_ids):
                cache.pop(name, None)
        model_sdf = str(Path(self.brick_models.get(name, self.brick_sdf)).resolve())
        request = EntityFactory(sdf_filename=model_sdf, name=name, allow_renaming=False)
        request.pose.position.x, request.pose.position.y, request.pose.position.z = pose.position
        (request.pose.orientation.x, request.pose.orientation.y,
         request.pose.orientation.z, request.pose.orientation.w) = pose.orientation_xyzw
        uncertain = self._create_once(request)
        new_id = self._confirm(lambda: self._supply_readback(name, pose, False, old_id),
                               f'dynamic supply replacement {name}')
        if uncertain is not None:
            if not hasattr(self, 'creation_checks'):
                self.creation_checks = {}
            self.creation_checks[name] = {'outcome': 'confirmed_after_timeout', 'entity_id': new_id,
                                          'verification': 'new ID, dynamic collision geometry and fresh pose'}

    def _verify_dynamic_supply(self, model, name, expected_pose):
        """Confirm the actual replacement, including collision size and pose."""
        return self._verify_supply(model, name, expected_pose, False)

    def _verify_supply(self, model, name, expected_pose, static):
        if model.id != self._pose_ids.get(name):
            raise GazeboCommandError(f'{name} has no fresh pose for its new entity ID')
        if len(model.link) != 1 or model.link[0].name != 'brick_link':
            raise GazeboCommandError(f'{name} has unexpected supply links')
        snapshot = self._request('state', Empty(), SerializedStepMap)
        if self._static_flag(model.id, snapshot) is not static:
            raise GazeboCommandError(f'{name} is not confirmed {"static" if static else "dynamic"}')
        link_id = model.link[0].id
        collisions = []
        for entity in snapshot.state.entities.values():
            components = {c.type: c.component for c in entity.components.values()}
            if (COLLISION_COMPONENT_TYPE not in components or
                    components.get(PARENT_ENTITY_COMPONENT_TYPE, b'').strip() != str(link_id).encode()):
                continue
            geometry = Geometry()
            try:
                geometry.ParseFromString(components.get(GEOMETRY_COMPONENT_TYPE, b''))
            except Exception as exc:
                raise GazeboCommandError(f'{name} has unreadable collision geometry') from exc
            if not geometry.HasField('box'):
                raise GazeboCommandError(f'{name} does not have box collision geometry')
            collisions.append((geometry.box.size.x, geometry.box.size.y, geometry.box.size.z))
        expected_size = self._brick_geometry(name)[2]
        if len(collisions) != 1 or any(abs(a-b) > 1e-9 for a,b in zip(collisions[0], expected_size)):
            raise GazeboCommandError(f'{name} collision dimensions do not match its selected full/half model')
        actual = self.pose(name)
        dot = abs(sum(a*b for a,b in zip(actual.orientation_xyzw, expected_pose.orientation_xyzw)))
        if math.dist(actual.position, expected_pose.position) > 0.005 or dot < math.cos(math.radians(2)/2):
            raise GazeboCommandError(f'{name} dynamic replacement does not match its pickup pose')

    @bounded_operation()
    def set_pose(self, name: str, pose: Pose) -> None:
        """Relocate an existing model without changing its collisions or dynamics."""
        entity_id = self.models().get(name)
        if entity_id is None:
            raise GazeboCommandError(f'cannot move missing model {name}')
        before = self._pose_sequence.get(name, 0)
        request = GazeboPose(id=entity_id, name=name)
        request.position.x, request.position.y, request.position.z = pose.position
        (request.orientation.x, request.orientation.y,
         request.orientation.z, request.orientation.w) = pose.orientation_xyzw
        self._send_once("set_pose", request)
        def positioned():
            if self._pose_ids.get(name) != entity_id or self._pose_sequence.get(name, 0) <= before:
                return False
            actual = self.pose(name)
            match = abs(sum(a*b for a,b in zip(actual.orientation_xyzw, pose.orientation_xyzw)))
            return math.dist(actual.position, pose.position) <= .005 and match >= math.cos(math.radians(2)/2)
        self._confirm(positioned, f'pose update of {name} entity {entity_id}')

    def _brick_geometry(self, name):
        path = Path(self.brick_models.get(name, self.brick_sdf)).resolve()
        cache = getattr(self, '_brick_geometry_cache', {})
        if path in cache:
            return cache[path]
        link = ET.parse(path).getroot().find('./model/link')
        if link is None:
            raise GazeboCommandError(f'brick model has no link: {path}')
        size = link.find('./collision/geometry/box/size')
        if size is None:
            raise GazeboCommandError(f'brick model must have box collision geometry: {path}')
        dimensions = tuple(float(value) for value in size.text.split())
        cache[path] = (path, link, dimensions)
        self._brick_geometry_cache = cache
        return cache[path]

    def static_wall_manifest(self):
        """Export fixed assembly geometry for a separate dynamic assessment."""
        return [dict(name=name, position=pose.position,
                     orientation_xyzw=pose.orientation_xyzw,
                     model_sdf=str(self._brick_geometry(name)[0]),
                     dimensions_m=self._brick_geometry(name)[2])
                for name, pose in self.placed.items()]

    def _static_sdf(self, model_name, placements):
        root = ET.Element('sdf', version='1.9')
        model = ET.SubElement(root, 'model', name=model_name)
        ET.SubElement(model, 'static').text = 'true'
        for name, pose in placements.items():
            _, source, _ = self._brick_geometry(name)
            link = ET.SubElement(model, 'link', name=name)
            x, y, z, w = pose.orientation_xyzw
            roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
            pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
            yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
            ET.SubElement(link, 'pose').text = ' '.join(
                format(value, '.17g') for value in (*pose.position, roll, pitch, yaw))
            # Keep collision materials and visual colours from the selected
            # full/half model. Static members need no dynamic mass/inertia.
            for element in source:
                if element.tag in ('collision', 'visual'):
                    link.append(copy.deepcopy(element))
        return ET.tostring(root, encoding='unicode')

    def consolidate(self, name: str, pose: Pose) -> None:
        """Replace one released dynamic brick with fixed wall geometry."""
        self.prefill_static([(name, pose)])

    @bounded_operation(120.0)
    def prefill_static(self, placements) -> None:
        """Consolidate members while retaining the previous fixed support model.

        Only explicitly supplied brick entities are removed. The old static
        model remains until the replacement model is read back successfully.
        Failure is explicit; it never records a successful placement.
        """
        additions = dict(placements)
        if not additions:
            return
        for name in additions:
            if not name.startswith('pickup_brick_'):
                raise GazeboCommandError(f'cannot consolidate non-brick entity {name}')
            if self._states.get(name) == 'attached':
                raise GazeboCommandError(f'{name} must be detached before consolidation')
        combined = dict(self.placed)
        combined.update(additions)
        # Validate all source models before removing any entity.
        generation = self._static_generation + 1
        candidate = f'placed_structure_{generation:04d}'
        sdf = self._static_sdf(candidate, combined)
        existing = self.models()
        if candidate in existing:
            raise GazeboCommandError(f'{candidate} already exists; use a fresh world')
        for name in additions:
            if name in existing:
                self.delete(name)
        # Full walls exceed Linux's per-argument size limit when sent as an
        # inline --req XML string. Keep one persistent file per adapter. The
        # previous creation has been read back before this path is reused.
        path = getattr(self, '_static_sdf_path', None)
        if path is None:
            path = Path(tempfile.gettempdir()) / f'irb4600_wall_{os.getpid()}_{id(self)}.sdf'
            self._static_sdf_path = path
        path.write_text(sdf, encoding='utf-8')
        self._create_once(EntityFactory(sdf_filename=str(path), name=candidate, allow_renaming=False))
        def complete_model():
            scene = self._request('scene/info', Empty(), Scene)
            actual = next((model for model in scene.model if model.name == candidate), None)
            # SceneBroadcaster can expose the model before its full link list.
            # Creation is confirmed only after every expected member appears.
            if actual is not None and {link.name for link in actual.link} == set(combined):
                if not self._is_static(actual.id):
                    raise GazeboCommandError(f'wall replacement {candidate} is not static')
                return actual
            return False
        actual = self._confirm(complete_model, f'complete static wall {candidate}', timeout=60.0)
        old = self.static_model_name
        if old is not None:
            self._remove_confirmed(old)
        self.placed = combined
        self.static_model_name = candidate
        self._static_generation = generation
        with self._condition:
            for name in additions:
                self._poses.pop(name, None)
                self._pose_sequence.pop(name, None)
                self._pose_time.pop(name, None)
                self._pose_ids.pop(name, None)
                self._link_ids.pop(name, None)
                self._states[name] = 'consolidated'

    @bounded_operation()
    def attach(self, name: str) -> None:
        topic = f"/assembly/{name}"
        if not hasattr(self, '_pending_plugins'):
            self._pending_plugins = set()
        if name not in self._plugins and name not in self._pending_plugins:
            def callback(message):
                with self._condition:
                    self._notifications[name] = message.data
                    self._condition.notify_all()
            self.node.subscribe(StringMsg, topic + "/state", callback)
            for action in ("attach", "detach"):
                self._publishers[name, action] = self.node.advertise(topic + "/" + action, Empty)
            request = EntityPlugin_V()
            request.entity.id = self._resolve_links(name)[0]
            request.entity.type = Entity.MODEL
            plugin = request.plugins.add()
            plugin.filename = "gz-sim-detachable-joint-system"
            plugin.name = "gz::sim::systems::DetachableJoint"
            plugin.innerxml = (f"<parent_link>{self.parent_link}</parent_link>"
                               f"<child_model>{name}</child_model><child_link>brick_link</child_link>"
                               f"<attach_topic>{topic}/attach</attach_topic>"
                               f"<detach_topic>{topic}/detach</detach_topic>"
                               f"<output_topic>{topic}/state</output_topic>")
            # Loading at grasp time intentionally creates the initial attachment.
            self._pending_plugins.add(name)
            try:
                self._send_once("entity/system/add", request)
            except GazeboCommandError:
                self._pending_plugins.discard(name)
                raise
        self._wait_for_joint(name, attached=True)
        self._plugins.add(name)
        self._pending_plugins.discard(name)

    def detach(self, name: str) -> None:
        if name not in self._plugins:
            raise GazeboCommandError(f"no attachment registered for {name}")
        self._wait_for_joint(name, attached=False)

    @bounded_operation()
    def delete(self, name: str) -> None:
        if self._states.get(name) == "attached":
            self.detach(name)
        self._remove_confirmed(name)
