"""Placement-input loading and brick lifecycle state machine.

The placement file is deliberately supplied at runtime.  The fixed pickup
layout remains a separate exported workcell asset and is paired by array order.
Gazebo operations are injected through a small adapter so this module can be
tested without starting Gazebo.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Protocol, Sequence

from .motion import Pose


class BrickState(str, Enum):
    AVAILABLE = "AVAILABLE"
    ATTACHED = "ATTACHED"
    TRANSPORTING = "TRANSPORTING"
    RELEASED = "RELEASED"
    SETTLING = "SETTLING"
    PLACED = "PLACED"
    CLEANED = "CLEANED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class Placement:
    index: int
    pose: Pose


def _pose(raw: object, index: int) -> Pose:
    if not isinstance(raw, dict):
        raise ValueError(f"placement {index} must be an object")
    try:
        return Pose(tuple(raw["position"]), tuple(raw["orientation"]))
    except KeyError as exc:
        raise ValueError(
            f"placement {index} requires position and orientation"
        ) from exc
    except (TypeError, ValueError) as exc:
        raise ValueError(f"placement {index} contains an invalid pose") from exc


def load_placements(path: str | Path) -> tuple[Placement, ...]:
    """Load any user-supplied placement JSON file."""

    source = Path(path)
    with source.open("r", encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict):
        raise ValueError("placement JSON must contain an object")
    # ``placements`` is the preferred name; ``brick_frames`` is accepted for
    # the existing GH/Rhino export convention.
    placements = document.get("placements", document.get("brick_frames"))
    if not isinstance(placements, list) or not placements:
        raise ValueError(
            'placement JSON requires a non-empty "placements" or "brick_frames" array'
        )
    return tuple(Placement(index, _pose(item, index)) for index, item in enumerate(placements))


def load_pickup_layout(path: str | Path) -> tuple[Pose, ...]:
    """Load the exported fixed Rhino/GH jig layout."""

    source = Path(path)
    with source.open("r", encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict):
        raise ValueError("pickup layout JSON must contain an object")
    bricks = document.get("pickup_bricks")
    if not isinstance(bricks, list) or not bricks:
        raise ValueError('pickup layout requires a non-empty "pickup_bricks" array')
    return tuple(_pose(item, index) for index, item in enumerate(bricks))


@dataclass(frozen=True)
class BrickPlan:
    name: str
    pickup: Pose
    placement: Pose
    state: BrickState = BrickState.AVAILABLE


def pair_pickups_with_placements(
    pickup_layout: Sequence[Pose], placements: Sequence[Placement]
) -> tuple[BrickPlan, ...]:
    if not pickup_layout:
        raise ValueError("pickup layout must contain at least one supply slot")
    # Pickup slots are reusable supply locations, not a wall-size limit.  A
    # later executor may replenish a slot after its brick is consumed.
    return tuple(
        BrickPlan(
            f"assembly_brick_{item.index:03d}",
            pickup_layout[item.index % len(pickup_layout)],
            item.pose,
        )
        for item in placements
    )


class BrickWorld(Protocol):
    """Gazebo adapter required by the lifecycle manager."""

    def spawn(self, name: str, pose: Pose) -> None: ...
    def attach(self, name: str) -> None: ...
    def detach(self, name: str) -> None: ...
    def delete(self, name: str) -> None: ...


class BrickLifecycle:
    def __init__(self, world: BrickWorld, plans: Sequence[BrickPlan]) -> None:
        self.world = world
        self._states = {plan.name: plan.state for plan in plans}
        self.plans = tuple(plans)

    def spawn_available(self, plan: BrickPlan) -> None:
        self._require(plan.name, BrickState.AVAILABLE)
        self.world.spawn(plan.name, plan.pickup)

    def attach(self, plan: BrickPlan) -> None:
        self._require(plan.name, BrickState.AVAILABLE)
        self.world.attach(plan.name)
        self._states[plan.name] = BrickState.ATTACHED

    def begin_transport(self, plan: BrickPlan) -> None:
        self._require(plan.name, BrickState.ATTACHED)
        self._states[plan.name] = BrickState.TRANSPORTING

    def release(self, plan: BrickPlan) -> None:
        self._require(plan.name, BrickState.TRANSPORTING)
        self.world.detach(plan.name)
        self._states[plan.name] = BrickState.RELEASED

    def begin_settling(self, plan: BrickPlan) -> None:
        self._require(plan.name, BrickState.RELEASED)
        self._states[plan.name] = BrickState.SETTLING

    def mark_placed(self, plan: BrickPlan) -> None:
        self._require(plan.name, BrickState.SETTLING)
        self._states[plan.name] = BrickState.PLACED

    def cleanup(self, plan: BrickPlan) -> None:
        if self._states[plan.name] is BrickState.CLEANED:
            return
        self.world.delete(plan.name)
        self._states[plan.name] = BrickState.CLEANED

    def state(self, plan: BrickPlan) -> BrickState:
        return self._states[plan.name]

    def _require(self, name: str, expected: BrickState) -> None:
        actual = self._states[name]
        if actual is not expected:
            raise RuntimeError(f"{name}: expected {expected.value}, got {actual.value}")
