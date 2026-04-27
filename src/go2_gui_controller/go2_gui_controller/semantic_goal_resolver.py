from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, isfinite, radians, sin

from .semantic_object_registry import SemanticObject


class SemanticGoalError(ValueError):
    pass


@dataclass(frozen=True)
class SemanticGoal:
    object_id: str
    relation: str
    frame_id: str
    x: float
    y: float
    yaw_rad: float


@dataclass(frozen=True)
class SemanticGoalResolver:
    approach_distance_m: float = 1.0
    min_approach_distance_m: float = 0.6
    max_approach_distance_m: float = 1.5
    object_margin_m: float = 0.2

    def resolve(
        self,
        semantic_object: SemanticObject,
        relation: str = "near",
        *,
        current_pose: tuple[float, float] | None = None,
    ) -> SemanticGoal:
        relation_key = _normalize_relation(relation)
        heading = self._resolve_heading_rad(semantic_object, current_pose=current_pose)
        distance = self.approach_distance_m

        if relation_key in ("near", "front"):
            dx = cos(heading) * distance
            dy = sin(heading) * distance
        elif relation_key == "back":
            dx = -cos(heading) * distance
            dy = -sin(heading) * distance
        elif relation_key == "left":
            dx = -sin(heading) * distance
            dy = cos(heading) * distance
        elif relation_key == "right":
            dx = sin(heading) * distance
            dy = -cos(heading) * distance
        else:
            raise SemanticGoalError(f"unknown_semantic_relation:{relation}")

        goal_x = semantic_object.x + dx
        goal_y = semantic_object.y + dy
        goal_yaw = atan2(semantic_object.y - goal_y, semantic_object.x - goal_x)
        goal = SemanticGoal(
            object_id=semantic_object.object_id,
            relation=relation_key,
            frame_id=semantic_object.frame_id,
            x=goal_x,
            y=goal_y,
            yaw_rad=goal_yaw,
        )
        self.validate(semantic_object, goal)
        return goal

    def validate(self, semantic_object: SemanticObject, goal: SemanticGoal) -> None:
        if goal.frame_id != "map":
            raise SemanticGoalError("invalid_semantic_goal:frame_id")
        if not all(isfinite(value) for value in (goal.x, goal.y, goal.yaw_rad)):
            raise SemanticGoalError("invalid_semantic_goal:non_finite")
        if not (self.min_approach_distance_m <= self.approach_distance_m <= self.max_approach_distance_m):
            raise SemanticGoalError("invalid_semantic_goal:approach_distance")
        if semantic_object.radius_m is not None:
            dx = goal.x - semantic_object.x
            dy = goal.y - semantic_object.y
            distance = (dx * dx + dy * dy) ** 0.5
            if distance < semantic_object.radius_m + self.object_margin_m:
                raise SemanticGoalError("invalid_semantic_goal:inside_object_margin")

    def _resolve_heading_rad(
        self,
        semantic_object: SemanticObject,
        *,
        current_pose: tuple[float, float] | None,
    ) -> float:
        if semantic_object.yaw_deg is not None:
            return radians(semantic_object.yaw_deg)
        if semantic_object.approach_yaw_deg is not None:
            return radians(semantic_object.approach_yaw_deg)
        if semantic_object.observer_x is not None and semantic_object.observer_y is not None:
            return atan2(semantic_object.y - semantic_object.observer_y, semantic_object.x - semantic_object.observer_x)
        if current_pose is not None:
            return atan2(semantic_object.y - current_pose[1], semantic_object.x - current_pose[0])
        return 0.0


def _normalize_relation(relation: str) -> str:
    if relation in ("", "default", "near"):
        return "near"
    return relation
