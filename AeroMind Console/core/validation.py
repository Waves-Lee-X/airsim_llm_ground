from __future__ import annotations

from typing import Any, Optional
from pydantic import BaseModel, Field, ValidationError, field_validator, model_validator
from pydantic.types import PositiveFloat, PositiveInt
from core.task_schema import MissionArea


class WaypointModel(BaseModel):
    x: float = Field(..., ge=-120.0, le=120.0, description="X coordinate in meters")
    y: float = Field(..., ge=-120.0, le=120.0, description="Y coordinate in meters")
    z: float = Field(..., ge=-30.0, le=0.0, description="Z coordinate (NED) in meters")


class MissionAreaModel(BaseModel):
    x_min: float = Field(0.0, ge=-120.0, le=120.0)
    x_max: float = Field(60.0, ge=-120.0, le=120.0)
    y_min: float = Field(-20.0, ge=-120.0, le=120.0)
    y_max: float = Field(20.0, ge=-120.0, le=120.0)

    @model_validator(mode='after')
    def check_area_validity(self) -> 'MissionAreaModel':
        if self.x_min >= self.x_max:
            raise ValueError("x_min must be less than x_max")
        if self.y_min >= self.y_max:
            raise ValueError("y_min must be less than y_max")
        return self

    def to_mission_area(self) -> MissionArea:
        return MissionArea(
            x_min=self.x_min,
            x_max=self.x_max,
            y_min=self.y_min,
            y_max=self.y_max
        )


class TakeoffParams(BaseModel):
    altitude_m: PositiveFloat = Field(8.0, ge=1.0, le=30.0, description="Target altitude in meters")


class GotoLocalParams(BaseModel):
    x: float = Field(..., ge=-120.0, le=120.0)
    y: float = Field(..., ge=-120.0, le=120.0)
    z: float = Field(..., ge=-30.0, le=0.0)
    speed_mps: PositiveFloat = Field(2.0, ge=0.5, le=4.0)
    face_target: bool = True


class WaypointRouteParams(BaseModel):
    waypoints: list[WaypointModel] = Field(..., min_length=1, max_length=40)
    speed_mps: PositiveFloat = Field(2.0, ge=0.5, le=4.0)
    avoidance: bool = True
    hold_at_end: bool = True


class AutonomousNavParams(BaseModel):
    x: float = Field(..., ge=-120.0, le=120.0)
    y: float = Field(..., ge=-120.0, le=120.0)
    z: float = Field(..., ge=-30.0, le=0.0)
    speed_mps: PositiveFloat = Field(1.5, ge=0.3, le=4.0)
    replan_interval_s: PositiveFloat = Field(1.0, ge=0.3, le=5.0)
    control_dt_s: PositiveFloat = Field(0.2, ge=0.1, le=1.0)
    lookahead_m: PositiveFloat = Field(3.0, ge=1.0, le=10.0)
    timeout_s: PositiveFloat = Field(120.0, ge=5.0, le=900.0)


class FormationFlightParams(BaseModel):
    count: PositiveInt = Field(3, ge=2, le=6)
    shape: str = Field('v', pattern=r'^(v|line|delta)$')
    distance_m: PositiveFloat = Field(40.0, ge=5.0, le=200.0)
    altitude_m: PositiveFloat = Field(8.0, ge=1.0, le=30.0)
    spacing_m: PositiveFloat = Field(6.0, ge=2.0, le=20.0)
    speed_mps: PositiveFloat = Field(2.0, ge=0.5, le=4.0)
    avoidance: bool = True


class RecoverFromCollisionParams(BaseModel):
    climb_m: PositiveFloat = Field(8.0, ge=2.0, le=15.0)
    backoff_m: PositiveFloat = Field(10.0, ge=2.0, le=20.0)
    force_relocate: bool = True


class SearchAreaParams(BaseModel):
    target: str = Field('object', min_length=1)
    area: MissionAreaModel
    altitude_m: PositiveFloat = Field(8.0, ge=1.0, le=30.0)
    spacing_m: PositiveFloat = Field(10.0, ge=2.0, le=30.0)
    speed_mps: PositiveFloat = Field(2.0, ge=0.5, le=4.0)
    avoidance: bool = True
    planned_avoidance: bool = True
    obstacle_distance_m: PositiveFloat = Field(8.0, ge=3.0, le=20.0)
    avoidance_offset_m: PositiveFloat = Field(6.0, ge=2.0, le=15.0)
    scan_margin_m: float = Field(4.0, ge=0.0, le=12.0)
    pre_scan: bool = True
    pre_scan_stop_on_high_risk: bool = True
    strategy: str = Field('lawnmower', pattern=r'^(lawnmower|spiral|strip)$')


class CollectImagesParams(BaseModel):
    area: MissionAreaModel
    altitude_m: PositiveFloat = Field(8.0, ge=1.0, le=30.0)
    spacing_m: PositiveFloat = Field(8.0, ge=2.0, le=30.0)
    speed_mps: PositiveFloat = Field(1.5, ge=0.5, le=4.0)
    camera: str = Field('front_center', pattern=r'^(front_center|bottom_center|0|front_left|front_right)$')
    dataset_name: str = Field('collect', min_length=1, max_length=80)
    max_images: PositiveInt = Field(200, ge=1, le=5000)
    capture_interval_s: PositiveFloat = Field(1.0, ge=0.2, le=30.0)
    avoidance: bool = True
    planned_avoidance: bool = True
    obstacle_distance_m: PositiveFloat = Field(8.0, ge=3.0, le=20.0)
    avoidance_offset_m: PositiveFloat = Field(6.0, ge=2.0, le=15.0)
    scan_margin_m: float = Field(4.0, ge=0.0, le=12.0)
    pre_scan: bool = True
    pre_scan_stop_on_high_risk: bool = True


class DetectObjectsParams(BaseModel):
    target: str = Field('object', min_length=1)
    camera: str = Field('front_center', pattern=r'^(front_center|bottom_center|0|front_left|front_right)$')


class ReturnHomeParams(BaseModel):
    safe_altitude_m: PositiveFloat = Field(8.0, ge=1.0, le=30.0)
    confirmed: bool = False


class ValidationResult(BaseModel):
    ok: bool
    data: dict[str, Any] = {}
    errors: list[str] = []

    @classmethod
    def success(cls, data: dict[str, Any]) -> 'ValidationResult':
        return cls(ok=True, data=data)

    @classmethod
    def failure(cls, errors: list[str]) -> 'ValidationResult':
        return cls(ok=False, errors=errors)


class ToolValidator:
    _SCHEMAS: dict[str, type[BaseModel]] = {
        'takeoff': TakeoffParams,
        'goto_local': GotoLocalParams,
        'waypoint_route': WaypointRouteParams,
        'autonomous_nav': AutonomousNavParams,
        'formation_flight': FormationFlightParams,
        'recover_from_collision': RecoverFromCollisionParams,
        'search_area': SearchAreaParams,
        'collect_images': CollectImagesParams,
        'detect_objects': DetectObjectsParams,
        'return_home': ReturnHomeParams,
    }

    @classmethod
    def validate(cls, tool: str, args: dict[str, Any]) -> ValidationResult:
        schema = cls._SCHEMAS.get(tool)
        if schema is None:
            if tool in {'hover', 'stop', 'land'}:
                return ValidationResult.success({})
            return ValidationResult.failure([f"Unknown tool: {tool}"])

        try:
            validated = schema(**args)
            return ValidationResult.success(validated.model_dump())
        except ValidationError as exc:
            errors = []
            for error in exc.errors():
                field_name = '.'.join(str(p) for p in error['loc'])
                errors.append(f"{field_name}: {error['msg']}")
            return ValidationResult.failure(errors)
