"""Content hashes that lock one experiment to reproducible inputs."""

from __future__ import annotations

import hashlib
import json
from typing import Any, Literal

from pydantic import Field

from aeromind_apm_lite.common.contracts.models import StrictModel


def canonical_hash(payload: Any) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class ExperimentLock(StrictModel):
    schema_version: Literal["1.0"] = "1.0"
    code_commit: str = Field(min_length=1, max_length=128)
    software_versions: dict[str, str]
    model_versions: dict[str, str]
    algorithm_versions: dict[str, str]
    scenario_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    configuration_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    random_seeds: tuple[int, ...]

    @classmethod
    def from_inputs(
        cls,
        *,
        code_commit: str,
        software_versions: dict[str, str],
        model_versions: dict[str, str],
        algorithm_versions: dict[str, str],
        scenario: Any,
        configuration: Any,
        random_seeds: tuple[int, ...] | list[int],
    ) -> "ExperimentLock":
        return cls(
            code_commit=code_commit,
            software_versions=software_versions,
            model_versions=model_versions,
            algorithm_versions=algorithm_versions,
            scenario_hash=canonical_hash(scenario),
            configuration_hash=canonical_hash(configuration),
            random_seeds=tuple(random_seeds),
        )


__all__ = ["ExperimentLock", "canonical_hash"]
