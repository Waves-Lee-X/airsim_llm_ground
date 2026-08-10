"""Deterministic directory and ZIP evidence package export and verification."""

from __future__ import annotations

import hashlib
import json
import os
import re
import tempfile
import zipfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Literal

from pydantic import Field, field_validator, model_validator

from aeromind_apm_lite.common.contracts.models import StrictModel

from .lock import ExperimentLock, canonical_hash


EVIDENCE_SCHEMA_VERSION = "1.0"


def _as_utc(value: datetime) -> datetime:
    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("timestamp must include a timezone")
    return value.astimezone(timezone.utc)


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _json_bytes(payload: object) -> bytes:
    return (
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")


class EvidenceError(ValueError):
    pass


class EvidenceModel(StrictModel):
    schema_version: Literal["1.0"] = EVIDENCE_SCHEMA_VERSION


class EvidenceArtifact(EvidenceModel):
    name: str = Field(
        min_length=1,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9._-]*$",
    )
    role: str = Field(
        min_length=1,
        max_length=64,
        pattern=r"^[a-z][a-z0-9_-]*$",
    )
    relative_path: str = Field(min_length=1, max_length=256)
    media_type: str = Field(min_length=1, max_length=128)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    size_bytes: int = Field(ge=0)

    @field_validator("relative_path")
    @classmethod
    def path_is_safe(cls, value: str) -> str:
        path = PurePosixPath(value)
        if path.is_absolute() or ".." in path.parts or value.startswith(("/", "\\")):
            raise ValueError("artifact relative_path must stay inside the package")
        if path.parts[0] != "artifacts":
            raise ValueError("artifact relative_path must start with artifacts/")
        return value


class EvidenceManifest(EvidenceModel):
    experiment_id: str = Field(min_length=1, max_length=128)
    created_at_utc: datetime
    lock: ExperimentLock
    requirements: tuple[str, ...] = Field(min_length=1)
    applicability: tuple[str, ...] = Field(min_length=1)
    limitations: tuple[str, ...] = ()
    conclusion: str = Field(min_length=1, max_length=4_096)
    artifacts: tuple[EvidenceArtifact, ...] = Field(min_length=1)

    _created_at_is_utc = field_validator("created_at_utc")(_as_utc)

    @model_validator(mode="after")
    def artifact_names_and_paths_are_unique(self) -> "EvidenceManifest":
        names = [item.name for item in self.artifacts]
        paths = [item.relative_path for item in self.artifacts]
        if len(names) != len(set(names)):
            raise ValueError("evidence artifact names must be unique")
        if len(paths) != len(set(paths)):
            raise ValueError("evidence artifact paths must be unique")
        return self


class EvidencePackage(EvidenceModel):
    package_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    manifest: EvidenceManifest

    @model_validator(mode="after")
    def package_hash_matches_manifest(self) -> "EvidencePackage":
        expected = canonical_hash(self.manifest.model_dump(mode="json"))
        if self.package_hash != expected:
            raise ValueError("evidence package hash mismatch")
        return self


@dataclass(frozen=True)
class _ArtifactPayload:
    name: str
    role: str
    relative_path: str
    media_type: str
    payload: bytes


class EvidencePackageBuilder:
    def __init__(
        self,
        *,
        experiment_id: str,
        created_at_utc: datetime,
        lock: ExperimentLock,
        requirements: tuple[str, ...] | list[str],
        applicability: tuple[str, ...] | list[str],
        limitations: tuple[str, ...] | list[str],
        conclusion: str,
    ) -> None:
        self.experiment_id = experiment_id
        self.created_at_utc = _as_utc(created_at_utc)
        self.lock = lock
        self.requirements = tuple(requirements)
        self.applicability = tuple(applicability)
        self.limitations = tuple(limitations)
        self.conclusion = conclusion
        self._artifacts: dict[str, _ArtifactPayload] = {}

    def add_json(self, name: str, payload: object, *, role: str) -> None:
        self.add_bytes(
            name,
            _json_bytes(payload),
            role=role,
            media_type="application/json",
            suffix=".json",
        )

    def add_bytes(
        self,
        name: str,
        payload: bytes,
        *,
        role: str,
        media_type: str = "application/octet-stream",
        suffix: str | None = None,
    ) -> None:
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}", name):
            raise EvidenceError("invalid evidence artifact name")
        if name in self._artifacts:
            raise EvidenceError(f"duplicate evidence artifact name: {name}")
        if suffix is None:
            suffix = ".txt" if media_type.startswith("text/") else ".bin"
        if not re.fullmatch(r"\.[A-Za-z0-9]{1,16}", suffix):
            raise EvidenceError("invalid evidence artifact suffix")
        self._artifacts[name] = _ArtifactPayload(
            name=name,
            role=role,
            relative_path=f"artifacts/{name}{suffix}",
            media_type=media_type,
            payload=bytes(payload),
        )

    def add_file(
        self,
        name: str,
        path: str | Path,
        *,
        role: str,
        media_type: str = "application/octet-stream",
    ) -> None:
        source = Path(path)
        try:
            payload = source.read_bytes()
        except OSError as exc:
            raise EvidenceError(f"failed to read artifact {source}: {exc}") from exc
        self.add_bytes(
            name,
            payload,
            role=role,
            media_type=media_type,
            suffix=source.suffix or None,
        )

    def build(self) -> tuple[EvidencePackage, dict[str, bytes]]:
        if not self._artifacts:
            raise EvidenceError("evidence package requires at least one artifact")
        ordered = [self._artifacts[name] for name in sorted(self._artifacts)]
        descriptors = tuple(
            EvidenceArtifact(
                name=item.name,
                role=item.role,
                relative_path=item.relative_path,
                media_type=item.media_type,
                sha256=_sha256(item.payload),
                size_bytes=len(item.payload),
            )
            for item in ordered
        )
        manifest = EvidenceManifest(
            experiment_id=self.experiment_id,
            created_at_utc=self.created_at_utc,
            lock=self.lock,
            requirements=self.requirements,
            applicability=self.applicability,
            limitations=self.limitations,
            conclusion=self.conclusion,
            artifacts=descriptors,
        )
        package = EvidencePackage(
            package_hash=canonical_hash(manifest.model_dump(mode="json")),
            manifest=manifest,
        )
        files = {item.relative_path: item.payload for item in ordered}
        files["manifest.json"] = _json_bytes(package.model_dump(mode="json"))
        return package, files

    def export(self, output: str | Path) -> EvidencePackage:
        destination = Path(output)
        if destination.exists():
            raise EvidenceError(f"evidence destination already exists: {destination}")
        package, files = self.build()
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.suffix.lower() == ".zip":
            _export_zip(destination, files)
        else:
            _export_directory(destination, files)
        return package


def _export_directory(destination: Path, files: dict[str, bytes]) -> None:
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{destination.name}.", dir=destination.parent)
    )
    try:
        for relative_path, payload in sorted(files.items()):
            path = temporary / PurePosixPath(relative_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
        os.replace(temporary, destination)
    except OSError as exc:
        raise EvidenceError(f"failed to export evidence directory: {exc}") from exc
    finally:
        if temporary.exists():
            for path in sorted(temporary.rglob("*"), reverse=True):
                if path.is_file():
                    path.unlink()
                elif path.is_dir():
                    path.rmdir()
            temporary.rmdir()


def _export_zip(destination: Path, files: dict[str, bytes]) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        with zipfile.ZipFile(temporary, "w") as archive:
            for relative_path, payload in sorted(files.items()):
                info = zipfile.ZipInfo(relative_path, date_time=(1980, 1, 1, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                info.create_system = 3
                info.external_attr = 0o100644 << 16
                archive.writestr(info, payload)
        os.replace(temporary, destination)
    except OSError as exc:
        raise EvidenceError(f"failed to export evidence zip: {exc}") from exc
    finally:
        if temporary.exists():
            temporary.unlink()


def verify_evidence_package(path: str | Path) -> EvidencePackage:
    source = Path(path)
    try:
        if source.is_dir():
            files = {
                item.relative_to(source).as_posix(): item.read_bytes()
                for item in source.rglob("*")
                if item.is_file()
            }
        else:
            with zipfile.ZipFile(source, "r") as archive:
                names = archive.namelist()
                if len(names) != len(set(names)):
                    raise EvidenceError("evidence archive contains duplicate paths")
                files = {name: archive.read(name) for name in names}
        package = EvidencePackage.model_validate(json.loads(files["manifest.json"]))
    except EvidenceError:
        raise
    except (OSError, KeyError, ValueError, zipfile.BadZipFile, json.JSONDecodeError) as exc:
        raise EvidenceError(f"failed to read evidence package {source}: {exc}") from exc

    expected_paths = {"manifest.json"}
    for artifact in package.manifest.artifacts:
        expected_paths.add(artifact.relative_path)
        payload = files.get(artifact.relative_path)
        if payload is None:
            raise EvidenceError(f"missing evidence artifact: {artifact.relative_path}")
        if _sha256(payload) != artifact.sha256:
            raise EvidenceError(f"hash mismatch for evidence artifact {artifact.name}")
        if len(payload) != artifact.size_bytes:
            raise EvidenceError(f"size mismatch for evidence artifact {artifact.name}")
    extras = set(files) - expected_paths
    if extras:
        raise EvidenceError("unindexed evidence files: " + ", ".join(sorted(extras)))
    return package


__all__ = [
    "EVIDENCE_SCHEMA_VERSION",
    "EvidenceArtifact",
    "EvidenceError",
    "EvidenceManifest",
    "EvidencePackage",
    "EvidencePackageBuilder",
    "verify_evidence_package",
]
