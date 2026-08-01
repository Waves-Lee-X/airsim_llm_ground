"""Deterministic color cross-validation and persistent vision evidence.

The VLM never has flight authority: deterministic OpenCV color detection runs
on the same frame, the two results are cross-checked, several recent analyses
are voted into a consensus, and every record is persisted immutably so that
later acceptance tests can audit the evidence trail.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import tempfile
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal
from uuid import UUID, uuid4

from pydantic import BaseModel, ConfigDict, Field, field_validator

VISION_SCHEMA_VERSION = "1.0"

DEFAULT_VISION_EVIDENCE_DIR = Path.home() / ".aeromind" / "visual-evidence"


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _as_utc(value: datetime) -> datetime:
    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("timestamp must include a timezone")
    return value.astimezone(timezone.utc)


def _canonical_hash(payload: Any) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class VisionEvidenceError(ValueError):
    """Raised when vision evidence cannot be persisted or validated."""


class ColorUnavailable(RuntimeError):
    """Raised when OpenCV is not installed or the frame cannot be decoded."""


class VisionModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)


# ---------------------------------------------------------------------------
# Deterministic color detection
# ---------------------------------------------------------------------------

#: Canonical color names understood by the ground station.
CANONICAL_COLORS = (
    "red",
    "green",
    "blue",
    "yellow",
    "orange",
    "purple",
    "pink",
    "white",
    "black",
    "gray",
)

_COLOR_ALIASES: dict[str, str] = {
    "red": "red",
    "红色": "red",
    "红": "red",
    "green": "green",
    "绿色": "green",
    "绿": "green",
    "blue": "blue",
    "蓝色": "blue",
    "蓝": "blue",
    "yellow": "yellow",
    "黄色": "yellow",
    "黄": "yellow",
    "orange": "orange",
    "橙色": "orange",
    "橙": "orange",
    "purple": "purple",
    "紫色": "purple",
    "紫": "purple",
    "pink": "pink",
    "粉色": "pink",
    "粉": "pink",
    "white": "white",
    "白色": "white",
    "白": "white",
    "black": "black",
    "黑色": "black",
    "黑": "black",
    "gray": "gray",
    "grey": "gray",
    "灰颜色": "gray",
    "灰": "gray",
}

#: Single-character Chinese color words for compound descriptions (蓝白色).
_CHINESE_COLOR_CHARS: dict[str, str] = {
    "红": "red",
    "绿": "green",
    "蓝": "blue",
    "黄": "yellow",
    "橙": "orange",
    "紫": "purple",
    "粉": "pink",
    "白": "white",
    "黑": "black",
    "灰": "gray",
}

#: HSV ranges per canonical color: (h_lo, s_lo, v_lo, h_hi, s_hi, v_hi).
#: Red needs two ranges because hue wraps around 180 in OpenCV.
_COLOR_HSV_RANGES: dict[str, tuple[tuple[int, int, int, int, int, int], ...]] = {
    "red": ((0, 50, 50, 10, 255, 255), (170, 50, 50, 180, 255, 255)),
    "green": ((35, 40, 40, 85, 255, 255),),
    "blue": ((100, 40, 40, 130, 255, 255),),
    "yellow": ((20, 40, 40, 35, 255, 255),),
    "orange": ((8, 40, 40, 20, 255, 255),),
    "purple": ((125, 40, 40, 150, 255, 255),),
    "pink": ((150, 40, 40, 175, 255, 255),),
    "white": ((0, 0, 200, 180, 30, 255),),
    "black": ((0, 0, 0, 180, 255, 50),),
    "gray": ((0, 0, 50, 180, 30, 200),),
}


def normalize_color(value: Any) -> str | None:
    """Return the canonical color name for a VLM/English/Chinese label.

    Accepts single palette words (orange / 橙色), hyphen/space/and-separated
    compounds (blue-gray, blue and white, 蓝白色) and light/dark modifiers;
    the first canonical token wins for compound descriptions.  This keeps
    free-form VLM color text comparable with the deterministic detector.
    """
    if value is None:
        return None
    text = str(value).strip().lower()
    cleaned = re.sub(r"[^a-z\u4e00-\u9fff]", "", text)
    direct = _COLOR_ALIASES.get(cleaned)
    if direct is not None:
        return direct
    for part in re.split(r"[^a-z\u4e00-\u9fff]+", text):
        if not part or part in {"light", "dark", "deep", "bright", "pale"}:
            continue
        alias = _COLOR_ALIASES.get(part)
        if alias is not None:
            return alias
    for name in CANONICAL_COLORS:
        if name in cleaned:
            return name
    for char, name in _CHINESE_COLOR_CHARS.items():
        if char in cleaned:
            return name
    return None


class ColorDetection(VisionModel):
    """One deterministic OpenCV HSV detection on a single camera frame."""

    color: Literal[tuple(CANONICAL_COLORS)]
    coverage_ratio: float = Field(ge=0.0, le=1.0)
    area_px: int = Field(ge=0)
    bbox: tuple[int, int, int, int] | None = Field(default=None)
    confidence: float = Field(ge=0.0, le=1.0)
    frame_sequence: int = Field(ge=0)
    detected_at_utc: datetime = Field(default_factory=utc_now)
    note: str = Field(default="", max_length=256)

    _detected_is_utc = field_validator("detected_at_utc")(_as_utc)

    def public_payload(self) -> dict[str, Any]:
        return self.model_dump(mode="json")


def _nearest_palette_color(hue: float) -> str | None:
    """Map a mean hue to the nearest palette hue range (circular hue)."""
    best_name: str | None = None
    best_distance = 1e9
    for name, ranges in _COLOR_HSV_RANGES.items():
        if name in {"white", "black", "gray"}:
            continue
        for h_lo, _s_lo, _v_lo, h_hi, _s_hi, _v_hi in ranges:
            lo, hi = float(h_lo), float(h_hi)
            if lo <= hue <= hi:
                return name
            mid = (lo + hi) / 2.0
            distance = min(abs(hue - mid), 180.0 - abs(hue - mid))
            if distance < best_distance:
                best_distance = distance
                best_name = name
    return best_name


class ColorDetector:
    """Run deterministic HSV color detection without any model authority."""

    _MAX_DECODE_DIMENSION = 960

    def __init__(
        self,
        *,
        min_coverage: float = 0.02,
        max_coverage: float = 0.95,
    ) -> None:
        if not 0.0 < min_coverage < max_coverage < 1.0:
            raise ValueError("require 0 < min_coverage < max_coverage < 1")
        self.min_coverage = min_coverage
        self.max_coverage = max_coverage

    @staticmethod
    def _classify_vivid_target(
        hsv: Any,
        region_total: int,
    ) -> tuple[str, float, tuple[int, int, int, int]] | None:
        """Classify the largest vivid blob's mean hue into the palette.

        The region around a VLM target is usually background-dominated, so a
        plain dominant-color vote would still return gray; the vivid blob (the
        actual object) is what the VLM described.
        """
        import cv2
        import numpy as np

        mask = cv2.inRange(hsv, (0, 80, 60), (179, 255, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))
        if area < 64.0:
            return None
        blob_mask = np.zeros_like(mask)
        cv2.drawContours(blob_mask, [largest], -1, 255, -1)
        mean_hue = float(cv2.mean(hsv[:, :, 0], mask=blob_mask)[0])
        mean_sat = float(cv2.mean(hsv[:, :, 1], mask=blob_mask)[0])
        if mean_sat < 60.0:
            return None
        color = _nearest_palette_color(mean_hue)
        if color is None:
            return None
        x, y, w, h = cv2.boundingRect(largest)
        coverage = min(1.0, area / region_total) if region_total > 0 else 0.0
        return color, coverage, (int(x), int(y), int(w), int(h))

    def detect(
        self,
        data: bytes,
        *,
        sequence: int,
        captured_at_utc: datetime,
        media_type: str = "image/jpeg",
        region_center: tuple[float, float] | None = None,
        region_size_fraction: float = 0.3,
    ) -> ColorDetection | None:
        """Return the dominant palette color, or None when not informative.

        ``region_center`` (normalized 0-1 pixel center) restricts the analysis
        to a patch around the detected target, so the measurement matches the
        object the VLM described instead of the whole (often background-
        dominated) frame.
        """
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover - depends on ground extra
            raise ColorUnavailable(
                "OpenCV 未安装，无法执行确定性颜色交叉验证"
            ) from exc
        if not data:
            return None
        import numpy as np

        encoded = np.frombuffer(data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            return ColorDetection(
                color="gray",
                coverage_ratio=0.0,
                area_px=0,
                frame_sequence=sequence,
                detected_at_utc=captured_at_utc,
                confidence=0.0,
                note="无法解码图像",
            )
        height, width = image.shape[:2]
        region_active = False
        if region_center is not None and len(region_center) == 2:
            cx, cy = float(region_center[0]), float(region_center[1])
            if 0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0:
                half = max(
                    4,
                    int(region_size_fraction * max(height, width) / 2.0),
                )
                x0 = max(0, int(cx * width) - half)
                y0 = max(0, int(cy * height) - half)
                x1 = min(width, int(cx * width) + half)
                y1 = min(height, int(cy * height) + half)
                if x1 - x0 >= 16 and y1 - y0 >= 16:
                    image = image[y0:y1, x0:x1]
                    height, width = image.shape[:2]
                    region_active = True
        scale = self._MAX_DECODE_DIMENSION / max(height, width)
        if scale < 1.0:
            image = cv2.resize(
                image,
                (max(1, int(width * scale)), max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        total = image.shape[0] * image.shape[1]
        if total <= 0:
            return None
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        if region_active:
            vivid = self._classify_vivid_target(hsv, height * width)
            if vivid is not None:
                color, coverage, bbox = vivid
                return ColorDetection(
                    color=color,
                    coverage_ratio=round(coverage, 6),
                    area_px=int(round(coverage * height * width)),
                    bbox=bbox,
                    confidence=round(min(1.0, 0.5 + coverage / 0.35), 4),
                    frame_sequence=sequence,
                    detected_at_utc=captured_at_utc,
                )

        best: tuple[float, str, float, tuple[int, int, int, int] | None] = (
            0.0,
            "gray",
            0.0,
            None,
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        for color, ranges in _COLOR_HSV_RANGES.items():
            mask = None
            for h_lo, s_lo, v_lo, h_hi, s_hi, v_hi in ranges:
                lower = np.array([h_lo, s_lo, v_lo], dtype=np.uint8)
                upper = np.array([h_hi, s_hi, v_hi], dtype=np.uint8)
                part = cv2.inRange(hsv, lower, upper)
                mask = part if mask is None else cv2.bitwise_or(mask, part)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            if not contours:
                continue
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            region_pixels = int(cv2.countNonZero(mask[y : y + h, x : x + w]))
            if region_pixels <= 0:
                continue
            coverage = region_pixels / total
            if coverage < self.min_coverage:
                continue
            if coverage > self.max_coverage:
                continue
            confidence = round(min(1.0, 0.15 + coverage / 0.35), 4)
            if coverage > best[0]:
                best = (coverage, color, confidence, (int(x), int(y), int(w), int(h)))

        coverage, color, confidence, bbox = best
        if coverage <= 0.0:
            return None
        return ColorDetection(
            color=color,
            coverage_ratio=round(coverage, 6),
            area_px=int(round(coverage * total)),
            bbox=bbox,
            confidence=confidence,
            frame_sequence=sequence,
            detected_at_utc=captured_at_utc,
        )


def cross_validate_color(
    vlm_color: Any,
    detection: ColorDetection | None,
    *,
    vlm_analyzed: bool = True,
) -> dict[str, Any]:
    """Compare the VLM color claim with the deterministic detector result.

    ``vlm_analyzed`` distinguishes "the model has not run yet" from "the
    model ran but returned no usable color", so the browser can explain both.
    """
    normalized = normalize_color(vlm_color)
    if detection is None:
        return {
            "method": "opencv_hsv",
            "vlm_color": normalized,
            "opencv_color": None,
            "agreement": None,
            "status": "unavailable",
            "note": "OpenCV 颜色检测不可用或画面中无有效色块",
        }
    if normalized is None:
        agreement = None
        if vlm_analyzed:
            status = "vlm_color_missing"
            note = "VLM 已识别但未给出可归一化的颜色，无法交叉验证"
        else:
            status = "vlm_not_analyzed"
            note = "尚未进行 VLM 识别，请先点击“识别当前帧”"
    elif normalized == detection.color:
        agreement = True
        status = "agreed"
        note = f"VLM({normalized}) 与 OpenCV({detection.color}) 一致"
    else:
        agreement = False
        status = "disagreed"
        note = f"VLM({normalized}) 与 OpenCV({detection.color}) 不一致"
    return {
        "method": "opencv_hsv",
        "vlm_color": normalized,
        "opencv_color": detection.color,
        "agreement": agreement,
        "status": status,
        "note": note,
        "detection": detection.public_payload(),
    }

# ---------------------------------------------------------------------------
# Multi-frame consensus
# ---------------------------------------------------------------------------


class VisionConsensusTracker:
    """Vote recent visual analyses into a deterministic consensus window."""

    def __init__(self, window: int = 5, required: int = 3) -> None:
        if window < 1 or required < 1 or required > window:
            raise ValueError("consensus requires 1 <= required <= window")
        self.window = window
        self.required = required
        self._recent: list[dict[str, Any]] = []

    def append(self, payload: dict[str, Any]) -> dict[str, Any]:
        result = payload.get("result") or {}
        target = result.get("target") or {}
        entry = {
            "analyzed_at_utc": payload.get("analyzed_at_utc"),
            "frame_sequence": (payload.get("frame") or {}).get("sequence"),
            "found": bool(target.get("found")),
            "label": str(target.get("label") or "").strip() or None,
            "box_id": str(target.get("box_id") or "").strip() or None,
            "color": normalize_color(target.get("color")),
            "face": str(target.get("face") or "").strip() or None,
        }
        self._recent.append(entry)
        if len(self._recent) > self.window:
            self._recent = self._recent[-self.window :]
        return self.consensus_payload()

    def consensus_payload(self) -> dict[str, Any]:
        votes: dict[str, dict[str, int]] = {}
        best: dict[str, Any] = {}
        for field in ("found", "label", "box_id", "color", "face"):
            counts: dict[str, int] = {}
            for entry in self._recent:
                value = entry.get(field)
                if value is None:
                    continue
                if field == "found":
                    key = "true" if value else "false"
                else:
                    key = str(value)
                counts[key] = counts.get(key, 0) + 1
            votes[field] = dict(sorted(counts.items(), key=lambda item: (-item[1], item[0])))
            top, count = next(iter(votes[field].items()), (None, 0))
            if count:
                best[field] = (
                    (top == "true") if field == "found" and top is not None else top
                )
            else:
                best[field] = None
        color_counts = votes.get("color") or {}
        confirmed_color = next(iter(color_counts), None) if color_counts else None
        confirmed = bool(
            confirmed_color is not None
            and color_counts.get(confirmed_color, 0) >= self.required
            and len(self._recent) >= self.required
        )
        return {
            "window_size": len(self._recent),
            "required": self.required,
            "confirmed": confirmed,
            "confirmed_color": confirmed_color if confirmed else None,
            "votes": votes,
            "best": best,
            "samples": list(self._recent),
        }


# ---------------------------------------------------------------------------
# Immutable evidence persistence
# ---------------------------------------------------------------------------


class VisionFrameInfo(VisionModel):
    sequence: int = Field(ge=0)
    captured_at_utc: datetime
    media_type: str = Field(min_length=1, max_length=64)
    size_bytes: int = Field(ge=0)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")

    _captured_is_utc = field_validator("captured_at_utc")(_as_utc)


class VisionAnalysisEvidence(VisionModel):
    """One immutable record combining VLM output, cross-validation and consensus."""

    schema_version: Literal["1.0"] = VISION_SCHEMA_VERSION
    evidence_id: UUID = Field(default_factory=uuid4)
    vehicle_id: int = Field(ge=1, le=255)
    analyzed_at_utc: datetime = Field(default_factory=utc_now)
    frame: VisionFrameInfo
    vision_model: str = Field(min_length=1, max_length=128)
    prompt: str = Field(default="", max_length=4096)
    result: dict[str, Any]
    raw_response: str = Field(default="", max_length=1_000_000)
    cross_validation: dict[str, Any]
    consensus: dict[str, Any]
    producer_version: str = Field(min_length=1, max_length=128)
    preview_only: bool = True
    flight_command_generated: bool = False

    _analyzed_is_utc = field_validator("analyzed_at_utc")(_as_utc)

    @property
    def evidence_hash(self) -> str:
        return _canonical_hash(self.model_dump(mode="json"))

    def public_payload(self) -> dict[str, Any]:
        return {
            "evidence_hash": self.evidence_hash,
            "evidence": self.model_dump(mode="json"),
        }


def _atomic_json_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
            json.dump(payload, temporary, ensure_ascii=False, indent=2, sort_keys=True)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        os.replace(temporary_path, path)
    except OSError as exc:
        raise VisionEvidenceError(f"failed to write {path}: {exc}") from exc
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def _atomic_bytes_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
            temporary.write(data)
            temporary.flush()
            os.fsync(temporary.fileno())
        os.replace(temporary_path, path)
    except OSError as exc:
        raise VisionEvidenceError(f"failed to write {path}: {exc}") from exc
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


class VisionEvidenceStore:
    """Atomically persist immutable vision evidence by evidence UUID."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self._lock = threading.RLock()

    def _path(self, evidence_id: UUID) -> Path:
        return self.root / f"{evidence_id}.json"

    def _frame_path(self, evidence_id: UUID) -> Path:
        return self.root / f"{evidence_id}.frame"

    def save(
        self,
        evidence: VisionAnalysisEvidence,
        frame_data: bytes | None = None,
    ) -> VisionAnalysisEvidence:
        with self._lock:
            path = self._path(evidence.evidence_id)
            if path.exists():
                existing = self.load(evidence.evidence_id)
                if existing.evidence_hash != evidence.evidence_hash:
                    raise VisionEvidenceError(
                        "evidence_id is immutable and already contains different data"
                    )
                return existing
            _atomic_json_write(path, evidence.public_payload())
            if frame_data is not None:
                if (
                    hashlib.sha256(frame_data).hexdigest()
                    != evidence.frame.sha256
                ):
                    raise VisionEvidenceError(
                        "frame snapshot does not match the evidence sha256"
                    )
                _atomic_bytes_write(self._frame_path(evidence.evidence_id), frame_data)
            return evidence

    def load(self, evidence_id: UUID | str) -> VisionAnalysisEvidence:
        with self._lock:
            try:
                parsed_id = UUID(str(evidence_id))
                payload = json.loads(self._path(parsed_id).read_text(encoding="utf-8"))
                evidence = VisionAnalysisEvidence.model_validate(
                    payload["evidence"]
                )
            except (OSError, ValueError, KeyError, TypeError) as exc:
                raise VisionEvidenceError(
                    f"failed to load vision evidence {evidence_id}: {exc}"
                ) from exc
            if payload.get("evidence_hash") != evidence.evidence_hash:
                raise VisionEvidenceError("vision evidence hash mismatch")
            return evidence

    def load_frame(self, evidence_id: UUID | str) -> bytes | None:
        with self._lock:
            parsed_id = UUID(str(evidence_id))
            path = self._frame_path(parsed_id)
            if not path.exists():
                return None
            try:
                return path.read_bytes()
            except OSError as exc:
                raise VisionEvidenceError(
                    f"failed to load vision evidence frame {evidence_id}: {exc}"
                ) from exc

    def list_summaries(self) -> list[dict[str, Any]]:
        if not self.root.is_dir():
            return []
        summaries = []
        for path in sorted(self.root.glob("*.json")):
            evidence = self.load(path.stem)
            cross = evidence.cross_validation or {}
            summaries.append(
                {
                    "evidence_id": str(evidence.evidence_id),
                    "evidence_hash": evidence.evidence_hash,
                    "vehicle_id": evidence.vehicle_id,
                    "analyzed_at_utc": evidence.analyzed_at_utc.isoformat(),
                    "frame_sequence": evidence.frame.sequence,
                    "vlm_color": cross.get("vlm_color"),
                    "opencv_color": cross.get("opencv_color"),
                    "agreement": cross.get("agreement"),
                    "confirmed": bool(evidence.consensus.get("confirmed")),
                    "preview_only": evidence.preview_only,
                    "has_frame": self._frame_path(evidence.evidence_id).exists(),
                }
            )
        return summaries
