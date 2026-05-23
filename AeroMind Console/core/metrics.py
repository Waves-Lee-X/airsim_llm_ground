from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass(frozen=True)
class MetricPoint:
    name: str
    value: float
    timestamp: float
    tags: Dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "value": self.value,
            "timestamp": self.timestamp,
            "tags": self.tags,
        }


@dataclass(frozen=True)
class LogEntry:
    level: str
    message: str
    timestamp: float
    module: str = ""
    function: str = ""
    line: int = 0
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "level": self.level,
            "message": self.message,
            "timestamp": self.timestamp,
            "module": self.module,
            "function": self.function,
            "line": self.line,
            "extra": self.extra,
        }


class MetricsCollector:
    def __init__(self, max_points: int = 1000) -> None:
        self._metrics: Dict[str, List[MetricPoint]] = {}
        self._max_points = max_points
        self._counters: Dict[str, int] = {}
        self._gauges: Dict[str, float] = {}
        self._timers: Dict[str, List[float]] = {}
        self._start_time = time.time()

    def record_counter(self, name: str, value: int = 1) -> None:
        self._counters[name] = self._counters.get(name, 0) + value
        self._add_metric(name, float(self._counters[name]))

    def record_gauge(self, name: str, value: float) -> None:
        self._gauges[name] = value
        self._add_metric(name, value)

    def record_timer(self, name: str, duration_ms: float) -> None:
        if name not in self._timers:
            self._timers[name] = []
        self._timers[name].append(duration_ms)
        if len(self._timers[name]) > 100:
            self._timers[name] = self._timers[name][-100:]
        self._add_metric(f"{name}_duration", duration_ms)

    def record_histogram(self, name: str, value: float) -> None:
        self._add_metric(name, value)

    def _add_metric(self, name: str, value: float, tags: Dict[str, str] | None = None) -> None:
        point = MetricPoint(
            name=name,
            value=value,
            timestamp=time.time(),
            tags=tags or {},
        )
        if name not in self._metrics:
            self._metrics[name] = []
        self._metrics[name].append(point)
        if len(self._metrics[name]) > self._max_points:
            self._metrics[name] = self._metrics[name][-self._max_points:]

    def get_metrics(self, name: str | None = None) -> List[MetricPoint]:
        if name:
            return self._metrics.get(name, [])
        all_points: List[MetricPoint] = []
        for points in self._metrics.values():
            all_points.extend(points)
        return sorted(all_points, key=lambda p: p.timestamp)

    def get_summary(self) -> Dict[str, Any]:
        summary = {
            "uptime_s": round(time.time() - self._start_time, 1),
            "counters": dict(self._counters),
            "gauges": dict(self._gauges),
            "timers": {},
            "metric_count": sum(len(points) for points in self._metrics.values()),
        }
        for name, values in self._timers.items():
            if values:
                summary["timers"][name] = {
                    "count": len(values),
                    "min": round(min(values), 2),
                    "max": round(max(values), 2),
                    "avg": round(sum(values) / len(values), 2),
                }
        return summary

    def reset(self) -> None:
        self._metrics.clear()
        self._counters.clear()
        self._gauges.clear()
        self._timers.clear()
        self._start_time = time.time()


class StructuredLogger:
    def __init__(self, log_file: Path | None = None) -> None:
        self._log_file = log_file
        self._entries: List[LogEntry] = []
        self._max_entries = 500
        if self._log_file:
            self._log_file.parent.mkdir(parents=True, exist_ok=True)

    def _log(self, level: str, message: str, **kwargs: Any) -> None:
        entry = LogEntry(
            level=level,
            message=message,
            timestamp=time.time(),
            **kwargs,
        )
        self._entries.insert(0, entry)
        if len(self._entries) > self._max_entries:
            self._entries = self._entries[:self._max_entries]
        self._write_to_file(entry)

    def debug(self, message: str, **kwargs: Any) -> None:
        self._log("DEBUG", message, **kwargs)

    def info(self, message: str, **kwargs: Any) -> None:
        self._log("INFO", message, **kwargs)

    def warn(self, message: str, **kwargs: Any) -> None:
        self._log("WARN", message, **kwargs)

    def error(self, message: str, **kwargs: Any) -> None:
        self._log("ERROR", message, **kwargs)

    def critical(self, message: str, **kwargs: Any) -> None:
        self._log("CRITICAL", message, **kwargs)

    def _write_to_file(self, entry: LogEntry) -> None:
        if not self._log_file:
            return
        try:
            line = json.dumps(entry.to_dict(), ensure_ascii=False) + "\n"
            with self._log_file.open("a", encoding="utf-8") as f:
                f.write(line)
        except Exception as log_err:
            print(f"[aeromind] metrics log write failed: {log_err}", file=sys.stderr)
    def get_entries(self, level: str | None = None) -> List[Dict[str, Any]]:
        filtered = self._entries
        if level:
            filtered = [e for e in filtered if e.level == level.upper()]
        return [e.to_dict() for e in filtered]

    def get_recent(self, limit: int = 50) -> List[Dict[str, Any]]:
        return [e.to_dict() for e in self._entries[:limit]]

    def clear(self) -> None:
        self._entries.clear()
        if self._log_file:
            try:
                self._log_file.write_text("", encoding="utf-8")
            except Exception as clear_err:
                print(f"[aeromind] metrics log clear failed: {clear_err}", file=sys.stderr)


class TelemetryMonitor:
    def __init__(self, metrics: MetricsCollector, logger: StructuredLogger) -> None:
        self._metrics = metrics
        self._logger = logger
        self._last_telemetry = {}
        self._telemetry_history: List[Dict[str, Any]] = []
        self._max_history = 200

    def record_telemetry(self, telemetry: Dict[str, Any]) -> None:
        self._last_telemetry = telemetry.copy()
        self._telemetry_history.append({
            "timestamp": time.time(),
            **telemetry,
        })
        if len(self._telemetry_history) > self._max_history:
            self._telemetry_history = self._telemetry_history[-self._max_history:]

        if "altitude_m" in telemetry:
            self._metrics.record_gauge("altitude", telemetry["altitude_m"])
        if "speed_mps" in telemetry:
            self._metrics.record_gauge("speed", telemetry["speed_mps"])
        if "x" in telemetry and "y" in telemetry:
            self._metrics.record_gauge("position_x", telemetry["x"])
            self._metrics.record_gauge("position_y", telemetry["y"])

    def get_last_telemetry(self) -> Dict[str, Any]:
        return self._last_telemetry

    def get_telemetry_history(self) -> List[Dict[str, Any]]:
        return self._telemetry_history

    def get_health_status(self) -> Dict[str, Any]:
        status = {
            "timestamp": time.time(),
            "telemetry_received": len(self._telemetry_history) > 0,
            "last_update_age_s": round(time.time() - self._telemetry_history[-1]["timestamp"], 1) if self._telemetry_history else None,
        }
        
        if self._last_telemetry:
            status["current_altitude"] = self._last_telemetry.get("altitude_m")
            status["current_speed"] = self._last_telemetry.get("speed_mps")
        
        return status

    def log_task_event(self, task_name: str, event_type: str, **kwargs: Any) -> None:
        self._logger.info(
            f"Task event: {task_name} - {event_type}",
            task=task_name,
            event=event_type,
            **kwargs,
        )
        self._metrics.record_counter(f"task_{event_type}")


class PerformanceTracker:
    def __init__(self) -> None:
        self._metrics = MetricsCollector()
        self._logger = StructuredLogger()
        self._telemetry = TelemetryMonitor(self._metrics, self._logger)
        self._contexts: Dict[str, float] = {}

    def start_timer(self, name: str) -> None:
        self._contexts[name] = time.time()

    def stop_timer(self, name: str) -> float:
        if name not in self._contexts:
            return 0.0
        duration_ms = (time.time() - self._contexts[name]) * 1000
        self._metrics.record_timer(name, duration_ms)
        del self._contexts[name]
        return duration_ms

    def track_execution(self, name: str) -> Any:
        class ExecutionTracker:
            def __enter__(self) -> 'ExecutionTracker':
                self._start = time.time()
                return self
            
            def __exit__(self, exc_type, exc_val, exc_tb) -> None:
                duration_ms = (time.time() - self._start) * 1000
                self._outer._metrics.record_timer(name, duration_ms)
        
        return ExecutionTracker()

    def get_metrics(self) -> Dict[str, Any]:
        return self._metrics.get_summary()

    def get_logs(self) -> List[Dict[str, Any]]:
        return self._logger.get_recent()

    def record_telemetry(self, telemetry: Dict[str, Any]) -> None:
        self._telemetry.record_telemetry(telemetry)

    def get_health_status(self) -> Dict[str, Any]:
        return self._telemetry.get_health_status()

    def log(self, level: str, message: str, **kwargs: Any) -> None:
        getattr(self._logger, level.lower())(message, **kwargs)