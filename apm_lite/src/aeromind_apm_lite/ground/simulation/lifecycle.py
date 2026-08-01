"""Owned AirSim and ArduPilot SITL process lifecycle."""

from __future__ import annotations

import asyncio
import base64
import json
import os
import signal
import subprocess
import time
from collections.abc import Awaitable, Callable, Mapping, Sequence
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, BinaryIO

from .airsim_settings import AirSimSettings
from .config import (
    ARDUPILOT_COMMIT,
    SimulationConfigError,
    SimulationLaunchConfig,
    SitlInstanceConfig,
)

ProcessFactory = Callable[..., Awaitable[Any]]
CommitReader = Callable[[Path], str]
ProcessSignaler = Callable[[Any, int], None]
Sleeper = Callable[[float], Awaitable[None]]


class SimulationLifecycleError(RuntimeError):
    """Raised when a managed simulator process cannot be safely operated."""


class RuntimeState(str, Enum):
    NEW = "new"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    STOPPED = "stopped"
    FAILED = "failed"


@dataclass(frozen=True)
class ProcessEvidence:
    name: str
    pid: int
    argv: tuple[str, ...]
    working_directory: Path
    log_path: Path
    started_monotonic_s: float
    returncode: int | None
    external_pid: int | None = None


@dataclass
class _ManagedProcess:
    evidence: ProcessEvidence
    process: Any
    log_stream: BinaryIO
    external_pid_path: Path | None = None
    external_pid: int | None = None
    external_marker: str | None = None


def read_git_head(source_root: Path) -> str:
    """Read a repository HEAD through git without accepting abbreviated hashes."""

    try:
        completed = subprocess.run(
            ("git", "-C", str(source_root), "rev-parse", "HEAD"),
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise SimulationLifecycleError(
            f"could not verify ArduPilot git HEAD at {source_root}: {exc}"
        ) from exc
    commit = completed.stdout.strip().lower()
    if len(commit) != 40 or any(char not in "0123456789abcdef" for char in commit):
        raise SimulationLifecycleError(
            f"ArduPilot git returned an invalid full commit: {commit!r}"
        )
    return commit


def windows_path_from_wsl(path: Path, *, distro_name: str | None = None) -> str:
    """Translate a WSL path for a Windows process without executing a shell."""

    absolute = Path(path)
    if not absolute.is_absolute():
        raise SimulationConfigError("Windows path conversion requires an absolute path")
    parts = absolute.parts
    if len(parts) >= 4 and parts[1] == "mnt" and len(parts[2]) == 1:
        drive = parts[2].upper()
        suffix = "\\".join(parts[3:])
        return f"{drive}:\\{suffix}"

    distro = distro_name or os.environ.get("WSL_DISTRO_NAME")
    if not distro:
        raise SimulationLifecycleError(
            "WSL_DISTRO_NAME is unavailable; place settings under /mnt/<drive> "
            "or provide distro_name"
        )
    suffix = "\\".join(parts[1:])
    return f"\\\\wsl.localhost\\{distro}\\{suffix}"


def parse_parameter_file(path: Path) -> Mapping[str, str]:
    """Parse the simple ArduPilot NAME VALUE / NAME,VALUE parameter format."""

    try:
        lines = path.read_text(encoding="ascii").splitlines()
    except OSError as exc:
        raise SimulationConfigError(f"could not read SITL parameter file {path}: {exc}") from exc

    parameters: dict[str, str] = {}
    for line_number, raw_line in enumerate(lines, start=1):
        content = raw_line.split("#", 1)[0].strip().replace(",", " ")
        if not content:
            continue
        fields = content.split()
        if len(fields) != 2:
            raise SimulationConfigError(
                f"invalid parameter row {path}:{line_number}: {raw_line!r}"
            )
        name, value = fields
        if name in parameters:
            raise SimulationConfigError(
                f"duplicate parameter {name} in {path}:{line_number}"
            )
        parameters[name] = value
    return parameters


def validate_m1_parameters(instance: SitlInstanceConfig) -> None:
    """Verify that the final defaults file contains the M1 safety freeze."""

    parameters = parse_parameter_file(instance.parameter_file)
    expected = {
        "MAV_SYSID": float(instance.system_id),
        "MAV_GCS_SYSID": float(instance.gcs_system_id),
        "MAV_GCS_SYSID_HI": 0.0,
        "FS_GCS_ENABLE": 5.0,
        "FS_GCS_TIMEOUT": 3.0,
        "GUID_TIMEOUT": 0.0,
        "ARMING_CHECK": 1.0,
        "SCHED_LOOP_RATE": 150.0,
        "SIM_GPS1_HZ": 5.0,
        "GPS1_RATE_MS": 200.0,
        "GPS1_DELAY_MS": 200.0,
    }
    for name, expected_value in expected.items():
        try:
            actual = float(parameters[name])
        except KeyError as exc:
            raise SimulationConfigError(
                f"required M1 SITL parameter {name} is missing from "
                f"{instance.parameter_file}"
            ) from exc
        except ValueError as exc:
            raise SimulationConfigError(
                f"M1 SITL parameter {name} is not numeric in {instance.parameter_file}"
            ) from exc
        if actual != expected_value:
            raise SimulationConfigError(
                f"M1 SITL parameter {name} must be {expected_value:g}, got {actual:g}"
            )


def sitl_argv(instance: SitlInstanceConfig) -> tuple[str, ...]:
    """Build the reproducible direct arducopter invocation for one instance."""

    defaults = (*instance.build.defaults_files, instance.parameter_file)
    return (
        str(instance.build.executable),
        "--wipe",
        "--model",
        instance.model,
        "--speedup",
        "1",
        "--defaults",
        ",".join(str(path) for path in defaults),
        "--sim-address",
        instance.simulator_address,
        "--instance",
        str(instance.instance),
        "--home",
        instance.home.argv_value,
        "--serial0",
        instance.mavlink.serial0_value,
        "--sysid",
        str(instance.system_id),
    )


def _default_signal_process(process: Any, signal_number: int) -> None:
    try:
        os.killpg(process.pid, signal_number)
    except (AttributeError, OSError):
        if signal_number == signal.SIGTERM:
            process.terminate()
        else:
            process.kill()


class SimulationRuntime:
    """Start, audit and cleanly stop an isolated AirSim/SITL runtime."""

    def __init__(
        self,
        config: SimulationLaunchConfig,
        *,
        process_factory: ProcessFactory = asyncio.create_subprocess_exec,
        commit_reader: CommitReader = read_git_head,
        process_signaler: ProcessSignaler = _default_signal_process,
        sleeper: Sleeper = asyncio.sleep,
        clock: Callable[[], float] = time.monotonic,
        distro_name: str | None = None,
    ) -> None:
        self._config = config
        self._process_factory = process_factory
        self._commit_reader = commit_reader
        self._process_signaler = process_signaler
        self._sleeper = sleeper
        self._clock = clock
        self._distro_name = distro_name
        self._state = RuntimeState.NEW
        self._managed: list[_ManagedProcess] = []
        self._history: list[_ManagedProcess] = []
        self._settings_path: Path | None = None

    @property
    def state(self) -> RuntimeState:
        return self._state

    @property
    def settings_path(self) -> Path | None:
        return self._settings_path

    @property
    def process_evidence(self) -> tuple[ProcessEvidence, ...]:
        return tuple(
            ProcessEvidence(
                name=item.evidence.name,
                pid=item.evidence.pid,
                argv=item.evidence.argv,
                working_directory=item.evidence.working_directory,
                log_path=item.evidence.log_path,
                started_monotonic_s=item.evidence.started_monotonic_s,
                returncode=item.process.returncode,
                external_pid=item.external_pid,
            )
            for item in self._history
        )

    @property
    def running(self) -> bool:
        return self._state == RuntimeState.RUNNING and all(
            item.process.returncode is None for item in self._managed
        )

    async def __aenter__(self) -> "SimulationRuntime":
        await self.start()
        return self

    async def __aexit__(self, exc_type, exc, traceback) -> None:
        await asyncio.shield(self.stop())

    async def start(self) -> None:
        if self._state != RuntimeState.NEW:
            raise SimulationLifecycleError(
                f"runtime can only start from new, current state is {self._state.value}"
            )
        self._state = RuntimeState.STARTING
        try:
            self._prepare_directories()
            self._validate_runtime()
            self._settings_path = AirSimSettings(self._config).write()
            if self._config.airsim_process is not None:
                await self._start_airsim()
            for instance in self._config.instances:
                await self._start_sitl(instance)
        except BaseException:
            self._state = RuntimeState.FAILED
            await asyncio.shield(self._shutdown_processes())
            raise
        self._state = RuntimeState.RUNNING

    async def stop(self) -> None:
        if self._state == RuntimeState.STOPPED:
            return
        if self._state == RuntimeState.NEW:
            self._state = RuntimeState.STOPPED
            return
        failed_before_stop = self._state == RuntimeState.FAILED
        self._state = RuntimeState.STOPPING
        errors = await self._shutdown_processes()
        if errors or failed_before_stop:
            self._state = RuntimeState.FAILED
        else:
            self._state = RuntimeState.STOPPED
        if errors:
            raise SimulationLifecycleError("; ".join(errors))

    def assert_healthy(self) -> None:
        if self._state != RuntimeState.RUNNING:
            raise SimulationLifecycleError(
                f"simulation runtime is {self._state.value}, not running"
            )
        exited = [
            f"{item.evidence.name}={item.process.returncode}"
            for item in self._managed
            if item.process.returncode is not None
        ]
        if exited:
            self._state = RuntimeState.FAILED
            raise SimulationLifecycleError(
                "managed simulation process exited: " + ", ".join(exited)
            )

    def _prepare_directories(self) -> None:
        paths = self._config.paths
        paths.state_directory.mkdir(parents=True, exist_ok=True)
        paths.log_directory.mkdir(parents=True, exist_ok=True)
        for instance in self._config.instances:
            paths.instance_directory(instance.instance).mkdir(
                parents=True,
                exist_ok=True,
            )

    def _validate_runtime(self) -> None:
        seen_roots: set[Path] = set()
        for instance in self._config.instances:
            instance.build.validate_runtime_files(instance.parameter_file)
            validate_m1_parameters(instance)
            root = instance.build.source_root
            if root in seen_roots:
                continue
            commit = self._commit_reader(root).lower()
            if commit != ARDUPILOT_COMMIT:
                raise SimulationLifecycleError(
                    "ArduPilot HEAD mismatch: expected "
                    f"{ARDUPILOT_COMMIT}, got {commit} at {root}"
                )
            seen_roots.add(root)
        if self._config.airsim_process is not None:
            self._config.airsim_process.validate_runtime_files()

    async def _start_airsim(self) -> None:
        process_config = self._config.airsim_process
        if process_config is None or self._settings_path is None:
            raise AssertionError("AirSim process and generated settings are required")
        settings = windows_path_from_wsl(
            self._settings_path,
            distro_name=self._distro_name,
        )
        arguments = tuple(
            windows_path_from_wsl(
                Path(argument),
                distro_name=self._distro_name,
            )
            if argument.startswith("/") and Path(argument).exists()
            else argument
            for argument in process_config.arguments
        )
        application_argv = (
            str(process_config.executable),
            *arguments,
            f"-settings={settings}",
        )
        working_directory = (
            process_config.working_directory or process_config.executable.parent
        )
        if self._uses_windows_interop(process_config.executable):
            pid_path = self._config.paths.state_directory / "airsim-windows.pid"
            pid_path.unlink(missing_ok=True)
            wrapper_argv = self._windows_process_wrapper_argv(
                executable=process_config.executable,
                arguments=application_argv[1:],
                working_directory=working_directory,
                pid_path=pid_path,
            )
            managed = await self._spawn(
                "airsim",
                wrapper_argv,
                working_directory,
                self._config.paths.airsim_log_path,
                evidence_argv=application_argv,
            )
            managed.external_pid_path = pid_path
            managed.external_marker = settings
            managed.external_pid = await self._wait_for_external_pid(pid_path)
            return
        await self._spawn(
            "airsim",
            application_argv,
            working_directory,
            self._config.paths.airsim_log_path,
        )

    async def _start_sitl(self, instance: SitlInstanceConfig) -> None:
        await self._spawn(
            f"sitl-{instance.instance}",
            sitl_argv(instance),
            self._config.paths.instance_directory(instance.instance),
            self._config.paths.sitl_log_path(instance.instance),
        )

    async def _spawn(
        self,
        name: str,
        argv: Sequence[str],
        working_directory: Path,
        log_path: Path,
        *,
        evidence_argv: Sequence[str] | None = None,
    ) -> _ManagedProcess:
        encoded_header = (
            json.dumps(
                {
                    "event": "process_start",
                    "name": name,
                    "argv": list(argv),
                    "monotonic_s": self._clock(),
                },
                ensure_ascii=True,
            )
            + "\n"
        ).encode("utf-8")
        stream = log_path.open("ab", buffering=0)
        stream.write(encoded_header)
        try:
            process = await asyncio.wait_for(
                self._process_factory(
                    *argv,
                    cwd=str(working_directory),
                    stdin=asyncio.subprocess.DEVNULL,
                    stdout=stream,
                    stderr=asyncio.subprocess.STDOUT,
                    start_new_session=True,
                ),
                timeout=self._config.startup_timeout_s,
            )
        except BaseException:
            stream.close()
            raise

        evidence = ProcessEvidence(
            name=name,
            pid=int(process.pid),
            argv=tuple(evidence_argv or argv),
            working_directory=working_directory,
            log_path=log_path,
            started_monotonic_s=self._clock(),
            returncode=process.returncode,
        )
        managed = _ManagedProcess(evidence, process, stream)
        self._managed.append(managed)
        self._history.append(managed)
        await self._sleeper(self._config.startup_grace_s)
        if process.returncode is not None:
            tail = self._log_tail(log_path)
            raise SimulationLifecycleError(
                f"{name} exited during startup with {process.returncode}; "
                f"log tail: {tail}"
            )
        return managed

    async def _shutdown_processes(self) -> list[str]:
        errors: list[str] = []
        for managed in reversed(self._managed):
            process = managed.process
            try:
                if managed.external_pid is not None and process.returncode is None:
                    try:
                        await self._stop_windows_process(managed)
                    except (OSError, RuntimeError) as exc:
                        errors.append(
                            f"failed to stop Windows {managed.evidence.name}: {exc}"
                        )
                if process.returncode is None:
                    self._process_signaler(process, signal.SIGTERM)
                    try:
                        await asyncio.wait_for(
                            process.wait(),
                            timeout=self._config.terminate_timeout_s,
                        )
                    except asyncio.TimeoutError:
                        self._process_signaler(process, signal.SIGKILL)
                        try:
                            await asyncio.wait_for(
                                process.wait(),
                                timeout=self._config.kill_timeout_s,
                            )
                        except asyncio.TimeoutError:
                            errors.append(
                                f"{managed.evidence.name} did not exit after SIGKILL"
                            )
            except (OSError, RuntimeError) as exc:
                errors.append(f"failed to stop {managed.evidence.name}: {exc}")
            finally:
                managed.log_stream.close()
        self._managed.clear()
        return errors

    def _uses_windows_interop(self, executable: Path) -> bool:
        return (
            self._process_factory is asyncio.create_subprocess_exec
            and executable.suffix.lower() == ".exe"
            and bool(os.environ.get("WSL_DISTRO_NAME"))
        )

    def _windows_process_wrapper_argv(
        self,
        *,
        executable: Path,
        arguments: Sequence[str],
        working_directory: Path,
        pid_path: Path,
    ) -> tuple[str, ...]:
        specification = {
            "Executable": windows_path_from_wsl(
                executable,
                distro_name=self._distro_name,
            ),
            "Arguments": list(arguments),
            "WorkingDirectory": windows_path_from_wsl(
                working_directory,
                distro_name=self._distro_name,
            ),
            "PidFile": windows_path_from_wsl(
                pid_path,
                distro_name=self._distro_name,
            ),
        }
        specification_b64 = base64.b64encode(
            json.dumps(specification, ensure_ascii=True).encode("utf-8")
        ).decode("ascii")
        script = (
            "$ErrorActionPreference='Stop';"
            "$json=[Text.Encoding]::UTF8.GetString("
            f"[Convert]::FromBase64String('{specification_b64}'));"
            "$spec=$json|ConvertFrom-Json;"
            "$child=Start-Process -FilePath $spec.Executable "
            "-ArgumentList ([string[]]$spec.Arguments) "
            "-WorkingDirectory $spec.WorkingDirectory -PassThru;"
            "[IO.File]::WriteAllText($spec.PidFile,[string]$child.Id);"
            "$child.WaitForExit();exit $child.ExitCode"
        )
        encoded_script = base64.b64encode(script.encode("utf-16le")).decode("ascii")
        return (
            "powershell.exe",
            "-NoProfile",
            "-NonInteractive",
            "-EncodedCommand",
            encoded_script,
        )

    async def _wait_for_external_pid(self, pid_path: Path) -> int:
        deadline = self._clock() + self._config.startup_timeout_s
        while self._clock() < deadline:
            try:
                value = int(pid_path.read_text(encoding="ascii").strip())
            except (OSError, ValueError):
                await self._sleeper(0.05)
                continue
            if value <= 0:
                raise SimulationLifecycleError(
                    f"Windows process PID file contains an invalid PID: {value}"
                )
            return value
        raise SimulationLifecycleError(
            f"Windows AirSim process did not publish its PID at {pid_path}"
        )

    async def _stop_windows_process(self, managed: _ManagedProcess) -> None:
        pid = managed.external_pid
        marker = managed.external_marker
        if pid is None or marker is None:
            return
        marker_b64 = base64.b64encode(marker.encode("utf-8")).decode("ascii")
        script = (
            f"$pidToStop={pid};"
            "$marker=[Text.Encoding]::UTF8.GetString("
            f"[Convert]::FromBase64String('{marker_b64}'));"
            "$process=Get-CimInstance Win32_Process -Filter "
            "\"ProcessId = $pidToStop\" -ErrorAction SilentlyContinue;"
            "if($null -eq $process){exit 0};"
            "if($process.CommandLine -notlike ('*'+$marker+'*')){exit 3};"
            "Stop-Process -Id $pidToStop -ErrorAction Stop"
        )
        encoded_script = base64.b64encode(script.encode("utf-16le")).decode("ascii")
        helper = await asyncio.create_subprocess_exec(
            "powershell.exe",
            "-NoProfile",
            "-NonInteractive",
            "-EncodedCommand",
            encoded_script,
            stdin=asyncio.subprocess.DEVNULL,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )
        try:
            returncode = await asyncio.wait_for(
                helper.wait(),
                timeout=self._config.terminate_timeout_s,
            )
        except asyncio.TimeoutError as exc:
            helper.kill()
            await helper.wait()
            raise SimulationLifecycleError(
                f"timed out stopping Windows AirSim PID {pid}"
            ) from exc
        if returncode != 0:
            raise SimulationLifecycleError(
                "refused to stop Windows AirSim PID "
                f"{pid}: process identity check returned {returncode}"
            )

    @staticmethod
    def _log_tail(path: Path, limit: int = 512) -> str:
        try:
            with path.open("rb") as stream:
                stream.seek(0, os.SEEK_END)
                size = stream.tell()
                stream.seek(max(0, size - limit))
                return stream.read().decode("utf-8", errors="replace").strip()
        except OSError:
            return "<unavailable>"
