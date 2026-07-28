import asyncio
import signal
from pathlib import Path

import pytest

from aeromind_apm_lite.ground.simulation import (
    ARDUPILOT_COMMIT,
    AirSimProcessConfig,
    ArduPilotBuild,
    RuntimeState,
    SimulationLaunchConfig,
    SimulationLifecycleError,
    SimulationPaths,
    SimulationRuntime,
    SitlInstanceConfig,
)

ROOT = Path(__file__).resolve().parents[1]


class FakeProcess:
    next_pid = 4000

    def __init__(self, *, returncode=None, ignore_term=False):
        self.pid = FakeProcess.next_pid
        FakeProcess.next_pid += 1
        self.returncode = returncode
        self.ignore_term = ignore_term
        self.signals = []
        self._done = asyncio.Event()
        if returncode is not None:
            self._done.set()

    async def wait(self):
        await self._done.wait()
        return self.returncode

    def receive_signal(self, signal_number):
        self.signals.append(signal_number)
        if signal_number == signal.SIGTERM and self.ignore_term:
            return
        self.returncode = -signal_number
        self._done.set()


class FakeProcessFactory:
    def __init__(self, processes=None):
        self.processes = list(processes or [])
        self.calls = []

    async def __call__(self, *argv, **kwargs):
        self.calls.append((argv, kwargs))
        if self.processes:
            return self.processes.pop(0)
        return FakeProcess()


async def no_sleep(_seconds):
    return None


def make_build(tmp_path):
    root = tmp_path / "ardupilot"
    executable = root / "build/sitl/bin/arducopter"
    defaults = root / "Tools/autotest/default_params"
    executable.parent.mkdir(parents=True)
    defaults.mkdir(parents=True)
    executable.write_bytes(b"fake")
    executable.chmod(0o755)
    (defaults / "copter.parm").write_text("FRAME_CLASS 1\n", encoding="ascii")
    (defaults / "airsim-quadX.parm").write_text("FRAME_TYPE 1\n", encoding="ascii")
    return ArduPilotBuild(source_root=root, executable=executable)


def make_config(tmp_path, *, airsim=False, terminate_timeout=0.1):
    process_config = None
    if airsim:
        executable = tmp_path / "UE4Editor.exe"
        project = tmp_path / "Blocks.uproject"
        executable.write_bytes(b"fake")
        project.write_text("{}", encoding="ascii")
        process_config = AirSimProcessConfig(
            executable=executable,
            arguments=(str(project), "-game", "-windowed"),
            working_directory=tmp_path,
        )
    instance = SitlInstanceConfig(
        instance=0,
        build=make_build(tmp_path),
        simulator_address="192.168.16.1",
        parameter_file=ROOT / "configs/sim/arducopter-m1.parm",
    )
    return SimulationLaunchConfig(
        instances=(instance,),
        paths=SimulationPaths(
            state_directory=tmp_path / "runtime",
            airsim_settings_path=tmp_path / "runtime/airsim/settings.json",
        ),
        wsl_ip="192.168.18.156",
        airsim_process=process_config,
        startup_timeout_s=0.2,
        startup_grace_s=0.0,
        terminate_timeout_s=terminate_timeout,
        kill_timeout_s=0.1,
    )


def runtime_for(config, factory):
    return SimulationRuntime(
        config,
        process_factory=factory,
        commit_reader=lambda _root: ARDUPILOT_COMMIT,
        process_signaler=lambda process, sig: process.receive_signal(sig),
        sleeper=no_sleep,
        distro_name="Ubuntu-22.04",
    )


def test_runtime_launches_sitl_with_logs_and_cleans_it_up(tmp_path):
    async def scenario():
        config = make_config(tmp_path)
        factory = FakeProcessFactory()
        runtime = runtime_for(config, factory)

        await runtime.start()

        assert runtime.state == RuntimeState.RUNNING
        assert runtime.running
        assert runtime.settings_path == config.paths.airsim_settings_path
        assert runtime.settings_path.is_file()
        assert len(factory.calls) == 1
        argv, kwargs = factory.calls[0]
        defaults = argv[argv.index("--defaults") + 1]
        assert str(config.instances[0].parameter_file) in defaults
        assert argv[argv.index("--sim-address") + 1] == "192.168.16.1"
        assert kwargs["start_new_session"] is True
        assert config.paths.sitl_log_path(0).is_file()
        runtime.assert_healthy()

        process = runtime._managed[0].process
        await runtime.stop()
        assert runtime.state == RuntimeState.STOPPED
        assert process.signals == [signal.SIGTERM]
        assert runtime.process_evidence[0].returncode == -signal.SIGTERM
        await runtime.stop()

    asyncio.run(scenario())


def test_optional_airsim_starts_first_with_independent_settings_argument(tmp_path):
    async def scenario():
        config = make_config(tmp_path, airsim=True)
        factory = FakeProcessFactory()
        runtime = runtime_for(config, factory)

        await runtime.start()

        assert [evidence.name for evidence in runtime.process_evidence] == [
            "airsim",
            "sitl-0",
        ]
        airsim_argv = factory.calls[0][0]
        assert airsim_argv[1].startswith(
            "\\\\wsl.localhost\\Ubuntu-22.04\\"
        )
        assert airsim_argv[1].endswith("Blocks.uproject")
        settings_argument = next(
            argument for argument in airsim_argv if argument.startswith("-settings=")
        )
        assert settings_argument.startswith(
            "-settings=\\\\wsl.localhost\\Ubuntu-22.04\\"
        )
        assert settings_argument.endswith("runtime\\airsim\\settings.json")
        assert config.paths.airsim_log_path.is_file()

        processes = [item.process for item in runtime._managed]
        await runtime.stop()
        assert processes[1].signals == [signal.SIGTERM]
        assert processes[0].signals == [signal.SIGTERM]

    asyncio.run(scenario())


def test_commit_drift_blocks_all_process_launches(tmp_path):
    async def scenario():
        config = make_config(tmp_path)
        factory = FakeProcessFactory()
        runtime = SimulationRuntime(
            config,
            process_factory=factory,
            commit_reader=lambda _root: "0" * 40,
            sleeper=no_sleep,
        )

        with pytest.raises(SimulationLifecycleError, match="HEAD mismatch"):
            await runtime.start()
        assert runtime.state == RuntimeState.FAILED
        assert factory.calls == []

    asyncio.run(scenario())


def test_early_process_exit_fails_start_and_preserves_log_evidence(tmp_path):
    async def scenario():
        config = make_config(tmp_path)
        factory = FakeProcessFactory([FakeProcess(returncode=17)])
        runtime = runtime_for(config, factory)

        with pytest.raises(SimulationLifecycleError, match="exited during startup"):
            await runtime.start()

        assert runtime.state == RuntimeState.FAILED
        assert runtime.process_evidence[0].returncode == 17
        log = config.paths.sitl_log_path(0).read_text(encoding="utf-8")
        assert '"event": "process_start"' in log

    asyncio.run(scenario())


def test_stubborn_process_escalates_from_term_to_kill(tmp_path):
    async def scenario():
        config = make_config(tmp_path, terminate_timeout=0.1)
        process = FakeProcess(ignore_term=True)
        factory = FakeProcessFactory([process])
        runtime = runtime_for(config, factory)
        await runtime.start()

        await runtime.stop()

        assert process.signals == [signal.SIGTERM, signal.SIGKILL]
        assert runtime.state == RuntimeState.STOPPED

    asyncio.run(scenario())


def test_context_manager_stops_process_after_body_exception(tmp_path):
    async def scenario():
        config = make_config(tmp_path)
        factory = FakeProcessFactory()
        runtime = runtime_for(config, factory)

        with pytest.raises(ValueError, match="injected"):
            async with runtime:
                raise ValueError("injected")

        assert runtime.state == RuntimeState.STOPPED

    asyncio.run(scenario())


def test_health_check_reports_process_that_exited_after_start(tmp_path):
    async def scenario():
        config = make_config(tmp_path)
        process = FakeProcess()
        factory = FakeProcessFactory([process])
        runtime = runtime_for(config, factory)
        await runtime.start()
        process.returncode = 9
        process._done.set()

        with pytest.raises(SimulationLifecycleError, match="sitl-0=9"):
            runtime.assert_healthy()
        assert runtime.state == RuntimeState.FAILED
        await runtime.stop()
        assert runtime.state == RuntimeState.FAILED

    asyncio.run(scenario())


def test_cancelling_start_after_spawn_still_terminates_owned_process(tmp_path):
    async def scenario():
        config = make_config(tmp_path)
        process = FakeProcess()
        factory = FakeProcessFactory([process])
        sleeping = asyncio.Event()

        async def blocked_sleep(_seconds):
            sleeping.set()
            await asyncio.Event().wait()

        runtime = SimulationRuntime(
            config,
            process_factory=factory,
            commit_reader=lambda _root: ARDUPILOT_COMMIT,
            process_signaler=lambda owned, sig: owned.receive_signal(sig),
            sleeper=blocked_sleep,
        )
        starting = asyncio.create_task(runtime.start())
        await sleeping.wait()

        starting.cancel()
        with pytest.raises(asyncio.CancelledError):
            await starting

        assert runtime.state == RuntimeState.FAILED
        assert process.signals == [signal.SIGTERM]
        assert runtime.process_evidence[0].returncode == -signal.SIGTERM

    asyncio.run(scenario())
