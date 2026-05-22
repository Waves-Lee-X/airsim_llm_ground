import unittest
import tempfile
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.auth import AuthManager, AuthConfig
from core.validation import ToolValidator
from core.safety_gate import SafetyGate, SafetyState
from core.di import ServiceManager
from core.exceptions import AeroMindError, ValidationError, SafetyError
from core.metrics import MetricsCollector, StructuredLogger
from core.storage import TaskStorage
from core.formation_manager import FormationManager


class TestAuthManager(unittest.TestCase):
    def test_auth_disabled(self):
        config = AuthConfig(enabled=False)
        manager = AuthManager(config)
        result = manager.authenticate(lambda: {"ok": True}, "127.0.0.1", {})
        self.assertEqual(result[0], 200)
        self.assertEqual(result[1], {"ok": True})

    def test_auth_enabled_with_valid_key(self):
        config = AuthConfig(enabled=True, api_key="test_key_123")
        manager = AuthManager(config)
        headers = {"X-API-Key": "test_key_123"}
        result = manager.authenticate(lambda: {"ok": True}, "127.0.0.1", headers)
        self.assertEqual(result[0], 200)

    def test_auth_enabled_with_invalid_key(self):
        config = AuthConfig(enabled=True, api_key="test_key_123")
        manager = AuthManager(config)
        headers = {"X-API-Key": "wrong_key"}
        result = manager.authenticate(lambda: {"ok": True}, "127.0.0.1", headers)
        self.assertEqual(result[0], 401)

    def test_rate_limit(self):
        config = AuthConfig(enabled=True, rate_limit_requests=2, rate_limit_window_s=1)
        manager = AuthManager(config)
        headers = {"X-API-Key": "test_key"}
        result1 = manager.authenticate(lambda: {"ok": True}, "127.0.0.1", headers)
        result2 = manager.authenticate(lambda: {"ok": True}, "127.0.0.1", headers)
        result3 = manager.authenticate(lambda: {"ok": True}, "127.0.0.1", headers)
        self.assertEqual(result1[0], 200)
        self.assertEqual(result2[0], 200)
        self.assertEqual(result3[0], 429)


class TestToolValidator(unittest.TestCase):
    def test_validate_takeoff_valid(self):
        result = ToolValidator.validate("takeoff", {"altitude_m": 10.0})
        self.assertTrue(result.ok)

    def test_validate_takeoff_invalid_altitude(self):
        result = ToolValidator.validate("takeoff", {"altitude_m": 50.0})
        self.assertFalse(result.ok)
        self.assertIn("altitude_m", "; ".join(result.errors))

    def test_validate_goto_local_valid(self):
        result = ToolValidator.validate("goto_local", {"x": 10.0, "y": 20.0, "z": -5.0})
        self.assertTrue(result.ok)

    def test_validate_goto_local_out_of_boundary(self):
        result = ToolValidator.validate("goto_local", {"x": 200.0, "y": 0.0, "z": -5.0})
        self.assertFalse(result.ok)

    def test_validate_search_area_valid(self):
        result = ToolValidator.validate("search_area", {
            "area": {"x_min": 0.0, "x_max": 50.0, "y_min": -20.0, "y_max": 20.0},
            "altitude_m": 10.0,
        })
        self.assertTrue(result.ok)

    def test_validate_search_area_invalid(self):
        result = ToolValidator.validate("search_area", {
            "area": {"x_min": 50.0, "x_max": 10.0},
            "altitude_m": 10.0,
        })
        self.assertFalse(result.ok)


class TestSafetyGate(unittest.TestCase):
    def test_validate_takeoff(self):
        gate = SafetyGate()
        decision = gate.validate("takeoff", {"altitude_m": 10.0}, connected=True)
        self.assertTrue(decision.accepted)

    def test_validate_land_requires_confirmation(self):
        gate = SafetyGate()
        decision = gate.validate("land", {}, connected=True)
        self.assertFalse(decision.accepted)
        self.assertIn("confirmed=true", decision.reason)

    def test_validate_land_with_confirmation(self):
        gate = SafetyGate()
        decision = gate.validate("land", {"confirmed": True}, connected=True)
        self.assertTrue(decision.accepted)

    def test_preflight_check_collision(self):
        gate = SafetyGate()
        gate.set_state_checker(lambda: SafetyState(has_collision=True))
        decision = gate.validate("takeoff", {"altitude_m": 10.0}, connected=True)
        self.assertFalse(decision.accepted)

    def test_preflight_check_no_collision(self):
        gate = SafetyGate()
        gate.set_state_checker(lambda: SafetyState(has_collision=False))
        decision = gate.validate("takeoff", {"altitude_m": 10.0}, connected=True)
        self.assertTrue(decision.accepted)


class TestDI(unittest.TestCase):
    def test_service_manager_register_and_get(self):
        sm = ServiceManager()
        obj = {"key": "value"}
        sm.register(dict, obj)
        result = sm.get(dict)
        self.assertIs(result, obj)

    def test_service_manager_lazy_init(self):
        sm = ServiceManager()
        called = [False]
        
        def factory():
            called[0] = True
            return {"lazy": "loaded"}
        
        sm.register_lazy(dict, factory)
        self.assertFalse(called[0])
        result = sm.get(dict)
        self.assertTrue(called[0])
        self.assertEqual(result, {"lazy": "loaded"})


class TestMetrics(unittest.TestCase):
    def test_metrics_collector_counter(self):
        collector = MetricsCollector()
        collector.record_counter("api_requests")
        collector.record_counter("api_requests")
        summary = collector.get_summary()
        self.assertEqual(summary["counters"]["api_requests"], 2)

    def test_metrics_collector_gauge(self):
        collector = MetricsCollector()
        collector.record_gauge("temperature", 25.5)
        summary = collector.get_summary()
        self.assertEqual(summary["gauges"]["temperature"], 25.5)

    def test_structured_logger(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            log_file = Path(tmpdir) / "test.log"
            logger = StructuredLogger(log_file)
            logger.info("Test message", module="test")
            entries = logger.get_entries()
            self.assertEqual(len(entries), 1)
            self.assertEqual(entries[0]["message"], "Test message")


class TestStorage(unittest.TestCase):
    def test_task_storage_create_and_get(self):
        with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as f:
            db_path = f.name
        
        try:
            storage = TaskStorage(db_path)
            task_id = storage.create_task("search_area", "搜索区域", '{"plan": []}')
            self.assertGreater(task_id, 0)
            
            task = storage.get_task(task_id)
            self.assertIsNotNone(task)
            self.assertEqual(task.task_type, "search_area")
            self.assertEqual(task.status, "pending")
            
            storage.update_task_status(task_id, "running")
            task = storage.get_task(task_id)
            self.assertEqual(task.status, "running")
        finally:
            Path(db_path).unlink(missing_ok=True)

    def test_task_storage_list(self):
        with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as f:
            db_path = f.name
        
        try:
            storage = TaskStorage(db_path)
            storage.create_task("takeoff", "起飞", '{}')
            storage.create_task("search_area", "搜索", '{}')
            
            tasks = storage.list_tasks()
            self.assertEqual(len(tasks), 2)
        finally:
            Path(db_path).unlink(missing_ok=True)


class TestFormationManager(unittest.TestCase):
    def test_create_v_formation(self):
        manager = FormationManager()
        pattern = manager.create_pattern("v", 3, 5.0)
        self.assertEqual(pattern.name, "v")
        self.assertEqual(len(pattern.slots), 3)
        self.assertEqual(pattern.slots[0].role, "leader")

    def test_create_line_formation(self):
        manager = FormationManager()
        pattern = manager.create_pattern("line", 4, 4.0)
        self.assertEqual(pattern.name, "line")
        self.assertEqual(len(pattern.slots), 4)

    def test_unknown_pattern(self):
        manager = FormationManager()
        pattern = manager.get_pattern("unknown", 3, 5.0)
        self.assertIsNone(pattern)

    def test_get_all_patterns(self):
        manager = FormationManager()
        patterns = manager.get_all_patterns(3, 5.0)
        self.assertGreater(len(patterns), 0)


class TestExceptions(unittest.TestCase):
    def test_aeromind_error(self):
        exc = AeroMindError("test error", code=123)
        self.assertEqual(str(exc), "test error")
        self.assertEqual(exc.http_status, 500)
        self.assertEqual(exc.error_code, "AM_ERROR")

    def test_validation_error(self):
        exc = ValidationError("invalid input")
        self.assertEqual(exc.http_status, 400)
        self.assertEqual(exc.error_code, "AM_VALIDATION_ERROR")

    def test_safety_error(self):
        exc = SafetyError("safety check failed")
        self.assertEqual(exc.http_status, 403)
        self.assertEqual(exc.error_code, "AM_SAFETY_ERROR")


if __name__ == "__main__":
    unittest.main()