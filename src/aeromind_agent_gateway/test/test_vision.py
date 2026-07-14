import base64
import os
import tempfile
import unittest
from unittest.mock import patch
import json

from aeromind_agent_gateway.config import GatewayConfig
from aeromind_agent_gateway.vision import VisionAnalyzer


PNG_1X1 = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII="
)


class VisionAnalyzerTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.config = GatewayConfig(
            host="127.0.0.1",
            port=8090,
            database_path=os.path.join(self.temp_dir.name, "agent.db"),
            workspace=self.temp_dir.name,
            access_token="",
            default_model="sonnet",
            max_turns=4,
            max_budget_usd=0.1,
            feishu_image_max_bytes=1024,
            vlm_api_url="https://example.invalid/v1",
            vlm_api_key="secret",
            vlm_model="qwen3-vl-plus",
        )
        self.analyzer = VisionAnalyzer(self.config)

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_png_validation(self):
        metadata = self.analyzer.validate(PNG_1X1)
        self.assertEqual(metadata["mime_type"], "image/png")
        self.assertEqual(metadata["width"], 1)

    def test_invalid_image_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "无法识别"):
            self.analyzer.validate(b"not-an-image")

    def test_oversized_image_is_rejected_before_decode(self):
        with self.assertRaisesRegex(ValueError, "超过大小限制"):
            self.analyzer.validate(b"x" * 1025)

    def test_openai_compatible_request(self):
        class Response:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read(self):
                return json.dumps(
                    {"choices": [{"message": {"content": "分析完成"}}]}
                ).encode()

        metadata = self.analyzer.validate(PNG_1X1)
        with patch("urllib.request.urlopen", return_value=Response()) as urlopen:
            result = self.analyzer._call_api(PNG_1X1, "检查目标", metadata)
        self.assertEqual(result, "分析完成")
        request = urlopen.call_args.args[0]
        payload = json.loads(request.data.decode())
        self.assertEqual(payload["model"], "qwen3-vl-plus")
        self.assertTrue(
            payload["messages"][0]["content"][1]["image_url"]["url"].startswith(
                "data:image/png;base64,"
            )
        )
        self.assertEqual(request.headers["Authorization"], "Bearer secret")


if __name__ == "__main__":
    unittest.main()
