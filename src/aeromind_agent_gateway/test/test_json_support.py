import json
import unittest

import numpy as np

from aeromind_agent_gateway.json_support import json_compatible, json_dumps


class JsonSupportTest(unittest.TestCase):
    def test_nested_numpy_values_are_strict_json_compatible(self):
        value = {
            "confidence": np.float32(0.75),
            "covariance": np.asarray([0.1, np.nan], dtype=np.float32),
            "count": np.int32(3),
        }

        normalized = json_compatible(value)
        encoded = json_dumps(value)

        self.assertEqual(normalized["confidence"], 0.75)
        self.assertIsNone(normalized["covariance"][1])
        self.assertEqual(normalized["count"], 3)
        self.assertNotIn("NaN", encoded)
        self.assertEqual(json.loads(encoded)["count"], 3)


if __name__ == "__main__":
    unittest.main()
