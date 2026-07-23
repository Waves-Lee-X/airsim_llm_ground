import json

import numpy as np

from aeromind_web.json_support import json_compatible


def test_json_compatible_converts_nested_numpy_values_and_non_finite_numbers():
    value = {
        "confidence": np.float32(0.75),
        "covariance": np.asarray([0.1, np.inf], dtype=np.float32),
        "count": np.int32(3),
    }

    normalized = json_compatible(value)
    encoded = json.dumps(normalized, allow_nan=False)

    assert normalized["confidence"] == 0.75
    assert normalized["covariance"][1] is None
    assert normalized["count"] == 3
    assert "Infinity" not in encoded
