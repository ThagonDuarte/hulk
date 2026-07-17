import hashlib
import json
import unittest

from utils.export_hydra import refresh_manifest_config_hash


class ExportHydraTests(unittest.TestCase):
    def test_refreshes_hash_after_export_preprocessing_change(self) -> None:
        manifest = {
            "architecture": "dfine-multitask",
            "preprocessing": {"input_size": [640, 640]},
            "config_hash": "stale",
        }
        manifest["preprocessing"]["input_size"] = [448, 544]

        refresh_manifest_config_hash(manifest)

        values = dict(manifest)
        digest = values.pop("config_hash")
        expected = hashlib.sha256(
            json.dumps(values, sort_keys=True).encode()
        ).hexdigest()
        self.assertEqual(digest, expected)
        self.assertNotEqual(digest, "stale")


if __name__ == "__main__":
    unittest.main()
