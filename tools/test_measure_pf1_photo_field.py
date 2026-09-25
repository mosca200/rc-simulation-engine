"""Focused PF1 capture-pose and radial-order evidence guards."""

import json
import tempfile
import unittest
from pathlib import Path

from tools.measure_pf1_photo_field import checked_capture_pose, occlusion_geometry
from tools.photo_field_pipeline.photo_field_assets import PILOT_EYE_RENDER_M


class OcclusionEvidenceTests(unittest.TestCase):
    def test_far_requires_pose_bound_to_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            png = root / "capture.png"
            png.write_bytes(b"capture")
            import hashlib
            digest = hashlib.sha256(b"capture").hexdigest()
            run = {"png": png, "receipt": {
                "presentation_frame_index": 180,
                "framebuffer_width": 1920,
                "framebuffer_height": 1080,
                "image_sha256": digest,
            }}
            with self.assertRaises(FileNotFoundError):
                checked_capture_pose(run)
            pose = {
                "schema_version": "1.0.0",
                "presentation_frame_index": 180,
                "framebuffer_width": 1920,
                "framebuffer_height": 1080,
                "image_sha256": digest,
                "aircraft_position_render_m": [50.0, 8.0, 0.0],
            }
            path = root / "runtime_capture_pose.json"
            path.write_text(json.dumps(pose), encoding="utf-8")
            self.assertEqual(checked_capture_pose(run), pose["aircraft_position_render_m"])
            pose["presentation_frame_index"] = 179
            path.write_text(json.dumps(pose), encoding="utf-8")
            with self.assertRaises(ValueError):
                checked_capture_pose(run)

    def test_near_and_far_require_conservative_distance_order(self):
        eye = list(PILOT_EYE_RENDER_M)
        self.assertTrue(occlusion_geometry("near", [0.0, 0.0, 0.0], eye)["radial_order_verified"])
        self.assertTrue(occlusion_geometry("far", [eye[0] + 40.0, 5.0, eye[2]], eye)["radial_order_verified"])
        with self.assertRaises(ValueError):
            occlusion_geometry("far", [eye[0] + 34.0, 5.0, eye[2]], eye)
        with self.assertRaises(ValueError):
            occlusion_geometry("far", [eye[0] + 40.0, -2.0, eye[2]], eye)


if __name__ == "__main__":
    unittest.main()
