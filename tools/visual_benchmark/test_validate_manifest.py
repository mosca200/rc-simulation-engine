#!/usr/bin/env python3
"""
Tests for RV2-VIS0 Golden Scene Manifest Validator.

Run with:
    python -m unittest tools/visual_benchmark/test_validate_manifest.py -v
"""

import json
import tempfile
import unittest
from pathlib import Path

from tools.visual_benchmark.validate_manifest import validate_manifest


class TestManifestValidator(unittest.TestCase):
    def setUp(self):
        """Create a valid base manifest."""
        self.valid_manifest = {
            "schema_version": "1.0.0",
            "scene_id": "test_scene",
            "description": "Test scene for validation",
            "renderer": {
                "version": "test@123",
                "scenery_preset": "FlyingField"
            },
            "scenery": {
                "time_of_day": "noon",
                "weather": "clear"
            },
            "camera": {
                "mode": "perspective",
                "position": [0.0, 1.0, -5.0],
                "orientation": {
                    "look_at": [0.0, 0.0, 0.0]
                },
                "fov_deg": 60
            },
            "resolution": {
                "width": 1920,
                "height": 1080
            },
            "exposure_ev": 0,
            "lighting": {
                "ambient_intensity": 15000,
                "sun_intensity": 100000
            },
            "sun": {
                "azimuth_deg": 180,
                "elevation_deg": 45
            },
            "aircraft": {
                "model": "models/acro_electric_01/model.json",
                "position": [0.0, 0.5, 0.0]
            },
            "warmup": 10,
            "capture": {
                "filename": "test.png",
                "format": "png"
            },
            "tags": ["test"]
        }

    def _write_manifest(self, manifest: dict) -> Path:
        """Write manifest to temp file and return path."""
        # Create a dummy model.json for aircraft.model validation
        temp_dir = Path(tempfile.mkdtemp())
        models_dir = temp_dir / "models" / "acro_electric_01"
        models_dir.mkdir(parents=True)
        (models_dir / "model.json").write_text("{}")

        manifest_path = temp_dir / "manifest.json"
        with open(manifest_path, "w") as f:
            json.dump(manifest, f)
        return manifest_path

    def test_valid_manifest(self):
        """Test that a valid manifest passes validation."""
        manifest_path = self._write_manifest(self.valid_manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 0)

    def test_missing_required_field(self):
        """Test that missing required field fails validation."""
        manifest = self.valid_manifest.copy()
        del manifest["scene_id"]
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_fov_too_low(self):
        """Test that FOV below minimum fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["fov_deg"] = 5
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_fov_too_high(self):
        """Test that FOV above maximum fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["fov_deg"] = 150
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_resolution_width(self):
        """Test that invalid resolution width fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["resolution"]["width"] = 100
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_resolution_height(self):
        """Test that invalid resolution height fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["resolution"]["height"] = 5000
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_nan_in_position(self):
        """Test that NaN in position fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["position"] = [0.0, float('nan'), -5.0]
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_infinity_in_exposure(self):
        """Test that Infinity in exposure fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["exposure_ev"] = float('inf')
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_capture_extension_mismatch(self):
        """Test that filename extension not matching format fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["capture"]["filename"] = "test.jpg"
        manifest["capture"]["format"] = "png"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_scene_id_format(self):
        """Test that invalid scene_id format fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["scene_id"] = "Invalid-Scene-ID"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_scene_id_too_short(self):
        """Test that scene_id too short fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["scene_id"] = "ab"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_schema_version(self):
        """Test that invalid schema_version format fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["schema_version"] = "v1.0"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_scenery_preset(self):
        """Test that invalid scenery preset fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["renderer"]["scenery_preset"] = "InvalidPreset"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_camera_mode(self):
        """Test that invalid camera mode fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["mode"] = "fisheye"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_sun_direction_not_normalized(self):
        """Test that non-normalized sun direction fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["sun"] = {"direction": [1.0, 1.0, 1.0]}  # Not normalized
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_valid_sun_direction_normalized(self):
        """Test that normalized sun direction passes validation."""
        manifest = self.valid_manifest.copy()
        manifest["sun"] = {"direction": [0.0, 0.707, -0.707]}  # Normalized
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 0)

    def test_invalid_quaternion_not_unit(self):
        """Test that non-unit quaternion fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["orientation"] = {"quaternion": [1.0, 1.0, 1.0, 1.0]}
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_valid_quaternion_unit(self):
        """Test that unit quaternion passes validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["orientation"] = {"quaternion": [1.0, 0.0, 0.0, 0.0]}
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 0)

    def test_near_greater_than_far(self):
        """Test that near plane >= far plane fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["near_plane_m"] = 100
        manifest["camera"]["far_plane_m"] = 10
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_duplicate_tags(self):
        """Test that duplicate tags fail validation."""
        manifest = self.valid_manifest.copy()
        manifest["tags"] = ["test", "test"]
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_empty_tags(self):
        """Test that empty tags array fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["tags"] = []
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_aircraft_model_not_found(self):
        """Test that non-existent aircraft model fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["aircraft"]["model"] = "models/nonexistent/model.json"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_aircraft_throttle(self):
        """Test that throttle outside [0, 1] fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["aircraft"]["throttle"] = 1.5
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_control_surface_deflection(self):
        """Test that control surface deflection outside [-45, 45] fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["aircraft"]["control_state"] = {"aileron_deg": 50}
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_file_not_found(self):
        """Test that non-existent manifest file returns exit code 2."""
        manifest_path = Path("/nonexistent/manifest.json")
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 2)

    def test_invalid_json(self):
        """Test that invalid JSON returns exit code 2."""
        temp_dir = Path(tempfile.mkdtemp())
        manifest_path = temp_dir / "manifest.json"
        manifest_path.write_text("{invalid json")
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 2)


if __name__ == "__main__":
    unittest.main()
