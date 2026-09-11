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
        """Create a valid base manifest aligned with runtime."""
        self.valid_manifest = {
            "schema_version": "1.0.0",
            "scene_id": "test_scene",
            "description": "Test scene for validation",
            "renderer": {
                "version": "v2"
            },
            "scenery": {
                "preset": "flying-field"
            },
            "camera": {
                "mode": "pilot",
                "vertical_fov_deg": 55,
                "pilot_position_render_m": [0.0, 1.8, 20.0]
            },
            "resolution": {
                "width": 1920,
                "height": 1080
            },
            "exposure_ev": 0,
            "aircraft": {
                "model": "models/acro_electric_01/model.json"
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

    def _write_raw_json(self, data: str) -> Path:
        """Write raw JSON string to temp file and return path."""
        temp_dir = Path(tempfile.mkdtemp())
        manifest_path = temp_dir / "manifest.json"
        manifest_path.write_text(data)
        return manifest_path

    # === Basic validation tests ===

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

    # === Finding 1: Debug labels tests ===

    def test_terrain_debug_albedo_valid(self):
        """Test that terrain_debug=albedo passes validation."""
        manifest = self.valid_manifest.copy()
        manifest["renderer"]["terrain_debug"] = "albedo"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 0)

    def test_terrain_debug_wireframe_invalid(self):
        """Test that terrain_debug=wireframe fails validation (not a runtime value)."""
        manifest = self.valid_manifest.copy()
        manifest["renderer"]["terrain_debug"] = "wireframe"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_vegetation_debug_culling_valid(self):
        """Test that vegetation_debug=culling passes validation."""
        manifest = self.valid_manifest.copy()
        manifest["renderer"]["vegetation_debug"] = "culling"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 0)

    def test_vegetation_debug_bounds_invalid(self):
        """Test that vegetation_debug=bounds fails validation (not a runtime value)."""
        manifest = self.valid_manifest.copy()
        manifest["renderer"]["vegetation_debug"] = "bounds"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    # === Finding 3: Boolean is not number tests ===

    def test_exposure_ev_boolean_fails(self):
        """Test that exposure_ev=true fails validation (bool is not number)."""
        manifest = self.valid_manifest.copy()
        manifest["exposure_ev"] = True
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_vertical_fov_deg_boolean_fails(self):
        """Test that vertical_fov_deg=true fails validation (bool is not number)."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["vertical_fov_deg"] = True
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_warmup_boolean_fails(self):
        """Test that warmup=true fails validation (bool is not integer)."""
        manifest = self.valid_manifest.copy()
        manifest["warmup"] = True
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_capture_frame_boolean_fails(self):
        """Test that capture.frame=false fails validation (bool is not integer)."""
        manifest = self.valid_manifest.copy()
        manifest["capture"]["frame"] = False
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    # === Finding 4: Top-level non-object tests ===

    def test_root_array_fails(self):
        """Test that root array fails validation without traceback."""
        manifest_path = self._write_raw_json("[]")
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_root_string_fails(self):
        """Test that root string fails validation without traceback."""
        manifest_path = self._write_raw_json('"hello"')
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_root_number_fails(self):
        """Test that root number fails validation without traceback."""
        manifest_path = self._write_raw_json("42")
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    # === Finding 8: JPEG quality tests ===

    def test_png_with_quality_fails(self):
        """Test that PNG format with quality field fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["capture"]["format"] = "png"
        manifest["capture"]["quality"] = 95
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_jpg_with_quality_passes(self):
        """Test that JPG format with quality field passes validation."""
        manifest = self.valid_manifest.copy()
        manifest["capture"]["filename"] = "test.jpg"
        manifest["capture"]["format"] = "jpg"
        manifest["capture"]["quality"] = 95
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 0)

    def test_exr_with_quality_fails(self):
        """Test that EXR format with quality field fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["capture"]["filename"] = "test.exr"
        manifest["capture"]["format"] = "exr"
        manifest["capture"]["quality"] = 95
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    # === Runtime alignment tests ===

    def test_unknown_top_level_property(self):
        """Test that unknown top-level property fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["unknown_field"] = "value"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_unknown_renderer_property(self):
        """Test that unknown renderer property fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["renderer"]["unknown_option"] = True
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_runtime_scenery_label(self):
        """Test that invalid runtime scenery label fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["scenery"]["preset"] = "TestField"  # Not a real runtime preset
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_renderer_version(self):
        """Test that invalid renderer version fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["renderer"]["version"] = "v3"  # Not supported
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    # === Camera reproducibility tests ===

    def test_missing_required_camera_reconstruction_data_pilot(self):
        """Test that missing pilot position fails validation."""
        manifest = self.valid_manifest.copy()
        del manifest["camera"]["pilot_position_render_m"]
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_missing_required_camera_reconstruction_data_chase(self):
        """Test that missing chase distance fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["mode"] = "chase"
        del manifest["camera"]["pilot_position_render_m"]
        # Missing chase_distance_behind_m and chase_height_above_m
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_chase_camera_with_pilot_fields(self):
        """Test that chase mode with pilot fields fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["mode"] = "chase"
        manifest["camera"]["chase_distance_behind_m"] = 3.5
        manifest["camera"]["chase_height_above_m"] = 1.25
        # pilot_position_render_m should not be present in chase mode
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_valid_chase_camera(self):
        """Test that valid chase camera passes validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["mode"] = "chase"
        manifest["camera"]["chase_distance_behind_m"] = 3.5
        manifest["camera"]["chase_height_above_m"] = 1.25
        del manifest["camera"]["pilot_position_render_m"]
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 0)

    # === NaN/Infinity tests ===

    def test_nan_fov(self):
        """Test that NaN FOV fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["vertical_fov_deg"] = float('nan')
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_infinity_throttle(self):
        """Test that Infinity throttle fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["aircraft"]["throttle"] = float('inf')
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_negative_infinity_exposure(self):
        """Test that -Infinity exposure fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["exposure_ev"] = float('-inf')
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_nan_in_position(self):
        """Test that NaN in position fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["pilot_position_render_m"] = [0.0, float('nan'), 20.0]
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    # === Crash prevention tests ===

    def test_wrong_type_vector_no_exception(self):
        """Test that wrong-type vector fails without exception."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["pilot_position_render_m"] = ["bad", 0, 1]
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_null_in_vector_no_exception(self):
        """Test that null in vector fails without exception."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["pilot_position_render_m"] = [None, 0, 0]
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_wrong_length_vector_no_exception(self):
        """Test that wrong-length vector fails without exception."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["pilot_position_render_m"] = [0.0, 1.0]  # Only 2 elements
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    # === Other validation tests ===

    def test_invalid_fov_too_low(self):
        """Test that FOV below minimum fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["vertical_fov_deg"] = 5
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_invalid_fov_too_high(self):
        """Test that FOV above maximum fails validation."""
        manifest = self.valid_manifest.copy()
        manifest["camera"]["vertical_fov_deg"] = 150
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

    # === Optional blocks validation ===

    def test_optional_reference_hardware_validated(self):
        """Test that optional reference_hardware block is validated."""
        manifest = self.valid_manifest.copy()
        manifest["reference_hardware"] = {"gpu": 123}  # Wrong type
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_optional_terrain_debug_validated(self):
        """Test that optional terrain_debug is validated."""
        manifest = self.valid_manifest.copy()
        manifest["renderer"]["terrain_debug"] = "invalid_mode"
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)

    def test_optional_start_on_ground_validated(self):
        """Test that optional start_on_ground is validated."""
        manifest = self.valid_manifest.copy()
        manifest["aircraft"]["start_on_ground"] = "yes"  # Wrong type
        manifest_path = self._write_manifest(manifest)
        exit_code = validate_manifest(manifest_path)
        self.assertEqual(exit_code, 1)


if __name__ == "__main__":
    unittest.main()
