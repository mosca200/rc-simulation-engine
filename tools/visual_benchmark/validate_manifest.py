#!/usr/bin/env python3
"""
RV2-VIS0 Golden Scene Manifest Validator

Validates a golden scene manifest JSON against the contract defined in
docs/architecture/rv2_vis0_visual_benchmark.md.

Usage:
    python tools/visual_benchmark/validate_manifest.py <manifest.json>

Exit codes:
    0 - Valid manifest
    1 - Invalid manifest (validation errors)
    2 - File not found or parse error
"""

import json
import math
import re
import sys
from pathlib import Path
from typing import Any


class ValidationError:
    def __init__(self, path: str, message: str):
        self.path = path
        self.message = message

    def __str__(self):
        return f"{self.path}: {self.message}"


class ManifestValidator:
    # Allowed top-level properties (from schema)
    ALLOWED_TOP_LEVEL = {
        "schema_version", "scene_id", "description", "renderer", "scenery",
        "camera", "resolution", "exposure_ev", "aircraft", "warmup",
        "capture", "tags", "reference_hardware", "reference_image"
    }

    # Allowed nested properties per block
    ALLOWED_RENDERER = {"version", "terrain_debug", "vegetation_debug"}
    ALLOWED_SCENERY = {"preset"}
    ALLOWED_CAMERA = {"mode", "vertical_fov_deg", "pilot_position_render_m",
                      "chase_distance_behind_m", "chase_height_above_m"}
    ALLOWED_RESOLUTION = {"width", "height"}
    ALLOWED_AIRCRAFT = {"model", "throttle", "start_on_ground"}
    ALLOWED_CAPTURE = {"filename", "format", "frame", "quality"}
    ALLOWED_REFERENCE_HARDWARE = {"gpu", "driver_version", "os", "notes"}

    def __init__(self, manifest: dict, base_path: Path):
        self.manifest = manifest
        self.base_path = base_path
        self.errors: list[ValidationError] = []

    def error(self, path: str, message: str):
        self.errors.append(ValidationError(path, message))

    def validate(self) -> bool:
        """Validate the manifest. Returns True if valid, False otherwise."""
        self._validate_unknown_top_level()
        self._validate_schema_version()
        self._validate_scene_id()
        self._validate_description()
        self._validate_renderer()
        self._validate_scenery()
        self._validate_camera()
        self._validate_resolution()
        self._validate_exposure()
        self._validate_aircraft()
        self._validate_warmup()
        self._validate_capture()
        self._validate_tags()
        self._validate_reference_hardware()
        self._validate_semantic_invariants()
        return len(self.errors) == 0

    def _validate_unknown_top_level(self):
        """Reject unknown top-level properties."""
        for key in self.manifest.keys():
            if key not in self.ALLOWED_TOP_LEVEL:
                self.error(key, f"unknown top-level property (allowed: {sorted(self.ALLOWED_TOP_LEVEL)})")

    def _validate_schema_version(self):
        version = self.manifest.get("schema_version")
        if not isinstance(version, str):
            self.error("schema_version", "must be a string")
            return
        if not re.match(r"^[0-9]+\.[0-9]+\.[0-9]+$", version):
            self.error("schema_version", f"must match semantic version format (X.Y.Z), got '{version}'")

    def _validate_scene_id(self):
        scene_id = self.manifest.get("scene_id")
        if not isinstance(scene_id, str):
            self.error("scene_id", "must be a string")
            return
        if len(scene_id) < 3:
            self.error("scene_id", f"must be at least 3 characters, got {len(scene_id)}")
        if len(scene_id) > 128:
            self.error("scene_id", f"must be at most 128 characters, got {len(scene_id)}")
        if not re.match(r"^[a-z][a-z0-9_]*$", scene_id):
            self.error("scene_id", f"must match pattern '^[a-z][a-z0-9_]*$', got '{scene_id}'")

    def _validate_description(self):
        desc = self.manifest.get("description")
        if not isinstance(desc, str):
            self.error("description", "must be a string")
            return
        if len(desc) < 10:
            self.error("description", f"must be at least 10 characters, got {len(desc)}")
        if len(desc) > 1000:
            self.error("description", f"must be at most 1000 characters, got {len(desc)}")

    def _validate_renderer(self):
        renderer = self.manifest.get("renderer")
        if not isinstance(renderer, dict):
            self.error("renderer", "must be an object")
            return

        # Reject unknown properties
        for key in renderer.keys():
            if key not in self.ALLOWED_RENDERER:
                self.error(f"renderer.{key}", f"unknown property (allowed: {sorted(self.ALLOWED_RENDERER)})")

        if "version" not in renderer:
            self.error("renderer.version", "required field missing")
        elif renderer["version"] not in ["v1", "v2"]:
            self.error("renderer.version", f"must be 'v1' or 'v2', got '{renderer['version']}'")

        if "terrain_debug" in renderer:
            if renderer["terrain_debug"] not in ["final", "wireframe", "normals", "uv", "bounds"]:
                self.error("renderer.terrain_debug", f"invalid value '{renderer['terrain_debug']}'")

        if "vegetation_debug" in renderer:
            if renderer["vegetation_debug"] not in ["final", "wireframe", "bounds", "lod"]:
                self.error("renderer.vegetation_debug", f"invalid value '{renderer['vegetation_debug']}'")

    def _validate_scenery(self):
        scenery = self.manifest.get("scenery")
        if not isinstance(scenery, dict):
            self.error("scenery", "must be an object")
            return

        # Reject unknown properties
        for key in scenery.keys():
            if key not in self.ALLOWED_SCENERY:
                self.error(f"scenery.{key}", f"unknown property (allowed: {sorted(self.ALLOWED_SCENERY)})")

        if "preset" not in scenery:
            self.error("scenery.preset", "required field missing")
        elif scenery["preset"] not in ["none", "flying-field"]:
            self.error("scenery.preset", f"must be 'none' or 'flying-field', got '{scenery['preset']}'")

    def _validate_camera(self):
        camera = self.manifest.get("camera")
        if not isinstance(camera, dict):
            self.error("camera", "must be an object")
            return

        # Reject unknown properties
        for key in camera.keys():
            if key not in self.ALLOWED_CAMERA:
                self.error(f"camera.{key}", f"unknown property (allowed: {sorted(self.ALLOWED_CAMERA)})")

        if "mode" not in camera:
            self.error("camera.mode", "required field missing")
            return
        elif camera["mode"] not in ["pilot", "chase"]:
            self.error("camera.mode", f"must be 'pilot' or 'chase', got '{camera['mode']}'")
            return

        if "vertical_fov_deg" not in camera:
            self.error("camera.vertical_fov_deg", "required field missing")
        else:
            self._validate_finite_number("camera.vertical_fov_deg", camera["vertical_fov_deg"], 10, 120)

        # Mode-specific required fields
        mode = camera["mode"]
        if mode == "pilot":
            if "pilot_position_render_m" not in camera:
                self.error("camera.pilot_position_render_m", "required when mode='pilot'")
            else:
                self._validate_vector3("camera.pilot_position_render_m", camera["pilot_position_render_m"])
            # Chase fields should not be present
            if "chase_distance_behind_m" in camera:
                self.error("camera.chase_distance_behind_m", "not allowed when mode='pilot'")
            if "chase_height_above_m" in camera:
                self.error("camera.chase_height_above_m", "not allowed when mode='pilot'")
        elif mode == "chase":
            if "chase_distance_behind_m" not in camera:
                self.error("camera.chase_distance_behind_m", "required when mode='chase'")
            else:
                self._validate_finite_number("camera.chase_distance_behind_m", camera["chase_distance_behind_m"], 0, 50, exclusive_min=True)
            if "chase_height_above_m" not in camera:
                self.error("camera.chase_height_above_m", "required when mode='chase'")
            else:
                self._validate_finite_number("camera.chase_height_above_m", camera["chase_height_above_m"], -10, 50)
            # Pilot fields should not be present
            if "pilot_position_render_m" in camera:
                self.error("camera.pilot_position_render_m", "not allowed when mode='chase'")

    def _validate_resolution(self):
        res = self.manifest.get("resolution")
        if not isinstance(res, dict):
            self.error("resolution", "must be an object")
            return

        # Reject unknown properties
        for key in res.keys():
            if key not in self.ALLOWED_RESOLUTION:
                self.error(f"resolution.{key}", f"unknown property (allowed: {sorted(self.ALLOWED_RESOLUTION)})")

        if "width" not in res:
            self.error("resolution.width", "required field missing")
        else:
            width = res["width"]
            if not isinstance(width, int):
                self.error("resolution.width", "must be an integer")
            elif width < 320 or width > 7680:
                self.error("resolution.width", f"must be in range [320, 7680], got {width}")

        if "height" not in res:
            self.error("resolution.height", "required field missing")
        else:
            height = res["height"]
            if not isinstance(height, int):
                self.error("resolution.height", "must be an integer")
            elif height < 240 or height > 4320:
                self.error("resolution.height", f"must be in range [240, 4320], got {height}")

    def _validate_exposure(self):
        exposure = self.manifest.get("exposure_ev")
        self._validate_finite_number("exposure_ev", exposure, -8, 8)

    def _validate_aircraft(self):
        aircraft = self.manifest.get("aircraft")
        if not isinstance(aircraft, dict):
            self.error("aircraft", "must be an object")
            return

        # Reject unknown properties
        for key in aircraft.keys():
            if key not in self.ALLOWED_AIRCRAFT:
                self.error(f"aircraft.{key}", f"unknown property (allowed: {sorted(self.ALLOWED_AIRCRAFT)})")

        if "model" not in aircraft:
            self.error("aircraft.model", "required field missing")
        else:
            model = aircraft["model"]
            if not isinstance(model, str):
                self.error("aircraft.model", "must be a string")
            elif not re.match(r"^models/[a-z0-9_]+/model\.json$", model):
                self.error("aircraft.model", f"must match pattern 'models/<name>/model.json', got '{model}'")
            else:
                # Check if model file exists
                # Try from manifest directory first (for tests), then from repo root
                model_path_local = self.base_path / model
                repo_root = self.base_path.parent.parent.parent
                model_path_repo = repo_root / model
                if not model_path_local.exists() and not model_path_repo.exists():
                    self.error("aircraft.model", f"file does not exist: {model}")

        if "throttle" in aircraft:
            self._validate_finite_number("aircraft.throttle", aircraft["throttle"], 0, 1)

        if "start_on_ground" in aircraft:
            if not isinstance(aircraft["start_on_ground"], bool):
                self.error("aircraft.start_on_ground", "must be a boolean")

    def _validate_warmup(self):
        warmup = self.manifest.get("warmup")
        if not isinstance(warmup, int):
            self.error("warmup", "must be an integer")
        elif warmup < 0 or warmup > 300:
            self.error("warmup", f"must be in range [0, 300], got {warmup}")

    def _validate_capture(self):
        capture = self.manifest.get("capture")
        if not isinstance(capture, dict):
            self.error("capture", "must be an object")
            return

        # Reject unknown properties
        for key in capture.keys():
            if key not in self.ALLOWED_CAPTURE:
                self.error(f"capture.{key}", f"unknown property (allowed: {sorted(self.ALLOWED_CAPTURE)})")

        if "filename" not in capture:
            self.error("capture.filename", "required field missing")
        else:
            filename = capture["filename"]
            if not isinstance(filename, str):
                self.error("capture.filename", "must be a string")
            elif not re.match(r"^[a-z0-9_]+\.(png|jpg|exr)$", filename):
                self.error("capture.filename", f"must match pattern '^[a-z0-9_]+\\.(png|jpg|exr)$', got '{filename}'")

        if "format" not in capture:
            self.error("capture.format", "required field missing")
        else:
            fmt = capture["format"]
            if fmt not in ["png", "jpg", "exr"]:
                self.error("capture.format", f"must be one of ['png', 'jpg', 'exr'], got '{fmt}'")

            # Check filename extension matches format
            if "filename" in capture:
                filename = capture["filename"]
                ext = filename.split(".")[-1]
                if ext != fmt:
                    self.error("capture", f"filename extension '{ext}' does not match format '{fmt}'")

        if "frame" in capture:
            frame = capture["frame"]
            if not isinstance(frame, int):
                self.error("capture.frame", "must be an integer")
            elif frame < 0:
                self.error("capture.frame", f"must be non-negative, got {frame}")

        if "quality" in capture:
            quality = capture["quality"]
            if not isinstance(quality, int):
                self.error("capture.quality", "must be an integer")
            elif quality < 1 or quality > 100:
                self.error("capture.quality", f"must be in range [1, 100], got {quality}")

    def _validate_tags(self):
        tags = self.manifest.get("tags")
        if not isinstance(tags, list):
            self.error("tags", "must be an array")
            return
        if len(tags) < 1:
            self.error("tags", "must have at least 1 tag")
        if len(tags) > 20:
            self.error("tags", f"must have at most 20 tags, got {len(tags)}")
        if len(tags) != len(set(tags)):
            self.error("tags", "must have unique items")
        for i, tag in enumerate(tags):
            if not isinstance(tag, str):
                self.error(f"tags[{i}]", "must be a string")
            elif not re.match(r"^[a-z][a-z0-9_-]*$", tag):
                self.error(f"tags[{i}]", f"must match pattern '^[a-z][a-z0-9_-]*$', got '{tag}'")

    def _validate_reference_hardware(self):
        hw = self.manifest.get("reference_hardware")
        if hw is None:
            return  # Optional
        if not isinstance(hw, dict):
            self.error("reference_hardware", "must be an object")
            return

        # Reject unknown properties
        for key in hw.keys():
            if key not in self.ALLOWED_REFERENCE_HARDWARE:
                self.error(f"reference_hardware.{key}", f"unknown property (allowed: {sorted(self.ALLOWED_REFERENCE_HARDWARE)})")

        for field in ["gpu", "driver_version", "os", "notes"]:
            if field in hw and not isinstance(hw[field], str):
                self.error(f"reference_hardware.{field}", "must be a string")

    def _validate_semantic_invariants(self):
        """Validate semantic invariants not expressible in JSON Schema."""
        # Check that resolution aspect ratio is reasonable
        res = self.manifest.get("resolution", {})
        width = res.get("width")
        height = res.get("height")
        if isinstance(width, int) and isinstance(height, int) and height > 0:
            aspect = width / height
            if aspect < 0.5 or aspect > 3.0:
                self.error("resolution", f"aspect ratio {aspect:.2f} is unusual (expected 0.5-3.0)")

    def _validate_finite_number(self, path: str, value: Any, min_val: float, max_val: float, exclusive_min: bool = False):
        """Validate that a value is a finite number within range."""
        if not isinstance(value, (int, float)):
            self.error(path, f"must be a number, got {type(value).__name__}")
            return
        if math.isnan(value):
            self.error(path, "must be finite (got NaN)")
            return
        if math.isinf(value):
            self.error(path, f"must be finite (got {'+' if value > 0 else '-'}Infinity)")
            return
        if exclusive_min:
            if value <= min_val or value > max_val:
                self.error(path, f"must be in range ({min_val}, {max_val}], got {value}")
        else:
            if value < min_val or value > max_val:
                self.error(path, f"must be in range [{min_val}, {max_val}], got {value}")

    def _validate_vector3(self, path: str, vec: Any):
        """Validate a 3D vector with finite components."""
        if not isinstance(vec, list):
            self.error(path, f"must be an array, got {type(vec).__name__}")
            return
        if len(vec) != 3:
            self.error(path, f"must be an array of 3 numbers, got {len(vec)}")
            return
        for i, val in enumerate(vec):
            if not isinstance(val, (int, float)):
                self.error(f"{path}[{i}]", f"must be a number, got {type(val).__name__}")
                return
            if math.isnan(val):
                self.error(f"{path}[{i}]", "must be finite (got NaN)")
                return
            if math.isinf(val):
                self.error(f"{path}[{i}]", f"must be finite (got {'+' if val > 0 else '-'}Infinity)")
                return


def validate_manifest(manifest_path: Path) -> int:
    """Validate a manifest file. Returns exit code."""
    if not manifest_path.exists():
        print(f"Error: file not found: {manifest_path}", file=sys.stderr)
        return 2

    try:
        with open(manifest_path, "r", encoding="utf-8") as f:
            manifest = json.load(f)
    except json.JSONDecodeError as e:
        print(f"Error: invalid JSON: {e}", file=sys.stderr)
        return 2

    validator = ManifestValidator(manifest, manifest_path.parent)
    is_valid = validator.validate()

    if is_valid:
        print(f"✓ Valid manifest: {manifest_path}")
        return 0
    else:
        print(f"✗ Invalid manifest: {manifest_path}")
        print(f"  {len(validator.errors)} error(s):")
        for error in validator.errors:
            print(f"    - {error}")
        return 1


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <manifest.json>", file=sys.stderr)
        sys.exit(2)

    manifest_path = Path(sys.argv[1])
    exit_code = validate_manifest(manifest_path)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
