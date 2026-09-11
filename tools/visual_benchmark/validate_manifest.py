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
    def __init__(self, manifest: dict, base_path: Path):
        self.manifest = manifest
        self.base_path = base_path
        self.errors: list[ValidationError] = []

    def error(self, path: str, message: str):
        self.errors.append(ValidationError(path, message))

    def validate(self) -> bool:
        """Validate the manifest. Returns True if valid, False otherwise."""
        self._validate_schema_version()
        self._validate_scene_id()
        self._validate_description()
        self._validate_renderer()
        self._validate_scenery()
        self._validate_camera()
        self._validate_resolution()
        self._validate_exposure()
        self._validate_lighting()
        self._validate_sun()
        self._validate_aircraft()
        self._validate_warmup()
        self._validate_capture()
        self._validate_tags()
        self._validate_reference_hardware()
        self._validate_semantic_invariants()
        return len(self.errors) == 0

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

        if "version" not in renderer:
            self.error("renderer.version", "required field missing")
        elif not isinstance(renderer["version"], str):
            self.error("renderer.version", "must be a string")

        if "scenery_preset" not in renderer:
            self.error("renderer.scenery_preset", "required field missing")
        elif renderer["scenery_preset"] not in ["FlyingField", "TestField", "Empty"]:
            self.error("renderer.scenery_preset", f"must be one of ['FlyingField', 'TestField', 'Empty'], got '{renderer['scenery_preset']}'")

        if "vsync" in renderer and not isinstance(renderer["vsync"], bool):
            self.error("renderer.vsync", "must be a boolean")

        if "msaa_samples" in renderer:
            if renderer["msaa_samples"] not in [1, 2, 4, 8]:
                self.error("renderer.msaa_samples", f"must be one of [1, 2, 4, 8], got {renderer['msaa_samples']}")

        if "taa_enabled" in renderer and not isinstance(renderer["taa_enabled"], bool):
            self.error("renderer.taa_enabled", "must be a boolean")

        if "hdr_enabled" in renderer and not isinstance(renderer["hdr_enabled"], bool):
            self.error("renderer.hdr_enabled", "must be a boolean")

    def _validate_scenery(self):
        scenery = self.manifest.get("scenery")
        if not isinstance(scenery, dict):
            self.error("scenery", "must be an object")
            return

        if "time_of_day" not in scenery:
            self.error("scenery.time_of_day", "required field missing")
        elif scenery["time_of_day"] not in ["dawn", "morning", "noon", "afternoon", "dusk", "night"]:
            self.error("scenery.time_of_day", f"invalid value '{scenery['time_of_day']}'")

        if "weather" not in scenery:
            self.error("scenery.weather", "required field missing")
        elif scenery["weather"] not in ["clear", "partly_cloudy", "overcast", "fog"]:
            self.error("scenery.weather", f"invalid value '{scenery['weather']}'")

        if "wind_speed_mps" in scenery:
            wind = scenery["wind_speed_mps"]
            if not isinstance(wind, (int, float)):
                self.error("scenery.wind_speed_mps", "must be a number")
            elif wind < 0 or wind > 50:
                self.error("scenery.wind_speed_mps", f"must be in range [0, 50], got {wind}")

    def _validate_camera(self):
        camera = self.manifest.get("camera")
        if not isinstance(camera, dict):
            self.error("camera", "must be an object")
            return

        if "mode" not in camera:
            self.error("camera.mode", "required field missing")
        elif camera["mode"] not in ["perspective", "orthographic"]:
            self.error("camera.mode", f"must be 'perspective' or 'orthographic', got '{camera['mode']}'")

        if "position" not in camera:
            self.error("camera.position", "required field missing")
        else:
            self._validate_vector3("camera.position", camera["position"])

        if "orientation" in camera:
            orient = camera["orientation"]
            if not isinstance(orient, dict):
                self.error("camera.orientation", "must be an object")
            else:
                if "quaternion" in orient:
                    self._validate_quaternion("camera.orientation.quaternion", orient["quaternion"])
                elif "look_at" in orient:
                    self._validate_vector3("camera.orientation.look_at", orient["look_at"])
                    if "up" in orient:
                        self._validate_vector3("camera.orientation.up", orient["up"])
                else:
                    self.error("camera.orientation", "must have either 'quaternion' or 'look_at'")

        if "fov_deg" not in camera:
            self.error("camera.fov_deg", "required field missing")
        else:
            fov = camera["fov_deg"]
            if not isinstance(fov, (int, float)):
                self.error("camera.fov_deg", "must be a number")
            elif fov < 10 or fov > 120:
                self.error("camera.fov_deg", f"must be in range [10, 120], got {fov}")

        if "near_plane_m" in camera:
            near = camera["near_plane_m"]
            if not isinstance(near, (int, float)) or near <= 0:
                self.error("camera.near_plane_m", f"must be positive, got {near}")

        if "far_plane_m" in camera:
            far = camera["far_plane_m"]
            if not isinstance(far, (int, float)) or far <= 0:
                self.error("camera.far_plane_m", f"must be positive, got {far}")

    def _validate_resolution(self):
        res = self.manifest.get("resolution")
        if not isinstance(res, dict):
            self.error("resolution", "must be an object")
            return

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
        if not isinstance(exposure, (int, float)):
            self.error("exposure_ev", "must be a number")
        elif exposure < -10 or exposure > 20:
            self.error("exposure_ev", f"must be in range [-10, 20], got {exposure}")
        elif math.isnan(exposure) or math.isinf(exposure):
            self.error("exposure_ev", "must be finite")

    def _validate_lighting(self):
        lighting = self.manifest.get("lighting")
        if not isinstance(lighting, dict):
            self.error("lighting", "must be an object")
            return

        if "ambient_intensity" not in lighting:
            self.error("lighting.ambient_intensity", "required field missing")
        else:
            ambient = lighting["ambient_intensity"]
            if not isinstance(ambient, (int, float)):
                self.error("lighting.ambient_intensity", "must be a number")
            elif ambient < 0 or ambient > 100000:
                self.error("lighting.ambient_intensity", f"must be in range [0, 100000], got {ambient}")

        if "sun_intensity" not in lighting:
            self.error("lighting.sun_intensity", "required field missing")
        else:
            sun = lighting["sun_intensity"]
            if not isinstance(sun, (int, float)):
                self.error("lighting.sun_intensity", "must be a number")
            elif sun < 0 or sun > 200000:
                self.error("lighting.sun_intensity", f"must be in range [0, 200000], got {sun}")

        if "shadow_cascades" in lighting:
            if lighting["shadow_cascades"] not in [1, 2, 3, 4]:
                self.error("lighting.shadow_cascades", f"must be one of [1, 2, 3, 4], got {lighting['shadow_cascades']}")

        if "shadow_resolution" in lighting:
            if lighting["shadow_resolution"] not in [512, 1024, 2048, 4096]:
                self.error("lighting.shadow_resolution", f"must be one of [512, 1024, 2048, 4096], got {lighting['shadow_resolution']}")

    def _validate_sun(self):
        sun = self.manifest.get("sun")
        if not isinstance(sun, dict):
            self.error("sun", "must be an object")
            return

        if "direction" in sun:
            self._validate_vector3("sun.direction", sun["direction"], normalized=True)
        elif "azimuth_deg" in sun and "elevation_deg" in sun:
            azimuth = sun["azimuth_deg"]
            elevation = sun["elevation_deg"]
            if not isinstance(azimuth, (int, float)):
                self.error("sun.azimuth_deg", "must be a number")
            elif azimuth < 0 or azimuth > 360:
                self.error("sun.azimuth_deg", f"must be in range [0, 360], got {azimuth}")
            if not isinstance(elevation, (int, float)):
                self.error("sun.elevation_deg", "must be a number")
            elif elevation < -90 or elevation > 90:
                self.error("sun.elevation_deg", f"must be in range [-90, 90], got {elevation}")
        else:
            self.error("sun", "must have either 'direction' or both 'azimuth_deg' and 'elevation_deg'")

    def _validate_aircraft(self):
        aircraft = self.manifest.get("aircraft")
        if not isinstance(aircraft, dict):
            self.error("aircraft", "must be an object")
            return

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

        if "position" not in aircraft:
            self.error("aircraft.position", "required field missing")
        else:
            self._validate_vector3("aircraft.position", aircraft["position"])

        if "orientation" in aircraft:
            orient = aircraft["orientation"]
            if not isinstance(orient, dict):
                self.error("aircraft.orientation", "must be an object")
            else:
                if "quaternion" in orient:
                    self._validate_quaternion("aircraft.orientation.quaternion", orient["quaternion"])
                elif "euler_deg" in orient:
                    euler = orient["euler_deg"]
                    if not isinstance(euler, list) or len(euler) != 3:
                        self.error("aircraft.orientation.euler_deg", "must be an array of 3 numbers")
                    else:
                        for i, val in enumerate(euler):
                            if not isinstance(val, (int, float)):
                                self.error(f"aircraft.orientation.euler_deg[{i}]", "must be a number")

        if "throttle" in aircraft:
            throttle = aircraft["throttle"]
            if not isinstance(throttle, (int, float)):
                self.error("aircraft.throttle", "must be a number")
            elif throttle < 0 or throttle > 1:
                self.error("aircraft.throttle", f"must be in range [0, 1], got {throttle}")

        if "control_state" in aircraft:
            cs = aircraft["control_state"]
            if not isinstance(cs, dict):
                self.error("aircraft.control_state", "must be an object")
            else:
                for surface in ["aileron_deg", "elevator_deg", "rudder_deg"]:
                    if surface in cs:
                        val = cs[surface]
                        if not isinstance(val, (int, float)):
                            self.error(f"aircraft.control_state.{surface}", "must be a number")
                        elif val < -45 or val > 45:
                            self.error(f"aircraft.control_state.{surface}", f"must be in range [-45, 45], got {val}")

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
        for field in ["gpu", "driver_version", "os", "notes"]:
            if field in hw and not isinstance(hw[field], str):
                self.error(f"reference_hardware.{field}", "must be a string")

    def _validate_semantic_invariants(self):
        """Validate semantic invariants not expressible in JSON Schema."""
        # Check that near < far
        camera = self.manifest.get("camera", {})
        near = camera.get("near_plane_m", 0.1)
        far = camera.get("far_plane_m", 10000)
        if near >= far:
            self.error("camera", f"near_plane_m ({near}) must be less than far_plane_m ({far})")

        # Check that resolution aspect ratio is reasonable
        res = self.manifest.get("resolution", {})
        width = res.get("width")
        height = res.get("height")
        if width and height:
            aspect = width / height
            if aspect < 0.5 or aspect > 3.0:
                self.error("resolution", f"aspect ratio {aspect:.2f} is unusual (expected 0.5-3.0)")

    def _validate_vector3(self, path: str, vec: Any, normalized: bool = False):
        if not isinstance(vec, list) or len(vec) != 3:
            self.error(path, "must be an array of 3 numbers")
            return
        for i, val in enumerate(vec):
            if not isinstance(val, (int, float)):
                self.error(f"{path}[{i}]", "must be a number")
            elif math.isnan(val) or math.isinf(val):
                self.error(f"{path}[{i}]", "must be finite")

        if normalized:
            length = math.sqrt(sum(v * v for v in vec))
            if abs(length - 1.0) > 0.01:
                self.error(path, f"must be normalized (length {length:.3f}, expected 1.0)")

    def _validate_quaternion(self, path: str, quat: Any):
        if not isinstance(quat, list) or len(quat) != 4:
            self.error(path, "must be an array of 4 numbers")
            return
        for i, val in enumerate(quat):
            if not isinstance(val, (int, float)):
                self.error(f"{path}[{i}]", "must be a number")
            elif math.isnan(val) or math.isinf(val):
                self.error(f"{path}[{i}]", "must be finite")

        # Check unit quaternion
        length = math.sqrt(sum(v * v for v in quat))
        if abs(length - 1.0) > 0.01:
            self.error(path, f"must be a unit quaternion (length {length:.3f}, expected 1.0)")


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
