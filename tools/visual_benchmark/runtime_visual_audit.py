#!/usr/bin/env python3
"""Strict stdlib reader for RuntimeVisualAudit 1.0.0."""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


AUDIT_KIND = "runtime_visual_audit"
AUDIT_SCHEMA_VERSION = "1.0.0"
PASS_IDS = (
    "shadow_near",
    "shadow_mid",
    "shadow_far",
    "scene",
    "temporal_resolve",
    "postprocess",
)

GROUP_FIELDS = {
    "identity": {
        "presentation_frame_index", "framebuffer_width", "framebuffer_height",
        "renderer_version",
    },
    "device": {"adapter_name", "backend", "driver", "driver_info"},
    "environment": {
        "environment_mode", "physical_atmosphere_active", "physical_ibl_active",
        "aerial_perspective_active",
    },
    "image_pipeline": {
        "hdr_scene_format", "exposure_ev", "tone_mapper", "temporal_resolve_active",
    },
    "shadows": {
        "path_active", "cascade_count", "map_resolution", "split_distances_m",
        "filtering", "filter_tap_count", "filter_tap_count_unavailable_reason",
    },
    "terrain": {
        "render_mode", "debug_mode", "material_path_active", "sampler_anisotropy",
        "material_scale", "material_scale_unavailable_reason",
    },
    "vegetation": {"vegetation_present", "debug_mode", "stats"},
    "profiling": {
        "presentation_frame_index", "cpu_frame_duration_ns", "gpu_timing_status",
        "gpu_timing_source_presentation_frame_index", "gpu_timing_frame_age",
        "gpu_timing_unavailable_reason", "passes",
    },
}
ROOT_FIELDS = {"schema_version", *GROUP_FIELDS}
VEGETATION_STATS_FIELDS = {
    "total", "visible", "culled_frustum", "culled_distance", "lod_counts",
    "scene_draw_calls", "shadow_draw_calls", "uploaded_instance_bytes",
}
PASS_FIELDS = {"pass_id", "label", "cpu_duration_ns", "gpu_duration_ns"}
GPU_STATUSES = {
    "timestamp_query_unsupported",
    "asynchronous_result_not_ready",
    "current_frame_sample",
    "previous_frame_sample",
}


@dataclass(frozen=True)
class RuntimeVisualAudit:
    """A structurally and semantically valid audit payload."""

    payload: dict

    def to_json(self) -> dict:
        return self.payload

    @property
    def identity(self) -> dict:
        return self.payload["identity"]


def _typename(value: Any) -> str:
    return "null" if value is None else type(value).__name__


def _exact_object(errors: list, path: str, value: Any, fields: set) -> Optional[dict]:
    if not isinstance(value, dict):
        errors.append(f"{path}: must be an object, got {_typename(value)}")
        return None
    unknown = sorted(set(value) - fields)
    missing = sorted(fields - set(value))
    if unknown:
        errors.append(f"{path}: unknown field(s) {unknown}")
    if missing:
        errors.append(f"{path}: missing required field(s) {missing}")
    return value


def _integer(errors: list, path: str, value: Any, minimum: int = 0) -> Optional[int]:
    if not isinstance(value, int) or isinstance(value, bool):
        errors.append(f"{path}: must be an integer, got {_typename(value)}")
        return None
    if value < minimum:
        errors.append(f"{path}: must be >= {minimum}, got {value}")
        return None
    return value


def _number(errors: list, path: str, value: Any, minimum: Optional[float] = None) -> Optional[float]:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        errors.append(f"{path}: must be a finite number, got {_typename(value)}")
        return None
    if minimum is not None and value < minimum:
        errors.append(f"{path}: must be >= {minimum}, got {value}")
        return None
    return value


def _string(errors: list, path: str, value: Any, *, nonempty: bool = True) -> Optional[str]:
    if not isinstance(value, str):
        errors.append(f"{path}: must be a string, got {_typename(value)}")
        return None
    if nonempty and not value.strip():
        errors.append(f"{path}: must not be empty")
        return None
    return value


def _boolean(errors: list, path: str, value: Any) -> Optional[bool]:
    if not isinstance(value, bool):
        errors.append(f"{path}: must be a boolean, got {_typename(value)}")
        return None
    return value


def _nullable_measurement(
    errors: list,
    value_path: str,
    value: Any,
    reason_path: str,
    reason: Any,
) -> None:
    if value is None:
        _string(errors, reason_path, reason)
    else:
        _number(errors, value_path, value, minimum=0)
        if reason is not None:
            errors.append(f"{reason_path}: must be null when {value_path} is available")


def parse_runtime_visual_audit(payload: Any) -> tuple:
    """Return ``(RuntimeVisualAudit | None, errors)`` and never coerce data."""
    errors: list = []
    root = _exact_object(errors, "audit", payload, ROOT_FIELDS)
    if root is None:
        return None, errors
    if root.get("schema_version") != AUDIT_SCHEMA_VERSION:
        errors.append(
            f"schema_version: expected '{AUDIT_SCHEMA_VERSION}', got "
            f"{root.get('schema_version')!r}"
        )

    groups = {
        name: _exact_object(errors, name, root.get(name), fields)
        for name, fields in GROUP_FIELDS.items()
    }

    identity = groups["identity"]
    if identity is not None:
        _integer(errors, "identity.presentation_frame_index", identity.get("presentation_frame_index"))
        _integer(errors, "identity.framebuffer_width", identity.get("framebuffer_width"), 1)
        _integer(errors, "identity.framebuffer_height", identity.get("framebuffer_height"), 1)
        _string(errors, "identity.renderer_version", identity.get("renderer_version"))

    device = groups["device"]
    if device is not None:
        _string(errors, "device.adapter_name", device.get("adapter_name"))
        _string(errors, "device.backend", device.get("backend"))
        _string(errors, "device.driver", device.get("driver"), nonempty=False)
        _string(errors, "device.driver_info", device.get("driver_info"), nonempty=False)

    environment = groups["environment"]
    if environment is not None:
        mode = _string(errors, "environment.environment_mode", environment.get("environment_mode"))
        if mode not in (None, "physical", "analytical_fallback"):
            errors.append("environment.environment_mode: expected 'physical' or 'analytical_fallback'")
        atmosphere = _boolean(errors, "environment.physical_atmosphere_active", environment.get("physical_atmosphere_active"))
        ibl = _boolean(errors, "environment.physical_ibl_active", environment.get("physical_ibl_active"))
        aerial = _boolean(errors, "environment.aerial_perspective_active", environment.get("aerial_perspective_active"))
        if mode == "analytical_fallback" and any(value is True for value in (atmosphere, ibl, aerial)):
            errors.append("environment: analytical_fallback cannot report physical paths active")
        # Inverse of the rule above. environment_from_runtime derives the mode
        # and both physical flags from the same V2EnvironmentMode, so "physical"
        # with either flag off is internally contradictory rather than a runtime
        # state. aerial_perspective_active is deliberately NOT implied: it is a
        # separately reported runtime path that physical mode may leave off.
        if mode == "physical":
            if atmosphere is False:
                errors.append(
                    "environment.physical_atmosphere_active: must be true when "
                    "environment_mode is 'physical'"
                )
            if ibl is False:
                errors.append(
                    "environment.physical_ibl_active: must be true when "
                    "environment_mode is 'physical'"
                )

    image = groups["image_pipeline"]
    if image is not None:
        _string(errors, "image_pipeline.hdr_scene_format", image.get("hdr_scene_format"))
        _number(errors, "image_pipeline.exposure_ev", image.get("exposure_ev"))
        _string(errors, "image_pipeline.tone_mapper", image.get("tone_mapper"))
        _boolean(errors, "image_pipeline.temporal_resolve_active", image.get("temporal_resolve_active"))

    shadows = groups["shadows"]
    if shadows is not None:
        _boolean(errors, "shadows.path_active", shadows.get("path_active"))
        count = _integer(errors, "shadows.cascade_count", shadows.get("cascade_count"))
        _integer(errors, "shadows.map_resolution", shadows.get("map_resolution"), 1)
        splits = shadows.get("split_distances_m")
        if not isinstance(splits, list):
            errors.append("shadows.split_distances_m: must be an array")
        else:
            if count is not None and len(splits) != count:
                errors.append("shadows.split_distances_m: length must equal cascade_count")
            for index, value in enumerate(splits):
                _number(errors, f"shadows.split_distances_m[{index}]", value, minimum=0)
        _string(errors, "shadows.filtering", shadows.get("filtering"))
        _nullable_measurement(
            errors, "shadows.filter_tap_count", shadows.get("filter_tap_count"),
            "shadows.filter_tap_count_unavailable_reason",
            shadows.get("filter_tap_count_unavailable_reason"),
        )

    terrain = groups["terrain"]
    if terrain is not None:
        _string(errors, "terrain.render_mode", terrain.get("render_mode"))
        _string(errors, "terrain.debug_mode", terrain.get("debug_mode"))
        _boolean(errors, "terrain.material_path_active", terrain.get("material_path_active"))
        _integer(errors, "terrain.sampler_anisotropy", terrain.get("sampler_anisotropy"), 1)
        _nullable_measurement(
            errors, "terrain.material_scale", terrain.get("material_scale"),
            "terrain.material_scale_unavailable_reason",
            terrain.get("material_scale_unavailable_reason"),
        )

    vegetation = groups["vegetation"]
    if vegetation is not None:
        present = _boolean(errors, "vegetation.vegetation_present", vegetation.get("vegetation_present"))
        _string(errors, "vegetation.debug_mode", vegetation.get("debug_mode"))
        stats = vegetation.get("stats")
        if present is True and stats is None:
            errors.append("vegetation.stats: must be present when vegetation_present is true")
        if present is False and stats is not None:
            errors.append("vegetation.stats: must be null when vegetation_present is false")
        if stats is not None:
            parsed_stats = _exact_object(errors, "vegetation.stats", stats, VEGETATION_STATS_FIELDS)
            if parsed_stats is not None:
                for field in VEGETATION_STATS_FIELDS - {"lod_counts"}:
                    _integer(errors, f"vegetation.stats.{field}", parsed_stats.get(field))
                lod_counts = parsed_stats.get("lod_counts")
                if not isinstance(lod_counts, list) or len(lod_counts) != 3:
                    errors.append("vegetation.stats.lod_counts: must be an array of three integers")
                else:
                    for index, value in enumerate(lod_counts):
                        _integer(errors, f"vegetation.stats.lod_counts[{index}]", value)

    profiling = groups["profiling"]
    if profiling is not None:
        frame = _integer(errors, "profiling.presentation_frame_index", profiling.get("presentation_frame_index"))
        _integer(errors, "profiling.cpu_frame_duration_ns", profiling.get("cpu_frame_duration_ns"))
        status = _string(errors, "profiling.gpu_timing_status", profiling.get("gpu_timing_status"))
        if status not in GPU_STATUSES and status is not None:
            errors.append(f"profiling.gpu_timing_status: unsupported value '{status}'")
        source = profiling.get("gpu_timing_source_presentation_frame_index")
        age = profiling.get("gpu_timing_frame_age")
        reason = profiling.get("gpu_timing_unavailable_reason")
        if status in ("timestamp_query_unsupported", "asynchronous_result_not_ready"):
            if source is not None or age is not None:
                errors.append("profiling: unavailable GPU timing must have null source frame and age")
            _string(errors, "profiling.gpu_timing_unavailable_reason", reason)
        elif status in ("current_frame_sample", "previous_frame_sample"):
            parsed_source = _integer(errors, "profiling.gpu_timing_source_presentation_frame_index", source)
            parsed_age = _integer(errors, "profiling.gpu_timing_frame_age", age)
            if reason is not None:
                errors.append("profiling.gpu_timing_unavailable_reason: must be null for a sample")
            if frame is not None and parsed_source is not None and parsed_age is not None:
                if parsed_source > frame or parsed_age != frame - parsed_source:
                    errors.append("profiling: GPU source frame/age is inconsistent with presentation frame")
                if status == "current_frame_sample" and parsed_age != 0:
                    errors.append("profiling: current_frame_sample must have age 0")
                if status == "previous_frame_sample" and parsed_age == 0:
                    errors.append("profiling: previous_frame_sample must have positive age")
        passes = profiling.get("passes")
        if not isinstance(passes, list):
            errors.append("profiling.passes: must be an array")
        else:
            seen = []
            for index, item in enumerate(passes):
                parsed_pass = _exact_object(errors, f"profiling.passes[{index}]", item, PASS_FIELDS)
                if parsed_pass is None:
                    continue
                pass_id = _string(errors, f"profiling.passes[{index}].pass_id", parsed_pass.get("pass_id"))
                seen.append(pass_id)
                _string(errors, f"profiling.passes[{index}].label", parsed_pass.get("label"))
                _integer(errors, f"profiling.passes[{index}].cpu_duration_ns", parsed_pass.get("cpu_duration_ns"))
                gpu_ns = parsed_pass.get("gpu_duration_ns")
                if gpu_ns is not None:
                    _number(errors, f"profiling.passes[{index}].gpu_duration_ns", gpu_ns, minimum=0)
                if status in ("timestamp_query_unsupported", "asynchronous_result_not_ready") and gpu_ns is not None:
                    errors.append(f"profiling.passes[{index}].gpu_duration_ns: must be null when timing is unavailable")
            if tuple(seen) != PASS_IDS:
                errors.append(f"profiling.passes: expected ordered pass ids {list(PASS_IDS)}, got {seen}")

    if identity is not None and profiling is not None:
        if identity.get("presentation_frame_index") != profiling.get("presentation_frame_index"):
            errors.append("identity/profiling presentation_frame_index mismatch")

    if errors:
        return None, errors
    return RuntimeVisualAudit(payload), []


def load_runtime_visual_audit(path: Path) -> tuple:
    if not path.exists():
        return None, [f"runtime visual audit not found: {path}"]
    if not path.is_file():
        return None, [f"runtime visual audit is not a regular file: {path}"]
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except json.JSONDecodeError as error:
        return None, [f"runtime visual audit is not valid JSON: {error}"]
    except OSError as error:
        return None, [f"runtime visual audit could not be read: {error}"]
    return parse_runtime_visual_audit(payload)


def check_audit_expectations(
    audit: RuntimeVisualAudit,
    expected_frame: Any,
    expected_width: Any,
    expected_height: Any,
    expected_renderer: str = "v2",
) -> list:
    errors = []
    identity = audit.identity
    comparisons = (
        ("presentation_frame_index", expected_frame),
        ("framebuffer_width", expected_width),
        ("framebuffer_height", expected_height),
        ("renderer_version", expected_renderer),
    )
    for field, expected in comparisons:
        if identity.get(field) != expected:
            errors.append(
                f"identity.{field}: audit declares {identity.get(field)!r} but "
                f"the trusted runtime receipt/request expects {expected!r}"
            )
    return errors
