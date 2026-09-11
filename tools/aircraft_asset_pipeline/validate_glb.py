#!/usr/bin/env python3
"""
RV2-7A offline GLB validator for RC Simulation Engine aircraft assets.

Validates a glTF 2.0 binary (.glb) container and, when a manifest is supplied,
the semantic asset contract that the runtime presentation layer depends on.

Design constraints (deliberate):
  * Python standard library only. No wgpu, no gltf wheel, no numpy.
  * Never mutates the file it validates.
  * Fails closed: any unmet contract is an error and the exit code is non-zero.
  * Importable: `blender_export_glb.py` calls `validate()` in-process.

Usage:
    python tools/aircraft_asset_pipeline/validate_glb.py <asset.glb> \
        --manifest tools/aircraft_asset_pipeline/acro_electric_01_manifest.json \
        [--profile production|blender_export] [--json-out FILE] [--quiet]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import struct
import sys

GLB_MAGIC = 0x46546C67
GLB_VERSION = 2
CHUNK_JSON = 0x4E4F534A
CHUNK_BIN = 0x004E4942

COMPONENT_TYPES = {
    5120: ("BYTE", 1, "b"),
    5121: ("UNSIGNED_BYTE", 1, "B"),
    5122: ("SHORT", 2, "h"),
    5123: ("UNSIGNED_SHORT", 2, "H"),
    5125: ("UNSIGNED_INT", 4, "I"),
    5126: ("FLOAT", 4, "f"),
}
TYPE_COMPONENT_COUNTS = {
    "SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4,
    "MAT2": 4, "MAT3": 9, "MAT4": 16,
}
DEFAULT_INDEX_COMPONENT_TYPES = (5121, 5123, 5125)
AXIS_NAMES = "XYZ"


class Report:
    """Collects errors / warnings / info lines and the resolved summary."""

    def __init__(self, path: str):
        self.path = path
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.info: list[str] = []
        self.summary: dict = {}

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warn(self, message: str) -> None:
        self.warnings.append(message)

    def note(self, message: str) -> None:
        self.info.append(message)

    @property
    def ok(self) -> bool:
        return not self.errors

    def as_dict(self) -> dict:
        return {
            "path": self.path,
            "ok": self.ok,
            "error_count": len(self.errors),
            "warning_count": len(self.warnings),
            "errors": self.errors,
            "warnings": self.warnings,
            "info": self.info,
            "summary": self.summary,
        }

    def render(self) -> str:
        lines = [f"validate_glb: {self.path}"]
        for key in sorted(self.summary):
            lines.append(f"  {key}: {self.summary[key]}")
        for message in self.info:
            lines.append(f"  [info] {message}")
        for message in self.warnings:
            lines.append(f"  [warn] {message}")
        for message in self.errors:
            lines.append(f"  [FAIL] {message}")
        lines.append(
            f"validate_glb: {'PASS' if self.ok else 'FAIL'} "
            f"({len(self.errors)} error(s), {len(self.warnings)} warning(s))"
        )
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# container
# ---------------------------------------------------------------------------

def _read_container(report: Report, raw: bytes):
    """Validate the 12-byte header + chunk table. Returns (json_doc, bin_bytes)."""
    if len(raw) < 12:
        report.error(f"file too small for a GLB header ({len(raw)} bytes)")
        return None, None

    magic, version, length = struct.unpack_from("<III", raw, 0)
    if magic != GLB_MAGIC:
        report.error(f"bad GLB magic 0x{magic:08X}, expected 0x{GLB_MAGIC:08X}")
        return None, None
    if version != GLB_VERSION:
        report.error(f"GLB version {version}, expected {GLB_VERSION}")
    if length != len(raw):
        report.error(f"GLB header length {length} != actual file size {len(raw)}")

    offset = 12
    json_chunk = None
    bin_chunk = None
    chunk_index = 0
    while offset < len(raw):
        if offset + 8 > len(raw):
            report.error(f"truncated chunk header at byte {offset}")
            break
        chunk_length, chunk_type = struct.unpack_from("<II", raw, offset)
        payload_start = offset + 8
        payload_end = payload_start + chunk_length
        if payload_end > len(raw):
            report.error(
                f"chunk {chunk_index} (type 0x{chunk_type:08X}) declares {chunk_length} bytes "
                f"but only {len(raw) - payload_start} remain"
            )
            break
        payload = raw[payload_start:payload_end]
        if chunk_type == CHUNK_JSON:
            if json_chunk is not None:
                report.error("more than one JSON chunk")
            json_chunk = payload
        elif chunk_type == CHUNK_BIN:
            if bin_chunk is not None:
                report.error("more than one BIN chunk")
            bin_chunk = payload
        else:
            report.error(f"unknown chunk type 0x{chunk_type:08X} at byte {offset}")
        offset = payload_end
        chunk_index += 1

    if json_chunk is None:
        report.error("no JSON chunk found")
        return None, bin_chunk
    if len(json_chunk) % 4 != 0:
        report.error(f"JSON chunk length {len(json_chunk)} is not 4-byte aligned")
    elif json_chunk and json_chunk[-1] != 0x20:
        report.warn("JSON chunk is not space-padded to a 4-byte boundary")
    if bin_chunk is not None and len(bin_chunk) % 4 != 0:
        report.error(f"BIN chunk length {len(bin_chunk)} is not 4-byte aligned")

    try:
        document = json.loads(json_chunk.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        report.error(f"JSON chunk is not valid UTF-8 JSON: {exc}")
        return None, bin_chunk
    if not isinstance(document, dict):
        report.error("JSON chunk root is not an object")
        return None, bin_chunk
    return document, bin_chunk


class Document:
    """Thin, bounds-checked view over the parsed glTF JSON + BIN chunk."""

    def __init__(self, report: Report, document: dict, bin_chunk: bytes | None):
        self.report = report
        self.json = document
        self.bin = bin_chunk or b""

    def array(self, key: str) -> list:
        value = self.json.get(key)
        if value is None:
            return []
        if not isinstance(value, list):
            self.report.error(f"top-level '{key}' must be an array")
            return []
        return value

    def get(self, collection: str, index):
        if not isinstance(index, int) or isinstance(index, bool):
            self.report.error(f"{collection} index must be an integer, got {index!r}")
            return None
        items = self.array(collection)
        if index < 0 or index >= len(items):
            self.report.error(f"{collection}[{index}] is out of range (count={len(items)})")
            return None
        item = items[index]
        if not isinstance(item, dict):
            self.report.error(f"{collection}[{index}] is not an object")
            return None
        return item

    def read_accessor(self, index):
        """Return (flat_values, accessor_type, component_type) or None."""
        accessor = self.get("accessors", index)
        if accessor is None:
            return None
        report = self.report
        a_type = accessor.get("type")
        component_type = accessor.get("componentType")
        count = accessor.get("count")
        if a_type not in TYPE_COMPONENT_COUNTS:
            report.error(f"accessor[{index}] has unsupported type {a_type!r}")
            return None
        if component_type not in COMPONENT_TYPES:
            report.error(f"accessor[{index}] has unsupported componentType {component_type!r}")
            return None
        if not isinstance(count, int) or count <= 0:
            report.error(f"accessor[{index}] has invalid count {count!r}")
            return None

        _name, component_size, fmt = COMPONENT_TYPES[component_type]
        per_element = TYPE_COMPONENT_COUNTS[a_type]
        element_bytes = component_size * per_element

        view_index = accessor.get("bufferView")
        if view_index is None:
            report.error(f"accessor[{index}] has no bufferView")
            return None
        view = self.get("bufferViews", view_index)
        if view is None:
            return None

        stride = view.get("byteStride")
        if stride is None:
            stride = element_bytes
        elif stride < element_bytes:
            report.error(
                f"bufferView[{view_index}] byteStride {stride} is smaller than the "
                f"{element_bytes}-byte accessor element"
            )
            return None

        start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
        needed = element_bytes if count == 1 else element_bytes + (count - 1) * stride
        if start + needed > len(self.bin):
            report.error(
                f"accessor[{index}] needs bytes [{start}, {start + needed}) but the BIN "
                f"chunk is only {len(self.bin)} bytes"
            )
            return None

        values: list = []
        unpack = struct.Struct("<" + fmt * per_element).unpack_from
        for i in range(count):
            values.extend(unpack(self.bin, start + i * stride))
        return values, a_type, component_type


# ---------------------------------------------------------------------------
# structural validation
# ---------------------------------------------------------------------------

def _validate_asset_header(report: Report, doc: Document) -> None:
    asset = doc.json.get("asset")
    if not isinstance(asset, dict):
        report.error("missing 'asset' object")
        return
    version = asset.get("version")
    if not isinstance(version, str) or not version.startswith("2."):
        report.error(f"asset.version {version!r} is not glTF 2.x")
    generator = asset.get("generator")
    if isinstance(generator, str):
        report.note(f"generator: {generator}")


def _validate_buffers(report: Report, doc: Document) -> None:
    buffers = doc.array("buffers")
    if not buffers:
        report.error("no buffers declared")
        return
    for i, buffer in enumerate(buffers):
        byte_length = buffer.get("byteLength")
        uri = buffer.get("uri")
        if not isinstance(byte_length, int) or byte_length <= 0:
            report.error(f"buffers[{i}].byteLength is invalid: {byte_length!r}")
            continue
        if uri is None:
            if i != 0:
                report.error(f"buffers[{i}] has no uri; only buffer 0 may use the BIN chunk")
            elif byte_length != len(doc.bin):
                report.error(
                    f"buffers[0].byteLength {byte_length} != BIN chunk length {len(doc.bin)}"
                )
        elif isinstance(uri, str) and uri.startswith("data:"):
            report.warn(f"buffers[{i}] uses a data URI; production assets must embed a BIN chunk")
        else:
            report.error(
                f"buffers[{i}] references external uri {uri!r}; aircraft assets must be self-contained"
            )


def _validate_buffer_views(report: Report, doc: Document) -> None:
    buffers = doc.array("buffers")
    for i, view in enumerate(doc.array("bufferViews")):
        byte_offset = view.get("byteOffset", 0)
        byte_length = view.get("byteLength")
        if not isinstance(byte_length, int) or byte_length <= 0:
            report.error(f"bufferViews[{i}].byteLength is invalid: {byte_length!r}")
            continue
        buffer_index = view.get("buffer")
        if not isinstance(buffer_index, int) or not 0 <= buffer_index < len(buffers):
            report.error(f"bufferViews[{i}].buffer {buffer_index!r} is out of range")
            continue
        declared = buffers[buffer_index].get("byteLength")
        if isinstance(declared, int) and byte_offset + byte_length > declared:
            report.error(
                f"bufferViews[{i}] range [{byte_offset}, {byte_offset + byte_length}) exceeds "
                f"buffers[{buffer_index}].byteLength {declared}"
            )
        if byte_offset + byte_length > len(doc.bin):
            report.error(
                f"bufferViews[{i}] range [{byte_offset}, {byte_offset + byte_length}) exceeds "
                f"the {len(doc.bin)}-byte BIN chunk"
            )
        stride = view.get("byteStride")
        if stride is not None and (
            not isinstance(stride, int) or stride < 4 or stride > 252 or stride % 4 != 0
        ):
            report.error(
                f"bufferViews[{i}].byteStride {stride!r} violates the glTF 4..252 multiple-of-4 rule"
            )


def _validate_accessors(report: Report, doc: Document) -> None:
    views = doc.array("bufferViews")
    for i, accessor in enumerate(doc.array("accessors")):
        if accessor.get("type") not in TYPE_COMPONENT_COUNTS:
            report.error(f"accessors[{i}].type {accessor.get('type')!r} is not a valid glTF type")
        if accessor.get("componentType") not in COMPONENT_TYPES:
            report.error(
                f"accessors[{i}].componentType {accessor.get('componentType')!r} is not valid"
            )
        count = accessor.get("count")
        if not isinstance(count, int) or count <= 0:
            report.error(f"accessors[{i}].count {count!r} must be a positive integer")
        view_index = accessor.get("bufferView")
        if view_index is not None and (
            not isinstance(view_index, int) or not 0 <= view_index < len(views)
        ):
            report.error(f"accessors[{i}].bufferView {view_index!r} is out of range")
        for bound_key in ("min", "max"):
            bound = accessor.get(bound_key)
            if bound is None:
                continue
            if not isinstance(bound, list):
                report.error(f"accessors[{i}].{bound_key} is not an array")
                continue
            for component in bound:
                if isinstance(component, bool) or not isinstance(component, (int, float)):
                    report.error(
                        f"accessors[{i}].{bound_key} contains a non-numeric entry: {component!r}"
                    )
                elif not math.isfinite(component):
                    report.error(
                        f"accessors[{i}].{bound_key} contains a non-finite entry: {component!r}"
                    )


def _validate_images_textures(report: Report, doc: Document, manifest: dict | None) -> None:
    images = doc.array("images")
    textures = doc.array("textures")
    samplers = doc.array("samplers")
    for i, image in enumerate(images):
        if "bufferView" in image:
            doc.get("bufferViews", image["bufferView"])
            if not image.get("mimeType"):
                report.error(f"images[{i}] uses bufferView but declares no mimeType")
        elif "uri" in image:
            report.warn(f"images[{i}] references external uri {image['uri']!r}")
        else:
            report.error(f"images[{i}] has neither bufferView nor uri")
    for i, texture in enumerate(textures):
        source = texture.get("source")
        if source is not None and (not isinstance(source, int) or not 0 <= source < len(images)):
            report.error(f"textures[{i}].source {source!r} is out of range")
        sampler = texture.get("sampler")
        if sampler is not None and (
            not isinstance(sampler, int) or not 0 <= sampler < len(samplers)
        ):
            report.error(f"textures[{i}].sampler {sampler!r} is out of range")

    if manifest is None:
        return
    contract = manifest.get("attribute_contract", {})
    if contract.get("texcoord_0_required") and not textures:
        report.error(
            "attribute_contract.texcoord_0_required is true but the asset carries no textures"
        )
    if not textures and not images and not contract.get("texcoord_0_required", False):
        report.note("asset is untextured, matching attribute_contract.texcoord_0_required=false")


# ---------------------------------------------------------------------------
# scene / primitive flattening
# ---------------------------------------------------------------------------

def _flatten_primitives(report: Report, doc: Document) -> list:
    """Flatten meshes in mesh-index order, primitives in document order (runtime contract)."""
    entries = []
    for mesh_index, mesh in enumerate(doc.array("meshes")):
        primitives = mesh.get("primitives")
        if not isinstance(primitives, list) or not primitives:
            report.error(f"meshes[{mesh_index}] has no primitives")
            continue
        for local_index, primitive in enumerate(primitives):
            if not isinstance(primitive, dict):
                report.error(f"meshes[{mesh_index}].primitives[{local_index}] is not an object")
                continue
            entries.append({"mesh_index": mesh_index, "local_index": local_index, "primitive": primitive})
    return entries


def _walk_scene(report: Report, doc: Document):
    """Return (mesh_index -> [node names], scene_index) for the default scene."""
    scenes = doc.array("scenes")
    nodes = doc.array("nodes")
    meshes = doc.array("meshes")
    if not scenes:
        report.error("no scenes declared")
        return {}, None
    scene_index = doc.json.get("scene", 0)
    if not isinstance(scene_index, int) or not 0 <= scene_index < len(scenes):
        report.error(f"'scene' index {scene_index!r} is out of range (scenes={len(scenes)})")
        return {}, None
    scene = scenes[scene_index]
    report.note(f"default scene {scene_index} name={scene.get('name')!r}")

    mesh_names: dict[int, list[str]] = {}
    visited: set[int] = set()
    roots = scene.get("nodes")
    if not isinstance(roots, list) or not roots:
        report.error(f"scenes[{scene_index}] declares no root nodes")
        return mesh_names, scene_index

    stack = list(roots)
    while stack:
        node_index = stack.pop()
        if not isinstance(node_index, int) or not 0 <= node_index < len(nodes):
            report.error(f"scene node reference {node_index!r} is out of range")
            continue
        if node_index in visited:
            report.error(f"node {node_index} is referenced twice (cycle or duplicate)")
            continue
        visited.add(node_index)
        node = nodes[node_index]
        name = node.get("name")
        label = name if isinstance(name, str) else f"<unnamed node {node_index}>"
        mesh_index = node.get("mesh")
        if mesh_index is not None:
            if not isinstance(mesh_index, int) or not 0 <= mesh_index < len(meshes):
                report.error(f"nodes[{node_index}].mesh {mesh_index!r} is out of range")
            else:
                mesh_names.setdefault(mesh_index, []).append(label)
        children = node.get("children")
        if isinstance(children, list):
            stack.extend(children)

    for mesh_index in sorted(set(range(len(meshes))) - set(mesh_names)):
        report.warn(f"meshes[{mesh_index}] is not referenced by any node of the default scene")
    return mesh_names, scene_index


def _measure_primitives(report: Report, doc: Document, entries: list, contract: dict) -> list:
    """Decode POSITION / NORMAL / index data per primitive and measure bounds."""
    required = contract.get("required", ["POSITION", "NORMAL"])
    texcoord_required = bool(contract.get("texcoord_0_required", False))
    indices_required = bool(contract.get("indices_required", True))
    allowed_index_types = tuple(contract.get("index_component_types_allowed", DEFAULT_INDEX_COMPONENT_TYPES))

    measured = []
    for entry in entries:
        mesh_index = entry["mesh_index"]
        primitive = entry["primitive"]
        where = f"meshes[{mesh_index}].primitives[{entry['local_index']}]"
        attributes = primitive.get("attributes")
        record = {
            "mesh_index": mesh_index,
            "vertices": 0,
            "indices": 0,
            "bounds_min": None,
            "bounds_max": None,
        }
        if not isinstance(attributes, dict):
            report.error(f"{where} has no attributes object")
            measured.append(record)
            continue

        for name in required:
            if name not in attributes:
                report.error(f"{where} is missing required attribute {name}")
        if texcoord_required and "TEXCOORD_0" not in attributes:
            report.error(f"{where} is missing TEXCOORD_0 required by the manifest contract")
        for name, accessor_index in attributes.items():
            doc.get("accessors", accessor_index)

        mode = primitive.get("mode", 4)
        if mode != 4:
            report.error(f"{where} mode {mode!r} is not 4 (TRIANGLES)")

        material = primitive.get("material")
        if material is None:
            report.error(f"{where} has no material assignment")
        else:
            doc.get("materials", material)

        if indices_required and "indices" not in primitive:
            report.error(f"{where} has no indices accessor")

        positions = None
        if "POSITION" in attributes:
            decoded = doc.read_accessor(attributes["POSITION"])
            if decoded is None:
                report.error(f"{where}: POSITION accessor could not be decoded")
            else:
                values, a_type, _component = decoded
                if a_type != "VEC3":
                    report.error(f"{where}: POSITION must be VEC3, got {a_type}")
                else:
                    bad = next((v for v in values if not math.isfinite(v)), None)
                    if bad is not None:
                        report.error(f"{where}: POSITION contains a non-finite component ({bad!r})")
                    else:
                        positions = values

        if "NORMAL" in attributes:
            decoded = doc.read_accessor(attributes["NORMAL"])
            if decoded is None:
                report.error(f"{where}: NORMAL accessor could not be decoded")
            else:
                values, a_type, _component = decoded
                if a_type != "VEC3":
                    report.error(f"{where}: NORMAL must be VEC3, got {a_type}")
                else:
                    worst = 0.0
                    non_finite = False
                    for i in range(0, len(values) - 2, 3):
                        nx, ny, nz = values[i], values[i + 1], values[i + 2]
                        if not (math.isfinite(nx) and math.isfinite(ny) and math.isfinite(nz)):
                            non_finite = True
                            break
                        worst = max(worst, abs(math.sqrt(nx * nx + ny * ny + nz * nz) - 1.0))
                    if non_finite:
                        report.error(f"{where}: NORMAL contains a non-finite component")
                    elif worst > 1.0e-3:
                        report.error(
                            f"{where}: NORMAL is not unit length (max deviation {worst:.3e} > 1e-3)"
                        )

        if positions is not None:
            vertex_count = len(positions) // 3
            record["vertices"] = vertex_count
            local_min = [math.inf] * 3
            local_max = [-math.inf] * 3
            for i in range(vertex_count):
                for axis in range(3):
                    value = positions[i * 3 + axis]
                    if value < local_min[axis]:
                        local_min[axis] = value
                    if value > local_max[axis]:
                        local_max[axis] = value
            record["bounds_min"] = local_min
            record["bounds_max"] = local_max

        indices_accessor = primitive.get("indices")
        if indices_accessor is not None:
            decoded = doc.read_accessor(indices_accessor)
            if decoded is None:
                report.error(f"{where}: index accessor could not be decoded")
            else:
                values, a_type, component_type = decoded
                if a_type != "SCALAR":
                    report.error(f"{where}: indices must be SCALAR, got {a_type}")
                if component_type not in allowed_index_types:
                    report.error(
                        f"{where}: index componentType {component_type} is not in the allowed "
                        f"set {allowed_index_types}"
                    )
                record["indices"] = len(values)
                if len(values) % 3 != 0:
                    report.error(f"{where}: index count {len(values)} is not a multiple of 3")
                if record["vertices"]:
                    overflow = [v for v in values if v >= record["vertices"]]
                    if overflow:
                        report.error(
                            f"{where}: {len(overflow)} index value(s) exceed the "
                            f"{record['vertices']}-vertex POSITION accessor (max {max(overflow)})"
                        )
                    degenerate = sum(
                        1
                        for t in range(0, len(values) - 2, 3)
                        if values[t] == values[t + 1]
                        or values[t + 1] == values[t + 2]
                        or values[t] == values[t + 2]
                    )
                    triangles = len(values) // 3
                    if triangles and degenerate * 1000 > triangles:
                        report.error(
                            f"{where}: {degenerate} index-degenerate triangle(s) exceed 0.1% "
                            f"of the primitive ({triangles} triangles)"
                        )
        measured.append(record)
    return measured


# ---------------------------------------------------------------------------
# orientation convention checks (no expression evaluation)
# ---------------------------------------------------------------------------

def _validate_orientation(report: Report, spec: dict, components: list, measured: list) -> None:
    if not spec:
        return
    tolerance = spec.get("tolerance_m", 0.002)
    by_semantic = {c["semantic_id"]: c["primitive_index"] for c in components}

    def bounds_of(semantic_id: str):
        index = by_semantic.get(semantic_id)
        if index is None or index >= len(measured):
            return None
        record = measured[index]
        if record["bounds_min"] is None:
            return None
        return record["bounds_min"], record["bounds_max"]

    foremost = spec.get("foremost_component")
    if foremost:
        got = bounds_of(foremost)
        if got is None:
            report.error(f"orientation check: foremost component {foremost!r} has no measured bounds")
        else:
            global_min_z = min(
                m["bounds_min"][2] for m in measured if m["bounds_min"] is not None
            )
            if got[0][2] - global_min_z > tolerance:
                report.error(
                    f"orientation check 'nose_forward_minus_z': {foremost} min Z {got[0][2]:.6f} "
                    f"is not the global minimum Z {global_min_z:.6f}; the nose may point at +Z"
                )

    up = spec.get("up_reference") or {}
    if up.get("above") and up.get("below"):
        high = bounds_of(up["above"])
        low = bounds_of(up["below"])
        if high is None or low is None:
            report.error("orientation check 'up_is_plus_y': missing component bounds")
        elif high[1][1] <= low[1][1]:
            report.error(
                f"orientation check 'up_is_plus_y': {up['above']} max Y {high[1][1]:.6f} is not "
                f"above {up['below']} max Y {low[1][1]:.6f}; the up axis may be flipped"
            )

    forward = spec.get("forward_reference") or {}
    if forward.get("front") and forward.get("behind"):
        front = bounds_of(forward["front"])
        behind = bounds_of(forward["behind"])
        if front is None or behind is None:
            report.error("orientation check 'forward_reference': missing component bounds")
        elif front[0][2] >= behind[0][2]:
            report.error(
                f"orientation check 'forward_reference': {forward['front']} min Z "
                f"{front[0][2]:.6f} is not forward of {forward['behind']} min Z {behind[0][2]:.6f}"
            )

    if spec.get("mirror_pairs_by_suffix"):
        for component in components:
            semantic_id = component["semantic_id"]
            if not semantic_id.endswith("_L"):
                continue
            sibling = semantic_id[:-2] + "_R"
            if sibling not in by_semantic:
                report.error(
                    f"orientation check 'mirror_pairs': {semantic_id} has no {sibling} sibling"
                )
                continue
            left = bounds_of(semantic_id)
            right = bounds_of(sibling)
            if left is None or right is None:
                report.error(f"orientation check 'mirror_pairs': missing bounds for {semantic_id}")
                continue
            if left[1][0] >= right[0][0]:
                report.error(
                    f"orientation check 'mirror_pairs': {semantic_id} max X {left[1][0]:.6f} is "
                    f"not left of {sibling} min X {right[0][0]:.6f}; +X must be aircraft right"
                )
            for bound, other in ((0, 1), (1, 0)):
                want = -left[bound][0]
                got = right[other][0]
                if abs(got - want) > tolerance:
                    report.error(
                        f"orientation check 'mirror_pairs': {semantic_id}/{sibling} are not "
                        f"mirrored about X=0 ({'min' if bound == 0 else 'max'} X {want:.6f} "
                        f"expected, {got:.6f} measured)"
                    )
            for axis in (1, 2):
                for bound, label in ((0, "min"), (1, "max")):
                    if abs(left[bound][axis] - right[bound][axis]) > tolerance:
                        report.error(
                            f"orientation check 'mirror_pairs': {semantic_id}/{sibling} differ on "
                            f"{label} {AXIS_NAMES[axis]} ({left[bound][axis]:.6f} vs "
                            f"{right[bound][axis]:.6f})"
                        )


# ---------------------------------------------------------------------------
# manifest-driven validation
# ---------------------------------------------------------------------------

def _validate_against_manifest(
    report: Report,
    doc: Document,
    manifest: dict,
    profile: str,
    entries: list,
    measured: list,
    mesh_names: dict,
    raw_size: int,
    sha256: str,
) -> None:
    profiles = manifest.get("export_profiles", {})
    if profile not in profiles:
        report.error(f"manifest declares no export profile {profile!r} (has {sorted(profiles)})")
        return
    active = profiles[profile]
    node_name_field = active.get("node_name_field", "legacy_node_name")
    material_order_field = active.get("material_order_field", "production")
    enforce_counts = bool(active.get("enforce_exact_counts", False))
    enforce_sha = bool(active.get("enforce_sha256", False))

    components = manifest.get("components", [])
    if not components:
        report.error("manifest declares no components")
        return

    # --- component table integrity ----------------------------------------
    indices = [c["primitive_index"] for c in components]
    if indices != list(range(len(components))):
        report.error(
            f"manifest primitive_index values must be dense ascending 0..N-1, got {indices}"
        )
    for field in ("semantic_id", node_name_field):
        values = [c.get(field) for c in components]
        repeated = sorted({v for v in values if values.count(v) > 1})
        if repeated:
            report.error(f"manifest components duplicate {field!r} values: {repeated}")
    if len(entries) != len(components):
        report.error(
            f"primitive count {len(entries)} != manifest component count {len(components)}"
        )

    # --- semantic node names in primitive order ---------------------------
    for component in components:
        semantic_id = component["semantic_id"]
        primitive_index = component["primitive_index"]
        expected_name = component.get(node_name_field)
        if not isinstance(expected_name, str) or not expected_name:
            report.error(f"component {semantic_id} has no {node_name_field!r} in the manifest")
            continue
        if primitive_index >= len(entries):
            report.error(
                f"component {semantic_id} maps to primitive {primitive_index} but the asset has "
                f"{len(entries)} primitive(s)"
            )
            continue
        mesh_index = entries[primitive_index]["mesh_index"]
        names = mesh_names.get(mesh_index, [])
        if expected_name not in names:
            report.error(
                f"primitive {primitive_index} must be node {expected_name!r} ({semantic_id}) "
                f"but resolves to {names!r}"
            )
        if len(names) > 1:
            report.warn(
                f"primitive {primitive_index} ({semantic_id}) is referenced by {len(names)} nodes "
                f"{names!r}; the runtime maps one primitive per component"
            )
        mesh = doc.get("meshes", mesh_index)
        if mesh is not None:
            primitive_count = len(mesh.get("primitives", []))
            if primitive_count != 1:
                report.error(
                    f"component {semantic_id} mesh {mesh_index} carries {primitive_count} "
                    f"primitives; the contract is one primitive per semantic component"
                )

    # --- moving surface separation ----------------------------------------
    moving = manifest.get("moving_surfaces", [])
    moving_indices = [m["primitive_index"] for m in moving]
    if len(set(moving_indices)) != len(moving_indices):
        report.error(f"manifest moving surfaces share primitive indices: {moving_indices}")
    for surface in moving:
        semantic_id = surface["semantic_id"]
        primitive_index = surface["primitive_index"]
        owner = next((c for c in components if c["semantic_id"] == semantic_id), None)
        if owner is None:
            report.error(f"moving surface {semantic_id!r} has no matching component entry")
            continue
        if owner["primitive_index"] != primitive_index:
            report.error(
                f"moving surface {semantic_id} declares primitive {primitive_index} but the "
                f"component table declares {owner['primitive_index']}"
            )
        if owner.get("moving_surface") != surface.get("surface_id"):
            report.error(
                f"component {semantic_id} moving_surface={owner.get('moving_surface')!r} "
                f"disagrees with surface_id={surface.get('surface_id')!r}"
            )
        if owner.get("family") != "control_surface":
            report.error(f"moving surface {semantic_id} must belong to the control_surface family")
        for key in ("hinge_axis_render_body", "hinge_origin_render_body_m"):
            vector = surface.get(key)
            if not isinstance(vector, list) or len(vector) != 3:
                report.error(f"moving surface {semantic_id} {key} must be a 3-element array")
            elif any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                for v in vector
            ):
                report.error(f"moving surface {semantic_id} {key} has a non-finite component")
        axis = surface.get("hinge_axis_render_body")
        if isinstance(axis, list) and len(axis) == 3 and all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
            for v in axis
        ):
            # Mirrors renderer::SurfaceHinge::new exactly: the axis only has to be
            # finite and non-degenerate, because the runtime normalizes it. A small
            # non-unit length is authored dihedral/anhedral tilt, not a defect.
            length = math.sqrt(sum(v * v for v in axis))
            if not (length > 1.0e-9):
                report.error(
                    f"moving surface {semantic_id} hinge axis {axis} is degenerate "
                    f"(|axis| = {length:.3e}); SurfaceHinge::new would reject it"
                )
            else:
                normalized = [v / length for v in axis]
                declared = surface.get("hinge_axis_normalized_render_body")
                if isinstance(declared, list) and len(declared) == 3:
                    for axis_index in range(3):
                        if abs(normalized[axis_index] - declared[axis_index]) > 1.0e-6:
                            report.error(
                                f"moving surface {semantic_id} normalized hinge axis "
                                f"[{axis_index}] {normalized[axis_index]:.9f} != manifest "
                                f"{declared[axis_index]}"
                            )
        gain = surface.get("visual_gain")
        if isinstance(gain, bool) or not isinstance(gain, (int, float)) or not math.isfinite(gain):
            report.error(f"moving surface {semantic_id} visual_gain is not finite: {gain!r}")
        if primitive_index >= len(entries):
            continue
        mesh_index = entries[primitive_index]["mesh_index"]
        names = mesh_names.get(mesh_index, [])
        if len(names) != 1:
            report.error(
                f"moving surface {semantic_id} (primitive {primitive_index}) must be owned by "
                f"exactly one node, found {names!r}; a merged or instanced surface cannot articulate"
            )

    # --- runtime mapping cross-check --------------------------------------
    runtime = manifest.get("runtime_mapping_authority", {}).get("model_json_presentation", {})
    for surface in moving:
        declared = runtime.get(surface["surface_id"])
        if declared is None:
            report.error(
                f"runtime_mapping_authority has no entry for surface {surface['surface_id']!r}"
            )
        elif declared != surface["primitive_index"]:
            report.error(
                f"surface {surface['surface_id']!r}: manifest primitive {surface['primitive_index']} "
                f"!= runtime model.json mapping {declared}"
            )

    # --- materials ---------------------------------------------------------
    material_spec = manifest.get("materials", {})
    catalog = material_spec.get("catalog", [])
    materials = doc.array("materials")
    names_in_asset = [m.get("name") for m in materials]
    names_in_manifest = [m["name"] for m in catalog]
    if len(materials) != len(catalog):
        report.error(f"material count {len(materials)} != manifest catalog {len(catalog)}")
    for expected in names_in_manifest:
        if expected not in names_in_asset:
            report.error(f"manifest material {expected!r} is missing from the asset")
    for actual in names_in_asset:
        if actual not in names_in_manifest:
            report.error(f"asset material {actual!r} is not declared in the manifest catalog")

    expected_order = material_spec.get("index_order_profiles", {}).get(material_order_field)
    if isinstance(expected_order, list) and names_in_asset != expected_order:
        report.error(
            f"material index order {names_in_asset} != manifest {material_order_field!r} profile "
            f"{expected_order}"
        )

    default_alpha = material_spec.get("alpha_mode_default", "OPAQUE")
    default_double = material_spec.get("double_sided_default", False)
    for entry in catalog:
        actual = next((m for m in materials if m.get("name") == entry["name"]), None)
        if actual is None:
            continue
        name = entry["name"]
        pbr = actual.get("pbrMetallicRoughness")
        if not isinstance(pbr, dict):
            report.error(f"material {name!r} has no pbrMetallicRoughness block")
            continue
        base = pbr.get("baseColorFactor", [1.0, 1.0, 1.0, 1.0])
        if not isinstance(base, list) or len(base) != 4:
            report.error(f"material {name!r} baseColorFactor must be a 4-element array")
        else:
            for channel in base:
                if isinstance(channel, bool) or not isinstance(channel, (int, float)):
                    report.error(f"material {name!r} baseColorFactor has a non-numeric channel")
                elif not math.isfinite(channel):
                    report.error(f"material {name!r} baseColorFactor has a non-finite channel")
                elif not 0.0 <= channel <= 1.0:
                    report.error(
                        f"material {name!r} baseColorFactor channel {channel} is outside [0, 1]"
                    )
            expected_base = entry.get("base_color_factor")
            if isinstance(expected_base, list) and len(expected_base) == 4:
                for axis, (got, want) in enumerate(zip(base, expected_base)):
                    if abs(got - want) > 1.0e-4:
                        report.error(
                            f"material {name!r} baseColorFactor[{axis}] {got} != manifest {want}"
                        )
        for factor_key, manifest_key, default in (
            ("metallicFactor", "metallic_factor", 1.0),
            ("roughnessFactor", "roughness_factor", 1.0),
        ):
            got = pbr.get(factor_key, default)
            if isinstance(got, bool) or not isinstance(got, (int, float)) or not math.isfinite(got):
                report.error(f"material {name!r} {factor_key} is not finite: {got!r}")
                continue
            if not 0.0 <= got <= 1.0:
                report.error(f"material {name!r} {factor_key} {got} is outside [0, 1]")
            want = entry.get(manifest_key)
            if isinstance(want, (int, float)) and abs(got - want) > 1.0e-4:
                report.error(f"material {name!r} {factor_key} {got} != manifest {want}")
        alpha_mode = actual.get("alphaMode", default_alpha)
        if alpha_mode not in ("OPAQUE", "MASK", "BLEND"):
            report.error(f"material {name!r} alphaMode {alpha_mode!r} is invalid")
        elif alpha_mode != entry.get("alpha_mode", default_alpha):
            report.error(
                f"material {name!r} alphaMode {alpha_mode!r} != manifest "
                f"{entry.get('alpha_mode', default_alpha)!r}"
            )
        if alpha_mode == "MASK":
            cutoff = actual.get("alphaCutoff", 0.5)
            if not isinstance(cutoff, (int, float)) or not 0.0 <= cutoff <= 1.0:
                report.error(f"material {name!r} alphaCutoff {cutoff!r} is outside [0, 1]")
        double_sided = actual.get("doubleSided", default_double)
        if not isinstance(double_sided, bool):
            report.error(f"material {name!r} doubleSided must be a boolean")
        elif double_sided != entry.get("double_sided", default_double):
            report.error(
                f"material {name!r} doubleSided {double_sided} != manifest "
                f"{entry.get('double_sided', default_double)}"
            )
        for texture_key in ("normalTexture", "occlusionTexture", "emissiveTexture"):
            if texture_key in actual:
                report.warn(
                    f"material {name!r} carries {texture_key}; RV2-7A validates no texture maps "
                    f"(RV2-8 scope)"
                )
        if pbr.get("metallicRoughnessTexture") is not None:
            report.warn(f"material {name!r} carries metallicRoughnessTexture (RV2-8 scope)")
        if actual.get("extensions"):
            report.warn(
                f"material {name!r} uses extensions {sorted(actual['extensions'])}; the renderer "
                f"path is core metallic-roughness only"
            )
    if doc.json.get("extensionsUsed"):
        report.error(
            f"extensionsUsed {doc.json['extensionsUsed']} is not permitted: the renderer consumes "
            f"core glTF 2.0 metallic-roughness only"
        )
    if doc.json.get("extensionsRequired"):
        report.error(
            f"extensionsRequired {doc.json['extensionsRequired']} would make the asset unloadable "
            f"by the production renderer"
        )

    # --- per-component material assignment --------------------------------
    for component in components:
        primitive_index = component["primitive_index"]
        if primitive_index >= len(entries):
            continue
        material_index = entries[primitive_index]["primitive"].get("material")
        if not isinstance(material_index, int) or not 0 <= material_index < len(materials):
            continue
        got = materials[material_index].get("name")
        want = component.get("material_name")
        if want is not None and got != want:
            report.error(
                f"component {component['semantic_id']} (primitive {primitive_index}) uses material "
                f"{got!r} but the manifest declares {want!r}"
            )

    # --- bounds ------------------------------------------------------------
    bounds_spec = manifest.get("bounds", {})
    tolerance = bounds_spec.get("tolerance_m", 0.002)
    absurd = bounds_spec.get("absurd_extent_limit_m", 5.0)
    valid = [m for m in measured if m["bounds_min"] is not None]
    if valid:
        global_min = [min(m["bounds_min"][a] for m in valid) for a in range(3)]
        global_max = [max(m["bounds_max"][a] for m in valid) for a in range(3)]
        report.summary["bounds_min"] = [round(v, 6) for v in global_min]
        report.summary["bounds_max"] = [round(v, 6) for v in global_max]

        expected_min = bounds_spec.get("global_min")
        expected_max = bounds_spec.get("global_max")
        if isinstance(expected_min, list) and isinstance(expected_max, list):
            for axis in range(3):
                if abs(global_min[axis] - expected_min[axis]) > tolerance:
                    report.error(
                        f"global bounds min[{AXIS_NAMES[axis]}] {global_min[axis]:.6f} != manifest "
                        f"{expected_min[axis]} (tolerance {tolerance} m)"
                    )
                if abs(global_max[axis] - expected_max[axis]) > tolerance:
                    report.error(
                        f"global bounds max[{AXIS_NAMES[axis]}] {global_max[axis]:.6f} != manifest "
                        f"{expected_max[axis]} (tolerance {tolerance} m)"
                    )
        for axis in range(3):
            extent = global_max[axis] - global_min[axis]
            if extent <= 0.0:
                report.error(f"global {AXIS_NAMES[axis]} extent is non-positive ({extent:.6f} m)")
            elif extent > absurd:
                report.error(
                    f"global {AXIS_NAMES[axis]} extent {extent:.3f} m exceeds the absurd-bounds "
                    f"limit {absurd} m; check the unit system and the applied scale"
                )
            if max(abs(global_min[axis]), abs(global_max[axis])) > absurd:
                report.error(
                    f"global {AXIS_NAMES[axis]} bounds leave the +/-{absurd} m sanity box "
                    f"(unit system or unapplied scale problem)"
                )

        for component in components:
            expected = component.get("expected_bounds")
            primitive_index = component["primitive_index"]
            if not isinstance(expected, dict) or primitive_index >= len(measured):
                continue
            record = measured[primitive_index]
            if record["bounds_min"] is None:
                report.error(
                    f"component {component['semantic_id']}: no measured bounds (missing POSITION?)"
                )
                continue
            for key in ("min", "max"):
                want = expected.get(key)
                if not isinstance(want, list) or len(want) != 3:
                    continue
                got = record["bounds_min"] if key == "min" else record["bounds_max"]
                for axis in range(3):
                    if abs(got[axis] - want[axis]) > tolerance:
                        report.error(
                            f"component {component['semantic_id']} bounds {key}"
                            f"[{AXIS_NAMES[axis]}] {got[axis]:.6f} != manifest {want[axis]} "
                            f"(tolerance {tolerance} m)"
                        )

        _validate_orientation(report, manifest.get("orientation_checks", {}), components, measured)

    # --- counts ------------------------------------------------------------
    report.summary["vertices"] = sum(m["vertices"] for m in measured)
    report.summary["indices"] = sum(m["indices"] for m in measured)
    report.summary["triangles"] = report.summary["indices"] // 3
    report.summary["primitives"] = len(entries)
    report.summary["meshes"] = len(doc.array("meshes"))
    report.summary["materials"] = len(materials)
    report.summary["nodes"] = len(doc.array("nodes"))
    report.summary["scenes"] = len(doc.array("scenes"))
    report.summary["accessors"] = len(doc.array("accessors"))
    report.summary["buffer_views"] = len(doc.array("bufferViews"))
    report.summary["buffers"] = len(doc.array("buffers"))
    report.summary["images"] = len(doc.array("images"))
    report.summary["textures"] = len(doc.array("textures"))
    report.summary["samplers"] = len(doc.array("samplers"))

    counts = manifest.get("counts", {}).get(profile, {})
    if enforce_counts:
        for key in (
            "vertices", "indices", "triangles", "primitives", "meshes", "materials",
            "nodes", "scenes", "accessors", "buffer_views", "buffers", "images", "textures",
        ):
            want = counts.get(key)
            actual = report.summary.get(key)
            if want is None or actual is None:
                continue
            if want != actual:
                report.error(
                    f"count {key}: asset has {actual}, manifest {profile!r} profile expects {want}"
                )
        want_size = counts.get("byte_size")
        if isinstance(want_size, int) and want_size != raw_size:
            report.error(f"byte size {raw_size} != manifest {profile!r} profile {want_size}")
    if enforce_sha:
        want_sha = counts.get("sha256")
        if isinstance(want_sha, str) and want_sha.lower() != sha256.lower():
            report.error(f"SHA-256 {sha256} != manifest {profile!r} profile {want_sha}")

    report.summary["asset_id"] = manifest.get("asset_id")
    report.summary["profile"] = profile
    report.summary["sha256"] = sha256
    report.summary["byte_size"] = raw_size


# ---------------------------------------------------------------------------
# semantic fingerprint (reproducibility comparison)
# ---------------------------------------------------------------------------

def fingerprint(glb_path: str) -> dict:
    """Deterministic semantic content fingerprint of a GLB.

    Two exports of the same source can differ byte-for-byte because Blender
    writes non-deterministic container metadata. This fingerprint captures only
    the semantic content the runtime and the manifest care about, so
    reproducibility can still be asserted honestly without faking byte-level
    determinism.
    """
    with open(glb_path, "rb") as handle:
        raw = handle.read()
    scratch = Report(glb_path)
    document, bin_chunk = _read_container(scratch, raw)
    if document is None:
        return {"error": "unreadable GLB container", "detail": scratch.errors}
    doc = Document(scratch, document, bin_chunk)

    entries = _flatten_primitives(scratch, doc)
    mesh_names, _scene = _walk_scene(scratch, doc)
    measured = _measure_primitives(scratch, doc, entries, {})
    materials = doc.array("materials")
    nodes = doc.array("nodes")
    meshes = doc.array("meshes")

    primitives = []
    for index, entry in enumerate(entries):
        record = measured[index] if index < len(measured) else {}
        mesh_index = entry["mesh_index"]
        primitive = entry["primitive"]
        material_index = primitive.get("material")
        material_name = None
        if isinstance(material_index, int) and 0 <= material_index < len(materials):
            material_name = materials[material_index].get("name")
        primitives.append({
            "primitive_index": index,
            "mesh_index": mesh_index,
            "mesh_name": (meshes[mesh_index] if mesh_index < len(meshes) else {}).get("name"),
            "node_names": sorted(mesh_names.get(mesh_index, [])),
            "attributes": sorted((primitive.get("attributes") or {}).keys()),
            "material_name": material_name,
            "vertices": record.get("vertices"),
            "indices": record.get("indices"),
            "bounds_min": _round_vector(record.get("bounds_min")),
            "bounds_max": _round_vector(record.get("bounds_max")),
        })

    images = []
    for index, image in enumerate(doc.array("images")):
        digest = None
        view_index = image.get("bufferView")
        if isinstance(view_index, int):
            view = doc.get("bufferViews", view_index)
            if view is not None:
                start = view.get("byteOffset", 0)
                digest = hashlib.sha256(
                    doc.bin[start:start + view.get("byteLength", 0)]
                ).hexdigest()
        images.append({"index": index, "name": image.get("name"), "sha256": digest})

    valid = [m for m in measured if m["bounds_min"] is not None]
    global_min = [min(m["bounds_min"][a] for m in valid) for a in range(3)] if valid else None
    global_max = [max(m["bounds_max"][a] for m in valid) for a in range(3)] if valid else None

    return {
        "generator": (document.get("asset") or {}).get("generator"),
        "scene_names": [s.get("name") for s in doc.array("scenes")],
        "node_order": [n.get("name") for n in nodes],
        "node_count": len(nodes),
        "mesh_count": len(doc.array("meshes")),
        "primitive_count": len(entries),
        "accessor_count": len(doc.array("accessors")),
        "buffer_view_count": len(doc.array("bufferViews")),
        "material_names": [m.get("name") for m in materials],
        "materials": [
            {
                "name": m.get("name"),
                "base_color_factor": (m.get("pbrMetallicRoughness") or {}).get("baseColorFactor"),
                "metallic_factor": (m.get("pbrMetallicRoughness") or {}).get("metallicFactor", 1.0),
                "roughness_factor": (m.get("pbrMetallicRoughness") or {}).get("roughnessFactor", 1.0),
                "alpha_mode": m.get("alphaMode", "OPAQUE"),
                "double_sided": m.get("doubleSided", False),
            }
            for m in materials
        ],
        "primitives": primitives,
        "images": images,
        "vertices_total": sum(m["vertices"] for m in measured),
        "indices_total": sum(m["indices"] for m in measured),
        "triangles_total": sum(m["indices"] for m in measured) // 3,
        "bounds_min": _round_vector(global_min),
        "bounds_max": _round_vector(global_max),
        "container": {
            "byte_size": len(raw),
            "sha256": hashlib.sha256(raw).hexdigest(),
        },
    }


def _round_vector(values):
    if values is None:
        return None
    return [round(float(v), 6) for v in values]


# ---------------------------------------------------------------------------
# entry points
# ---------------------------------------------------------------------------

def validate(glb_path: str, manifest_path: str | None = None, profile: str = "production") -> Report:
    report = Report(glb_path)
    if not os.path.isfile(glb_path):
        report.error(f"file not found: {glb_path}")
        return report
    with open(glb_path, "rb") as handle:
        raw = handle.read()
    sha256 = hashlib.sha256(raw).hexdigest()
    report.summary["sha256"] = sha256
    report.summary["byte_size"] = len(raw)

    document, bin_chunk = _read_container(report, raw)
    if document is None:
        return report

    doc = Document(report, document, bin_chunk)
    _validate_asset_header(report, doc)
    _validate_buffers(report, doc)
    _validate_buffer_views(report, doc)
    _validate_accessors(report, doc)

    manifest = None
    if manifest_path is not None:
        if not os.path.isfile(manifest_path):
            report.error(f"manifest not found: {manifest_path}")
            return report
        try:
            with open(manifest_path, "r", encoding="utf-8") as handle:
                manifest = json.load(handle)
        except (OSError, json.JSONDecodeError) as exc:
            report.error(f"manifest could not be parsed: {exc}")
            return report

    contract = (manifest or {}).get("attribute_contract", {})
    entries = _flatten_primitives(report, doc)
    mesh_names, _scene_index = _walk_scene(report, doc)
    measured = _measure_primitives(report, doc, entries, contract)
    _validate_images_textures(report, doc, manifest)

    if manifest is None:
        report.warn("no manifest supplied: only structural glTF 2.0 checks were performed")
        report.summary["primitives"] = len(entries)
        report.summary["vertices"] = sum(m["vertices"] for m in measured)
        report.summary["indices"] = sum(m["indices"] for m in measured)
        report.summary["triangles"] = report.summary["indices"] // 3
    else:
        _validate_against_manifest(
            report, doc, manifest, profile, entries, measured, mesh_names, raw_size=len(raw),
            sha256=sha256,
        )
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Offline (no wgpu) glTF 2.0 GLB validator for RC Simulation Engine aircraft assets."
    )
    parser.add_argument("glb", help="path to the .glb file to validate")
    parser.add_argument("--manifest", help="path to the semantic asset manifest JSON")
    parser.add_argument(
        "--profile", default="production",
        help="manifest export profile to enforce (default: production)",
    )
    parser.add_argument("--json-out", help="write the report as JSON to this path")
    parser.add_argument("--quiet", action="store_true", help="only print when validation fails")
    arguments = parser.parse_args(argv)

    report = validate(arguments.glb, arguments.manifest, arguments.profile)
    if arguments.json_out:
        directory = os.path.dirname(os.path.abspath(arguments.json_out))
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(arguments.json_out, "w", encoding="utf-8") as handle:
            json.dump(report.as_dict(), handle, indent=2)
            handle.write("\n")
    if not arguments.quiet or not report.ok:
        print(report.render())
    return 0 if report.ok else 1


if __name__ == "__main__":
    sys.exit(main())
