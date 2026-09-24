"""PF1: reproducible acquisition of the Poly Haven `meadow` panorama sources.

Queries the Poly Haven public API, saves both verbatim JSON payloads, downloads
the three documented source files, and verifies every download against the size
and MD5 the API publishes - and against the SHA-256 / size / MD5 pinned in
``photo_field_assets.py`` when PF1 was calibrated. Any disagreement fails closed:
the receipt is still written, but the exit code is non-zero.

The source cache lives under ``tmp/`` and is gitignored: the 8k Radiance source
alone is 108 MB and is re-downloadable, so it is never committed.

``--offline`` reuses the cached API payloads and the cached downloads and never
touches the network. It fails closed when a payload or a source is absent,
because a receipt that cannot cite the API is not provenance.

Standard library only.

Usage:
    python -X utf8 tools/photo_field_pipeline/fetch_photo_field_sources.py
    python -X utf8 tools/photo_field_pipeline/fetch_photo_field_sources.py --offline
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from photo_field_assets import (  # noqa: E402
    ASSET_ID,
    ASSET_NAME,
    ATTRIBUTION,
    AUTHORS,
    FILES_URL,
    INFO_URL,
    LICENSE,
    LICENSE_URL,
    PROVIDER,
    RECEIPT_NAME,
    SLUG,
    SOURCE_FILES,
    USER_AGENT,
    PhotoFieldError,
    configure_streams,
    md5_file,
    sha256_file,
    source_cache_dir,
)

TIMEOUT_SECONDS = 3600
API_TIMEOUT_SECONDS = 120
API_DIR_NAME = "api"


def fetch_json(url: str) -> bytes:
    """GET a URL with an identifying User-Agent and return the raw bytes."""
    request = urllib.request.Request(
        url, headers={"User-Agent": USER_AGENT, "Accept": "application/json"}
    )
    with urllib.request.urlopen(request, timeout=API_TIMEOUT_SECONDS) as response:
        return response.read()


def download(url: str, destination: pathlib.Path) -> int:
    """Stream a URL to disk, returning the byte count written."""
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    written = 0
    started = time.time()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(request, timeout=TIMEOUT_SECONDS) as response:
        with destination.open("wb") as handle:
            while True:
                chunk = response.read(1 << 22)
                if not chunk:
                    break
                handle.write(chunk)
                written += len(chunk)
                elapsed = max(time.time() - started, 1e-6)
                print(
                    f"    {written / 1e6:7.1f} MB ({written / elapsed / 1e6:.2f} MB/s)",
                    end="\r",
                    flush=True,
                )
    print(" " * 48, end="\r", flush=True)
    return written


def load_api_payloads(
    cache: pathlib.Path, offline: bool
) -> dict[str, dict[str, object]]:
    """Load (or fetch) the verbatim info/files API payloads."""
    api_dir = cache / API_DIR_NAME
    api_dir.mkdir(parents=True, exist_ok=True)
    payloads: dict[str, dict[str, object]] = {}
    for name, url in (("info", INFO_URL), ("files", FILES_URL)):
        target = api_dir / f"{name}.json"
        if offline:
            if not target.is_file():
                raise PhotoFieldError(
                    f"--offline was given but {target} is absent; run once without "
                    "--offline so the API payloads are captured verbatim",
                    exit_code=2,
                )
            raw = target.read_bytes()
            print(f"  [offline] reusing {name}.json ({len(raw)} bytes)")
        else:
            raw = fetch_json(url)
            target.write_bytes(raw)
            print(f"  fetched {url} -> {target.name} ({len(raw)} bytes)")
        try:
            parsed = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise PhotoFieldError(f"{target}: the API payload is not valid JSON: {error}") from error
        payloads[name] = {
            "path": target,
            "url": url,
            "byte_size": len(raw),
            "sha256": sha256_file(target),
            "json": parsed,
        }
    return payloads


def resolve_leaf(payload: dict, api_path: tuple, role: str) -> dict:
    """Reach one download leaf of the /files payload, failing closed."""
    node: object = payload
    for key in api_path:
        if not isinstance(node, dict) or key not in node:
            raise PhotoFieldError(
                f"the /files payload has no {'/'.join(api_path)} leaf for the {role} "
                f"source (missing {key!r}); the Poly Haven layout changed and the "
                "pinned constants must be reviewed"
            )
        node = node[key]
    if not isinstance(node, dict) or "url" not in node:
        raise PhotoFieldError(
            f"the /files leaf {'/'.join(api_path)} is not a download object: {node!r}"
        )
    return node


def acquire(
    descriptor,
    leaf: dict,
    cache: pathlib.Path,
    offline: bool,
    fetched_utc: str,
) -> tuple[dict, bool]:
    """Verify (and if needed download) one source file. Returns (record, ok)."""
    url = leaf["url"]
    api_size = int(leaf["size"])
    api_md5 = leaf.get("md5")

    if url != descriptor.url:
        raise PhotoFieldError(
            f"{descriptor.role}: the API offers {url!r} but PF1 pinned "
            f"{descriptor.url!r}; refusing to fetch an unrecognised leaf"
        )
    if api_size != descriptor.api_size:
        raise PhotoFieldError(
            f"{descriptor.role}: the API size {api_size} != the pinned "
            f"{descriptor.api_size}"
        )
    if api_md5 != descriptor.api_md5:
        raise PhotoFieldError(
            f"{descriptor.role}: the API MD5 {api_md5!r} != the pinned "
            f"{descriptor.api_md5!r}"
        )

    target = cache / descriptor.file_name
    print(f"\n=== {descriptor.role} -> {descriptor.file_name} ===")
    print(f"  {descriptor.purpose}")
    downloaded = False
    if target.is_file() and target.stat().st_size == api_size:
        print(f"  present ({api_size} bytes), skipping download")
    elif offline:
        raise PhotoFieldError(
            f"--offline was given but {target} is absent or has the wrong size; run "
            "once without --offline to acquire it",
            exit_code=2,
        )
    else:
        written = download(url, target)
        downloaded = True
        print(f"  downloaded {written} bytes from {url}")

    local_size = target.stat().st_size
    local_md5 = md5_file(target)
    local_sha256 = sha256_file(target)
    size_ok = local_size == api_size
    md5_ok = local_md5 == api_md5
    pinned_ok = descriptor.pinned_sha256 is None or local_sha256 == descriptor.pinned_sha256

    print(f"  size   {local_size} (API {api_size}) -> {'OK' if size_ok else 'MISMATCH'}")
    print(f"  md5    {local_md5} (API {api_md5}) -> {'OK' if md5_ok else 'MISMATCH'}")
    print(f"  sha256 {local_sha256}")
    if descriptor.pinned_sha256 is not None:
        print(
            f"  pinned sha256 -> {'OK' if pinned_ok else 'MISMATCH'} "
            f"({descriptor.pinned_sha256})"
        )
    else:
        print("  pinned sha256 -> none recorded (decode probe, verified by API MD5)")

    record = {
        "role": descriptor.role,
        "file_name": descriptor.file_name,
        "purpose": descriptor.purpose,
        "api_path": list(descriptor.api_path),
        "published_name": url.rsplit("/", 1)[-1],
        "url": url,
        "api_size": api_size,
        "api_md5": api_md5,
        "local_size": local_size,
        "local_md5": local_md5,
        "local_sha256": local_sha256,
        "pinned_sha256": descriptor.pinned_sha256,
        "size_verified": size_ok,
        "md5_verified": md5_ok,
        "pinned_sha256_verified": pinned_ok,
        "downloaded": downloaded,
        "fetched_utc": fetched_utc,
        "user_agent": USER_AGENT,
    }
    return record, bool(size_ok and md5_ok and pinned_ok)


def main(argv: list[str] | None = None) -> int:
    configure_streams()
    parser = argparse.ArgumentParser(
        prog="fetch_photo_field_sources.py",
        description=(
            "Acquire and verify the Poly Haven `meadow` sources of the PF1 photo "
            "field into the gitignored source cache."
        ),
    )
    parser.add_argument(
        "--offline",
        action="store_true",
        help="reuse the cached API payloads and downloads instead of contacting Poly Haven",
    )
    parser.add_argument(
        "--cache-dir",
        metavar="PATH",
        default=None,
        help=f"source cache root (default {source_cache_dir()})",
    )
    args = parser.parse_args(argv)

    cache = (
        pathlib.Path(args.cache_dir).resolve() if args.cache_dir else source_cache_dir()
    )
    print(f"PF1 source acquisition: {PROVIDER} {SLUG} ({ATTRIBUTION})")
    print(f"  cache: {cache}")
    print(f"  mode: {'offline' if args.offline else 'online'}")

    try:
        cache.mkdir(parents=True, exist_ok=True)
        payloads = load_api_payloads(cache, args.offline)
        info = payloads["info"]["json"]
        files = payloads["files"]["json"]
        if not isinstance(info, dict) or not isinstance(files, dict):
            raise PhotoFieldError("the API payloads are not JSON objects")

        files_hash = info.get("files_hash")
        if not isinstance(files_hash, str) or not files_hash:
            raise PhotoFieldError("the /info payload carries no files_hash")

        fetched_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        receipt: dict[str, object] = {
            "provider": PROVIDER,
            "slug": SLUG,
            "asset_id": ASSET_ID,
            "asset_name": info.get("name") or ASSET_NAME,
            "authors": info.get("authors") or AUTHORS,
            "license": LICENSE,
            "license_url": LICENSE_URL,
            "attribution": ATTRIBUTION,
            "source_page": f"https://polyhaven.com/a/{SLUG}",
            "user_agent": USER_AGENT,
            "offline": bool(args.offline),
            "verification_basis": (
                "cached_polyhaven_api_payloads" if args.offline else "polyhaven_api"
            ),
            "info_url": INFO_URL,
            "files_url": FILES_URL,
            "info_payload_sha256": payloads["info"]["sha256"],
            "files_payload_sha256": payloads["files"]["sha256"],
            "files_hash": files_hash,
            "fetched_utc": fetched_utc,
            "api_payloads": {
                name: {
                    "url": payload["url"],
                    "path": str(pathlib.Path(str(payload["path"])).relative_to(cache)).replace(
                        "\\", "/"
                    ),
                    "byte_size": payload["byte_size"],
                    "sha256": payload["sha256"],
                }
                for name, payload in payloads.items()
            },
            "files": [],
        }

        failures = 0
        for descriptor in SOURCE_FILES:
            leaf = resolve_leaf(files, descriptor.api_path, descriptor.role)
            record, ok = acquire(descriptor, leaf, cache, args.offline, fetched_utc)
            receipt["files"].append(record)
            if not ok:
                failures += 1

        receipt["all_verified"] = failures == 0
        target = cache / RECEIPT_NAME
        target.write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
        print(f"\nwrote {target}")
        print(f"  all_verified {receipt['all_verified']}")
        print(ATTRIBUTION)
        if failures:
            print(
                f"\nFAIL: {failures} source file(s) did not match the published digests",
                file=sys.stderr,
            )
            return 1
        print("\nall PF1 source files verified against the Poly Haven API")
        return 0
    except PhotoFieldError as error:
        print(f"error: {error.message}", file=sys.stderr)
        return error.exit_code
    except urllib.error.URLError as error:
        print(
            f"error: network failure contacting Poly Haven: {error}\n"
            "  (with a populated cache, --offline re-verifies it against the "
            "cached API payloads)",
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
