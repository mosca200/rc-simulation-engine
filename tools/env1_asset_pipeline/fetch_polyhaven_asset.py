"""ENV1-A: reproducible acquisition of the Poly Haven `sparse_grass` sources.

Queries the Poly Haven public API, saves both verbatim JSON payloads, downloads
the documented source maps, and verifies every download against the size and
MD5 the API publishes before recording a local SHA-256. Any mismatch fails
closed: the receipt is still written, but the exit code is non-zero.

The source cache lives under ``tmp/`` and is gitignored — the 4k sources total
roughly 212 MB and are re-downloadable, so they are never committed.

Standard library only.

Usage:
    python -X utf8 tools/env1_asset_pipeline/fetch_polyhaven_asset.py
    python -X utf8 tools/env1_asset_pipeline/fetch_polyhaven_asset.py --offline
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time
import urllib.request

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from env1_assets import (  # noqa: E402
    ATTRIBUTION,
    SOURCE_FORMAT,
    SOURCE_RESOLUTION,
    USER_AGENT,
    Env1AssetError,
    configure_streams,
    descriptor_for_slug,
    md5_file,
    sha256_file,
    source_cache_dir_for,
)

RECEIPT_NAME = f"fetch_receipt_{SOURCE_RESOLUTION}_{SOURCE_FORMAT}.json"
TIMEOUT_SECONDS = 600


def fetch_json(url: str) -> bytes:
    """GET a URL with an identifying User-Agent and return the raw bytes."""
    request = urllib.request.Request(
        url, headers={"User-Agent": USER_AGENT, "Accept": "application/json"}
    )
    with urllib.request.urlopen(request, timeout=120) as response:
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


def load_api_payloads(cache: pathlib.Path, offline: bool, urls: dict) -> dict[str, dict]:
    """Load (or fetch) the verbatim info/files API payloads."""
    api_dir = cache / "api"
    api_dir.mkdir(parents=True, exist_ok=True)
    payloads: dict[str, dict] = {}
    for name, url in urls.items():
        target = api_dir / f"{name}.json"
        if offline:
            if not target.is_file():
                raise Env1AssetError(
                    f"--offline was given but {target} is absent; run without "
                    "--offline first",
                    exit_code=2,
                )
            raw = target.read_bytes()
            print(f"  [offline] reusing {name}.json ({len(raw)} bytes)")
        else:
            raw = fetch_json(url)
            target.write_bytes(raw)
            print(f"  fetched {url} -> {target.name} ({len(raw)} bytes)")
        payloads[name] = {
            "path": target,
            "raw": raw,
            "sha256": sha256_file(target),
            "json": json.loads(raw.decode("utf-8")),
        }
    return payloads


def main(argv: list[str] | None = None) -> int:
    configure_streams()
    parser = argparse.ArgumentParser(
        prog="fetch_polyhaven_asset.py",
        description=(
            "Acquire and verify the Poly Haven source maps of one registered "
            "open asset into the gitignored ENV1 source cache."
        ),
    )
    parser.add_argument(
        "--slug",
        metavar="SLUG",
        default="sparse_grass",
        help="registered open asset to acquire (default sparse_grass)",
    )
    parser.add_argument(
        "--offline",
        action="store_true",
        help="reuse the cached API payloads instead of contacting Poly Haven",
    )
    parser.add_argument(
        "--cache-dir",
        metavar="PATH",
        default=None,
        help="source cache root (default: the registered cache of --slug)",
    )
    args = parser.parse_args(argv)

    descriptor = descriptor_for_slug(args.slug)
    api_urls = {"info": descriptor.info_url, "files": descriptor.files_url}

    cache = (
        pathlib.Path(args.cache_dir).resolve()
        if args.cache_dir
        else source_cache_dir_for(slug=descriptor.slug)
    )
    print(f"ENV1 source acquisition: {descriptor.slug} ({ATTRIBUTION})")
    print(f"  cache: {cache}")

    try:
        payloads = load_api_payloads(cache, args.offline, api_urls)
        files = payloads["files"]["json"]
        info = payloads["info"]["json"]

        source_dir = cache / SOURCE_RESOLUTION
        source_dir.mkdir(parents=True, exist_ok=True)

        receipt: dict[str, object] = {
            "provider": "Poly Haven",
            "slug": descriptor.slug,
            "asset_id": descriptor.asset_id,
            "attribution": ATTRIBUTION,
            "resolution": SOURCE_RESOLUTION,
            "format": SOURCE_FORMAT,
            "user_agent": USER_AGENT,
            "info_url": descriptor.info_url,
            "files_url": descriptor.files_url,
            "info_payload_sha256": payloads["info"]["sha256"],
            "files_payload_sha256": payloads["files"]["sha256"],
            "api_files_hash": info.get("files_hash"),
            "asset_name": info.get("name"),
            "authors": info.get("authors"),
            "fetched_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "files": [],
        }

        failures = 0
        for map_type, role, file_name in descriptor.source_maps:
            print(f"\n=== {map_type} ({role}) -> {file_name} ===")
            try:
                leaf = files[map_type][SOURCE_RESOLUTION][SOURCE_FORMAT]
            except KeyError as error:
                raise Env1AssetError(
                    f"the files API has no {map_type}/{SOURCE_RESOLUTION}/"
                    f"{SOURCE_FORMAT} leaf (missing {error})"
                ) from error

            url = leaf["url"]
            expected_size = int(leaf["size"])
            expected_md5 = leaf.get("md5")
            target = source_dir / file_name
            published_name = url.rsplit("/", 1)[-1]
            if published_name != file_name:
                raise Env1AssetError(
                    f"unexpected source file name: the API offers {published_name!r} "
                    f"but this tool documents {file_name!r}"
                )

            if target.is_file() and target.stat().st_size == expected_size:
                print(f"  present ({expected_size} bytes), skipping download")
            else:
                written = download(url, target)
                print(f"  downloaded {written} bytes from {url}")

            actual_size = target.stat().st_size
            actual_md5 = md5_file(target)
            actual_sha = sha256_file(target)
            size_ok = actual_size == expected_size
            md5_ok = expected_md5 is None or actual_md5 == expected_md5
            print(f"  size   {actual_size} (API {expected_size}) -> "
                  f"{'OK' if size_ok else 'MISMATCH'}")
            print(f"  md5    {actual_md5} (API {expected_md5}) -> "
                  f"{'OK' if md5_ok else 'MISMATCH'}")
            print(f"  sha256 {actual_sha}")
            if not (size_ok and md5_ok):
                failures += 1

            receipt["files"].append(
                {
                    "map_type": map_type,
                    "role": role,
                    "file_name": file_name,
                    "url": url,
                    "api_size": expected_size,
                    "api_md5": expected_md5,
                    "local_size": actual_size,
                    "local_md5": actual_md5,
                    "local_sha256": actual_sha,
                    "size_verified": size_ok,
                    "md5_verified": md5_ok,
                }
            )

        receipt["all_verified"] = failures == 0
        receipt["api_payloads"] = {
            name: {
                "url": url,
                "path": str(payloads[name]["path"].relative_to(cache)).replace(
                    "\\", "/"
                ),
                "sha256": payloads[name]["sha256"],
            }
            for name, url in api_urls.items()
        }

        receipt_path = cache / RECEIPT_NAME
        receipt_path.write_text(
            json.dumps(receipt, indent=2) + "\n", encoding="utf-8"
        )
        print(f"\nwrote {receipt_path}")
        print(ATTRIBUTION)
        if failures:
            print(
                f"\nFAIL: {failures} source file(s) did not match the API digest",
                file=sys.stderr,
            )
            return 1
        print("\nall source files verified against the Poly Haven API")
        return 0
    except Env1AssetError as error:
        print(f"error: {error.message}", file=sys.stderr)
        return error.exit_code
    except urllib.error.URLError as error:
        print(f"error: network failure contacting Poly Haven: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
