import json
import hashlib
import ipaddress
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]

EXPECTED_RELEASE_FILES = {
    ".gitattributes",
    ".gitignore",
    "API_CONTRACT.md",
    "BASELINE.md",
    "BASELINE_WALK_AND_HEAD.md",
    "CALIBRATION.md",
    "HOW_THE_TRACKER_WORKS.md",
    "MOTION_GUARD_INTEGRATION.md",
    "OPERATIONS.md",
    "PACKAGE_MANIFEST.md",
    "README.md",
    "SDK_AND_RUNTIME.md",
    "SHA256SUMS.txt",
    "TESTING_CHECKLIST.md",
    "TROUBLESHOOTING.md",
    "V10_SOURCE_NOTES.md",
    "config/README.md",
    "config/tracker.example.json",
    "pc/README.md",
    "pc/arena_runner.py",
    "pc/booster_config.py",
    "pc/booster_motion.py",
    "pc/booster_nav.py",
    "pc/track_uwb.py",
    "pc/tracker_config.py",
    "pytest.ini",
    "requirements.txt",
    "robot/README.md",
    "robot/runtime/__init__.py",
    "robot/runtime/k1_control_server.py",
    "robot/runtime/k1_guard_protocol.py",
    "robot/runtime/k1_mode_watcher.py",
    "robot/runtime/k1_motion_guard.py",
    "robot/runtime/k1_motion_zero_once.py",
    "robot/runtime/k1_runtime_health.py",
    "robot/runtime/k1_telemetry_daemon.py",
    "robot/runtime/runtime.env.example",
    "robot/runtime/runtime_manifest.json",
    "robot/runtime/systemd/k1-control-server.service",
    "robot/runtime/systemd/k1-mode-watcher.service",
    "robot/runtime/systemd/k1-motion-guard.service",
    "robot/runtime/systemd/k1-panel-v10.target",
    "robot/runtime/systemd/k1-telemetry.service",
    "tests/README.md",
    "tests/conftest.py",
    "tests/test_bundle.py",
    "tests/test_config.py",
    "tests/test_motion_client_contract.py",
    "tests/test_motion_guard.py",
    "tests/test_navigation.py",
    "tests/test_pc_tracking_safety.py",
    "tests/test_robot_runtime_safety.py",
    "tests/test_status_contract.py",
}


def _release_files() -> set[str]:
    """Return distributable files, ignoring only generated Python caches."""
    return {
        path.relative_to(ROOT).as_posix()
        for path in ROOT.rglob("*")
        if path.is_file()
        and not any(part in {"__pycache__", ".pytest_cache"} for part in path.parts)
        and path.suffix.casefold() != ".pyc"
    }


def test_partner_release_inventory_is_exact():
    actual = _release_files()
    assert actual == EXPECTED_RELEASE_FILES, (
        f"missing package files: {sorted(EXPECTED_RELEASE_FILES - actual)}; "
        f"unexpected package files: {sorted(actual - EXPECTED_RELEASE_FILES)}"
    )


def test_whole_package_sha256_manifest_matches_final_bytes():
    manifest_path = ROOT / "SHA256SUMS.txt"
    entries = {}
    for line in manifest_path.read_text(encoding="ascii").splitlines():
        if not line or line.startswith("#"):
            continue
        digest, separator, relative = line.partition("  ")
        assert separator and re.fullmatch(r"[0-9a-f]{64}", digest)
        assert relative not in entries
        entries[relative] = digest

    expected_hashed_files = EXPECTED_RELEASE_FILES - {"SHA256SUMS.txt"}
    assert set(entries) == expected_hashed_files
    for relative, expected_digest in entries.items():
        actual_digest = hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
        assert actual_digest == expected_digest, f"hash mismatch: {relative}"


def test_robot_runtime_manifest_is_exact_allowlist():
    runtime = ROOT / "robot" / "runtime"
    manifest = json.loads((runtime / "runtime_manifest.json").read_text(encoding="utf-8"))
    actual = {
        path.relative_to(runtime).as_posix()
        for path in runtime.rglob("*")
        if path.is_file() and "__pycache__" not in path.parts and not path.name.endswith(".pyc")
    }
    assert actual == set(manifest["files"])
    assert manifest["status"] == "offline_review_only_not_deployed"


def test_relative_markdown_links_resolve():
    pattern = re.compile(r"\[[^\]]+\]\(([^)]+)\)")
    broken = []
    for document in ROOT.rglob("*.md"):
        body = document.read_text(encoding="utf-8")
        for target in pattern.findall(body):
            if target.startswith(("http://", "https://", "#")):
                continue
            relative = target.split("#", 1)[0]
            if relative and not (document.parent / relative).exists():
                broken.append(f"{document.relative_to(ROOT)} -> {target}")
    assert not broken, "broken local Markdown links:\n" + "\n".join(sorted(broken))


def test_raw_private_and_capture_material_is_absent():
    forbidden_names = {"logs", "captures", "audio", "lockin", ".env", "secrets"}
    forbidden_suffixes = {".wav", ".jpg", ".jpeg", ".png", ".log", ".jsonl", ".pem", ".key"}
    offenders = []
    for path in ROOT.rglob("*"):
        if any(part.casefold() in forbidden_names for part in path.relative_to(ROOT).parts):
            offenders.append(path.relative_to(ROOT).as_posix())
        if path.is_file() and path.suffix.casefold() in forbidden_suffixes:
            offenders.append(path.relative_to(ROOT).as_posix())
    assert not sorted(set(offenders))


def test_no_known_private_addresses_or_factory_passwords_in_runtime_sources():
    ip_pattern = re.compile(
        r"(?<![0-9])(?:[0-9]{1,3}\.){3}[0-9]{1,3}(?:/[0-9]{1,2})?"
    )
    allowed_network_literals = {
        "10.0.0.0/8",
        "172.16.0.0/12",
        "192.168.0.0/16",
    }
    private_networks = tuple(
        ipaddress.ip_network(value) for value in allowed_network_literals
    )
    credential_patterns = (
        re.compile(r"AutoAddPolicy"),
        re.compile(r"password\s*[:=]\s*['\"][^'\"]+", re.IGNORECASE),
        re.compile(r"Authorization\s*[:=]\s*['\"]Bearer\s+(?!\{)"),
        re.compile(r"gh[pousr]_[A-Za-z0-9]{20,}"),
        re.compile(r"AKIA[0-9A-Z]{16}"),
        re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    )
    offenders = []
    text_suffixes = {".py", ".json", ".service", ".md", ".txt", ".ini"}
    for path in ROOT.rglob("*"):
        if not path.is_file() or path.suffix.casefold() not in text_suffixes:
            continue
        if path.resolve() == Path(__file__).resolve():
            continue
        body = path.read_text(encoding="utf-8")
        if any(pattern.search(body) for pattern in credential_patterns):
            offenders.append(path.relative_to(ROOT).as_posix())
            continue
        if "tests" in path.relative_to(ROOT).parts:
            # RFC1918 examples are necessary to exercise the live network
            # gate. Credential signatures above are still checked.
            continue
        for literal in ip_pattern.findall(body):
            try:
                value = ipaddress.ip_interface(literal)
            except ValueError:
                offenders.append(
                    f"{path.relative_to(ROOT).as_posix()} (invalid IP {literal})"
                )
                continue
            if not any(value.ip in network for network in private_networks):
                continue
            if literal in allowed_network_literals:
                continue
            offenders.append(
                f"{path.relative_to(ROOT).as_posix()} (private IP {literal})"
            )
    assert not offenders
