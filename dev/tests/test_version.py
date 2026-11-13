"""Tests of dev/version.py's SDK pin checks, on fake sdk_version files.

Run with `python3 -m pytest dev/tests` (the Release workflow does).
"""

import importlib.util
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "version.py"
GRIPPERS = "grippers/robotiq_grippers_sdk_vendor/sdk_version"
TACTILE = "robotiq_tactile_sdk_vendor/sdk_version"
SHA = "0ee890a1cd30b14ef8b57427d3e2e840528d1bf0"


@pytest.fixture
def version(tmp_path, monkeypatch):
    """dev/version.py, pointed at an empty git repository."""
    spec = importlib.util.spec_from_file_location("version", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    monkeypatch.setattr(module, "REPO", tmp_path)
    return module


def pin(repo, **pins):
    """Track an sdk_version file per pin: grippers=..., tactile=..."""
    paths = {"grippers": GRIPPERS, "tactile": TACTILE}
    for name, value in pins.items():
        path = repo / paths[name]
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"{value}\n")
        subprocess.run(["git", "-C", str(repo), "add", paths[name]], check=True)


def existing(*tags):
    return lambda tag: tag in tags


@pytest.mark.parametrize(
    "grippers,tactile,ok",
    [
        ("v3.0.0", "v3.0.0", True),
        (SHA, "v3.0.0", True),
        (SHA[:7], "v3.0.0", False),
        (SHA.upper(), "v3.0.0", False),
        ("v1.2.3-rc1", "v3.0.0", False),
        ("3.0.0", "v3.0.0", False),
        ("main", "v3.0.0", False),
        ("v3.0.0", SHA, False),
    ],
)
def test_pin_format_accepts_a_release_tag_or_a_full_sha(version, grippers, tactile, ok):
    pin(version.REPO, grippers=grippers, tactile=tactile)
    assert version.check_pin_format() == (0 if ok else 1)


@pytest.mark.parametrize(
    "grippers,tactile,ok",
    [
        ("v3.0.0", "v3.0.0", True),
        (SHA, "v3.0.0", False),
        ("v9.9.9", "v3.0.0", False),
        ("v3.0.0", "v9.9.9", False),
    ],
)
def test_a_release_pins_existing_release_tags(version, grippers, tactile, ok):
    pin(version.REPO, grippers=grippers, tactile=tactile)
    assert version.check_pins(existing("v3.0.0")) == (0 if ok else 1)


def test_a_commit_pin_is_not_looked_up(version):
    pin(version.REPO, grippers=SHA, tactile="v3.0.0")
    asked = []
    version.check_pins(lambda tag: asked.append(tag) or True)
    assert asked == ["v3.0.0"]


def test_the_tactile_pin_is_tag_only(version):
    assert TACTILE in version.TAG_ONLY_PINS


def test_no_pins_is_an_error(version):
    with pytest.raises(SystemExit, match="no sdk_version pins"):
        version.check_pin_format()


def test_an_unreachable_remote_exits_with_a_message(version, monkeypatch):
    def fail(*args, **kwargs):
        raise subprocess.CalledProcessError(
            128, args[0], stderr="fatal: unable to access\n"
        )

    monkeypatch.setattr(version.subprocess, "run", fail)
    with pytest.raises(
        SystemExit, match="could not list robotiq-sdk's tags to check v3.0.0"
    ):
        version.remote_tag_exists("v3.0.0")


def test_the_real_pins_are_well_formed():
    assert subprocess.run([str(SCRIPT), "--check-pin-format"]).returncode == 0
