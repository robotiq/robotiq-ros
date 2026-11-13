#!/usr/bin/env python3
"""Keep every copy of the version equal to the repo-root VERSION file.

The repository is versioned as a whole: one version for all packages, one tag
per release. package.xml has to carry a literal version string — ament, rosdep
and bloom parse it statically, so it cannot reference a variable. mcp/ needs no
copy: its pyproject.toml reads VERSION at build time. VERSION is the
authoritative copy and this script is what propagates it; the --check mode runs
in pre-commit and CI so the copies cannot drift.

    dev/version.py                 # report the current version and any drift
    dev/version.py --check         # exit non-zero on drift (pre-commit, CI)
    dev/version.py --set 1.2.0     # write VERSION and every copy
    dev/version.py --check-tag v1.1.0   # assert a release tag matches VERSION,
                                        # and every SDK pin is a release tag
    dev/version.py --check-pins    # assert every SDK pin is an existing
                                   # robotiq-sdk release tag (the release gate)
    dev/version.py --check-pin-format   # assert every SDK pin is a tag or a
                                        # full commit SHA (every PR)

Between releases an SDK pin may be a robotiq-sdk main commit, so that ROS work
need not wait for an SDK release; a release must pin SDK release tags (see
CONTRIBUTING.md, "The SDKs"). --check-pins is that release gate.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
VERSION_FILE = REPO / "VERSION"

# <version> appears once per package.xml, and only the first occurrence is the
# package's own version, so the substitution is deliberately count-limited.
VERSION_TAG = re.compile(r"(<version>)([^<]*)(</version>)")
COPIES = {"package.xml": VERSION_TAG}
SEMVER = re.compile(r"^\d+\.\d+\.\d+$")
# robotiq-sdk release tags, as an SDK vendor package's sdk_version pins them.
SDK_RELEASE_TAG = re.compile(r"^v\d+\.\d+\.\d+$")
# A commit pin is a full SHA: vcstool fetches it with `git fetch origin <sha>`,
# which takes no abbreviation.
SDK_COMMIT = re.compile(r"^[0-9a-f]{40}$")
SDK_REPO_URL = "https://github.com/robotiq/robotiq-sdk.git"
# tactile_helpers.sh clones this pin with `git clone --branch`, which takes a
# tag or a branch but not a commit.
TAG_ONLY_PINS = {"robotiq_tactile_sdk_vendor/sdk_version"}


def version_copies():
    """Every file we own that carries a copy of the version, with its pattern.

    Enumerated through git rather than a glob: tracked files only, so colcon
    build/install trees carrying copies of package.xml stay out, and submodule
    contents (gitlinks, not trees) are excluded for free.
    """
    listed = subprocess.run(
        ["git", "-C", str(REPO), "ls-files", *(f"*{name}" for name in COPIES)],
        capture_output=True,
        text=True,
        check=True,
    )
    return sorted(
        (REPO / line, COPIES[Path(line).name]) for line in listed.stdout.split()
    )


def read_version():
    """The version from VERSION: first line that is not blank or a comment.

    The version stays on line 1 so `head -1 VERSION` remains a valid way to read
    it; the comment block listing the copies follows below.
    """
    for line in VERSION_FILE.read_text().splitlines():
        stripped = line.strip()
        if stripped and not stripped.startswith("#"):
            return stripped
    raise SystemExit(f"{VERSION_FILE} holds no version line")


def listed_copies():
    """The copy paths named in VERSION's reminder block."""
    listed = []
    for line in VERSION_FILE.read_text().splitlines():
        stripped = line.lstrip("#").strip()
        if stripped.endswith(tuple(COPIES)):
            listed.append(stripped)
    return sorted(listed)


def sdk_pins():
    """(path, pin) for every tracked sdk_version file."""
    listed = subprocess.run(
        ["git", "-C", str(REPO), "ls-files", "*sdk_version"],
        capture_output=True,
        text=True,
        check=True,
    )
    pins = [(line, (REPO / line).read_text().strip()) for line in listed.stdout.split()]
    if not pins:
        raise SystemExit("no sdk_version pins found")
    return pins


def report(problems, ok):
    """0 and `ok` when there are no problems, else 1 with each of them."""
    if not problems:
        print(ok)
        return 0
    for problem in problems:
        print(problem, file=sys.stderr)
    return 1


def check_pin_format():
    """Every pin is a release tag or a full commit SHA; tag-only pins are tags."""
    problems = []
    for path, pin in sdk_pins():
        if SDK_RELEASE_TAG.match(pin):
            continue
        if path in TAG_ONLY_PINS:
            problems.append(
                f"{path}: {pin} is not a release tag (tactile_helpers.sh clones it by name)"
            )
        elif not SDK_COMMIT.match(pin):
            problems.append(
                f"{path}: {pin} is neither a release tag nor a full 40-character commit SHA"
            )
    return report(problems, "every SDK pin is a release tag or a full commit SHA")


def remote_tag_exists(tag):
    """Whether robotiq-sdk has release tag `tag`; exits with a clear message
    when GitHub cannot be reached, rather than a traceback."""
    try:
        found = subprocess.run(
            ["git", "ls-remote", "--tags", SDK_REPO_URL, f"refs/tags/{tag}"],
            capture_output=True,
            text=True,
            check=True,
            timeout=60,
        )
    except subprocess.CalledProcessError as error:
        raise SystemExit(
            f"could not list robotiq-sdk's tags to check {tag}: {error.stderr.strip()}"
        )
    except (subprocess.TimeoutExpired, OSError) as error:
        raise SystemExit(f"could not list robotiq-sdk's tags to check {tag}: {error}")
    return bool(found.stdout.strip())


def check_pins(tag_exists=remote_tag_exists):
    """Every pin is a robotiq-sdk release tag that exists: the release gate.

    `tag_exists` asks robotiq-sdk; tests pass a stand-in.
    """
    problems = []
    for path, pin in sdk_pins():
        if not SDK_RELEASE_TAG.match(pin):
            problems.append(
                f"{path}: {pin} is not a release tag; a release must pin one"
            )
        elif not tag_exists(pin):
            problems.append(f"{path}: robotiq-sdk has no tag {pin}")
    return report(problems, "every SDK pin is an existing robotiq-sdk release tag")


def render_version_file(version):
    """VERSION's full contents: the version, then the generated reminder.

    The list is generated rather than hand-kept, and --check compares it against
    the tracked copies, so it cannot quietly go stale when a package is added or
    removed.
    """
    lines = [
        version,
        "",
        "# Authoritative version for the whole repository. Every file below carries",
        "# a copy, because ament, rosdep and bloom parse the version statically and",
        "# it cannot reference a variable (mcp/pyproject.toml reads this file instead):",
        "#",
    ]
    lines += [f"#   {p.relative_to(REPO)}" for p, _ in version_copies()]
    lines += [
        "#",
        "# Bump with `dev/version.py --set X.Y.Z`, which rewrites this file and every",
        "# copy above. A pre-commit hook fails if they drift, or if this list stops",
        "# matching the files in the repo.",
    ]
    return "\n".join(lines) + "\n"


def copy_version(path, pattern):
    match = pattern.search(path.read_text())
    return match.group(2) if match else None


def drift(expected):
    return [
        (p.relative_to(REPO), copy_version(p, pattern))
        for p, pattern in version_copies()
        if copy_version(p, pattern) != expected
    ]


def write_version(path, pattern, version):
    text = path.read_text()
    updated = pattern.sub(rf"\g<1>{version}\g<3>", text, count=1)
    if updated != text:
        path.write_text(updated)
        return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--check", action="store_true", help="fail on drift")
    group.add_argument("--set", metavar="X.Y.Z", help="set the repo version")
    group.add_argument(
        "--check-tag",
        metavar="TAG",
        help="assert TAG matches VERSION and every SDK pin is a release tag",
    )
    group.add_argument(
        "--check-pins",
        action="store_true",
        help="assert every SDK pin is an existing robotiq-sdk release tag",
    )
    group.add_argument(
        "--check-pin-format",
        action="store_true",
        help="assert every SDK pin is a release tag or a full commit SHA",
    )
    args = parser.parse_args()

    if args.set:
        if not SEMVER.match(args.set):
            parser.error(f"not a semantic version: {args.set}")
        VERSION_FILE.write_text(render_version_file(args.set))
        for path, pattern in version_copies():
            if write_version(path, pattern, args.set):
                print(f"updated {path.relative_to(REPO)}")
        print(f"repo version is now {args.set}")
        return 0

    if args.check_pins:
        return check_pins()
    if args.check_pin_format:
        return check_pin_format()

    expected = read_version()

    if args.check_tag:
        # Release tags are the repo version with a leading v.
        if args.check_tag.lstrip("vV") != expected:
            print(
                f"tag {args.check_tag} does not match VERSION ({expected})",
                file=sys.stderr,
            )
            return 1
        print(f"tag {args.check_tag} matches VERSION")
        return check_pins()

    tracked = [str(p.relative_to(REPO)) for p, _ in version_copies()]
    stale = listed_copies() != sorted(tracked)
    mismatched = drift(expected)

    if not mismatched and not stale:
        print(f"{expected}: {len(tracked)} version copies agree")
        return 0

    if stale:
        listed = set(listed_copies())
        print("VERSION's file list no longer matches the repo:", file=sys.stderr)
        for path in sorted(listed ^ set(tracked)):
            side = "listed, no longer in the repo" if path in listed else "not listed"
            print(f"  {path}: {side}", file=sys.stderr)

    if mismatched:
        print(f"VERSION says {expected}, but:", file=sys.stderr)
        for path, found in mismatched:
            print(f"  {path}: {found}", file=sys.stderr)
    if args.check:
        print("run dev/version.py --set <version> to fix", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
