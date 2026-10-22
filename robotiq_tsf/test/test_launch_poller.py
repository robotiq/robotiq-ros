# Copyright (c) 2026 Robotiq
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Every launch file's default poller is an executable this package installs."""
import os
import pathlib

import pytest
from ament_index_python.packages import get_package_prefix
from launch import LaunchContext
from launch.launch_description_sources import (
    get_launch_description_from_python_launch_file,
)
from launch.utilities import perform_substitutions

_LAUNCH_DIR = pathlib.Path(__file__).resolve().parent.parent / "launch"
_LAUNCH_FILES = sorted(
    p for p in _LAUNCH_DIR.glob("*.launch.py") if '"poller"' in p.read_text()
)


@pytest.mark.parametrize("launch_file", _LAUNCH_FILES, ids=lambda p: p.name)
def test_default_poller_is_installed(launch_file):
    description = get_launch_description_from_python_launch_file(str(launch_file))
    (poller,) = [a for a in description.get_launch_arguments() if a.name == "poller"]
    executable = perform_substitutions(LaunchContext(), poller.default_value)
    path = (
        pathlib.Path(get_package_prefix("robotiq_tsf"))
        / "lib"
        / "robotiq_tsf"
        / executable
    )
    assert os.access(
        path, os.X_OK
    ), f"{launch_file.name}: default poller {executable} is not installed"
