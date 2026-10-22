# Sourced by docker/run.sh and sensor_install.sh. Sets TACTILE_HELPERS to the
# tactile_sensors helper scripts (udev rule, permissions, device discovery) at
# the release robotiq_tactile_sdk_vendor pins, fetching it on first use. The
# directory is missing if the fetch failed, and callers skip the helpers then.
_tactile_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_tactile_version="$(<"${_tactile_root}/robotiq_tactile_sdk_vendor/sdk_version")"
_tactile_dir="${_tactile_root}/build/tactile_sensors-${_tactile_version}"
if [[ ! -d "${_tactile_dir}" ]]; then
  git -c advice.detachedHead=false clone -q --depth 1 --branch "${_tactile_version}" \
    https://github.com/robotiq/tactile_sensors.git "${_tactile_dir}" \
    || echo "WARNING: could not fetch tactile_sensors ${_tactile_version}; its helper scripts are unavailable." >&2
fi
TACTILE_HELPERS="${_tactile_dir}/utils/scripts"
unset _tactile_root _tactile_version _tactile_dir
