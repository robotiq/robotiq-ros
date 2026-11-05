# Sourced by docker/run.sh and sensor_install.sh. Sets TACTILE_HELPERS to the
# TSF-85 helper scripts of robotiq-sdk (udev rule, permissions, device
# discovery) at the release robotiq_tactile_sdk_vendor pins, fetching it on
# first use. The directory is missing if the fetch failed, and callers skip
# the helpers then.
_tactile_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_tactile_version="$(<"${_tactile_root}/robotiq_tactile_sdk_vendor/sdk_version")"
_tactile_dir="${_tactile_root}/build/robotiq-sdk-${_tactile_version}"
if [[ ! -d "${_tactile_dir}" ]]; then
  git -c advice.detachedHead=false clone -q --depth 1 --branch "${_tactile_version}" \
    https://github.com/robotiq/robotiq-sdk.git "${_tactile_dir}" \
    || echo "WARNING: could not fetch robotiq-sdk ${_tactile_version}; its helper scripts are unavailable." >&2
fi
TACTILE_HELPERS="${_tactile_dir}/tsf/utils/scripts"
unset _tactile_root _tactile_version _tactile_dir
