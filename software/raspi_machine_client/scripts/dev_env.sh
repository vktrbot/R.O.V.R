#!/usr/bin/env bash

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_ROVR_RESTORE_NOUNSET=0
if [[ $- == *u* ]]; then
  _ROVR_RESTORE_NOUNSET=1
  set +u
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
fi

if [[ -f "${ROOT_DIR}/.venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/.venv/bin/activate"
fi

VENV_SITE_PACKAGES="$(printf '%s\n' "${ROOT_DIR}"/.venv/lib/python*/site-packages | head -n1)"
if [[ ! -d "${VENV_SITE_PACKAGES}" ]]; then
  VENV_SITE_PACKAGES=""
fi

ROVR_CRC_PYTHON_SRC="${ROVR_CRC_PYTHON_SRC:-${ROOT_DIR}/communications/controller/src}"
EXTRA_PYTHONPATH=""
if [[ -d "${ROVR_CRC_PYTHON_SRC}" ]]; then
  EXTRA_PYTHONPATH="${ROVR_CRC_PYTHON_SRC}:"
fi

export PYTHONPATH="${VENV_SITE_PACKAGES}:${EXTRA_PYTHONPATH}${ROOT_DIR}/src/rovr_common:${ROOT_DIR}/src/rovr_chassis:${ROOT_DIR}/src/rovr_arm:${ROOT_DIR}/src/rovr_camera:${ROOT_DIR}/src/rovr_bringup:${PYTHONPATH:-}"
export ROVR_WORKSPACE_ROOT="${ROOT_DIR}"
export ROVR_CONFIG_DIR="${ROOT_DIR}/config"

if [[ "${_ROVR_RESTORE_NOUNSET}" -eq 1 ]]; then
  set -u
fi
