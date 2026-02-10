#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "==> Syncing submodules"
git -C "${ROOT_DIR}" submodule sync --recursive

echo "==> Initializing top-level submodules"
git -C "${ROOT_DIR}" submodule update --init

if [[ -d "${ROOT_DIR}/third_party/NavDP" ]]; then
  if git -C "${ROOT_DIR}/third_party/NavDP" ls-tree -d HEAD baselines/logoplanner/Pi3 >/dev/null 2>&1; then
    if [[ -f "${ROOT_DIR}/third_party/NavDP/.gitmodules" ]]; then
      echo "==> Initializing nested submodules in third_party/NavDP"
      git -C "${ROOT_DIR}/third_party/NavDP" submodule sync --recursive
      git -C "${ROOT_DIR}/third_party/NavDP" submodule update --init --recursive
    else
      echo "==> NavDP has Pi3 gitlink without .gitmodules; cloning Pi3 manually"
      pi3_commit="$(git -C "${ROOT_DIR}/third_party/NavDP" ls-tree HEAD baselines/logoplanner/Pi3 | awk '{print $3}')"
      pi3_dir="${ROOT_DIR}/third_party/NavDP/baselines/logoplanner/Pi3"

      if [[ -z "${pi3_commit}" ]]; then
        echo "ERROR: Unable to resolve Pi3 gitlink commit from NavDP."
        exit 1
      fi

      if [[ ! -d "${pi3_dir}/.git" ]]; then
        rm -rf "${pi3_dir}"
        git clone https://github.com/yyfz/Pi3 "${pi3_dir}"
      fi

      if ! git -C "${pi3_dir}" checkout -q "${pi3_commit}"; then
        echo "==> Fetching full history for Pi3 to reach ${pi3_commit}"
        git -C "${pi3_dir}" fetch --all --tags
        git -C "${pi3_dir}" checkout -q "${pi3_commit}"
      fi
    fi
  fi
fi

while IFS= read -r path; do
  if [[ -d "${ROOT_DIR}/${path}" && -f "${ROOT_DIR}/${path}/.gitmodules" ]]; then
    echo "==> Initializing nested submodules in ${path}"
    git -C "${ROOT_DIR}/${path}" submodule sync --recursive
    git -C "${ROOT_DIR}/${path}" submodule update --init --recursive
  fi
done < <(git -C "${ROOT_DIR}" config -f .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}')

echo "==> Third-party fetch complete"
