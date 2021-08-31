#!/usr/bin/env bash

set -o nounset

function die()
{
    echo >&2 "FAILURE: $*"
    exit 1
}

export SOFTWARE_PACKAGE_DIR=$(dirname "$0")

python -m pip --disable-pip-version-check install yakut~=0.6 || die "Failed to install Yakut"

yakut -v orchestrate "$SOFTWARE_PACKAGE_DIR/serial.orc.yaml"
result=$?
[[ $result == 222 ]] || die "Unexpected exit code: $result"
echo "Exit code $result is valid, test passed"
