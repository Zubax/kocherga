#!/usr/bin/env bash

set -o nounset

function die()
{
    echo >&2 "FAILURE: $*"
    exit 1
}

export SOFTWARE_PACKAGE_DIR=$(dirname "$0")

echo "TESTING: Cyphal/serial"
yakut -v orchestrate "$SOFTWARE_PACKAGE_DIR/serial.orc.yaml"
result=$?
[[ $result == 222 ]] || die "Unexpected exit code: $result"
echo "Exit code $result is valid, test passed"

echo "TESTING: Cyphal/CAN"
yakut -v orchestrate "$SOFTWARE_PACKAGE_DIR/can.orc.yaml"
result=$?
[[ $result == 223 ]] || die "Unexpected exit code: $result"
echo "Exit code $result is valid, test passed"

echo "PLEASE TEST v0 MANUALLY! RUN THIS: $SOFTWARE_PACKAGE_DIR/manual_v0.orc.yaml"
