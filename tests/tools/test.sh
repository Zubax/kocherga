#!/bin/bash

set -o nounset
set -o xtrace

function die()
{
    echo >&2 "FAILURE: $*"
    exit 1
}

cd "${0%/*}" || die "Couldn't cd into this script's directory"

SCRIPT=../../tools/kocherga_image.py

$SCRIPT self-test || die "Self-test unsuccessful"
echo

$SCRIPT demo-le.bin -vv --assign-vcs-revision-id=0xbadc_0ffe_e0dd_f00d --assign-flag-dirty=1 --assign-flag-release=0
[ -f demo-le-1.16.badc0ffee0ddf00d.01a43c554cb3de13.app.dirty.bin ] || die "Output file has not been created"
echo

$SCRIPT demo-le.bin -vv --assign-vcs-revision-id=0xbadc_0ffe_e0dd_f00d --assign-version=2.5
[ -f demo-le-2.5.badc0ffee0ddf00d.ca47fd04aefc15b6.app.release.dirty.bin ] || die "Output file has not been created"
echo

$SCRIPT nonexistent -vv && die "Expected failure"
echo

$SCRIPT invalid.bin -vv && die "Expected failure"
echo

rm ./*.app*.bin
