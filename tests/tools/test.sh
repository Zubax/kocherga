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

$SCRIPT demo-le.bin -vv --assign-vcs-commit=0xbadc_0ffe_e0dd_f00d --assign-dirty=1
[ -f demo-le-0.0.badc0ffee0ddf00d.0d4366e7e1936c2e,dirty.app.bin ] || die "Output file has not been created"
echo

$SCRIPT demo-le.bin -vv --assign-vcs-commit=0xbadc_0ffe_e0dd_f00d --assign-version=2.5
[ -f demo-le-2.5.badc0ffee0ddf00d.82b1d20386026627.app.bin ] || die "Output file has not been created"
echo

$SCRIPT nonexistent -vv && die "Expected failure"
echo

$SCRIPT invalid.bin -vv && die "Expected failure"
echo

rm ./*.app.bin
