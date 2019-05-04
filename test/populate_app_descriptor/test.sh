#!/bin/bash

set -e

../../populate_app_descriptor.py test.image-1.0.bin

# Padded correctly? File created?
size=$(stat --printf="%s" *.application.bin)
[ $(($size % 8)) -eq 0 ]
