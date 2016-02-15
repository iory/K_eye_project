#!/bin/bash -eu

DLIB_VERSION="18.18"

wget -P /tmp http://dlib.net/files/dlib-$DLIB_VERSION.tar.bz2
tar jxf /tmp/dlib-$DLIB_VERSION.tar.bz2 -C /tmp
cp -r /tmp/dlib-$DLIB_VERSION/dlib .
