#!/bin/bash
set -e

source /Users/Nik/cmfv-closedloop/.env

EXTRA_BUILD_FLAGS="-D OCV" pio run -d /Users/Nik/cmfv-closedloop/arduino -t upload -p $OCV_PORT
EXTRA_BUILD_FLAGS="-D FCV" pio run -d /Users/Nik/cmfv-closedloop/arduino -t upload -p $FCV_PORT