set -e

source ../.env

EXTRA_BUILD_FLAGS="-D OCV" pio run -d . -t upload -p $OCV_PORT
EXTRA_BUILD_FLAGS="-D FCV" pio run -d . -t upload -p $FCV_PORT