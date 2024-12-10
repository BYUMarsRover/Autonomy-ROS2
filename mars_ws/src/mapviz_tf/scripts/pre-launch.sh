#!/bin/bash
SOURCE="${BASH_SOURCE[0]}"
SCRIPT_DIR="$(cd -P "$(dirname "$SOURCE")" >/dev/null 2>&1 && pwd)"
cp "$SCRIPT_DIR/.mapviz_config" ~/
export MAPVIZ_SCRIPT_DIR=$SCRIPT_DIR
bash $SCRIPT_DIR/start_server.sh
