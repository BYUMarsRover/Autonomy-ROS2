#!/bin/bash
# TODO: also check if docker group permissions are working
if [[ $(which docker) ]]; then
        bash "$MAPVIZ_SCRIPT_DIR/stop_server.sh"
        docker run -d -p 8080:8080 -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
        wait
        # TODO: replace sleep with waiting until the server is actually up and running
        sleep 1
    else
        bash "$MAPVIZ_SCRIPT_DIR/setup_docker.sh"
        $BASH
fi
