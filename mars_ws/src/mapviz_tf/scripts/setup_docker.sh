#!/bin/bash
# setup Docker (used for mapviz)
sudo apt install -y docker
sudo apt install -y docker.io
sudo groupadd docker
sudo gpasswd -a $USER docker
sudo service docker restart
sudo service docker.io restart
echo "docker group added; please reboot your computer to finalize changes"
