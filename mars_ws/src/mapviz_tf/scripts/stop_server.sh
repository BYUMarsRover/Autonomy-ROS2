echo "Stopping docker containers"
echo "This may take a few seconds..."
docker stop $(docker ps -aq)
docker rm $(docker ps -aq)
