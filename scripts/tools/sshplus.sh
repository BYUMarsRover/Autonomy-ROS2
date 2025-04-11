#!/bin/bash

# Target IP and SSH user
IP="192.168.1.120"
SSH_USER="marsrover"

echo "Pinging $IP until successful..."

# Run ping in the background and direct its output to stdout
stdbuf -oL ping "$IP" &

# Store the PID of the background ping process
PING_PID=$!

# Loop until the host responds to a ping (waiting for any response)
while ! nc -z "$IP" 22; do
    sleep 1  # Wait a little before trying again
done

# Kill the background ping process once we're done
kill $PING_PID

# Once ping is successful and the host is reachable via SSH, show the result
echo "Connection made, attempting ssh..."

# Attempt SSH connection
ssh "$SSH_USER@$IP"
