tmux new-session -d -s cougars -n "coug"
tmux send-keys -t cougars:coug.0 "date" Enter
# Start the main process (replace this with your actual process)

/usr/sbin/sshd -D

exec "$@"
