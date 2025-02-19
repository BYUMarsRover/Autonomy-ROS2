tmux new-session -d -s autonomy_ros2 -n "mobility"
tmux send-keys -t autonomy_ros2:mobility.0 "date" Enter
# Start the main process (replace this with your actual process)

/usr/sbin/sshd -D

exec "$@"
