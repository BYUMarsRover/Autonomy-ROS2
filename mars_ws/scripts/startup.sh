tmux new-session -d -s autonomy_ros2 -n "mobility"
# Enable full color output in tmux
tmux set-option -g default-terminal "screen-256color"
tmux send-keys -t autonomy_ros2:mobility.0 "date" Enter

# Enable pane labels
tmux set -g pane-border-status top
tmux set -g pane-border-format "#{pane_title}"
tmux set-option -g allow-rename off
tmux set-window-option -g allow-rename off
tmux set-window-option -g automatic-rename off
# Keep history for the mosh terminal
tmux set -g history-limit 30000
# Enable clicking between windows
tmux set -g mouse on

tmux select-pane -t 0 -T mobility

tmux send-keys "ros2 launch mobility rover_xbox_launch.py" Enter

# Start the main process (replace this with your actual process)

sudo /usr/sbin/sshd -D

exec "$@"
