# Published by obstacle_detection.py

bool is_stuck               # True if rover hasn't moved recently despite commands
bool is_wall                # True if wall like obstacle
bool center_obs             # True if +/- 0.5 m window from center contains obstacle
bool goal_obs               # True if desired heading window contains obstacle
float32 obstacle_centroid   # Average angle for all detected obstacles
float32 weight              # average of 1/(dist*xdist) for all points
float32 pos_weight
float32 neg_weight
