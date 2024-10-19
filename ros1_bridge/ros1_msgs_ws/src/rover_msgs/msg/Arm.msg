#This message is used to control the joints of the arm
#We have turret, shoulder, elbow, wrist, and hand (grippers), and finger push 
#The turret is controlled through a "PWM" and will have a feedback angle
#The shoulder and elbow are linear actuators that will be position controlled
#The hand is controlled through direction and speed
#True is used for forward/extend

int16 turret_pos

uint16 shoulder_pos

uint16 elbow_pos

uint8 wrist_speed
bool wrist_dir

int8 hand_speed

int8 finger_enable
