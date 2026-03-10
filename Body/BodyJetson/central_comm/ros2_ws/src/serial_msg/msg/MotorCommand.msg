# Each array index is one motor command entry; all arrays must be the same length.
# motor id: 0=mid_foot 1=head 2=left_shoulder 3=right_shoulder 4=left_foot 5=right_foot
uint8[<=6]   ids
bool[<=6]    enable
bool[<=6]    direction
bool[<=6]    angle_set
bool[<=6]    velocity_set
float32[<=6] angle
uint8[<=6]   velocity
