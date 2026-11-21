# One entity of the truth stream (docs/design/truth-stream.md): what does not change per frame.
string name
uint32 index            # the entity's slot in the episode, assigned at spawn, never reused
string role             # ego, scenario, autopilot, agent
string kind             # ego, vehicle, pedestrian, misc_object
string subtype          # SSv2's (car, truck, pedestrian, ...)
uint32 sim_id           # the simulator's id of the body
string asset            # the simulator's model of the body
float64[3] declared_center      # the scenario's box, entity frame, m
float64[3] declared_dimensions  # length, width, height, m
float64[3] origin_offset        # x, y (m), yaw (rad): body origin from entity origin
bool has_asset_box
float64[3] asset_box_location   # the simulator's box, body frame (ROS-handed), m
float64[3] asset_box_extent     # half sizes, m
