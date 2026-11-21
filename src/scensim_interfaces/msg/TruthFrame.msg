# The world after one SSv2 step, on /scensim/truth (docs/design/truth-stream.md). Rows are the
# entities alive this frame; `index` is each row's slot (TruthEntity.index). Poses and
# velocities are flat, row-major, for readers that reshape them into tensors.
uint64 episode
uint64 frame            # SSv2's frame
int64 sim_time_ns       # the time SSv2 is told
uint64 sim_frame        # the simulator's frame of the snapshot
uint32[] index
string[] names
bool[] present          # the simulator had the body this frame; otherwise its row's values are 0
float64[] poses         # 6 per row: x, y, z (m), roll, pitch, yaw (rad); map frame
float64[] velocities    # 3 per row: world frame, m/s
float64[] angular_velocities  # 3 per row: world frame, rad/s
bool[] velocity_measured      # the world server measured it (else the bridge differenced poses)
TruthContact[] contacts
TruthSignal[] signals   # colours changed since the previous frame
