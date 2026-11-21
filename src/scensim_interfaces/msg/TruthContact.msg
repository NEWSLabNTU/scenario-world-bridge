# A reporting body touching another during the step.
string name             # the reporting entity
string other            # the other entity's name; empty for the environment or an unknown body
uint32 other_sim_id     # 0 when unknown
string other_type       # the simulator's type of the other body, or its prim path
float64 impulse         # total normal impulse over the step, N s
