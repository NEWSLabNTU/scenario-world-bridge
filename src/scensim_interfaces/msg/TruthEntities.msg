# The entities alive now, latched (transient local) on /scensim/truth/entities and published
# again at every spawn and despawn (docs/design/truth-stream.md).
uint64 episode
string map              # the map directory's name
float64 step_time       # s
TruthEntity[] entities
