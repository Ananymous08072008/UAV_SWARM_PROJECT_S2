# One frame on the emulated UAV mesh radio.
# Nodes publish frames on /radio/tx; radio_net delivers each one (or drops it) on
# /<dst>/radio/rx according to the link model, injected disturbances and latency.

string src              # sender node id: "gcs", "uav1", "uav2", ...
string dst              # receiver node id, or "*" to broadcast to every neighbour
uint32 seq              # per-sender sequence number (PDR accounting)
string kind             # HEARTBEAT, STATUS, TASKS, POI_REPORT, TASK_DONE, TASK_RELEASE, RECALL, ACK
string msg_id           # set on reliable frames: the receiver ACKs it and drops duplicates
string payload          # JSON body
builtin_interfaces/Time stamp   # sim time the sender first created the frame
uint8 attempt           # 1 = first transmission, 2+ = ARQ retransmission
uint8 hops              # hops travelled so far (multi-hop levels)
string[] route          # nodes the frame has passed through
