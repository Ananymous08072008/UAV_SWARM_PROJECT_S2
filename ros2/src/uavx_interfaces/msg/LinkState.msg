# State of one radio link (undirected) as measured by radio_net.
string a
string b
float32 distance_m
float32 snr_db
float32 pdr             # smoothed packet delivery ratio 0..1 (includes injected loss)
float32 latency_ms
bool up                 # link usable (hysteresis: up at 0.55 PDR, down below 0.45)
string impairment       # "", "outage", "packet_loss", "jamming", ...
