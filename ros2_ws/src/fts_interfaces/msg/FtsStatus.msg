# FTS status as reported on link 2. Times are simulation time in
# microseconds; -1 means not set.
uint64 t_us
string state
string cause
uint8 pbit_fail
int64 t_trigger_us
int64 t_relay_us
int64 t_chute_us
# FTS time of the newest status when the last command was sent
int64 t_last_cmd_us
bool link_enabled
