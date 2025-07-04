#pragma once
#include "hbridge_peltier.h"

/// Parse an ASCII command line (no CR/LF) and act on it.
/// Returns 0 on OK (already ACK'd), negative on syntax error.
int host_cmd_execute(char *line, HBridge *hb);
