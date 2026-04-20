"""
Central registry of Redis keys owned by picohost.

Mirrors the layout of ``eigsep_redis.keys`` and
``eigsep_observing.keys``: every key/stream/set/prefix touched by
this package lives here so collisions are visible at import time
and new names can be audited in one place. None of these overlap
with eigsep_redis (``metadata``, ``stream:status``, ``config``, …)
or eigsep_observing (``stream:corr``, ``corr_config``, …).
"""

PICO_CONFIG_KEY = "pico_config"
PICO_CMD_STREAM = "stream:pico_cmd"
PICO_RESP_STREAM = "stream:pico_resp"
PICO_CLAIM_PREFIX = "pico_claim"
PICO_HEARTBEAT_PREFIX = "pico"


def pico_heartbeat_name(device):
    """Return the name of the HeartbeatWriter for a given device.

    The writer renders the key as ``heartbeat:{name}``; with the
    ``pico:`` prefix the final key is ``heartbeat:pico:{device}`` —
    which keeps per-device liveness out of the same namespace as the
    panda client heartbeat (``heartbeat:client``).
    """
    return f"{PICO_HEARTBEAT_PREFIX}:{device}"


def pico_claim_key(device):
    """Return the soft-claim key for a given device."""
    return f"{PICO_CLAIM_PREFIX}:{device}"
