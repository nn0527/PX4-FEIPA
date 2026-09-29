"""Side-effect-free flight-state safety checks."""


def flight_state_ready(
    state,
    state_is_fresh,
    require_connected=True,
    require_armed=True,
    require_offboard=True,
):
    if not state_is_fresh:
        return False, "flight state timeout"
    if state is None:
        return False, "flight state unavailable"
    if require_connected and not state.connected:
        return False, "MAVROS is not connected"
    if require_armed and not state.armed:
        return False, "vehicle is not armed"
    if require_offboard and state.mode.upper() != "OFFBOARD":
        return False, "vehicle is not in OFFBOARD"
    return True, "flight state ready"

