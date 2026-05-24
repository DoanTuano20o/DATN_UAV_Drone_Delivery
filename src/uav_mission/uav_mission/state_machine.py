from __future__ import annotations

from enum import Enum


class MissionState(str, Enum):
    IDLE = "IDLE"
    WAIT_GUIDED = "WAIT_GUIDED"

    # =========================
    # TAKEOFF / GOTO
    # =========================
    TAKEOFF_CMD_SENT = "TAKEOFF_CMD_SENT"
    HOLD_BEFORE_GOTO = "HOLD_BEFORE_GOTO"
    GOTO_MARKER_REGION = "GOTO_MARKER_REGION"

    # =========================
    # BIG MARKER
    # =========================
    SEARCH_BIG_PATTERN = "SEARCH_BIG_PATTERN"
    ALIGN_BIG = "ALIGN_BIG"
    DESCEND_BIG = "DESCEND_BIG"
    LOCK_BIG = "LOCK_BIG"
    DESCEND_TO_SMALL_ALT = "DESCEND_TO_SMALL_ALT"

    # =========================
    # SMALL MARKER
    # =========================
    SEARCH_SMALL = "SEARCH_SMALL"
    ALIGN_SMALL = "ALIGN_SMALL"
    LOCK_SMALL = "LOCK_SMALL"

    # =========================
    # OLD LANDING FLOW
    # Giữ lại để không lỗi code cũ, nhưng luồng mới sẽ không dùng nữa.
    # =========================
    LAND_ON_SMALL_ARUCO = "LAND_ON_SMALL_ARUCO"
    LAND = "LAND"

    # =========================
    # NEW DROP + RTL FLOW
    # Luồng mới:
    # LOCK_SMALL -> DESCEND_TO_DROP_ALT -> DROP_PAYLOAD
    # -> WAIT_DROP_DONE -> RTL_RETURN
    # =========================
    DESCEND_TO_DROP_ALT = "DESCEND_TO_DROP_ALT"
    DROP_PAYLOAD = "DROP_PAYLOAD"
    WAIT_DROP_DONE = "WAIT_DROP_DONE"
    RTL_RETURN = "RTL_RETURN"

    # =========================
    # END / SAFETY
    # =========================
    MANUAL_OVERRIDE = "MANUAL_OVERRIDE"
    DONE = "DONE"
    FAILSAFE = "FAILSAFE"
