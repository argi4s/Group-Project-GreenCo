from pymavlink import mavutil
import time
import math

# ----------------------
# Connection
# ----------------------
mav = mavutil.mavlink_connection("tcp:127.0.0.1:14550")
mav.wait_heartbeat()
print("[OK] Connected to FC")

# ----------------------
# Target definition
# ----------------------
# home=51.4234178,-2.6715506,50,155
TARGET_LAT = 51.423425
TARGET_LON = -2.673540
TARGET_ALT = 20      # meters above home
LAND_ALT   = 0       # land

GUIDED_RATE = 1.0    # Hz
GUIDED_TIME = 100     # seconds streaming target

# ----------------------
# ACK utility
# ----------------------
def wait_ack(command, timeout=3):
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(type="COMMAND_ACK", blocking=False)
        if msg and msg.command == command:
            return msg.result
        time.sleep(0.05)
    return None

# ----------------------
# Mode change with ACK
# ----------------------
def set_mode_and_confirm(mode, timeout=5):
    mav.set_mode_apm(mode)

    start = time.time()
    while time.time() - start < timeout:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb:
            current_mode = mavutil.mode_string_v10(hb)
            if current_mode == mode:
                print(f"[MODE] Confirmed {mode}")
                return True

    print(f"[MODE FAIL] {mode}")
    return False


# ----------------------
# Guided target
# ----------------------
def send_guided_target(lat, lon, alt):
    mav.mav.set_position_target_global_int_send(
        0,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,   # position only
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

def get_global_position():
    while True:
        msg = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        if msg:
            now = time.time()
            return (
                msg.lat / 1e7,
                msg.lon / 1e7,
                msg.relative_alt / 1000.0,
                msg.time_boot_ms
            )



def distance_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat/2)**2 +
         math.cos(math.radians(lat1)) *
         math.cos(math.radians(lat2)) *
         math.sin(dlon/2)**2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def goto_guided_until_reached(lat_t, lon_t, alt_t,
                             threshold=2.0,
                             timeout=120):
    print("[GUIDED] Navigating to target")
    start = time.time()

    while True:
        if time.time() - start > timeout:
            raise RuntimeError("[GUIDED] Timeout reaching target")

        pos = get_global_position()
        if not pos:
            continue

        lat, lon, alt = pos
        d = distance_m(lat, lon, lat_t, lon_t)

        send_guided_target(lat_t, lon_t, alt_t)

        print(f"[GUIDED] Distance: {d:.1f} m")

        if d < threshold:
            print("[GUIDED] Target reached")
            break

        time.sleep(1.0)


# ----------------------
# Land
# ----------------------
def land():
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0,
        0, 0,
        0
    )
    ack = wait_ack(mavutil.mavlink.MAV_CMD_NAV_LAND)
    print("[LAND ACK]", ack)

# ----------------------
# Takeoff
# ----------------------
def takeoff(alt):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt
    )
    ack = wait_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
    print("[TAKEOFF ACK]", ack)

# ----------------------
# RTL
# ----------------------
def rtl():
    set_mode_and_confirm("RTL")

# velocity check
def get_velocity():
    msg = mav.recv_match(type="VFR_HUD", blocking=True)
    return msg.groundspeed


# ======================
# FSM EXECUTION
# ======================

# ----------------------
# GUIDED NAVIGATION LOOP
# ----------------------

threshold = 2.0          # meters
STOP_SPEED = 0.2         # m/s
RATE = 1.0               # Hz

print("[GUIDED] Navigating to target")

print("\n[FSM] AUTO → GUIDED")
if not set_mode_and_confirm("GUIDED"):
    raise RuntimeError("Failed to enter GUIDED")


last_time = None

while True:
    # --- Get fresh position ---
    pos = get_global_position()
    if not pos:
        continue

    lat, lon, alt, t = pos

    # Reject stale telemetry
    if last_time is not None and t == last_time:
        continue
    last_time = t

    # --- Compute distance ---
    d = distance_m(lat, lon, TARGET_LAT, TARGET_LON)

    # --- Get velocity ---
    groundspeed = get_velocity()

    # --- Stream target ---
    send_guided_target(TARGET_LAT, TARGET_LON, TARGET_ALT)

    print(f"[GUIDED] d={d:.1f} m | v={groundspeed:.2f} m/s")

    # --- Stop conditions ---
    if d < threshold:
        print("[GUIDED] Target reached")
        break

    if d > threshold and groundspeed < STOP_SPEED:
        print("[WARN] Vehicle stopped before reaching target")
        break

    time.sleep(1.0 / RATE)


# ----------------------
# LAND
# ----------------------
print("\n[FSM] Landing near target")
if not set_mode_and_confirm("LAND"):
    raise RuntimeError("Failed to enter LAND")

land()

print("\n[WAIT] Waiting for GCS confirmation to re-takeoff")
while True:
    cmd = input("Type 'confirm' to takeoff > ").strip().lower()
    if cmd == "confirm":
        break

# ----------------------
# TAKEOFF
# ----------------------
print("\n[FSM] Re-takeoff")
if not set_mode_and_confirm("GUIDED"):
    raise RuntimeError("Failed to re-enter GUIDED")

takeoff(TARGET_ALT)

time.sleep(5)

# ----------------------
# RTL
# ----------------------
print("\n[FSM] RTL")
rtl()
msg = mav.recv_match(type="HOME_POSITION", blocking=True)
print(msg.latitude/1e7, msg.longitude/1e7)


print("[DONE] Sequence complete")
