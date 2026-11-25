"""
gantry_tcp_v2.py     -  LAPTOP side of the gantry project (ME32088)

What it does
------------
1. Reads a USB camera and detects ArUco markers   (built on CV_Python_Aruco_4_10.py)
2. Decides which ONE marker the gantry and laser should deal with next:
   - every marker counts (no ID filter); a marker is lasered once, then remembered by its ID and ignored
   - the focus is the not-yet-done marker nearest the centre of the camera feed
   - it must stay in the centre band for a few frames in a row before the gantry stops for it
   - no reversing: a marker that slips past the band without being caught is given up and the gantry carries on
   - once the number of targets typed in at the start have all been dealt with, the gantry is stopped for good
3. Sends short fixed-size binary commands to the Raspberry Pi over TCP (built on TCP_Client_RepeatSend.py).
   Communication is ONE WAY: the Pi sends nothing back, so the script uses fixed waits for the gantry to stop
   (STOP_WAIT_S) and for the shot to finish (SHOT_WAIT_S).

The Raspberry Pi side is a Simulink model (NOT Python).

Message laptop -> Pi  (2 bytes, each uint8):   [CMD, SERVO]
    SERVO is the servo PWM duty cycle x 2000, so a decimal duty cycle fits in one whole-number byte.
    Simulink multiplies it by 0.0005 to get the decimal back for the PWM block:
        duty 0.050 -> byte 100   laser 45 deg LEFT of straight down
        duty 0.075 -> byte 150   NEUTRAL, straight down (default position)
        duty 0.100 -> byte 200   laser 45 deg RIGHT of straight down
    CMD 0 = FORWARD   no marker in the band: keep driving forward       SERVO = 150 (neutral)
    CMD 1 = MARKER    marker in the band: stop the gantry for it        SERVO = 0         -> stop, do not fire yet
                                                                         SERVO = 100..200  -> aim the laser there and fire
    CMD 3 = FINISHED  all targets dealt with: stop and stay stopped    SERVO = 150 (neutral)
    (There is no BACKWARD any more, and the old CMD 2 is now CMD 1.)
Nothing comes back from the Pi.

How to run it: home the gantry first (homing switch), start the Simulink model on the Pi, then run this script.
It first asks in the terminal how many targets there are (NUM_TARGETS = None), then starts searching on its own
after START_DELAY_S - no key presses needed.  Press q in the video window (or Ctrl+C) to stop; the gantry stops at once.
"""

import logging
import math
import socket
import statistics
import struct
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("gantry")

# =============================================================================
#  SETTINGS  -  everything you are likely to change is in this block
# =============================================================================

# ---- Network ---------------------------------------------------------------
TCP_IP = "138.38.226.194"   # <-- CHANGE: IP address of the Raspberry Pi
TCP_PORT = 25000            # Pi "TCP/IP Receive" block, Local IP port   (laptop = client, Pi = server)
CONNECT_TIMEOUT_S = 0.5     # how long one connection attempt may block the video loop
RECONNECT_PERIOD_S = 2.0    # time between reconnection attempts
HEARTBEAT_PERIOD_S = 0.1    # the current command is re-sent at least this often (Pi watchdog must be several times longer)
LINK_QUIET_S = 0.8          # after a failed send, stay silent this long before reconnecting, so the Pi's watchdog has
                            # certainly fired and reset it to SAFE.  Must be LONGER than the chart's T_LINK.

# ---- Targets -----------------------------------------------------------------
NUM_TARGETS = None          # None: ask in the terminal when the script starts.  Or set a number to skip the question.
                            # When this many markers have been fired at (or given up on), FINISHED [3, 150] is sent.

# ---- Start-up ----------------------------------------------------------------
START_DELAY_S = 2.0         # after connecting, wait this long (gantry held still) before driving forward.
                            # Gives the camera time to settle.  Home the gantry BEFORE running the script.

# ---- Camera / ArUco --------------------------------------------------------
CAMERA_INDEX = 0                    # which camera (0 = first)
CAMERA_BACKEND = cv2.CAP_ANY        # on Windows, cv2.CAP_DSHOW often opens faster
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
ARUCO_DICT = aruco.DICT_4X4_50      # same dictionary as the tutorial script
MARKER_SIZE_MM = 100                # printed marker side (mm), black square edge to edge
PROCESSING_PERIOD_S = 0.0           # minimum time between processed frames (0 = every frame).
                                    # Do not use sleep() to slow down: it makes the camera buffer lag behind.
CAMERA_FAIL_LIMIT = 10              # this many failed reads in a row = camera failure
CAMERA_RETRY_S = 2.0                # time between attempts to re-open a failed camera

# ---- Centre band (fractions of the frame, so resolution does not matter) ---
TRAVEL_AXIS = "x"                   # image axis the gantry travels along: "x" (left-right) or "y" (up-down)
MARKER_FLOW = -1                    # <-- CHECK ON THE RIG: while the gantry drives FORWARD, markers move across the image
                                    #     towards -x (or -y) -> -1, towards +x (or +y) -> +1.  Used to tell a marker that is
                                    #     still AHEAD from one the gantry has already PASSED (missed -> given up).
BAND_HALF_WIDTH_FRAC = 0.50         # half-width of the centre band; 0.50 makes it span 100% of the travel axis
BAND_OFFSET_FRAC = 0.0              # shift the band centre along the travel axis (+ = in the +image-axis direction).
                                    # Use this to make up for the distance travelled during detection + stopping.
CROSS_HALF_FRAC = 0.15              # marker must also be within this fraction of the centre line across the travel axis
                                    #   (0.15 -> box is 30% of the frame height; was 0.30 = 60%)
CONFIRM_FRAMES = 3                  # consecutive frames a marker must be seen in the band before STOP is sent
CHOOSE_BY = "central"               # if several markers qualify: "central" (closest to band centre) or "largest"

# ---- Markers stopped short of ------------------------------------------------
MAX_ATTEMPTS = 3                    # if the gantry stops SHORT of a marker (out of laser reach, still ahead), it creeps
                                    # forward and tries again up to this many times before giving the marker up

# ---- Aiming ----------------------------------------------------------------
# Camera calibration (same .npz format as CV_Python_Aruco_4_10.py: keys 'CM' = camera matrix, 'dist_coef').
# With it, cv2.solvePnP gives each marker's true position in mm relative to the camera (x, y AND distance z),
# corrected for lens distortion - so the angle is right at the edges of the frame, not only in the centre.
# If the file is missing, the script falls back to the simple method (marker size as scale + LASER_HEIGHT_MM).
USE_CALIBRATION = True
CALIBRATION_FILE = "calibration.npz"        # your calibration file; a bare name = same folder as this script.
                                            #     Must be made at the same resolution as FRAME_WIDTH x FRAME_HEIGHT.
LASER_ABOVE_CAMERA_MM = -10.0       # laser pivot about 1 cm BELOW the camera lens.  (calibrated method) how far the laser's servo pivot is ABOVE the camera
                                    #     lens (negative if it is below).  Laser height over the marker = camera distance z
                                    #     (measured by solvePnP) + this.
ANGLE_MODE = "computed"             # "computed": tilt from the marker position measured AFTER the gantry has stopped
                                    # "fixed"   : always FIXED_TILT_DEG (0 = laser straight down at the band centre)
FIXED_TILT_DEG = 0.0
SERVO_RANGE_DEG = 45                # the laser may tilt this far either side of straight down (a 90 degree sweep)
# Servo PWM duty cycles (from the servo set-up): left 45 deg, neutral (straight down), right 45 deg
SERVO_DUTY_LEFT = 0.05
SERVO_DUTY_NEUTRAL = 0.075          # the laser's DEFAULT position: used whenever it is not aiming at a marker
SERVO_DUTY_RIGHT = 0.10
DUTY_SCALE = 2000                   # duty sent as a whole-number byte = duty x 2000 (0.075 -> 150); Pi multiplies by 0.0005
SERVO_DIRECTION = +1                # Match the computed tilt direction to the servo's physical orientation.
LASER_HEIGHT_MM = 580.0             # simple method only (no calibration file): camera ~590 mm above the table, laser ~10 mm lower
LASER_OFFSET_MM = 0.0               # laser pivot position relative to the camera axis along the travel axis (mm); about in line.
                                    #   Each 10 mm here moves the aim by ~1 deg at 580 mm.  Tune if shots are consistently off.
MEASURE_FRAMES = 5                  # frames (stationary gantry) whose median gives the aiming angle

# ---- Fixed waits (no feedback from the Pi, so these must agree with the Simulink chart constants) ----
STOP_WAIT_S = 1.0                   # [1, 0] sent -> assume the gantry is stopped and still after this long.
                                    # Must be LONGER than chart T_STOP (0.5 s) + the time the carriage takes to stop.
MEASURE_TIMEOUT_S = 5.0             # marker must be seen within this time after STOP_WAIT_S.
                                    # Update the Pi chart's aim-wait budget to exceed STOP_WAIT_S + this timeout (6.0 s).
SHOT_WAIT_S = 5.0                   # [1, servo] sent -> assume the shot is finished after this long.
                                    # Must be LONGER than chart T_SERVO + T_LASER (3 s).  If it is too short, nothing unsafe
                                    # happens (the chart ignores FORWARD until the shot is over), the gantry just waits.

# ---- Misc ------------------------------------------------------------------
SHOW_WINDOW = True                  # False for headless runs
DRY_RUN = False                     # True: no network at all, just print the messages that would be sent

# =============================================================================
#  PROTOCOL  (must match the Simulink model)
# =============================================================================
CMD_FORWARD, CMD_STOP, CMD_FINISHED = 0, 1, 3     # CMD_STOP = 1: marker in the band (was 2).  No BACKWARD.
CMD_NAMES = {0: "FORWARD", 1: "MARKER", 3: "FINISHED"}
SERVO_BYTE_NEUTRAL = round(SERVO_DUTY_NEUTRAL * DUTY_SCALE)   # 150: SERVO byte in FORWARD / FINISHED (Pi checks it)
SERVO_BYTE_MIN = round(SERVO_DUTY_LEFT * DUTY_SCALE)          # 100  (must match the Pi's checks and Saturation block)
SERVO_BYTE_MAX = round(SERVO_DUTY_RIGHT * DUTY_SCALE)         # 200
SERVO_NO_FIRE = 0                             # SERVO byte in STOP while the gantry is stopping and being measured
MSG_FORMAT = "!BB"                            # [CMD, SERVO]; "!" = network byte order, no padding; B = unsigned 8-bit


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def tilt_to_duty(tilt_deg):
    """Tilt from straight down (deg, + = right) -> servo PWM duty cycle (decimal).
    Straight line through (-45, 0.05), (0, 0.075), (+45, 0.10)."""
    half_span = (SERVO_DUTY_RIGHT - SERVO_DUTY_LEFT) / 2               # 0.025 per 45 deg
    return SERVO_DUTY_NEUTRAL + half_span * tilt_deg / SERVO_RANGE_DEG


def pack_command(cmd, duty=None):
    """Build the 2-byte message [CMD, SERVO].  Single bytes, so byte order cannot go wrong.
    FORWARD/FINISHED always carry the neutral position (150).  CMD 1 (marker) carries 0 (stop only) or a duty cycle
    0.05..0.10 sent as 100..200 (aim and fire).  Keeping the second byte to these values is what lets the Pi
    spot a message that is out of step."""
    if cmd in (CMD_FORWARD, CMD_FINISHED):
        b = SERVO_BYTE_NEUTRAL
    elif duty is None:
        b = SERVO_NO_FIRE
    else:
        b = int(round(clamp(duty * DUTY_SCALE, SERVO_BYTE_MIN, SERVO_BYTE_MAX)))
    return struct.pack(MSG_FORMAT, cmd, b)


# =============================================================================
#  CAMERA + ARUCO  (same detector set-up as CV_Python_Aruco_4_10.py)
# =============================================================================
@dataclass
class Marker:
    id: int
    cx: float        # centre, pixels
    cy: float
    side: float      # mean side length, pixels
    corners: np.ndarray


def make_detector():
    aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
    return aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())


def detect_markers(detector, gray):
    """Return a list of Marker.  (Works for OpenCV 4.7+ and 5: no markers -> empty list.)"""
    corners, ids, _rejected = detector.detectMarkers(gray)
    found = []
    if len(corners) > 0 and ids is not None:
        for c, marker_id in zip(corners, ids.flatten()):
            pts = c.reshape(4, 2)
            cx, cy = pts.mean(axis=0)
            side = float(np.mean([np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]))
            found.append(Marker(int(marker_id), float(cx), float(cy), side, c))
    return found


def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, CAMERA_BACKEND)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)     # keep only the newest frame -> low latency (ignored by some drivers)
    return cap


# =============================================================================
#  CENTRE BAND
# =============================================================================
AHEAD, IN_BAND, PASSED = "ahead", "in band", "passed"


class Region:
    """Band across the image centre. 'along' = travel axis, 'cross' = the other image axis."""

    def __init__(self, width, height):
        n_along, n_cross = (width, height) if TRAVEL_AXIS == "x" else (height, width)
        self.centre_along = n_along / 2 + BAND_OFFSET_FRAC * n_along
        self.half_along = BAND_HALF_WIDTH_FRAC * n_along
        self.centre_cross = n_cross / 2
        self.half_cross = CROSS_HALF_FRAC * n_cross
        self.n_along = n_along
        self.width, self.height = width, height

    def split(self, m):
        return (m.cx, m.cy) if TRAVEL_AXIS == "x" else (m.cy, m.cx)

    def on_track(self, m):
        """Marker lies within the cross-band, i.e. on the line the gantry follows."""
        return abs(self.split(m)[1] - self.centre_cross) <= self.half_cross

    def where(self, m):
        """AHEAD (not reached yet), IN_BAND, or PASSED (the gantry has gone past it)."""
        rel = (self.split(m)[0] - self.centre_along) * MARKER_FLOW    # > 0: downstream of the centre = passed
        if rel > self.half_along:
            return PASSED
        if rel < -self.half_along:
            return AHEAD
        return IN_BAND

    def centrality(self, m):
        """Smaller = nearer the band centre."""
        return abs(self.split(m)[0] - self.centre_along)

    def draw(self, frame, colour=(255, 255, 0)):
        if TRAVEL_AXIS == "x":
            x0, x1 = int(self.centre_along - self.half_along), int(self.centre_along + self.half_along)
            y0, y1 = int(self.centre_cross - self.half_cross), int(self.centre_cross + self.half_cross)
        else:
            y0, y1 = int(self.centre_along - self.half_along), int(self.centre_along + self.half_along)
            x0, x1 = int(self.centre_cross - self.half_cross), int(self.centre_cross + self.half_cross)
        cv2.rectangle(frame, (x0, y0), (x1, y1), colour, 1)
        cv2.drawMarker(frame, (self.width // 2, self.height // 2), colour, cv2.MARKER_CROSS, 20, 1)


# ---- camera calibration (loaded in main) ----
CAM_MATRIX = None
DIST_COEFFS = None


def load_calibration():
    """Load CM and dist_coef from the calibration .npz.  Returns (None, None) if it is switched off or missing."""
    if not USE_CALIBRATION:
        log.info("Calibration switched off - using the simple method (LASER_HEIGHT_MM)")
        return None, None
    path = Path(CALIBRATION_FILE)
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / path
    try:
        data = np.load(path)
        cm, dist = data["CM"], data["dist_coef"]
        log.info("Loaded camera calibration from %s", path)
        return cm, dist
    except (OSError, KeyError) as e:
        log.warning("Could not load calibration %s (%s) - using the simple method (LASER_HEIGHT_MM)", path, e)
        return None, None


def marker_position_mm(m):
    """(x, y, z) of the marker centre in mm, in the camera's frame (x right, y down, z = distance from the lens),
    from cv2.solvePnP with the calibration - same call as the tutorial script.  None if no calibration or it fails."""
    if CAM_MATRIX is None:
        return None
    half = MARKER_SIZE_MM / 2
    obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
    ok, _rvec, tvec = cv2.solvePnP(obj, m.corners.reshape(4, 1, 2).astype(np.float32), CAM_MATRIX, DIST_COEFFS,
                                   flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return tuple(float(v) for v in tvec.flatten()) if ok else None


def tilt_reachable(tilt_deg):
    """True if the servo can point the laser there (within 45 degrees of straight down, either side)."""
    return abs(tilt_deg) <= SERVO_RANGE_DEG


def compute_tilt(m, region):
    """Tilt from straight down (deg, + = right) that points the laser at marker m.  Assumes the camera looks
    straight down.  Not limited to the servo range; tilt_reachable() decides whether the laser can get there.

    Calibrated method (calibration file loaded): solvePnP gives the marker's position in mm, corrected for lens
      distortion, and its distance z from the camera, so the laser height is measured, not guessed:
          sideways offset d = x (or y) - LASER_OFFSET_MM,   height h = z + LASER_ABOVE_CAMERA_MM
    Simple method (no calibration): marker size gives mm per pixel, height = LASER_HEIGHT_MM.
    Either way: tilt = atan(d / h)."""
    if ANGLE_MODE == "fixed":
        return float(FIXED_TILT_DEG)
    pos = marker_position_mm(m)
    if pos is not None:
        x, y, z = pos
        along_mm = x if TRAVEL_AXIS == "x" else y                # offset from the CAMERA axis along the travel axis
        d_mm = along_mm - LASER_OFFSET_MM                        # offset from the LASER axis
        h_mm = z + LASER_ABOVE_CAMERA_MM                         # laser pivot height above the marker
    else:
        mm_per_px = MARKER_SIZE_MM / m.side
        along_px = region.split(m)[0] - region.n_along / 2       # offset from the CAMERA axis (not the band centre)
        d_mm = along_px * mm_per_px - LASER_OFFSET_MM            # offset from the LASER axis
        h_mm = LASER_HEIGHT_MM
    theta = math.degrees(math.atan2(d_mm, h_mm))                 # tilt needed
    return SERVO_DIRECTION * theta


class Confirmer:
    """Counts consecutive frames in which each ID has been inside the band.  Any gap resets that ID."""

    def __init__(self, n):
        self.n, self.streak = n, {}

    def update(self, ids_now):
        self.streak = {i: self.streak.get(i, 0) + 1 for i in ids_now}   # IDs not seen now are dropped = reset
        return {i for i, c in self.streak.items() if c >= self.n}

    def reset(self):
        self.streak = {}


def choose_marker(candidates, region):
    """Pick ONE marker from several qualifying ones."""
    if CHOOSE_BY == "largest":
        return max(candidates, key=lambda m: m.side)
    return min(candidates, key=region.centrality)


# =============================================================================
#  LINK TO THE PI  (one TCP client socket, commands out only)
# =============================================================================
class PiLink:
    """Sends commands to the Pi.  The Pi never replies, so the only sign of a dead link is a failed send;
    the Pi protects itself with its own watchdog (no valid message for 0.5 s -> SAFE)."""

    def __init__(self):
        self.cmd = None
        self.last_attempt = -1e9
        self.lost_at = -1e9
        self.last_tx = 0.0

    @property
    def up(self):
        return self.cmd is not None

    def close(self):
        try:
            if self.cmd:
                self.cmd.close()
        except OSError:
            pass
        self.cmd = None

    def lose(self, reason):
        if self.cmd:
            log.warning("Link to Pi lost: %s", reason)
            self.lost_at = time.monotonic()
        self.close()

    def service(self, now):
        """Call every loop: (re)connect if the link is down."""
        if self.up:
            return
        if now - self.last_attempt >= RECONNECT_PERIOD_S and now - self.lost_at >= LINK_QUIET_S:
            self.last_attempt = now
            try:
                s = socket.create_connection((TCP_IP, TCP_PORT), timeout=CONNECT_TIMEOUT_S)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # send small packets immediately (no Nagle)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                s.settimeout(0.2)                                          # a stuck send cannot freeze the video loop
                self.cmd = s
                log.info("Connected to Pi %s port %d", TCP_IP, TCP_PORT)
            except OSError as e:
                self.close()
                log.info("Could not connect to %s: %s", TCP_IP, e)

    def send(self, data, now):
        if not self.cmd:
            return False
        try:
            self.cmd.sendall(data)
            self.last_tx = now
            return True
        except OSError as e:
            self.lose(f"send failed: {e}")
            return False


class DryLink:
    """Same interface as PiLink but only prints.  For testing the camera/decision logic without a Pi."""
    up = True
    last_tx = 0.0

    def __init__(self):
        self.last_printed = None

    def service(self, now): pass
    def close(self): pass
    def lose(self, reason): log.warning("[DRY RUN] would drop the link: %s", reason)

    def send(self, data, now):
        self.last_tx = now
        if data != self.last_printed:                         # print changes only, not every heartbeat
            cmd, b = struct.unpack(MSG_FORMAT, data)
            log.info("[DRY RUN] now sending %s servo byte=%d (duty %.4f)  bytes=%s",
                     CMD_NAMES[cmd], b, b / DUTY_SCALE, list(data))
            self.last_printed = data
        return True


# =============================================================================
#  THE DECISION LOGIC  (state machine on the laptop)
# =============================================================================
STARTING, SEARCH, WAIT_STOPPED, MEASURE, WAIT_DONE, FAULT, FINISHED = (
    "STARTING", "SEARCH", "WAIT_STOPPED", "MEASURE", "WAIT_DONE", "FAULT", "FINISHED")


def ask_num_targets():
    """Ask in the terminal how many targets there are (the brief says 3 to 8, revealed on the day)."""
    while True:
        try:
            text = input("How many targets are there? ").strip()
        except EOFError:
            raise SystemExit("No number of targets given - stopping.")
        if text.isdigit() and int(text) > 0:
            return int(text)
        print("   Please type a whole number greater than 0, e.g. 5")


class Controller:
    def __init__(self, link, num_targets):
        self.link = link
        self.num_targets = num_targets
        self.confirmer = Confirmer(CONFIRM_FRAMES)
        self.state = STARTING
        self.t_state = time.monotonic()
        self.out = pack_command(CMD_STOP)          # the command currently being sent (re-sent as the heartbeat)
        self.target_id = None
        self.tilts = deque(maxlen=MEASURE_FRAMES)
        self.last_tilt = None
        self.last_where = None
        self.fault_reason = ""
        self.done_ids = []                         # markers lasered, in order
        self.given_up = []                         # markers abandoned: missed, out of reach, or not seen after stopping
        self.attempts = {}                         # id -> times the gantry has stopped short of it
        self.approached = set()                    # ids seen ahead of / in the band (so "passed" really means missed)

    # ---- helpers ----
    def go(self, state, now):
        if state != self.state:
            log.info("State: %s -> %s", self.state, state)
        self.state, self.t_state = state, now

    def command(self, cmd, now, duty=None):
        """Change the command being sent; it goes out at once and is then repeated as the heartbeat."""
        data = pack_command(cmd, duty)
        if data != self.out:
            log.info("Send %s %s", CMD_NAMES[cmd], list(data))
        self.out = data
        return self.link.send(data, now)

    def finished_with(self, marker_id):
        return marker_id in self.done_ids or marker_id in self.given_up

    def give_up(self, marker_id, why):
        log.warning("Giving up on marker %d: %s", marker_id, why)
        if marker_id not in self.given_up:
            self.given_up.append(marker_id)

    def all_targets_dealt_with(self):
        return len(self.done_ids) + len(self.given_up) >= self.num_targets

    def resume_forward(self, now):
        """Carry on to the next marker - or, if every target has been dealt with, stop for good."""
        self.target_id = None
        self.confirmer.reset()
        if self.all_targets_dealt_with():
            self.finish(now)
            return
        self.command(CMD_FORWARD, now)
        self.go(SEARCH, now)

    def finish(self, now):
        log.info("All %d targets dealt with (fired at: %s, given up: %s) - sending FINISHED, gantry stops",
                 self.num_targets, self.done_ids, self.given_up)
        self.command(CMD_FINISHED, now)                      # [3, 150]: stop, laser neutral; repeated as the heartbeat
        self.go(FINISHED, now)

    def stop_for(self, m, now):
        self.target_id = m.id
        self.tilts.clear()
        self.confirmer.reset()
        self.command(CMD_STOP, now)                          # [1, 0]: marker in band - stop, do not fire yet
        log.info("STOP for marker %d", m.id)
        self.go(WAIT_STOPPED, now)

    def link_lost(self, now):
        """The Pi goes SAFE by itself (watchdog).  The marker in hand was not finished, so it stays on the list."""
        if self.state == FINISHED:
            return                                           # keep sending FINISHED once reconnected
        if self.state != FAULT:
            self.go(STARTING, now)
        self.out = pack_command(CMD_STOP)
        self.confirmer.reset()

    def camera_fault(self, now):
        if self.state not in (FAULT, FINISHED):
            log.error("FAULT: camera failure - stopping the gantry")
            self.fault_reason = "camera failure"
            self.command(CMD_STOP, now)                      # stop at once; the heartbeat then stops too
            self.confirmer.reset()
            self.go(FAULT, now)

    # ---- one step per processed frame ----
    def step(self, now, markers, region):
        if self.state == FINISHED:                           # nothing more to do: FINISHED keeps being re-sent
            return
        if self.state == FAULT:                              # camera is back (step only runs on a good frame)
            log.info("Camera working again - restarting")
            self.go(STARTING, now)

        # markers still to deal with, on the gantry's line
        todo = [m for m in markers if not self.finished_with(m.id) and region.on_track(m)]
        for m in todo:
            if region.where(m) != PASSED:
                self.approached.add(m.id)

        if self.state == STARTING:
            self.command(CMD_STOP, now)                      # hold still while the camera settles
            if now - self.t_state >= START_DELAY_S:
                log.info("Starting: driving forward")
                self.resume_forward(now)
            return

        if self.state == SEARCH:
            # Marker not (yet) confirmed in the band -> CMD 0 (FORWARD) keeps being sent.
            # Marker confirmed in the band           -> CMD 1 (stop for it).
            in_band = [m for m in todo if region.where(m) == IN_BAND]
            confirmed_ids = self.confirmer.update({m.id for m in in_band})
            confirmed = [m for m in in_band if m.id in confirmed_ids]
            if confirmed:
                if len(in_band) > 1:
                    log.info("Several markers in band %s -> choosing by '%s'", sorted(m.id for m in in_band), CHOOSE_BY)
                self.stop_for(choose_marker(confirmed, region), now)
                return

            # No reversing: a marker that went past the band without being caught is given up,
            # so it still counts towards NUM_TARGETS and FINISHED can be reached.
            for m in todo:
                if region.where(m) == PASSED and m.id in self.approached:
                    self.give_up(m.id, "went past the band without being caught")
            if self.all_targets_dealt_with():
                self.finish(now)
            return

        if self.state == WAIT_STOPPED:
            if now - self.t_state >= STOP_WAIT_S:              # no feedback: assume stopped after a fixed wait
                self.go(MEASURE, now)
            return

        if self.state == MEASURE:
            same = [m for m in markers if m.id == self.target_id]
            if same:
                m = choose_marker(same, region)
                self.tilts.append(compute_tilt(m, region))
                self.last_where = region.where(m)
            else:
                self.tilts.clear()                                   # need CONSECUTIVE frames
            if len(self.tilts) >= MEASURE_FRAMES:
                self.last_tilt = statistics.median(self.tilts)
                if not tilt_reachable(self.last_tilt):
                    log.warning("Marker %d needs a tilt of %.1f deg, more than %d deg from straight down - laser NOT fired",
                                self.target_id, self.last_tilt, SERVO_RANGE_DEG)
                    if self.last_where == AHEAD:                     # stopped short: creep forward into the band
                        self.attempts[self.target_id] = self.attempts.get(self.target_id, 0) + 1
                        if self.attempts[self.target_id] > MAX_ATTEMPTS:
                            self.give_up(self.target_id, "could not get it within reach")
                    else:                                            # overshot: no reversing, so give it up
                        self.give_up(self.target_id, "out of the laser's reach after stopping (overshot)")
                    self.resume_forward(now)
                    return
                duty = tilt_to_duty(self.last_tilt)
                self.command(CMD_STOP, now, duty)                    # [1, duty x 2000]: aim and fire
                log.info("Aim marker %d: tilt %.1f deg -> duty %.4f (byte %d)",
                         self.target_id, self.last_tilt, duty, round(duty * DUTY_SCALE))
                pos = marker_position_mm(m) if same else None
                if pos:
                    log.info("   marker position from camera: x %.0f mm, y %.0f mm, distance z %.0f mm", *pos)
                self.go(WAIT_DONE, now)
            elif now - self.t_state > MEASURE_TIMEOUT_S:
                self.give_up(self.target_id, "not visible after stopping (overshot?)")
                self.resume_forward(now)
            return

        if self.state == WAIT_DONE:
            if now - self.t_state >= SHOT_WAIT_S:              # no feedback: assume the shot is over after a fixed wait
                log.info("Marker %d done", self.target_id)
                self.done_ids.append(self.target_id)
                self.resume_forward(now)


# =============================================================================
#  DISPLAY
# =============================================================================
def draw_overlay(frame, region, markers, ctl, link):
    region.draw(frame)
    for m in markers:
        if ctl.finished_with(m.id):
            colour = (128, 128, 128)                                  # grey: already done / given up
        elif m.id == ctl.target_id:
            colour = (0, 0, 255)                                      # red: the current focus
        else:
            colour = (0, 255, 0)                                      # green: still to do
        pts = m.corners.reshape(4, 2).astype(int)
        cv2.polylines(frame, [pts], True, colour, 2)
        cv2.putText(frame, f"id {m.id}", (int(m.cx) - 20, int(m.cy) - int(m.side / 2) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
    cmd, b = struct.unpack(MSG_FORMAT, ctl.out)
    lines = [f"PC: {ctl.state}   link: {'UP' if link.up else 'DOWN'}",
             f"sending: {CMD_NAMES[cmd]} {list(ctl.out)}  (servo duty {b / DUTY_SCALE:.4f})",
             f"done: {ctl.done_ids}   given up: {ctl.given_up}   targets: {ctl.num_targets}"]
    if ctl.state == FAULT:
        lines.append(f"FAULT: {ctl.fault_reason}")
    for i, text in enumerate(lines):
        cv2.putText(frame, text, (10, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)


# =============================================================================
#  MAIN LOOP
# =============================================================================
def main(max_seconds=None):
    global CAM_MATRIX, DIST_COEFFS
    CAM_MATRIX, DIST_COEFFS = load_calibration()
    num_targets = NUM_TARGETS if NUM_TARGETS else ask_num_targets()
    log.info("Number of targets: %d", num_targets)
    detector = make_detector()
    link = DryLink() if DRY_RUN else PiLink()
    ctl = Controller(link, num_targets)
    if SHOW_WINDOW:
        cv2.namedWindow("Frame", cv2.WINDOW_AUTOSIZE)

    cap = open_camera()
    camera_fail = 0 if cap.isOpened() else CAMERA_FAIL_LIMIT
    last_cam_retry = time.monotonic()
    last_processed = 0.0
    was_up = False
    region = None
    markers = []
    frame = None
    t_start = time.monotonic()
    log.info("Running. Make sure the gantry has been homed. Press q in the video window to stop.")

    try:
        while True:
            now = time.monotonic()
            if max_seconds and now - t_start > max_seconds:
                break

            # ---- camera ----
            ok, img = cap.read() if cap.isOpened() else (False, None)
            if ok:
                camera_fail, frame = 0, img
            else:
                camera_fail += 1
                if camera_fail >= CAMERA_FAIL_LIMIT and now - last_cam_retry > CAMERA_RETRY_S:
                    last_cam_retry = now
                    log.error("Camera not delivering frames - trying to re-open")
                    cap.release()
                    cap = open_camera()
            camera_ok = camera_fail < CAMERA_FAIL_LIMIT

            # ---- link ----
            link.service(now)
            if was_up and not link.up:
                ctl.link_lost(now)
            if link.up and not was_up and ctl.state not in (FAULT, FINISHED):
                ctl.go(STARTING, now)                        # fresh connection: hold still, then drive forward
            was_up = link.up

            # ---- detection + decision (only on a fresh frame, at most every PROCESSING_PERIOD_S) ----
            if ok and now - last_processed >= PROCESSING_PERIOD_S:
                last_processed = now
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                markers = detect_markers(detector, gray)
                region = Region(frame.shape[1], frame.shape[0])
                if link.up:
                    ctl.step(now, markers, region)
            elif not camera_ok and link.up:
                ctl.camera_fault(now)

            # ---- heartbeat: re-send the current command ----
            # Only sent while the camera is healthy.  If the camera or this loop freezes, the messages stop
            # and the Pi's watchdog stops the motor and switches the laser off.
            if link.up and (camera_ok or ctl.state == FINISHED) and ctl.state != FAULT \
                    and now - link.last_tx >= HEARTBEAT_PERIOD_S:
                link.send(ctl.out, now)

            # ---- display (q quits) ----
            if SHOW_WINDOW and frame is not None:
                shown = frame.copy()
                if region:
                    draw_overlay(shown, region, markers, ctl, link)
                cv2.imshow("Frame", shown)
            key = (cv2.waitKey(1) & 0xFF) if SHOW_WINDOW else 255
            if key == ord('q'):
                break
    except KeyboardInterrupt:
        log.info("Ctrl+C")
    finally:
        # Best effort: stop the gantry NOW without firing; the Pi's watchdog then takes it to SAFE.
        if link.up:
            for _ in range(2):
                link.send(pack_command(CMD_STOP), time.monotonic())
        link.close()
        cap.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        log.info("Stopped. Done: %s  given up: %s", ctl.done_ids, ctl.given_up)
    return ctl


if __name__ == "__main__":
    main()
