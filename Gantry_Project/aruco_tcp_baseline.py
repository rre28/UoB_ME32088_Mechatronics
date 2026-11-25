"""
aruco_tcp_baseline.py  -  LAPTOP side, TEST / BASELINE version (ME32088 gantry project)

What it does
------------
1. Opens the USB camera and looks for ArUco markers        (based on CV_Python_Aruco_4_10.py)
2. When ONE chosen marker (TARGET_ID) has been seen for CONFIRM_FRAMES frames in a row,
   it sends that marker's ID number to the Raspberry Pi    (based on TCP_Client_RepeatSend.py)
3. The ID is sent only once (a "latch"). Press 'r' to re-arm it and test again.
4. Optional: a heartbeat that sends NO_MARKER (255) every HEARTBEAT_PERIOD_S seconds so the Pi
   can tell the link is alive. Switched OFF by default (HEARTBEAT_PERIOD_S = None).

The Raspberry Pi side is a Simulink model (TCP/IP Receive block), NOT Python.

Message laptop -> Pi : 1 byte, a uint8 (0-255)
        MARKER_ID  = the ArUco ID of the target that was confirmed (e.g. 3)
        255        = "no marker" heartbeat (only if the heartbeat is switched on;
                     255 can never be a real ID in DICT_4X4_50, whose IDs are 0-49)
    One byte means byte order cannot go wrong.
    Simulink TCP/IP Receive settings: Data type = uint8, Data size = 1.

    Note: a Display block shows 0 before anything has arrived, so for testing use a target
    marker that is NOT ID 0, otherwise you cannot tell "received 0" from "nothing yet".

Differences from the tutorial scripts (on purpose):
  - No camera calibration / pose estimation: we only need the marker ID and its pixel position,
    so Sample_Calibration.npz is not needed.
  - No time.sleep() in the camera loop: sleeping makes the camera buffer lag behind.
  - The socket has a timeout, and the script reconnects if the Pi drops out.

Keys (click the video window first):  r = re-arm the STOP latch    q = quit

Later stages (see the full prompt / claude/aruco_tcp_client.py) will add: several target IDs,
a centre band in the frame, choosing between several markers, servo angle calculation,
acknowledgements back from the Pi, and a full safety state machine.
"""

import logging
import socket
import struct
import time

import cv2
import cv2.aruco as aruco

logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(message)s", datefmt="%H:%M:%S")

# =============================================================================
#  SETTINGS  -  change these to suit your set-up
# =============================================================================
TCP_IP = "138.38.228.244"     # <-- CHANGE: IP address of the Raspberry Pi (this is the tutorial's address)
TCP_PORT = 25000              # must match "Local IP port" in the Pi's TCP/IP Receive block
                              # (laptop = CLIENT, Pi = SERVER, same as the tutorial)
CONNECT_TIMEOUT_S = 1.0       # how long one connection attempt may take
RECONNECT_PERIOD_S = 2.0      # wait this long between reconnection attempts

CAMERA_INDEX = 0                     # 0 = first camera; try 1 if the laptop's built-in webcam opens instead
ARUCO_DICT = aruco.DICT_4X4_50       # same dictionary as the tutorial script

TARGET_ID = 0                 # <-- CHANGE: the ONE marker ID that should be sent (use 1+ for testing, see top)
CONFIRM_FRAMES = 3            # frames in a row the marker must be seen before its ID is sent (filters flicker)

HEARTBEAT_PERIOD_S = None     # None = off.  e.g. 0.2 = send NO_MARKER (255) at least every 0.2 s
NO_MARKER = 255               # value sent by the heartbeat; never a real marker ID

DRY_RUN = False               # True = no network at all; just print what WOULD be sent (camera-only test)

# =============================================================================
#  MESSAGE FORMAT  (must match the Simulink model)
# =============================================================================
MSG_FORMAT = "B"              # B = one unsigned 8-bit integer (uint8), matches Data type uint8, Data size 1


def pack_message(value):
    """Build the 1-byte message. The value must be 0-255 (one byte)."""
    return struct.pack(MSG_FORMAT, int(value) & 0xFF)


# =============================================================================
#  TCP CONNECTION TO THE PI  (TCP_Client_RepeatSend.py, made robust)
# =============================================================================
class PiConnection:
    """Keeps one TCP client socket to the Pi. Reconnects by itself if the link drops."""

    def __init__(self, ip, port):
        self.ip, self.port = ip, port
        self.sock = None
        self.last_attempt = -1e9      # time of the last connection attempt
        self.last_send = 0.0          # time of the last successful send

    @property
    def connected(self):
        return self.sock is not None

    def try_connect(self, now):
        """Attempt a connection, but not more often than every RECONNECT_PERIOD_S."""
        if self.connected or now - self.last_attempt < RECONNECT_PERIOD_S:
            return
        self.last_attempt = now
        try:
            # Same as the tutorial: socket(AF_INET, SOCK_STREAM) + connect(), but with a timeout
            self.sock = socket.create_connection((self.ip, self.port), timeout=CONNECT_TIMEOUT_S)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # send small messages straight away
            logging.info("Connected to Pi at %s:%d", self.ip, self.port)
        except OSError as e:
            self.sock = None
            logging.warning("Could not connect to %s:%d (%s) - retrying in %.0f s",
                            self.ip, self.port, e, RECONNECT_PERIOD_S)

    def send(self, data, now):
        """Send all bytes. Returns True if it worked. On failure the socket is closed (and retried later)."""
        if not self.connected:
            return False
        try:
            self.sock.sendall(data)
            self.last_send = now
            return True
        except OSError as e:
            logging.error("Send failed (%s) - link lost", e)
            self.close()
            return False

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None


class DryRunConnection(PiConnection):
    """Same interface, but only prints. Lets you test the camera side without the Pi."""

    def __init__(self):
        super().__init__("dry-run", 0)
        self.sock = "dry-run"         # pretend to be connected

    def try_connect(self, now):
        pass

    def send(self, data, now):
        self.last_send = now
        (value,) = struct.unpack(MSG_FORMAT, data)
        if value != NO_MARKER:        # don't flood the terminal with heartbeats
            logging.info("[DRY RUN] would send marker ID %d  bytes=%s", value, list(data))
        return True

    def close(self):
        pass


# =============================================================================
#  ARUCO DETECTION  (CV_Python_Aruco_4_10.py, without the pose estimation)
# =============================================================================
def make_detector():
    aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
    parameters = aruco.DetectorParameters()
    return aruco.ArucoDetector(aruco_dict, parameters)


def find_target(detector, gray, target_id):
    """
    Detect all markers in a grey image.
    Returns (target_corners or None, list of all IDs seen, all corners, ids array).
    Only the TARGET_ID marker is returned as the target - any other marker is ignored,
    so a second marker in view can never cause a STOP.
    """
    corners, ids, _rejected = detector.detectMarkers(gray)
    # In OpenCV 4 'ids' is None when nothing is found; in OpenCV 5 it is empty -> check corners
    if len(corners) == 0 or ids is None:
        return None, [], corners, ids
    all_ids = [int(i) for i in ids.flatten()]
    for c, marker_id in zip(corners, all_ids):
        if marker_id == target_id:
            return c, all_ids, corners, ids
    return None, all_ids, corners, ids


class StopTrigger:
    """
    Decides WHEN to send STOP:
      - the target must be seen in CONFIRM_FRAMES consecutive frames (one missed frame resets the count)
      - after STOP is sent the trigger is 'latched' and will not fire again until reset() is called
    """

    def __init__(self, confirm_frames):
        self.confirm_frames = confirm_frames
        self.count = 0
        self.latched = False

    def update(self, target_seen):
        """Call once per frame. Returns True on the ONE frame where STOP should be sent."""
        if self.latched:
            return False
        self.count = self.count + 1 if target_seen else 0
        if self.count >= self.confirm_frames:
            self.latched = True
            return True
        return False

    def reset(self):
        self.count = 0
        self.latched = False


# =============================================================================
#  MAIN LOOP
# =============================================================================
def main():
    detector = make_detector()
    trigger = StopTrigger(CONFIRM_FRAMES)
    pi = DryRunConnection() if DRY_RUN else PiConnection(TCP_IP, TCP_PORT)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        logging.error("Could not open camera %d", CAMERA_INDEX)
        raise SystemExit(1)
    logging.info("Camera %d opened. Target marker ID = %d. Keys: r = re-arm, q = quit", CAMERA_INDEX, TARGET_ID)

    cv2.namedWindow("Frame", cv2.WINDOW_AUTOSIZE)
    last_ids = None
    t_prev = time.time()

    try:
        while True:
            now = time.time()

            # ---- 1. keep the link to the Pi up ----
            pi.try_connect(now)

            # ---- 2. grab a frame ----
            ok, frame = cap.read()
            if not ok:
                # Camera failed: stop here. The Pi stops receiving heartbeats, so in the
                # final version its watchdog will stop the motor and switch the laser off.
                logging.error("Could not read a frame from camera %d - exiting", CAMERA_INDEX)
                break

            # ---- 3. detect markers ----
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            target_corners, all_ids, corners, ids = find_target(detector, gray, TARGET_ID)

            if tuple(all_ids) != last_ids:                 # only log when what we see changes
                logging.info("Markers in view: %s", all_ids if all_ids else "none")
                last_ids = tuple(all_ids)

            # ---- 4. decide whether to send the marker ID (once) ----
            if trigger.update(target_corners is not None):
                msg = pack_message(TARGET_ID)
                if pi.send(msg, now):
                    logging.info(">>> Sent marker ID %d   bytes=%s", TARGET_ID, list(msg))
                else:
                    logging.warning("Marker ID could not be sent (not connected) - will try again")
                    trigger.reset()                         # un-latch so it is re-sent once reconnected

            # ---- 5. optional heartbeat (255 = no marker) so the Pi can see the link is alive ----
            if HEARTBEAT_PERIOD_S and pi.connected and now - pi.last_send >= HEARTBEAT_PERIOD_S:
                pi.send(pack_message(NO_MARKER), now)

            # ---- 6. draw and show ----
            if len(corners) > 0:
                aruco.drawDetectedMarkers(frame, corners, ids)
            if target_corners is not None:              # thick green outline on the target only
                cv2.polylines(frame, [target_corners.reshape(4, 2).astype(int)], True, (0, 255, 0), 4)

            fps = 1.0 / max(now - t_prev, 1e-6)
            t_prev = now
            status = "LATCHED (press r)" if trigger.latched else f"seen {trigger.count}/{CONFIRM_FRAMES}"
            link = "DRY RUN" if DRY_RUN else ("CONNECTED" if pi.connected else "NOT CONNECTED")
            for i, text in enumerate([f"Target ID {TARGET_ID}: {status}",
                                      f"Pi: {link}",
                                      f"FPS: {fps:.1f}"]):
                cv2.putText(frame, text, (10, 30 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("Frame", frame)

            # ---- 7. keys ----
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('r'):
                trigger.reset()
                logging.info("Latch reset - waiting for marker %d again", TARGET_ID)

    except KeyboardInterrupt:
        logging.info("Ctrl+C pressed")
    finally:
        pi.close()
        cap.release()
        cv2.destroyAllWindows()
        logging.info("Stopped")


if __name__ == "__main__":
    main()
