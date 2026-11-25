"""
pi_message_tester_auto.py  -  AUTOMATIC version of pi_message_tester.py.

Sends a fixed list of 2-byte messages [CMD, ANGLE] to the Raspberry Pi with a set gap between them, no key
presses needed, and prints the time of each one so you can line them up with the Simulink Scope.

How to use
----------
1. Deploy the Simulink model with Monitor & Tune and open the Scope (new_msg, ideally also Command and Angle).
2. Set PI_IP below, or pass it:   python pi_message_tester_auto.py 192.168.1.50
3. Run it.  It counts down, then runs every test.  Total run time is about 30 s with the default gap.

What the Scope should show (new_msg)
------------------------------------
GAP_S apart:  6 spikes (valid), then 5 gaps (invalid, no spike), then 1 spike ([0, 0]), then a 3 s comb.
"""
import socket
import sys
import time

PI_IP = "138.38.226.194"      # <-- CHANGE to your Pi's IP address
PORT = 25000                  # same as the TCP/IP Receive block
GAP_S = 2.0                   # time between tests (s): makes each spike, or missing spike, easy to see
COUNTDOWN_S = 3               # seconds before the first message, so you can look at the Scope
STREAM_S = 3.0                # length of the heartbeat-stream test (s)

# (name, [cmd, ang], "once" or "stream", expected new_msg pulse?)
TESTS = [
    ("Forward",               [0, 90],    "once",   True),
    ("Backward",              [1, 90],    "once",   True),
    ("Stop and hold",         [2, 0],     "once",   True),
    ("Fire at 45",            [2, 45],    "once",   True),
    ("Fire at 90",            [2, 90],    "once",   True),
    ("Fire at 135",           [2, 135],   "once",   True),
    ("BAD command 3",         [3, 90],    "once",   False),
    ("BAD angle 44",          [2, 44],    "once",   False),
    ("BAD angle 136",         [2, 136],   "once",   False),
    ("BAD angle 30",          [0, 30],    "once",   False),
    ("Junk 255, 255",         [255, 255], "once",   False),
    ("Forward with angle 0",  [0, 0],     "once",   True),    # passes the blocks; the chart ignores it later
    ("Heartbeat stream",      [0, 90],    "stream", True),    # 10 per second -> comb of spikes
]


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else PI_IP
    print(f"Connecting to {ip}:{PORT} ...")
    s = socket.create_connection((ip, PORT), timeout=3)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)       # send each 2-byte message straight away
    print("Connected.")
    for n in range(COUNTDOWN_S, 0, -1):
        print(f"  starting in {n} ...")
        time.sleep(1)

    t0 = time.monotonic()
    log = []
    try:
        for i, (name, msg, how, expect) in enumerate(TESTS, 1):
            t = time.monotonic() - t0
            if how == "once":
                s.sendall(bytes(msg))
                note = "spike" if expect else "NO spike"
            else:
                t_end = time.monotonic() + STREAM_S
                count = 0
                while time.monotonic() < t_end:
                    s.sendall(bytes(msg))
                    count += 1
                    time.sleep(0.1)
                note = f"comb of {count} spikes over {STREAM_S:.0f} s"
            print(f"t = {t:5.1f} s   test {i:2d}  {name:<22} sent {str(msg):<11} expect {note}")
            log.append((t, i, name, msg, note))
            time.sleep(GAP_S)
    except KeyboardInterrupt:
        print("\nStopped early.")
    finally:
        s.close()

    print("\nSummary (times are from the first message; on the Scope add the time the first spike appears):")
    for t, i, name, msg, note in log:
        print(f"  {t:5.1f} s  {i:2d}. {str(msg):<11} -> {note}")
    print("Pass: a spike for every 'spike' line, nothing for every 'NO spike' line, and a comb at the end.")


if __name__ == "__main__":
    main()
