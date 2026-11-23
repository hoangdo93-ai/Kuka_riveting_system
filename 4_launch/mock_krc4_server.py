#!/usr/bin/env python3
"""
Mock KRC4 Controller Server

Simulates a KUKA KRC4 controller's command/feedback interface for local
testing of KRC4Interface / KRC4MotionBackend without physical hardware.

PROTOCOL (simplified, demonstrative - see PHASE5_COMPLETE_OVERVIEW.md for
the design rationale). Newline-delimited pseudo-XML messages over TCP.

Incoming (from KRC4MotionBackend):
    <Cmd seq="3" type="LIN"><Pos X="100.000" Y="0.000" Z="100.000"
         A="0.000" B="0.000" C="0.000"/><Vel>500.000</Vel></Cmd>
    <Cmd type="ESTOP"/>
    <Cmd type="HEARTBEAT"/>

Outgoing (to KRC4MotionBackend):
    <Status seq="3" state="MOVING" X="40.000" Y="0.000" Z="40.000" err="0"/>
    <Status seq="3" state="COMPLETE" X="100.000" Y="0.000" Z="100.000" err="0"/>

Usage:
    python3 mock_krc4_server.py
    python3 mock_krc4_server.py --port 54600 --speed-factor 5.0
"""

import argparse
import re
import socket
import threading
import time


CMD_RE = re.compile(
    r'<Cmd\s+(?:seq="(?P<seq>\d+)"\s+)?type="(?P<type>\w+)"\s*(?:/>|>'
    r'(?:<Pos\s+X="(?P<x>[-\d.]+)"\s+Y="(?P<y>[-\d.]+)"\s+Z="(?P<z>[-\d.]+)"'
    r'\s+A="[-\d.]+"\s+B="[-\d.]+"\s+C="[-\d.]+"/>)?'
    r'(?:<Vel>(?P<vel>[-\d.]+)</Vel>)?</Cmd>)'
)


class MockRobotState:
    """Holds the simulated robot's current Cartesian position and E-stop flag."""

    def __init__(self):
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.estop = False
        self.lock = threading.Lock()


def handle_client(conn: socket.socket, addr, speed_factor: float, update_hz: float):
    print(f"[mock_krc4] Client connected: {addr}")
    state = MockRobotState()
    buffer = ""

    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            buffer += data.decode("utf-8", errors="ignore")

            while True:
                match = CMD_RE.search(buffer)
                if not match:
                    break
                buffer = buffer[match.end():]
                handle_command(conn, state, match, speed_factor, update_hz)

    except (ConnectionResetError, BrokenPipeError):
        pass
    finally:
        print(f"[mock_krc4] Client disconnected: {addr}")
        conn.close()


def handle_command(conn, state: MockRobotState, match, speed_factor, update_hz):
    cmd_type = match.group("type")
    seq = match.group("seq") or "0"

    if cmd_type == "ESTOP":
        with state.lock:
            state.estop = True
        send_status(conn, seq, "ESTOP", state)
        print("[mock_krc4] *** E-STOP received ***")
        return

    if cmd_type == "HEARTBEAT":
        send_status(conn, seq, "ESTOP" if state.estop else "MOVING", state)
        return

    if cmd_type == "LIN":
        if state.estop:
            send_status(conn, seq, "ESTOP", state)
            return

        with state.lock:
            sx, sy, sz = state.x, state.y, state.z

        tx = float(match.group("x")) if match.group("x") is not None else sx
        ty = float(match.group("y")) if match.group("y") is not None else sy
        tz = float(match.group("z")) if match.group("z") is not None else sz
        vel = float(match.group("vel")) if match.group("vel") is not None else 500.0

        distance = ((tx - sx) ** 2 + (ty - sy) ** 2 + (tz - sz) ** 2) ** 0.5
        duration = max(distance / max(vel, 1.0), 0.05) / max(speed_factor, 0.01)
        steps = max(int(duration * update_hz), 1)

        print(f"[mock_krc4] LIN seq={seq} -> ({tx:.1f}, {ty:.1f}, {tz:.1f}) "
              f"dist={distance:.1f}mm over {duration:.2f}s ({steps} steps)")

        for i in range(1, steps + 1):
            frac = i / steps
            with state.lock:
                if state.estop:
                    send_status(conn, seq, "ESTOP", state)
                    return
                state.x = sx + (tx - sx) * frac
                state.y = sy + (ty - sy) * frac
                state.z = sz + (tz - sz) * frac
            send_status(conn, seq, "MOVING", state)
            time.sleep(1.0 / update_hz)

        with state.lock:
            state.x, state.y, state.z = tx, ty, tz
        send_status(conn, seq, "COMPLETE", state)
        return

    print(f"[mock_krc4] Unknown command type: {cmd_type}")


def send_status(conn, seq, motion_state, state: MockRobotState):
    with state.lock:
        msg = (f'<Status seq="{seq}" state="{motion_state}" '
               f'X="{state.x:.3f}" Y="{state.y:.3f}" Z="{state.z:.3f}" err="0"/>\n')
    try:
        conn.sendall(msg.encode("utf-8"))
    except (BrokenPipeError, ConnectionResetError):
        pass


def main():
    parser = argparse.ArgumentParser(description="Mock KUKA KRC4 controller server")
    parser.add_argument("--port", type=int, default=54600,
                         help="TCP port to listen on (default: 54600)")
    parser.add_argument("--speed-factor", type=float, default=1.0,
                         help="Speed multiplier for simulated motion, e.g. 5.0 "
                              "for 5x faster local testing (default: 1.0)")
    parser.add_argument("--update-hz", type=float, default=20.0,
                         help="Feedback publish rate while moving (default: 20Hz)")
    args = parser.parse_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", args.port))
    server.listen(1)

    print(f"[mock_krc4] Listening on port {args.port} "
          f"(speed_factor={args.speed_factor}, update_hz={args.update_hz})")
    print("[mock_krc4] Waiting for KRC4MotionBackend connection... (Ctrl+C to stop)")

    try:
        while True:
            conn, addr = server.accept()
            client_thread = threading.Thread(
                target=handle_client,
                args=(conn, addr, args.speed_factor, args.update_hz),
                daemon=True)
            client_thread.start()
    except KeyboardInterrupt:
        print("\n[mock_krc4] Shutting down")
    finally:
        server.close()


if __name__ == "__main__":
    main()
