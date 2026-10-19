"""Inspect AAI18 binary MQTT data or send a session-bound START/STOP.

Requires only Python 3 and the installed Mosquitto command-line tools.
Run with the motor battery disconnected until the Vehicle Controller's
e-stop and braking paths have been tested on the real robot.
"""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
STATE_FILE = ROOT / ".tools" / "mqtt-command-state.json"
TOPIC_ROOT = "aai18/robot"


def executable(name):
    found = shutil.which(name)
    if found:
        return found
    candidate = Path("C:/Program Files/mosquitto") / (name + ".exe")
    if candidate.exists():
        return str(candidate)
    raise SystemExit(f"Cannot find {name}. Install Mosquitto or add it to PATH.")


def bits(payload, offset, width, signed=False):
    value = (int.from_bytes(payload, "little") >> offset) & ((1 << width) - 1)
    if signed and (value & (1 << (width - 1))):
        value -= 1 << width
    return value, offset + width


def decode(topic, payload):
    if not payload:
        raise ValueError("empty payload")
    offset = 0
    version, offset = bits(payload, offset, 3)
    kind, offset = bits(payload, offset, 2)
    reserved, offset = bits(payload, offset, 3)
    if version != 1 or reserved != 0:
        raise ValueError("unsupported frame header")
    if kind == 3 and topic.endswith("/heartbeat") and len(payload) == 1:
        return {"status": "offline"}
    if kind == 2 and topic.endswith("/heartbeat") and len(payload) == 9:
        session, offset = bits(payload, offset, 32)
        uptime, offset = bits(payload, offset, 32)
        return {"session": f"{session:08x}", "uptime_ms": uptime}
    if kind != 1 or not topic.endswith("/telemetry") or len(payload) < 13:
        raise ValueError("wrong topic, kind or length")
    session, offset = bits(payload, offset, 32)
    uptime, offset = bits(payload, offset, 32)
    sequence, offset = bits(payload, offset, 16)
    state, offset = bits(payload, offset, 3)
    valid, offset = bits(payload, offset, 5)
    faults, offset = bits(payload, offset, 6)
    result = {"session": f"{session:08x}", "uptime_ms": uptime,
              "seq": sequence, "state": state, "valid": valid,
              "fault_bits": faults}
    if valid & 1:
        for name, width in (("speed_l_mm_s", 12), ("speed_r_mm_s", 12),
                            ("encoder_l", 24), ("encoder_r", 24)):
            result[name], offset = bits(payload, offset, width, signed=True)
    if valid & 2:
        result["line_error"], offset = bits(payload, offset, 12, signed=True)
    if valid & 4:
        result["barcode"], offset = bits(payload, offset, 4)
    if valid & 8:
        result["hump_peak_mm"], offset = bits(payload, offset, 10)
    if valid & 16:
        result["obstacle_distance_mm"], offset = bits(payload, offset, 12)
        result["obstacle_width_mm"], offset = bits(payload, offset, 12)
    if len(payload) != (offset + 7) // 8:
        raise ValueError("unexpected trailing bytes")
    unused_bits = len(payload) * 8 - offset
    if unused_bits and (payload[-1] >> (8 - unused_bits)):
        raise ValueError("nonzero padding bits")
    return result


def broker_args(args):
    return ["-h", args.broker, "-p", str(args.port)]


def read_heartbeat(args):
    command = [executable("mosquitto_sub"), *broker_args(args),
               "-t", TOPIC_ROOT + "/heartbeat", "-C", "1", "-F", "%x"]
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=8, check=True)
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError) as error:
        raise SystemExit(f"No live heartbeat from the Pico: {error}") from error
    try:
        payload = bytes.fromhex(result.stdout.strip())
        frame = decode(TOPIC_ROOT + "/heartbeat", payload)
        return int(frame["session"], 16)
    except (ValueError, KeyError) as error:
        raise SystemExit("Heartbeat was offline or malformed; retry after Wi-Fi connects.") from error


def send(args):
    session = read_heartbeat(args)
    if STATE_FILE.exists():
        saved = json.loads(STATE_FILE.read_text(encoding="utf-8"))
    else:
        saved = {}
    if saved.get("session") == session:
        sequence = (saved["sequence"] + 1) & 0xffff
    else:
        sequence = 1
    action = 1 if args.action == "start" else 2
    payload = bytes([1 | (action << 5)]) + session.to_bytes(4, "little") + \
        sequence.to_bytes(2, "little")
    command = [executable("mosquitto_pub"), *broker_args(args),
               "-t", TOPIC_ROOT + "/command", "-s"]
    subprocess.run(command, input=payload, check=True)
    STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
    STATE_FILE.write_text(json.dumps({"session": session, "sequence": sequence}),
                          encoding="utf-8")
    print(f"Sent {args.action.upper()} for session {session:08x}, seq {sequence}.")


def watch(args):
    command = [executable("mosquitto_sub"), *broker_args(args),
               "-t", TOPIC_ROOT + "/telemetry",
               "-t", TOPIC_ROOT + "/heartbeat", "-F", "%t %x"]
    with subprocess.Popen(command, stdout=subprocess.PIPE, text=True) as process:
        try:
            for line in process.stdout:
                try:
                    topic, hex_payload = line.strip().split(" ", 1)
                    print(topic, json.dumps(decode(topic, bytes.fromhex(hex_payload))),
                          flush=True)
                except ValueError as error:
                    print(f"Ignored malformed frame: {error}", file=sys.stderr)
        except KeyboardInterrupt:
            process.terminate()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("watch", "start", "stop"))
    parser.add_argument("--broker", required=True, help="broker PC IPv4 address")
    parser.add_argument("--port", type=int, default=1883)
    args = parser.parse_args()
    if args.action == "watch":
        watch(args)
    else:
        send(args)


if __name__ == "__main__":
    main()
