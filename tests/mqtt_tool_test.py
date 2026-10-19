"""Check the PC decoder against fixed wire examples."""

import importlib.util
from pathlib import Path


tool_path = Path(__file__).resolve().parents[1] / "scripts" / "mqtt-tool.py"
spec = importlib.util.spec_from_file_location("mqtt_tool", tool_path)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

heartbeat = bytes.fromhex("11 78 56 34 12 e8 03 00 00")
assert tool.decode("aai18/robot/heartbeat", heartbeat) == {
    "session": "12345678", "uptime_ms": 1000
}

empty = bytes.fromhex("09 78 56 34 12 e8 03 00 00 00 00 00 00")
decoded = tool.decode("aai18/robot/telemetry", empty)
assert decoded["session"] == "12345678"
assert decoded["uptime_ms"] == 1000
assert decoded["seq"] == 0
assert decoded["valid"] == 0

assert tool.decode("aai18/robot/heartbeat", b"\x19") == {"status": "offline"}
