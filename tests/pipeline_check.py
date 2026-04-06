#!/usr/bin/env python3
"""
pipeline_check.py — Comprehensive RoboForge pipeline verification script.
Run inside Docker: docker exec roboforge_bridge python3 /ros_ws/tests/pipeline_check.py
Or locally: python3 tests/pipeline_check.py
"""

import sys
import json
import time
import subprocess
import socket
import traceback
from dataclasses import dataclass, field
from typing import List, Optional

# ── Configuration ────────────────────────────────────────────────────────
ROS_BRIDGE_WS  = "ws://localhost:9090"
REST_API_URL   = "http://localhost:8765"
ROS_DOMAIN_ID  = 42

@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str
    latency_ms: float = 0.0

results: List[CheckResult] = []

def check(name: str):
    """Decorator for pipeline checks."""
    def decorator(fn):
        def wrapper():
            t0 = time.monotonic()
            try:
                passed, detail = fn()
                lat = (time.monotonic() - t0) * 1000
                results.append(CheckResult(name, passed, detail, lat))
                status = "✓" if passed else "✗"
                print(f"  {status} {name}: {detail} ({lat:.0f}ms)")
            except Exception as e:
                lat = (time.monotonic() - t0) * 1000
                results.append(CheckResult(name, False, str(e), lat))
                print(f"  ✗ {name}: EXCEPTION — {e}")
        return wrapper
    return decorator


# ═══════════════════════════════════════════════════════════════════════════
# CHECK 1: REST API Health
# ═══════════════════════════════════════════════════════════════════════════
@check("REST API /health")
def check_rest_health():
    import urllib.request
    resp = urllib.request.urlopen(f"{REST_API_URL}/health", timeout=5)
    data = json.loads(resp.read())
    if data.get("status") == "ok":
        return True, f"status=ok, moveit_ready={data.get('moveit_ready')}"
    return False, f"Unexpected status: {data}"


# ═══════════════════════════════════════════════════════════════════════════
# CHECK 2: REST API /api/hardware/ports
# ═══════════════════════════════════════════════════════════════════════════
@check("REST API /api/hardware/ports")
def check_rest_ports():
    import urllib.request
    resp = urllib.request.urlopen(f"{REST_API_URL}/api/hardware/ports", timeout=5)
    data = json.loads(resp.read())
    ports = data.get("ports", [])
    return True, f"Found {len(ports)} port(s)"


# ═══════════════════════════════════════════════════════════════════════════
# CHECK 3: REST API /api/logs
# ═══════════════════════════════════════════════════════════════════════════
@check("REST API /api/logs")
def check_rest_logs():
    import urllib.request
    resp = urllib.request.urlopen(f"{REST_API_URL}/api/logs?n=5", timeout=5)
    data = json.loads(resp.read())
    entries = data.get("entries", [])
    return True, f"{len(entries)} log entries"


# ═══════════════════════════════════════════════════════════════════════════
# CHECK 4: WebSocket connectivity (basic TCP check)
# ═══════════════════════════════════════════════════════════════════════════
@check("WebSocket port 9090 reachable")
def check_ws_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3)
    try:
        s.connect(("localhost", 9090))
        s.close()
        return True, "Port 9090 is open"
    except Exception as e:
        return False, str(e)


# ═══════════════════════════════════════════════════════════════════════════
# CHECK 5: ROS 2 node list (requires ROS 2 environment)
# ═══════════════════════════════════════════════════════════════════════════
@check("ROS 2 node list")
def check_ros_nodes():
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True, text=True, timeout=10,
            env={**dict(__import__('os').environ), "ROS_DOMAIN_ID": str(ROS_DOMAIN_ID)}
        )
        nodes = [n.strip() for n in result.stdout.strip().split('\n') if n.strip()]
        if not nodes:
            return False, "No ROS 2 nodes found"
        bridge = any('bridge' in n.lower() for n in nodes)
        return True, f"{len(nodes)} nodes found, bridge={'✓' if bridge else '✗'}: {', '.join(nodes[:5])}"
    except FileNotFoundError:
        return False, "ros2 CLI not found — run inside Docker container"


# ═══════════════════════════════════════════════════════════════════════════
# CHECK 6: /joint_states topic is publishing
# ═══════════════════════════════════════════════════════════════════════════
@check("/joint_states topic alive")
def check_joint_states():
    try:
        result = subprocess.run(
            ["ros2", "topic", "hz", "/joint_states", "--window", "20"],
            capture_output=True, text=True, timeout=8,
            env={**dict(__import__('os').environ), "ROS_DOMAIN_ID": str(ROS_DOMAIN_ID)}
        )
        # Parse average rate from output
        for line in result.stdout.split('\n'):
            if 'average rate' in line:
                rate = float(line.split(':')[1].strip().split()[0])
                return rate > 10, f"Rate: {rate:.1f} Hz"
        return False, "Could not determine rate"
    except FileNotFoundError:
        return False, "ros2 CLI not found — run inside Docker"
    except subprocess.TimeoutExpired:
        return False, "Timeout — topic may not be publishing"


# ═══════════════════════════════════════════════════════════════════════════
# CHECK 7: /compute_ik service available
# ═══════════════════════════════════════════════════════════════════════════
@check("MoveIt /compute_ik service")
def check_ik_service():
    try:
        result = subprocess.run(
            ["ros2", "service", "list"],
            capture_output=True, text=True, timeout=10,
            env={**dict(__import__('os').environ), "ROS_DOMAIN_ID": str(ROS_DOMAIN_ID)}
        )
        services = result.stdout.strip().split('\n')
        ik_found = any('/compute_ik' in s for s in services)
        fk_found = any('/compute_fk' in s for s in services)
        return ik_found, f"IK={'✓' if ik_found else '✗'}, FK={'✓' if fk_found else '✗'}"
    except FileNotFoundError:
        return False, "ros2 CLI not found"


# ═══════════════════════════════════════════════════════════════════════════
# CHECK 8: Full IK roundtrip via WebSocket
# ═══════════════════════════════════════════════════════════════════════════
@check("IK roundtrip via WebSocket")
def check_ik_roundtrip():
    try:
        import websocket
        ws = websocket.create_connection(ROS_BRIDGE_WS, timeout=5)
        ik_request = json.dumps({
            "op": "call_service",
            "service": "/compute_ik",
            "id": "pipeline-check-ik",
            "args": {
                "ik_request": {
                    "group_name": "manipulator",
                    "avoid_collisions": True,
                    "robot_state": {
                        "joint_state": {
                            "name": ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"],
                            "position": [0, 0, 0, 0, 0, 0]
                        }
                    },
                    "pose_stamped": {
                        "header": {"frame_id": "base_link"},
                        "pose": {
                            "position": {"x": 0.5, "y": 0.0, "z": 0.8},
                            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
                        }
                    },
                    "timeout": {"sec": 5, "nanosec": 0}
                }
            }
        })
        ws.send(ik_request)
        response = json.loads(ws.recv())
        ws.close()
        
        if response.get("result") and response.get("values", {}).get("error_code", {}).get("val") == 1:
            joints = response["values"]["solution"]["joint_state"]["position"]
            return True, f"Solved: [{', '.join(f'{j:.3f}' for j in joints[:6])}]"
        return False, f"IK failed: {response.get('values', {}).get('error_code', {})}"
    except ImportError:
        return False, "websocket-client not installed (pip install websocket-client)"
    except Exception as e:
        return False, str(e)


# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════
def main():
    print("=" * 60)
    print("  RoboForge v8.2 — Pipeline Verification")
    print("=" * 60)
    print()

    checks = [
        check_rest_health,
        check_rest_ports,
        check_rest_logs,
        check_ws_port,
        check_ros_nodes,
        check_joint_states,
        check_ik_service,
        check_ik_roundtrip,
    ]

    for fn in checks:
        fn()

    print()
    print("─" * 60)
    passed = sum(1 for r in results if r.passed)
    total = len(results)
    print(f"  Results: {passed}/{total} checks passed")
    
    if passed == total:
        print("  ✓ Pipeline is fully operational")
    else:
        print("  ✗ Some checks failed — review output above")
        for r in results:
            if not r.passed:
                print(f"    ✗ {r.name}: {r.detail}")
    
    print("─" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
