#!/usr/bin/env python3
"""
realtime_diagnostic.py — Hardware-in-the-loop diagnostic script for real motors and encoders.
Tests serial communication, encoder reading, motor response, and control loop latency.

Usage:
  python3 tests/realtime_diagnostic.py --port /dev/ttyUSB0
  python3 tests/realtime_diagnostic.py --port COM3 --baud 1000000
"""

import serial
import serial.tools.list_ports
import struct
import time
import math
import argparse
import json
import sys
from dataclasses import dataclass
from typing import List, Optional

# ── Serial Protocol Constants ────────────────────────────────────────────
ENCODER_HEADER   = bytes([0xAA, 0x55])
ENCODER_PKT_SIZE = 15   # 2 header + 12 data + 1 checksum

DC_CMD_HEADER    = 0xCC
BLDC_CMD_HEADER  = 0xBB


@dataclass
class JointDiag:
    joint: int
    serial_ok: bool = False
    encoder_ok: bool = False
    motor_ok: bool = False
    raw_count: int = 0
    angle_deg: float = 0.0
    response_latency_ms: float = 0.0
    error: str = ""


def discover_ports():
    """List all available serial ports."""
    ports = serial.tools.list_ports.comports()
    print("\n  Available Serial Ports:")
    for p in ports:
        print(f"    {p.device}: {p.description} [VID:PID = {p.vid}:{p.pid}]")
    return ports


def read_encoder_packet(ser: serial.Serial, timeout: float = 2.0) -> Optional[List[int]]:
    """Read one valid encoder packet from serial. Returns list of 6 raw counts or None."""
    buf = bytearray()
    start = time.monotonic()
    
    while time.monotonic() - start < timeout:
        chunk = ser.read(64)
        if not chunk:
            continue
        buf.extend(chunk)
        
        while len(buf) >= ENCODER_PKT_SIZE:
            idx = buf.find(ENCODER_HEADER)
            if idx == -1:
                buf.clear()
                break
            if idx > 0:
                del buf[:idx]
            if len(buf) < ENCODER_PKT_SIZE:
                break
            
            packet = bytes(buf[:ENCODER_PKT_SIZE])
            del buf[:ENCODER_PKT_SIZE]
            
            # Validate CRC
            data_bytes = packet[2:-1]
            crc = 0
            for b in data_bytes:
                crc ^= b
            if crc != packet[-1]:
                continue
            
            # Unpack 6 × uint16
            counts = struct.unpack('>6H', data_bytes)
            return list(counts)
    
    return None


def send_motor_nudge(ser: serial.Serial, joint_idx: int, motor_type: str = 'DC'):
    """Send a tiny motor nudge command for testing."""
    if motor_type == 'DC':
        # Duty = 33768 (slightly above center 32768 = stopped)
        duty = 33768  # ~3% forward
        packet = struct.pack('>BBH', DC_CMD_HEADER, joint_idx, duty)
        ser.write(packet)
    elif motor_type == 'BLDC':
        # Small 3-phase duty for FOC test
        packet = struct.pack('>BB3H', BLDC_CMD_HEADER, joint_idx, 33000, 32768, 32768)
        ser.write(packet)


def send_motor_stop(ser: serial.Serial, joint_idx: int, motor_type: str = 'DC'):
    """Send a stop command."""
    if motor_type == 'DC':
        packet = struct.pack('>BBH', DC_CMD_HEADER, joint_idx, 32768)  # center = stopped
        ser.write(packet)
    elif motor_type == 'BLDC':
        packet = struct.pack('>BB3H', BLDC_CMD_HEADER, joint_idx, 32768, 32768, 32768)
        ser.write(packet)


def run_diagnostic(port: str, baud: int = 1000000, num_joints: int = 6) -> List[JointDiag]:
    """Run full hardware diagnostic."""
    results = [JointDiag(joint=i+1) for i in range(num_joints)]
    
    print(f"\n  Opening serial port: {port} @ {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"  ✗ Failed to open {port}: {e}")
        for r in results:
            r.error = f"Serial port error: {e}"
        return results
    
    # ── Phase 1: Serial Communication Check ──────────────────────────────
    print("\n  Phase 1: Serial Communication")
    for r in results:
        r.serial_ok = True
    print("    ✓ Serial port opened successfully")
    
    # ── Phase 2: Encoder Read Test ───────────────────────────────────────
    print("\n  Phase 2: Encoder Read Test")
    for attempt in range(3):
        counts = read_encoder_packet(ser, timeout=2.0)
        if counts:
            print(f"    ✓ Received encoder packet (attempt {attempt + 1})")
            for i, c in enumerate(counts):
                if i < num_joints:
                    results[i].encoder_ok = True
                    results[i].raw_count = c
                    results[i].angle_deg = (c / 4096.0) * 360.0
                    print(f"      J{i+1}: raw={c:5d}  angle={results[i].angle_deg:.2f}°")
            break
    else:
        print("    ✗ No valid encoder packets received after 3 attempts")
    
    # ── Phase 3: Motor Micro-Nudge Test ──────────────────────────────────
    print("\n  Phase 3: Motor Micro-Nudge Test (±0.1°)")
    for i in range(num_joints):
        if not results[i].encoder_ok:
            results[i].error = "Skipped — encoder not responding"
            print(f"    J{i+1}: Skipped (no encoder)")
            continue
        
        # Read initial position
        initial_counts = read_encoder_packet(ser, timeout=1.0)
        if not initial_counts:
            results[i].error = "Could not read initial position"
            continue
        
        initial = initial_counts[i]
        
        # Send micro nudge
        t0 = time.monotonic()
        send_motor_nudge(ser, i, 'DC')
        time.sleep(0.1)  # Let motor move
        send_motor_stop(ser, i, 'DC')
        
        # Read new position
        time.sleep(0.05)
        final_counts = read_encoder_packet(ser, timeout=1.0)
        latency = (time.monotonic() - t0) * 1000
        
        if final_counts:
            delta = abs(final_counts[i] - initial)
            results[i].response_latency_ms = latency
            
            if delta > 0:  # Any movement detected
                results[i].motor_ok = True
                print(f"    J{i+1}: ✓ Motor responded (Δ={delta} counts, {latency:.1f}ms)")
            else:
                results[i].error = "No encoder change after motor pulse"
                print(f"    J{i+1}: ✗ Motor did not respond (Δ=0)")
        else:
            results[i].error = "No encoder response after nudge"
            print(f"    J{i+1}: ✗ No encoder response after nudge")
    
    # ── Phase 4: Control Loop Latency Measurement ────────────────────────
    print("\n  Phase 4: Control Loop Latency")
    latencies = []
    for _ in range(20):
        t0 = time.monotonic()
        counts = read_encoder_packet(ser, timeout=0.1)
        if counts:
            latencies.append((time.monotonic() - t0) * 1000)
    
    if latencies:
        avg_lat = sum(latencies) / len(latencies)
        max_lat = max(latencies)
        print(f"    Avg read latency: {avg_lat:.2f}ms")
        print(f"    Max read latency: {max_lat:.2f}ms")
        print(f"    Effective rate: {1000/avg_lat:.0f} Hz" if avg_lat > 0 else "    N/A")
    else:
        print("    ✗ Could not measure latency")
    
    ser.close()
    return results


def main():
    parser = argparse.ArgumentParser(description='RoboForge Real-Time Hardware Diagnostic')
    parser.add_argument('--port', type=str, default=None, help='Serial port (e.g., COM3 or /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=1000000, help='Baud rate (default: 1000000)')
    parser.add_argument('--joints', type=int, default=6, help='Number of joints (default: 6)')
    parser.add_argument('--list-ports', action='store_true', help='List available serial ports and exit')
    args = parser.parse_args()

    print("=" * 60)
    print("  RoboForge v8.2 — Real-Time Hardware Diagnostic")
    print("=" * 60)

    if args.list_ports or not args.port:
        ports = discover_ports()
        if not args.port:
            if not ports:
                print("\n  No serial ports found. Connect your encoder/motor board.")
                return 1
            args.port = ports[0].device
            print(f"\n  Auto-selected: {args.port}")

    results = run_diagnostic(args.port, args.baud, args.joints)

    # ── Summary ──────────────────────────────────────────────────────────
    print("\n" + "─" * 60)
    print("  DIAGNOSTIC SUMMARY")
    print("─" * 60)
    
    all_pass = True
    for r in results:
        serial_s = "✓" if r.serial_ok else "✗"
        enc_s = "✓" if r.encoder_ok else "✗"
        mot_s = "✓" if r.motor_ok else "✗"
        status = "PASS" if (r.serial_ok and r.encoder_ok and r.motor_ok) else "FAIL"
        if status == "FAIL":
            all_pass = False
        print(f"  J{r.joint}: [{status}] Serial={serial_s}  Encoder={enc_s}  Motor={mot_s}  "
              f"Count={r.raw_count}  Latency={r.response_latency_ms:.1f}ms  "
              f"{'✓' if not r.error else r.error}")
    
    print("─" * 60)
    if all_pass:
        print("  ✓ ALL JOINTS OPERATIONAL")
    else:
        print("  ✗ SOME JOINTS HAVE ISSUES — review output above")
    print("─" * 60)

    # Write JSON report
    report = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "port": args.port,
        "baud": args.baud,
        "results": [
            {
                "joint": r.joint, "serial_ok": r.serial_ok,
                "encoder_ok": r.encoder_ok, "motor_ok": r.motor_ok,
                "raw_count": r.raw_count, "angle_deg": r.angle_deg,
                "latency_ms": r.response_latency_ms, "error": r.error
            }
            for r in results
        ]
    }
    with open("tests/diagnostic_report.json", "w") as f:
        json.dump(report, f, indent=2)
    print(f"\n  Report saved to tests/diagnostic_report.json")

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
