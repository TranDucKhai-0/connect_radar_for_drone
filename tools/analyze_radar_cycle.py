#!/usr/bin/env python3
"""
Radar Cycle & Frame Integrity Monitor
===========================================
This tool measures the transmission cycle time (in milliseconds) of MR72 and U10 radars.
For MR72, it identifies the start of a frame by the Object List Status message (0x60A + m_id * 0x10)
and validates if all expected Object General Info messages (0x60B + m_id * 0x10) are received.
For U10, it parses the CAN payload stream to identify frame boundary Magic Words and validates packet completeness.

It supports:
1. Live CAN monitoring on a SocketCAN interface (e.g., can0).
2. Parsing a candump text log file.
"""

import sys
import os
import time
import struct
import argparse
import re
import math
import select
from collections import deque

# Constants for socketcan
CAN_EFF_MASK = 0x1FFFFFFF
CAN_ERR_FLAG = 0x20000000
CAN_RTR_FLAG = 0x40000000

# Base IDs for MR72
MR72_OBJECT_LIST_STATUS = 0x60A
MR72_OBJECT_GENERAL_INFO = 0x60B

# Unified regex to parse candump lines
# Formats supported:
#   1. (1689218204.123456) can0 60A#0500000000000000
#   2. (1689218204.123456)  can0  60A   [8]  05 00 00 00 00 00 00 00
#   3. can0 60A [8] 05 00 00 00 00 00 00 00
#   4. can0 60A#0500000000000000
CANDUMP_PATTERN = re.compile(
    r'^\s*(?:\((?P<ts>\d+\.\d+)\))?\s*(?P<iface>\S+)\s+(?P<id>[0-9A-Fa-f]+)'
    r'(?:#(?P<data_hex>[0-9A-Fa-f]*)|(?:\s+\[(?P<dlc>\d+)\]\s+(?P<data_bytes>[0-9A-Fa-f\s]+)))\s*$'
)

class RadarStats:
    def __init__(self, radar_id, radar_type, base_can_id, target_cycle):
        self.radar_id = radar_id
        self.radar_type = radar_type
        self.base_can_id = hex(base_can_id)
        self.target_cycle = target_cycle
        
        # Frame timing tracking
        self.last_header_time = None  # timestamp in ms
        self.cycle_times = []        # list of cycle times in ms
        
        # Frame integrity tracking
        self.expected_objs = 0
        self.received_objs = set()
        
        # Counts
        self.total_frames = 0
        self.complete_frames = 0
        self.incomplete_frames = 0
        
        # Temporary frame trackers for warning output
        self.frame_number = 0

    def start_new_frame(self, timestamp_ms, expected_objs, warnings_queue):
        # 1. Close current frame (if any) and check integrity
        if self.total_frames > 0:
            actual_count = len(self.received_objs)
            if actual_count == self.expected_objs:
                self.complete_frames += 1
            else:
                self.incomplete_frames += 1
                msg = (f"[WARNING] Radar {self.radar_type} ID {self.radar_id} Frame #{self.frame_number}: "
                       f"Incomplete frame. Expected {self.expected_objs} objects, "
                       f"got {actual_count}.")
                warnings_queue.append(msg)

        # 2. Record cycle time
        if self.last_header_time is not None:
            cycle = timestamp_ms - self.last_header_time
            self.cycle_times.append(cycle)
            # Limit the memory size of cycle_times for statistics
            if len(self.cycle_times) > 1000:
                self.cycle_times.pop(0)
        
        # 3. Initialize new frame
        self.last_header_time = timestamp_ms
        self.expected_objs = expected_objs
        self.received_objs.clear()
        self.total_frames += 1
        self.frame_number += 1

    def add_object(self, obj_id):
        self.received_objs.add(obj_id)

    def get_stats(self):
        if not self.cycle_times:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        
        avg_c = sum(self.cycle_times) / len(self.cycle_times)
        min_c = min(self.cycle_times)
        max_c = max(self.cycle_times)
        
        # Standard deviation (Jitter)
        variance = sum((x - avg_c) ** 2 for x in self.cycle_times) / len(self.cycle_times)
        jitter = math.sqrt(variance)
        
        # Get last cycle
        last_c = self.cycle_times[-1]
        
        return last_c, avg_c, min_c, max_c, jitter

class U10Parser:
    def __init__(self, radar_id, stats, warnings_queue):
        self.radar_id = radar_id
        self.stats = stats
        self.warnings_queue = warnings_queue
        self.buffer = bytearray()
        self.state = 0  # 0: FIND_SYNC, 1: READ_HEADER, 2: READ_PAYLOAD
        self.expected_length = 0
        
    def feed(self, data_bytes, timestamp_ms):
        self.buffer.extend(data_bytes)
        
        is_parsing = True
        while is_parsing:
            if self.state == 0:  # FIND_SYNC
                if len(self.buffer) < 8:
                    is_parsing = False
                    break
                
                magic = b'\x02\x01\x04\x03\x06\x05\x08\x07'
                idx = self.buffer.find(magic)
                if idx != -1:
                    # Found Magic Word!
                    # If we had a partial packet/header before, it means the previous frame was incomplete
                    if idx > 0 and self.expected_length > 0:
                        self.stats.incomplete_frames += 1
                        self.stats.total_frames += 1
                        msg = (f"[WARNING] Radar U10 ID {self.radar_id} Frame #{self.stats.frame_number}: "
                               f"Incomplete frame. Expected {self.stats.expected_objs} points, "
                               f"got incomplete packet (found sync word after {idx} bytes).")
                        self.warnings_queue.append(msg)
                    
                    self.buffer = self.buffer[idx:]
                    self.state = 1
                else:
                    if len(self.buffer) > 7:
                        self.buffer = self.buffer[-7:]
                    is_parsing = False
            
            elif self.state == 1:  # READ_HEADER
                if len(self.buffer) < 16:
                    is_parsing = False
                    break
                
                # Length is at bytes 12-15
                total_length = struct.unpack("<I", self.buffer[12:16])[0]
                if total_length < 40 or total_length > 8192:
                    # Invalid length, discard first byte and search again
                    self.buffer = self.buffer[1:]
                    self.state = 0
                else:
                    self.expected_length = total_length
                    self.state = 2
            
            elif self.state == 2:  # READ_PAYLOAD
                if len(self.buffer) < self.expected_length:
                    is_parsing = False
                    break
                
                # We have a complete packet!
                packet = self.buffer[:self.expected_length]
                self.buffer = self.buffer[self.expected_length:]
                self.state = 0
                
                # Parse packet header
                frame_num = struct.unpack("<I", packet[20:24])[0]
                points_num = struct.unpack("<I", packet[28:32])[0]
                
                # Record cycle timing
                if self.stats.last_header_time is not None:
                    cycle = timestamp_ms - self.stats.last_header_time
                    self.stats.cycle_times.append(cycle)
                    if len(self.stats.cycle_times) > 1000:
                        self.stats.cycle_times.pop(0)
                        
                    # Check cycle deviation
                    if abs(cycle - self.stats.target_cycle) / self.stats.target_cycle > 0.20:
                        self.warnings_queue.append(
                            f"[WARNING] Radar U10 ID {self.radar_id} Frame #{self.stats.frame_number}: "
                            f"Abnormal cycle time: {cycle:.2f} ms (expected {self.stats.target_cycle:.1f} ms)"
                        )
                
                self.stats.last_header_time = timestamp_ms
                self.stats.expected_objs = points_num
                self.stats.total_frames += 1
                self.stats.complete_frames += 1
                self.stats.frame_number = frame_num

def identify_radar(can_id):
    # Check if MR72
    # MR72 list status: 0x60A + r_id * 0x10
    # MR72 object info: 0x60B + r_id * 0x10
    if (can_id & 0xFF0F) == 0x60A:
        r_id = (can_id - 0x60A) // 0x10
        if 0 <= r_id < 8:
            return "MR72", r_id, 0x60A + r_id * 0x10
    elif (can_id & 0xFF0F) == 0x60B:
        r_id = (can_id - 0x60B) // 0x10
        if 0 <= r_id < 8:
            return "MR72", r_id, 0x60A + r_id * 0x10
            
    # Check if U10
    # U10: 0x0D1 to 0x0DF (ID 1 to 15)
    if 0x0D1 <= can_id <= 0x0DF:
        r_id = can_id - 0x0D0
        return "U10", r_id, can_id
        
    return None, None, None

def parse_candump_line(line):
    match = CANDUMP_PATTERN.match(line)
    if not match:
        return None
    
    # Extract timestamp
    ts_str = match.group('ts')
    ts = float(ts_str) if ts_str else None
    
    # Extract ID
    can_id = int(match.group('id'), 16)
    
    # Extract Data bytes
    data_hex = match.group('data_hex')
    if data_hex is not None:
        try:
            data = bytes.fromhex(data_hex)
        except ValueError:
            data = b''
    else:
        data_bytes = match.group('data_bytes').replace(" ", "").strip()
        try:
            data = bytes.fromhex(data_bytes)
        except ValueError:
            data = b''
            
    return ts, can_id, data

def draw_dashboard(radars, warnings, is_live=True, file_path=None, seen_can_ids=None):
    # Terminal clear and move cursor to top-left
    if is_live:
        sys.stdout.write("\033[H\033[J")
        
    print("=" * 114)
    print("                                   RADAR CYCLE & INTEGRITY MONITOR")
    if is_live:
        print(f" Live Monitoring | Press Ctrl+C to Stop")
    else:
        print(f" Log File Analysis | File: {file_path}")
    print("=" * 114)
    print(f"┌──────────┬──────────┬──────────────┬──────────────┬──────────────┬──────────────┬─────────────┬─────────────┬─────────────┐")
    print(f"│ Radar ID │ Type     │ Total Frames │ Last Cyc(ms) │ Avg Cyc(ms)  │ Min Cyc(ms)  │ Max Cyc(ms) │ Jitter(ms)  │ Incomplete  │")
    print(f"├──────────┼──────────┼──────────────┼──────────────┼──────────────┼──────────────┼─────────────┼─────────────┼─────────────┤")
    
    active_radars = [r for r in radars.values() if r.total_frames > 0]
    if not active_radars:
        print(f"│                                           No active radar detected yet...                                          │")
    else:
        for r in sorted(active_radars, key=lambda x: (x.radar_type, x.radar_id)):
            last_c, avg_c, min_c, max_c, jitter = r.get_stats()
            inc_percent = (r.incomplete_frames / r.total_frames * 100) if r.total_frames > 0 else 0
            
            # Format outputs
            radar_label = f"0x{r.base_can_id[2:].upper()}"
            id_cell = f"{radar_label:<8}"
            type_cell = f"{r.radar_type:<8}"
            frames_cell = f"{r.total_frames:<12}"
            
            last_val = f"{last_c:10.2f}  " if last_c > 0 else "    N/A     "
            if is_live and last_c > 0 and abs(last_c - r.target_cycle) / r.target_cycle > 0.20:
                # Color code: Red output using ANSI
                last_cell = f"\033[91m{last_val}\033[0m"
            else:
                last_cell = last_val
                
            avg_cell = f"{avg_c:10.2f}  " if avg_c > 0 else "    N/A     "
            min_cell = f"{min_c:10.2f}  " if min_c > 0 else "    N/A     "
            max_cell = f"{max_c:9.2f}  " if max_c > 0 else "    N/A    "
            jit_cell = f"{jitter:9.2f}  " if jitter > 0 else "    N/A    "
            
            inc_val = f"{r.incomplete_frames} ({inc_percent:.1f}%)"
            inc_val_padded = f"{inc_val:<11}"
            if is_live and r.incomplete_frames > 0:
                inc_cell = f"\033[93m{inc_val_padded}\033[0m"
            else:
                inc_cell = inc_val_padded
                
            print(f"│ {id_cell} │ {type_cell} │ {frames_cell} │ {last_cell} │ {avg_cell} │ {min_cell} │ {max_cell} │ {jit_cell} │ {inc_cell} │")
            
    print(f"└──────────┴──────────┴──────────────┴──────────────┴──────────────┴──────────────┴─────────────┴─────────────┴─────────────┘")
    
    if is_live and seen_can_ids is not None:
        hex_ids = ", ".join(f"0x{x:03X}" for x in sorted(seen_can_ids))
        print(f"Diagnostics - All Detected CAN IDs: {hex_ids if hex_ids else 'None yet'}")
        print("-" * 114)
        
    print("\nRecent Warnings / Anomalies:")
    print("-" * 114)
    
    # In live mode, show only the last 15 warnings to avoid screen clutter.
    # In file mode, show all warnings (up to the 1000 capacity).
    display_warnings = list(warnings)[-15:] if is_live else warnings
    for w in display_warnings:
        if "[WARNING]" in w and is_live:
            # Color code yellow
            print(f"\033[93m{w}\033[0m")
        elif "[CRITICAL]" in w and is_live:
            # Color code red
            print(f"\033[91m{w}\033[0m")
        else:
            print(w)
    if not warnings:
        print(" (No warnings or frame integrity issues detected)")
    print("-" * 114)
    sys.stdout.flush()

def main():
    parser = argparse.ArgumentParser(description="Monitor MR72 and U10 Radar transmission cycles and frame integrity.")
    parser.add_argument("-i", "--interface", default="can0", help="CAN interface to listen to (default: can0)")
    parser.add_argument("-f", "--file", default=None, help="Path to candump log file to parse instead of live capture")
    parser.add_argument("-t", "--target-cycle", type=float, default=None, help="Expected cycle time in milliseconds (default: 30.0 for MR72, 50.0 for U10)")
    args = parser.parse_args()

    target_cycle_mr72 = args.target_cycle if args.target_cycle is not None else 30.0
    target_cycle_u10 = args.target_cycle if args.target_cycle is not None else 50.0

    radars = {}
    u10_parsers = {}
    warnings = deque(maxlen=1000)

    if args.file:
        # Mode: Parse Log File
        if not os.path.exists(args.file):
            print(f"Error: Log file '{args.file}' does not exist.")
            sys.exit(1)
            
        print(f"Analyzing log file: {args.file}...")
        
        total_lines = 0
        parsed_lines = 0
        
        with open(args.file, "r") as f:
            for line in f:
                total_lines += 1
                res = parse_candump_line(line)
                if not res:
                    continue
                parsed_lines += 1
                ts_sec, can_id, data = res
                
                # Determine radar type and ID
                radar_type, r_id, base_can_id = identify_radar(can_id)
                if not radar_type:
                    continue
                
                key = (radar_type, r_id)
                if key not in radars:
                    t_cycle = target_cycle_mr72 if radar_type == "MR72" else target_cycle_u10
                    radars[key] = RadarStats(r_id, radar_type, base_can_id, t_cycle)
                
                # If no timestamp in the log line, use line number * target_cycle as timestamp
                if ts_sec is None:
                    ts_ms = total_lines * radars[key].target_cycle
                else:
                    ts_ms = ts_sec * 1000.0
                
                if radar_type == "MR72":
                    # Check which message type
                    # 1. 0x60A + ID * 0x10 -> Object List Status
                    if (can_id & 0xFF0F) == MR72_OBJECT_LIST_STATUS:
                        if len(data) >= 1:
                            nof_objects = data[0]
                            
                            # Check cycle time deviation warning
                            r = radars[key]
                            if r.last_header_time is not None:
                                cycle = ts_ms - r.last_header_time
                                if abs(cycle - r.target_cycle) / r.target_cycle > 0.20:
                                    warnings.append(
                                        f"[WARNING] Radar {r.radar_type} ID {r_id} Frame #{r.frame_number}: "
                                        f"Abnormal cycle time: {cycle:.2f} ms (expected {r.target_cycle:.1f} ms)"
                                    )
                                    
                            r.start_new_frame(ts_ms, nof_objects, warnings)
                            
                    # 2. 0x60B + ID * 0x10 -> Object General Info
                    elif (can_id & 0xFF0F) == MR72_OBJECT_GENERAL_INFO:
                        if len(data) >= 1:
                            obj_id = data[0]
                            radars[key].add_object(obj_id)
                            
                elif radar_type == "U10":
                    if key not in u10_parsers:
                        u10_parsers[key] = U10Parser(r_id, radars[key], warnings)
                    u10_parsers[key].feed(data, ts_ms)
                            
        # Finalize last frames for all radars
        for key, r in radars.items():
            if r.total_frames > 0:
                if r.radar_type == "MR72":
                    actual_count = len(r.received_objs)
                    if actual_count == r.expected_objs:
                        r.complete_frames += 1
                    else:
                        r.incomplete_frames += 1
                        warnings.append(
                            f"[WARNING] Radar {r.radar_type} ID {r.radar_id} Frame #{r.frame_number}: "
                            f"Incomplete frame. Expected {r.expected_objs} objects, got {actual_count}."
                        )
                elif r.radar_type == "U10":
                    parser = u10_parsers.get(key)
                    if parser and parser.state != 0:
                        r.incomplete_frames += 1
                        r.total_frames += 1
                        warnings.append(
                            f"[WARNING] Radar {r.radar_type} ID {r.radar_id} Frame #{r.frame_number}: "
                            f"Incomplete frame. File ended before payload was fully received."
                        )
                    
        print(f"Finished parsing log file. Parsed {parsed_lines} of {total_lines} lines.")
        draw_dashboard(radars, list(warnings), is_live=False, file_path=args.file)
        
    else:
        # Mode: Live CAN Monitoring
        try:
            import socket
        except ImportError:
            print("Error: 'socket' module not found. Cannot run live CAN monitor.")
            sys.exit(1)

        # Create raw CAN socket
        try:
            s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.bind((args.interface,))
            s.setblocking(False)
        except OSError as e:
            print(f"Error binding to CAN interface '{args.interface}': {e}")
            print("Make sure the interface is up (e.g. 'sudo ip link set can0 up type can bitrate 500000')")
            sys.exit(1)

        print(f"Successfully bound to {args.interface}. Starting live monitor...")
        
        last_draw_time = 0
        seen_can_ids = set()
        try:
            while True:
                # Wait for socket data with 100ms timeout
                rlist, _, _ = select.select([s], [], [], 0.1)
                
                # Draw interface if 200ms has elapsed since last draw
                now_s = time.time()
                if now_s - last_draw_time >= 0.2:
                    draw_dashboard(radars, list(warnings), is_live=True, seen_can_ids=seen_can_ids)
                    last_draw_time = now_s
                
                if not rlist:
                    continue
                
                try:
                    # Read a raw socketcan frame (16 bytes)
                    # struct can_frame is:
                    #   can_id  : 32-bit (unsigned int)
                    #   can_dlc : 8-bit (unsigned char)
                    #   __pad   : 3 bytes
                    #   data    : 8 bytes
                    cf, addr = s.recvfrom(16)
                    if len(cf) < 16:
                        continue
                    
                    can_id_raw, can_dlc, r1, r2, r3, data = struct.unpack("=IB3b8s", cf)
                    
                    # Skip error/rtr frames
                    if can_id_raw & (CAN_ERR_FLAG | CAN_RTR_FLAG):
                        continue
                        
                    can_id = can_id_raw & CAN_EFF_MASK
                    seen_can_ids.add(can_id)
                    ts_ms = time.time() * 1000.0
                    
                    # Determine radar type and ID
                    radar_type, r_id, base_can_id = identify_radar(can_id)
                    if not radar_type:
                        continue
                    
                    key = (radar_type, r_id)
                    if key not in radars:
                        t_cycle = target_cycle_mr72 if radar_type == "MR72" else target_cycle_u10
                        radars[key] = RadarStats(r_id, radar_type, base_can_id, t_cycle)
                    
                    if radar_type == "MR72":
                        # 1. 0x60A + ID * 0x10 -> Object List Status
                        if (can_id & 0xFF0F) == MR72_OBJECT_LIST_STATUS:
                            if len(data) >= 1:
                                nof_objects = data[0]
                                
                                # Check cycle time deviation warning
                                r = radars[key]
                                if r.last_header_time is not None:
                                    cycle = ts_ms - r.last_header_time
                                    if abs(cycle - r.target_cycle) / r.target_cycle > 0.20:
                                        warnings.append(
                                            f"[WARNING] Radar {r.radar_type} ID {r_id} Frame #{r.frame_number}: "
                                            f"Abnormal cycle time: {cycle:.2f} ms (expected {r.target_cycle:.1f} ms)"
                                        )
                                
                                radars[key].start_new_frame(ts_ms, nof_objects, warnings)
                                
                        # 2. 0x60B + ID * 0x10 -> Object General Info
                        elif (can_id & 0xFF0F) == MR72_OBJECT_GENERAL_INFO:
                            if len(data) >= 1:
                                obj_id = data[0]
                                radars[key].add_object(obj_id)
                                
                    elif radar_type == "U10":
                        if key not in u10_parsers:
                            u10_parsers[key] = U10Parser(r_id, radars[key], warnings)
                        # Slice data to actual received length
                        actual_data = data[:can_dlc]
                        u10_parsers[key].feed(actual_data, ts_ms)
                            
                except BlockingIOError:
                    pass
                except OSError as e:
                    warnings.append(f"[CRITICAL] OS Error reading socket: {e}")
                    
        except KeyboardInterrupt:
            print("\nMonitoring stopped by user.")
        finally:
            s.close()

if __name__ == "__main__":
    main()
