#!/usr/bin/env python3
import socket
import struct
import time
import sys

# MAVLink X25 CRC calculation
def crc_accumulate_buffer(buf, crc=0xFFFF):
    for b in buf:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc

def pack_global_position_int(seq, relative_alt_mm):
    # Message ID 33 (GLOBAL_POSITION_INT), CRC_EXTRA = 104
    msg_id = 33
    crc_extra = 104
    
    # Payload format:
    # time_boot_ms (I), lat (i), lon (i), alt (i), relative_alt (i), vx (h), vy (h), vz (h), hdg (H)
    time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF
    lat = 210000000
    lon = 105000000
    alt = relative_alt_mm + 10000  # MSL
    vx = 0
    vy = 0
    vz = 0
    hdg = 0  # degrees * 100 (0 means heading is north)
    
    payload = struct.pack("<IiiiihhhH", 
                          time_boot_ms, 
                          lat, 
                          lon, 
                          alt, 
                          relative_alt_mm, 
                          vx, 
                          vy, 
                          vz, 
                          hdg)
    
    # Header format:
    # Magic (0xFE), Length (28), Seq, SysID (1), CompID (1), MsgID (33)
    header = struct.pack("<BBBBBB", 0xFE, len(payload), seq, 1, 1, msg_id)
    
    # CRC includes header (excluding Magic 0xFE) + payload + CRC_EXTRA
    crc_data = header[1:] + payload + bytes([crc_extra])
    crc = crc_accumulate_buffer(crc_data)
    
    packet = header + payload + struct.pack("<H", crc)
    return packet

def main():
    listen_port = 14551
    if len(sys.argv) > 1:
        try:
            listen_port = int(sys.argv[1])
        except ValueError:
            print(f"Invalid port. Using default: {listen_port}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", listen_port))
    print(f"[*] Mock MAVLink Router listening on UDP 127.0.0.1:{listen_port}")
    print("[*] Waiting for dummy packet ('X') from radar_app...")

    client_addr = None
    while True:
        data, addr = sock.recvfrom(1024)
        if data == b'X':
            client_addr = addr
            print(f"[+] Received registration dummy packet from radar_app at {client_addr}")
            break
        else:
            print(f"[?] Received unknown packet from {addr}: {data}")

    print("\n[+] Starting flight simulation...")
    seq = 0
    
    try:
        # Phase 1: Ascent (Bay lên: 0m -> 15m)
        print("\n--- PHASE 1: ASCENDING (0m -> 15m) ---")
        for alt_cm in range(0, 1500, 50): # 0cm đến 1500cm (15m), mỗi bước 50cm
            alt_mm = alt_cm * 10
            packet = pack_global_position_int(seq, alt_mm)
            sock.sendto(packet, client_addr)
            print(f"Sent: Altitude = {alt_cm} cm (Target Min = 1001 cm)")
            seq = (seq + 1) % 256
            time.sleep(0.2) # 5 Hz

        # Phase 2: Hover (Bay lơ lửng: 15m trong 5 giây)
        print("\n--- PHASE 2: HOVERING AT 15m ---")
        for _ in range(25):
            packet = pack_global_position_int(seq, 15000)
            sock.sendto(packet, client_addr)
            print("Sent: Altitude = 1500 cm (Hovering)")
            seq = (seq + 1) % 256
            time.sleep(0.2)

        # Phase 3: Descent (Bay xuống: 15m -> 2m)
        print("\n--- PHASE 3: DESCENDING (15m -> 2m) ---")
        for alt_cm in range(1500, 200, -50):
            alt_mm = alt_cm * 10
            packet = pack_global_position_int(seq, alt_mm)
            sock.sendto(packet, client_addr)
            print(f"Sent: Altitude = {alt_cm} cm (Target Disconnect = 500 cm)")
            seq = (seq + 1) % 256
            time.sleep(0.2)

        # Phase 4: Hover (Lơ lửng ở độ cao thấp trong 5 giây)
        print("\n--- PHASE 4: HOVERING AT 2m ---")
        for _ in range(25):
            packet = pack_global_position_int(seq, 2000)
            sock.sendto(packet, client_addr)
            print("Sent: Altitude = 200 cm (Low altitude)")
            seq = (seq + 1) % 256
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        sock.close()
        print("[*] Socket closed. Exiting.")

if __name__ == "__main__":
    main()
