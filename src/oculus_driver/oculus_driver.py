#!/usr/bin/env python3
"""
Oculus M750D Python Driver
Implements the Oculus SDK communication protocol from Oculus.h (SDK v1.15.168)

Protocol:
  - TCP port: 52100
  - Send: OculusSimpleFireMessage2   (fire command)
  - Receive: OculusSimplePingResult2  (ping data + image)
  - Image layout: [nRanges rows × nBeams cols], uint8 or uint16 intensity
"""

import socket
import struct
import numpy as np
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable

# ────────────────────────────────────────────────────
# Protocol constants (from Oculus.h)
# ────────────────────────────────────────────────────
OCULUS_CHECK_ID       = 0x4f53
MSG_SIMPLE_FIRE       = 0x15
MSG_SIMPLE_PING_RESULT= 0x23
MSG_DUMMY             = 0xff
DEFAULT_PORT          = 52100

DATA_SIZE_8BIT  = 0
DATA_SIZE_16BIT = 1

PING_RATE_NORMAL  = 0x00   # ~10 Hz
PING_RATE_HIGH    = 0x01   # ~15 Hz
PING_RATE_HIGHEST = 0x02   # ~40 Hz


# ────────────────────────────────────────────────────
# OculusMessageHeader  (14 bytes, packed)
# ────────────────────────────────────────────────────
HEADER_FMT  = '<HHHHHIH'   # little-endian
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 16 bytes

# OculusSimpleFireMessage2 body (after header)
# masterMode(B) pingRate(B) netSpeed(B) gamma(B) flags(B) range(d)
# gainPercent(d) sos(d) salinity(d) extFlags(I) reserved0[2](II)
# beaconFreq(I) reserved1[5](IIIII)
FIRE_BODY_FMT = '<BBBBBddddIIIIIIIII'
FIRE_BODY_SIZE = struct.calcsize(FIRE_BODY_FMT)

# OculusSimplePingResult2 body (after header + fire message)
# pingId(I) status(I) freq(d) temp(d) pressure(d)
# heading(d) pitch(d) roll(d) sos_used(d) pingStartTime(d)
# dataSize(B) rangeRes(d) nRanges(H) nBeams(H)
# spare0-3(IIII) imageOffset(I) imageSize(I) messageSize(I)
PING_RESULT_FMT  = '<IIddddddddbdHHIIIIIII'
PING_RESULT_SIZE = struct.calcsize(PING_RESULT_FMT)


@dataclass
class SonarPing:
    """One complete ping frame from the sonar"""
    ping_id:         int
    timestamp:       float    # seconds since sonar powerup
    frequency:       float    # Hz (750k or 1.2M)
    temperature:     float    # °C
    pressure:        float    # bar
    heading:         float    # degrees
    pitch:           float
    roll:            float
    speed_of_sound:  float    # m/s (actual used)
    range_resolution: float   # m per range bin
    n_ranges:        int
    n_beams:         int
    bearings_deg:    np.ndarray   # shape (n_beams,), in degrees
    image:           np.ndarray   # shape (n_ranges, n_beams), float32 [0..1]

    def to_pointcloud(self, intensity_threshold: float = 0.05):
        """
        Convert polar image to Cartesian point cloud.

        Returns:
            points: np.ndarray  shape (N, 2)  columns = [x, y]  in metres
            intensity: np.ndarray  shape (N,)  normalized [0..1]
            times: np.ndarray  shape (N,)  approximate timestamp per point
        """
        brg_rad = np.deg2rad(self.bearings_deg)               # (n_beams,)
        ranges  = np.arange(self.n_ranges) * self.range_resolution  # (n_ranges,)

        # Meshgrid: rows=range, cols=bearing
        R, B = np.meshgrid(ranges, brg_rad, indexing='ij')    # (n_ranges, n_beams)

        x = R * np.sin(B)   # port (-) / starboard (+)
        y = R * np.cos(B)   # forward

        mask = self.image > intensity_threshold

        pts   = np.column_stack([x[mask], y[mask]])
        intens = self.image[mask]

        # Approx per-point time (linear across ping duration ~10ms)
        t = np.full(pts.shape[0], self.timestamp)

        return pts, intens, t


class OculusDriver:
    """
    Manages TCP connection to the Oculus sonar.
    Usage:
        driver = OculusDriver('192.168.2.6')
        driver.start(callback=my_fn)   # my_fn(ping: SonarPing)
        ...
        driver.stop()
    """

    def __init__(self, host: str, port: int = DEFAULT_PORT,
                 mode: int = 1, range_m: float = 10.0,
                 gain: float = 40.0, speed_of_sound: float = 1500.0,
                 salinity: float = 0.0):
        self.host    = host
        self.port    = port
        self.mode    = mode          # 1=750kHz  2=1.2MHz
        self.range_m = range_m
        self.gain    = gain
        self.sos     = speed_of_sound
        self.salinity = salinity

        self._sock:   Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._callback: Optional[Callable] = None
        self._rx_buf  = bytearray()

    def start(self, callback: Callable[[SonarPing], None]):
        self._callback = callback
        self._running  = True
        self._thread   = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._sock:
            self._sock.close()

    def _build_fire_message(self) -> bytes:
        # Header
        body_size = FIRE_BODY_SIZE
        header = struct.pack(HEADER_FMT,
            OCULUS_CHECK_ID,  # oculusId
            0,                # srcDeviceId
            0,                # dstDeviceId
            MSG_SIMPLE_FIRE,  # msgId
            2,                # msgVersion
            body_size,        # payloadSize
            0                 # partNumber
        )
        # Body: OculusSimpleFireMessage2
        body = struct.pack(FIRE_BODY_FMT,
            self.mode,           # masterMode
            PING_RATE_NORMAL,    # pingRate
            0,                   # networkSpeed (0=unrestricted)
            127,                 # gammaCorrection
            0x19,               # flags (gain assist off)
            self.range_m,
            self.gain,
            self.sos,
            self.salinity,
            0, 0, 0, 0,        # extFlags, reserved0[2], beaconFreq
            0, 0, 0, 0, 0       # reserved1[5]
        )
        return header + body

    def _run(self):
        while self._running:
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._sock.settimeout(5.0)
                self._sock.connect((self.host, self.port))
                print(f"[OculusDriver] Connected to {self.host}:{self.port}")

                # Send initial fire command
                self._sock.sendall(self._build_fire_message())
                self._rx_buf.clear()

                while self._running:
                    chunk = self._sock.recv(65536)
                    if not chunk:
                        break
                    self._rx_buf.extend(chunk)
                    self._process_buffer()

            except Exception as e:
                print(f"[OculusDriver] Error: {e}, reconnecting in 2s…")
                time.sleep(2)
            finally:
                if self._sock:
                    self._sock.close()

    def _process_buffer(self):
        while len(self._rx_buf) >= HEADER_SIZE:
            # Find OCULUS_CHECK_ID sync word 0x4f53
            sync_pos = -1
            for i in range(len(self._rx_buf) - 1):
                if self._rx_buf[i] == 0x53 and self._rx_buf[i+1] == 0x4f:
                    sync_pos = i
                    break

            if sync_pos < 0:
                self._rx_buf.clear()
                return
            if sync_pos > 0:
                del self._rx_buf[:sync_pos]

            if len(self._rx_buf) < HEADER_SIZE:
                return

            hdr = struct.unpack_from(HEADER_FMT, self._rx_buf, 0)
            oculus_id, src, dst, msg_id, msg_ver, payload_size, part_num = hdr

            total_size = HEADER_SIZE + payload_size
            if len(self._rx_buf) < total_size:
                return   # wait for more data

            pkt = bytes(self._rx_buf[:total_size])
            del self._rx_buf[:total_size]

            if msg_id == MSG_SIMPLE_PING_RESULT:
                ping = self._parse_ping(pkt)
                if ping and self._callback:
                    self._callback(ping)

    def _parse_ping(self, pkt: bytes) -> Optional[SonarPing]:
        try:
            offset = HEADER_SIZE + FIRE_BODY_SIZE
            fields = struct.unpack_from(PING_RESULT_FMT, pkt, offset)

            (ping_id, status, freq, temp, pressure,
             heading, pitch, roll, sos_used, ping_start_time,
             data_size, range_res, n_ranges, n_beams,
             sp0, sp1, sp2, sp3,
             image_offset, image_size, msg_size) = fields

            # Bearings array: n_beams × int16, right after the result header
            brg_offset = offset + PING_RESULT_SIZE
            brg_raw = struct.unpack_from(f'<{n_beams}h', pkt, brg_offset)
            bearings_deg = np.array(brg_raw, dtype=np.float32) * 0.01

            # Image data
            img_start = image_offset   # relative to start of message
            img_end   = img_start + image_size

            if data_size == DATA_SIZE_8BIT:
                raw = np.frombuffer(pkt[img_start:img_end], dtype=np.uint8)
                image = raw.reshape((n_ranges, n_beams)).astype(np.float32) / 255.0
            else:
                raw = np.frombuffer(pkt[img_start:img_end], dtype=np.uint16)
                image = raw.reshape((n_ranges, n_beams)).astype(np.float32) / 65535.0

            return SonarPing(
                ping_id=ping_id,
                timestamp=ping_start_time,
                frequency=freq,
                temperature=temp,
                pressure=pressure,
                heading=heading,
                pitch=pitch,
                roll=roll,
                speed_of_sound=sos_used,
                range_resolution=range_res,
                n_ranges=n_ranges,
                n_beams=n_beams,
                bearings_deg=bearings_deg,
                image=image,
            )
        except Exception as e:
            print(f"[OculusDriver] Parse error: {e}")
            return None


# ──────────────────────────────────────────────────────
# Quick standalone test (no ROS needed)
# Run:  python3 oculus_driver.py
# ──────────────────────────────────────────────────────
if __name__ == '__main__':
    import matplotlib.pyplot as plt

    received = []

    def on_ping(ping: SonarPing):
        received.append(ping)
        pts, intens, _ = ping.to_pointcloud(intensity_threshold=0.1)
        print(f"Ping {ping.ping_id}: {ping.n_beams} beams × {ping.n_ranges} ranges "
              f"| {len(pts)} points | T={ping.temperature:.1f}°C")
        if len(received) == 5:
            plt.figure(figsize=(10, 10))
            plt.scatter(pts[:, 0], pts[:, 1],
                        c=intens, cmap='hot', s=1, alpha=0.7)
            plt.axis('equal')
            plt.xlabel('X (m)'); plt.ylabel('Y (m)')
            plt.title('Oculus M750D Point Cloud')
            plt.colorbar(label='Intensity')
            plt.savefig('/tmp/sonar_pointcloud.png', dpi=150)
            print("Saved /tmp/sonar_pointcloud.png")

    driver = OculusDriver(
        host='192.168.2.6',
        mode=1,          # 1 = low frequency (750kHz), better range
        range_m=10.0,   # 10 metre range
        gain=50.0,       # 40% gain
        speed_of_sound=1500.0,
        salinity=0.0    # 0 = fresh water, 35 = seawater
    )
    driver.start(on_ping)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        driver.stop()
        print("Stopped.")
#Test the driver (no ROS needed)
#python3 oculus_driver.py
# You should see:
# [OculusDriver] Connected to 192.168.2.6:52100
# Ping 1: 256 beams × 500 ranges | 3421 points | T=22.3°C
# Ping 2: ...
# After 5 pings: saves /tmp/sonar_pointcloud.png
