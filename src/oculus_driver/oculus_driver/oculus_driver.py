#!/usr/bin/env python3
"""
Oculus M750D Python Driver
Implements the Oculus SDK communication protocol from Oculus.h (SDK v1.15.168)

Protocol:
  - TCP port: 52100
  - Send: OculusSimpleFireMessage2   (fire command)
  - Receive: OculusSimplePingResult2  (ping data + image)
  - Image layout: [nRanges rows x nBeams cols], uint8 or uint16 intensity

New in this version:
  - set_range(m)      → live range change, re-fires immediately
  - set_gain(%)       → live gain change
  - set_ping_rate(code) → live frequency change (use PING_RATES dict)
"""

import socket
import struct
import numpy as np
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable

# ── Protocol constants (from Oculus.h) ───────────────────────────────────────
OCULUS_CHECK_ID        = 0x4f53
MSG_SIMPLE_FIRE        = 0x15
MSG_SIMPLE_PING_RESULT = 0x23
MSG_DUMMY              = 0xff
DEFAULT_PORT           = 52100

DATA_SIZE_8BIT  = 0
DATA_SIZE_16BIT = 1

# Ping rate codes from Oculus.h PingRateType enum
PING_RATE_NORMAL  = 0x00   # ~10 Hz
PING_RATE_HIGH    = 0x01   # ~15 Hz
PING_RATE_HIGHEST = 0x02   # ~40 Hz
PING_RATE_LOW     = 0x03   # ~5 Hz
PING_RATE_LOWEST  = 0x04   # ~2 Hz
PING_RATE_STANDBY = 0x05   # 0 Hz (pause)

# ── Struct formats (little-endian, packed) ────────────────────────────────────
# OculusMessageHeader: oculusId srcDeviceId dstDeviceId msgId msgVersion payloadSize partNumber
HEADER_FMT  = '<HHHHHIH'
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 16 bytes

# OculusSimpleFireMessage2 body (after header)
# masterMode pingRate netSpeed gamma flags range gain sos salinity
# extFlags reserved0[2] beaconFreq reserved1[5]
FIRE_BODY_FMT  = '<BBBBBddddIIIIIIIII'
FIRE_BODY_SIZE = struct.calcsize(FIRE_BODY_FMT)

# OculusSimplePingResult2 body (after header + fire message body)
# pingId status freq temp pressure heading pitch roll sosUsed pingStartTime
# dataSize rangeRes nRanges nBeams spare0 spare1 spare2 spare3
# imageOffset imageSize messageSize
PING_RESULT_FMT  = '<IIddddddddbdHHIIIIIII'
PING_RESULT_SIZE = struct.calcsize(PING_RESULT_FMT)


@dataclass
class SonarPing:
    """One complete ping frame from the sonar."""
    ping_id:          int
    timestamp:        float     # seconds since sonar powerup
    frequency:        float     # Hz (750k or 1.2M)
    temperature:      float     # °C
    pressure:         float     # bar
    heading:          float     # degrees
    pitch:            float
    roll:             float
    speed_of_sound:   float     # m/s (actual used)
    range_resolution: float     # m per range bin
    n_ranges:         int
    n_beams:          int
    bearings_deg:     np.ndarray   # shape (n_beams,)
    image:            np.ndarray   # shape (n_ranges, n_beams), float32 [0..1]

    def to_pointcloud(self, intensity_threshold: float = 0.05):
        """Convert polar image to Cartesian (x, y) point cloud."""
        brg_rad = np.deg2rad(self.bearings_deg)
        ranges  = np.arange(self.n_ranges) * self.range_resolution

        R, B = np.meshgrid(ranges, brg_rad, indexing='ij')

        x = R * np.sin(B)
        y = R * np.cos(B)

        mask   = self.image > intensity_threshold
        pts    = np.column_stack([x[mask], y[mask]])
        intens = self.image[mask]
        t      = np.full(pts.shape[0], self.timestamp)
        return pts, intens, t


class OculusDriver:
    """
    Manages TCP connection to the Oculus sonar.

    Usage:
        driver = OculusDriver('192.168.2.6', range_m=5.0, gain=50.0)
        driver.start(callback=my_fn)   # my_fn(ping: SonarPing)
        driver.set_range(2.0)          # live update
        driver.set_gain(60.0)          # live update
        driver.set_ping_rate(PING_RATE_HIGH)  # live update
        driver.stop()
    """

    def __init__(self,
                 host:           str,
                 port:           int   = DEFAULT_PORT,
                 mode:           int   = 1,
                 range_m:        float = 10.0,
                 gain:           float = 40.0,
                 speed_of_sound: float = 1500.0,
                 salinity:       float = 0.0,
                 ping_rate:      int   = PING_RATE_NORMAL):

        self.host     = host
        self.port     = port
        self._lock    = threading.Lock()   # protects fire params

        # Fire parameters (protected by _lock for live updates)
        self._mode    = mode
        self._range_m = range_m
        self._gain    = gain
        self._sos     = speed_of_sound
        self._salinity = salinity
        self._ping_rate = ping_rate

        self._sock:    Optional[socket.socket]  = None
        self._thread:  Optional[threading.Thread] = None
        self._running  = False
        self._callback: Optional[Callable]      = None
        self._rx_buf   = bytearray()

        # Flag: fire params changed → send new fire message on next cycle
        self._needs_refire = False

    # ── Live parameter setters ────────────────────────────────────────────
    def set_mode(self, mode: int):
        with self._lock:
            self._mode         = mode
            self._needs_refire = True
    def set_range(self, range_m: float):
        with self._lock:
            self._range_m     = float(range_m)
            self._needs_refire = True

    def set_gain(self, gain: float):
        with self._lock:
            self._gain        = float(gain)
            self._needs_refire = True

    def set_ping_rate(self, ping_rate_code: int):
        with self._lock:
            self._ping_rate   = ping_rate_code
            self._needs_refire = True

    def set_speed_of_sound(self, sos: float):
        with self._lock:
            self._sos         = float(sos)
            self._needs_refire = True

    # ── Public API ───────────────────────────────────────────────────────
    def start(self, callback: Callable[[SonarPing], None]):
        self._callback = callback
        self._running  = True
        self._thread   = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

    # ── Build fire message from current params ────────────────────────────
    def _build_fire_message(self) -> bytes:
        with self._lock:
            mode      = self._mode
            ping_rate = self._ping_rate
            range_m   = self._range_m
            gain      = self._gain
            sos       = self._sos
            salinity  = self._salinity
            self._needs_refire = False

        header = struct.pack(HEADER_FMT,
            OCULUS_CHECK_ID,   # oculusId  (0x4f53)
            0,                 # srcDeviceId
            0,                 # dstDeviceId
            MSG_SIMPLE_FIRE,   # msgId     (0x15)
            2,                 # msgVersion
            FIRE_BODY_SIZE,    # payloadSize
            0                  # partNumber
        )
        body = struct.pack(FIRE_BODY_FMT,
            mode,              # masterMode
            ping_rate,         # pingRate
            0,                 # networkSpeed (0 = unrestricted)
            127,               # gammaCorrection
            0x19,              # flags (gain assist off)
            range_m,           # range (metres)
            gain,              # gainPercent
            sos,               # speedOfSound
            salinity,          # salinity
            0, 0, 0,           # extFlags, reserved0[2]
            0,                 # beaconLocatorFrequency
            0, 0, 0, 0, 0      # reserved1[5]
        )
        return header + body

    # ── Main connection loop ──────────────────────────────────────────────
    def _run(self):
        while self._running:
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._sock.settimeout(5.0)
                self._sock.connect((self.host, self.port))
                self._sock.settimeout(2.0)   # shorter timeout for recv loop
                print(f"[OculusDriver] Connected to {self.host}:{self.port}")

                # Send initial fire command
                self._sock.sendall(self._build_fire_message())
                self._rx_buf.clear()

                while self._running:
                    # ── Re-fire if params changed ──
                    if self._needs_refire:
                        self._sock.sendall(self._build_fire_message())

                    try:
                        chunk = self._sock.recv(65536)
                    except socket.timeout:
                        continue   # no data yet, check needs_refire again

                    if not chunk:
                        break
                    self._rx_buf.extend(chunk)
                    self._process_buffer()

            except Exception as e:
                print(f"[OculusDriver] Error: {e}, reconnecting in 2s…")
                time.sleep(2)
            finally:
                if self._sock:
                    try:
                        self._sock.close()
                    except Exception:
                        pass

    # ── Parse incoming buffer ─────────────────────────────────────────────
    def _process_buffer(self):
        while len(self._rx_buf) >= HEADER_SIZE:
            # Find sync word 0x4f53 (little-endian: 0x53, 0x4f)
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
            _, _, _, msg_id, _, payload_size, _ = hdr

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
            # Skip: header (HEADER_SIZE) + fire message body (FIRE_BODY_SIZE)
            offset = HEADER_SIZE + FIRE_BODY_SIZE

            fields = struct.unpack_from(PING_RESULT_FMT, pkt, offset)
            (ping_id, status, freq, temp, pressure,
             heading, pitch, roll, sos_used, ping_start_time,
             data_size, range_res, n_ranges, n_beams,
             sp0, sp1, sp2, sp3,
             image_offset, image_size, msg_size) = fields

            # Bearings array: n_beams × int16, immediately after the result struct
            brg_offset = offset + PING_RESULT_SIZE
            brg_raw    = struct.unpack_from(f'<{n_beams}h', pkt, brg_offset)
            bearings_deg = np.array(brg_raw, dtype=np.float32) * 0.01

            # Image data — image_offset is from start of the whole packet
            img_start = image_offset
            img_end   = img_start + image_size

            if data_size == DATA_SIZE_8BIT:
                raw   = np.frombuffer(pkt[img_start:img_end], dtype=np.uint8)
                image = raw.reshape((n_ranges, n_beams)).astype(np.float32) / 255.0
            else:
                raw   = np.frombuffer(pkt[img_start:img_end], dtype=np.uint16)
                image = raw.reshape((n_ranges, n_beams)).astype(np.float32) / 65535.0

            return SonarPing(
                ping_id          = ping_id,
                timestamp        = ping_start_time,
                frequency        = freq,
                temperature      = temp,
                pressure         = pressure,
                heading          = heading,
                pitch            = pitch,
                roll             = roll,
                speed_of_sound   = sos_used,
                range_resolution = range_res,
                n_ranges         = n_ranges,
                n_beams          = n_beams,
                bearings_deg     = bearings_deg,
                image            = image,
            )
        except Exception as e:
            print(f"[OculusDriver] Parse error: {e}")
            return None


# ── Standalone test (no ROS) ──────────────────────────────────────────────────
if __name__ == '__main__':
    import matplotlib.pyplot as plt

    received = []

    def on_ping(ping: SonarPing):
        received.append(ping)
        img = ping.image
        print(f"Ping {ping.ping_id:4d}: "
              f"n_ranges={ping.n_ranges}  n_beams={ping.n_beams}  "
              f"range_res={ping.range_resolution:.4f}m  "
              f"max_range={ping.n_ranges*ping.range_resolution:.2f}m")
        print(f"  Intensity: min={img.min():.4f}  "
              f"max={img.max():.4f}  mean={img.mean():.4f}")
        print(f"  Pixels > 0.05: {(img>0.05).sum()}")

        if len(received) == 10:
            pts, intens, _ = ping.to_pointcloud(intensity_threshold=0.02)
            plt.figure(figsize=(10, 10))
            plt.scatter(pts[:, 0], pts[:, 1],
                        c=intens, cmap='hot', s=1, alpha=0.7)
            plt.axis('equal')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title(f'Oculus M750D — range {ping.n_ranges*ping.range_resolution:.1f}m')
            plt.colorbar(label='Intensity')
            plt.savefig('/tmp/sonar_pointcloud.png', dpi=150)
            print("Saved /tmp/sonar_pointcloud.png")

    driver = OculusDriver(
        host           = '192.168.2.6',
        mode           = 1,
        range_m        = 1.0,
        gain           = 50.0,
        speed_of_sound = 1500.0,
        salinity       = 0.0,
        ping_rate      = PING_RATE_NORMAL,
    )
    driver.start(on_ping)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        driver.stop()
        print("Stopped.")
