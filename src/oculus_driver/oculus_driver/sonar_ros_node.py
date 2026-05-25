#!/usr/bin/env python3
"""
Oculus M750d ROS2 node — ViewPoint-identical fan image.
- Full wide cone (130° FOV rendered correctly)
- Range rings with distance labels (0.2m, 0.4m, 0.6m, 0.8m, 1.0m...)
- Bearing lines at -60°, -30°, 0°, +30°, +60°
- Proper wide aspect ratio canvas (wider than tall like ViewPoint)
- Live range/gain/ping_rate control
- Lag-free threaded render
"""
import rclpy
from rclpy.node import Node
import numpy as np
import threading
import queue

from sensor_msgs.msg import PointCloud2, PointField, LaserScan, Image
from std_msgs.msg import Header
from oculus_driver.oculus_driver import OculusDriver, SonarPing

# ── Oculus ViewPoint palette (warm gold) ─────────────────────────────────────
_PAL_RAW = [
    (2,1,0),(4,2,0),(6,4,0),(8,5,0),(10,6,0),(12,7,0),(14,9,0),(15,10,0),(17,11,0),(19,12,0),(21,14,0),(23,15,0),(25,16,0),(27,17,0),(29,18,0),(31,20,0),
    (33,21,0),(35,22,0),(37,23,0),(39,25,0),(41,26,0),(43,27,0),(45,28,0),(46,29,0),(48,31,0),(50,32,0),(52,33,0),(54,34,0),(56,36,0),(58,37,0),(60,38,0),(62,39,0),
    (64,41,0),(66,42,0),(68,43,0),(70,44,0),(72,45,0),(74,47,0),(76,48,0),(77,49,0),(79,50,0),(81,52,0),(83,53,0),(85,54,0),(87,55,0),(89,57,0),(91,58,0),(93,59,0),
    (95,60,0),(97,61,0),(99,63,0),(101,64,0),(103,65,0),(105,66,0),(107,68,0),(108,69,0),(110,70,0),(112,71,0),(114,72,0),(116,74,0),(118,75,0),(120,76,0),(122,77,0),(124,79,0),
    (126,80,0),(128,81,0),(130,82,0),(131,83,0),(133,85,0),(135,86,0),(137,87,0),(139,88,0),(141,89,0),(143,91,0),(145,92,0),(146,93,0),(148,94,0),(150,95,0),(152,97,0),(154,98,0),
    (156,99,0),(158,100,0),(160,101,0),(161,103,0),(163,104,0),(165,105,0),(167,106,0),(169,107,0),(171,109,0),(173,110,0),(175,111,0),(176,112,0),(178,113,0),(180,115,0),(182,116,0),(184,117,0),
    (186,118,0),(188,119,0),(190,120,0),(191,122,0),(193,123,0),(195,124,0),(197,125,0),(199,126,0),(201,128,0),(203,129,0),(205,130,0),(206,131,0),(208,132,0),(210,134,0),(212,135,0),(214,136,0),
    (216,137,0),(218,138,0),(220,140,0),(221,141,0),(223,142,0),(225,143,0),(227,144,0),(229,146,0),(231,147,0),(233,148,0),(235,149,0),(236,150,0),(238,152,0),(240,153,0),(242,154,0),(242,157,6),
    (242,157,6),(243,159,12),(243,162,19),(244,165,25),(244,167,31),(244,170,37),(245,173,43),(245,176,49),(246,178,56),(246,181,62),(246,184,68),(247,186,74),(247,189,80),(248,192,86),(248,194,93),(248,197,99),
    (249,200,105),(249,203,111),(249,205,117),(250,208,124),(250,211,130),(251,213,136),(251,216,142),(251,219,148),(252,221,154),(252,224,161),(253,227,167),(253,229,173),(253,232,179),(254,235,185),(254,238,192),(255,240,198),
    (255,243,204),(255,243,205),(255,243,205),(255,243,206),(255,244,206),(255,244,207),(255,244,208),(255,244,208),(255,244,209),(255,244,209),(255,244,210),(255,244,210),(255,245,211),(255,245,212),(255,245,212),(255,245,213),
    (255,245,213),(255,245,214),(255,245,215),(255,245,215),(255,246,216),(255,246,216),(255,246,217),(255,246,218),(255,246,218),(255,246,219),(255,246,219),(255,247,220),(255,247,221),(255,247,221),(255,247,222),(255,247,222),
    (255,247,223),(255,247,223),(255,247,224),(255,248,224),(255,248,225),(255,248,225),(255,248,226),(255,248,226),(255,248,227),(255,248,227),(255,248,228),(255,249,228),(255,249,229),(255,249,230),(255,249,230),(255,249,231),
    (255,249,231),(255,249,232),(255,249,232),(255,250,233),(255,250,233),(255,250,234),(255,250,234),(255,250,235),(255,250,235),(255,250,236),(255,250,236),(255,251,237),(255,251,237),(255,251,238),(255,251,238),(255,251,239),
    (255,251,239),(255,251,240),(255,251,240),(255,252,241),(255,252,241),(255,252,242),(255,252,243),(255,252,243),(255,252,244),(255,252,244),(255,252,245),(255,253,245),(255,253,246),(255,253,246),(255,253,247),(255,253,247),
    (255,253,248),(255,253,248),(255,253,249),(255,254,249),(255,254,250),(255,254,250),(255,254,251),(255,254,251),(255,254,252),(255,254,252),(255,254,253),(255,255,253),(255,255,254),(255,255,254),(255,255,255),(255,255,255),
]
PALETTE = np.array(_PAL_RAW, dtype=np.uint8)   # (256, 3)

# Ping rate codes
PING_RATES = {
    'standby': 0x05, 'lowest': 0x04, 'low': 0x03,
    'normal':  0x00, 'high':   0x01, 'highest': 0x02,
}

# Grid overlay colours (RGB)
COL_RING   = (80,  80,  80)   # dark grey arc lines
COL_LABEL  = (200, 200, 200)  # light grey text
COL_BEAM   = (60,  60,  60)   # bearing lines
COL_CENTER = (100, 100, 100)  # centre vertical line


def _draw_thick_arc(canvas, cx, cy, r, brg_min_d, brg_max_d, colour, thickness=1):
    """
    Draw an arc on the canvas (no cv2 needed — pure NumPy rasterization).
    Samples N points along the arc and sets pixels.
    """
    if r < 1:
        return
    H, W = canvas.shape[:2]
    n_pts = max(int(r * abs(brg_max_d - brg_min_d) * np.pi / 180 * 2), 8)
    angles = np.linspace(np.radians(brg_min_d), np.radians(brg_max_d), n_pts)
    xs = (cx + r * np.sin(angles)).astype(np.int32)
    ys = (cy - r * np.cos(angles)).astype(np.int32)
    for t in range(-thickness, thickness + 1):
        valid = (xs + t >= 0) & (xs + t < W) & (ys >= 0) & (ys < H)
        canvas[ys[valid], xs[valid] + t] = colour
        valid2 = (xs >= 0) & (xs < W) & (ys + t >= 0) & (ys + t < H)
        canvas[ys[valid2] + t, xs[valid2]] = colour


def _draw_line(canvas, x0, y0, x1, y1, colour, thickness=1):
    """Bresenham line on canvas."""
    H, W = canvas.shape[:2]
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = int(x0), int(y0)
    while True:
        for tx in range(-thickness, thickness + 1):
            for ty in range(-thickness, thickness + 1):
                nx, ny = x + tx, y + ty
                if 0 <= nx < W and 0 <= ny < H:
                    canvas[ny, nx] = colour
        if x == int(x1) and y == int(y1):
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x   += sx
        if e2 < dx:
            err += dx
            y   += sy


def _put_text_simple(canvas, text, x, y, colour, scale=2):
    """
    Bitmap font, each glyph stored as rows top→bottom, each row is a bitmask
    of 5 columns (bit4=leftmost, bit0=rightmost).
    scale=2 → each pixel becomes a 2×2 block (readable at normal sonar image size).
    """
    # Each entry: 7 rows, each row is 5-bit column mask (MSB = left)
    GLYPHS = {
        '0': [0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110],
        '1': [0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        '2': [0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111],
        '3': [0b11111, 0b00010, 0b00100, 0b00010, 0b00001, 0b10001, 0b01110],
        '4': [0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
        '5': [0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110],
        '6': [0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110],
        '7': [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000],
        '8': [0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110],
        '9': [0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100],
        '.': [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01100, 0b01100],
        'm': [0b00000, 0b00000, 0b11010, 0b10101, 0b10101, 0b10001, 0b10001],
        ' ': [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000],
    }
    H, W = canvas.shape[:2]
    char_w = 5 * scale + scale   # 5 cols + 1 gap
    cx = x
    for ch in text:
        rows = GLYPHS.get(ch, GLYPHS[' '])
        for row_i, row_bits in enumerate(rows):
            py0 = y + row_i * scale
            for col_i in range(5):
                if row_bits & (1 << (4 - col_i)):
                    px0 = cx + col_i * scale
                    for dy in range(scale):
                        for dx in range(scale):
                            px, py = px0 + dx, py0 + dy
                            if 0 <= px < W and 0 <= py < H:
                                canvas[py, px] = colour
        cx += char_w


class FanRenderer:
    """
    Renders the sonar fan exactly like Oculus ViewPoint:
    - Wide aspect ratio (fits full 130° cone)
    - Black background outside cone
    - Range rings with distance labels
    - Bearing lines at cardinal angles
    - Proper min-max contrast stretch per ping
    """

    def __init__(self, img_w: int, img_h: int):
        self.img_w = img_w
        self.img_h = img_h
        self._cfg       = None
        self._ri        = None
        self._bi        = None
        self._flat_mask = None
        self._W         = None
        self._H         = None
        self._scale     = None
        self._ox        = None
        self._oy        = None
        self._brg_min   = None
        self._brg_max   = None
        self._max_range = None

    def _build_lut(self, ping: SonarPing):
        brg_min   = float(ping.bearings_deg[0])
        brg_max   = float(ping.bearings_deg[-1])
        max_range = ping.n_ranges * ping.range_resolution
        fov_deg   = brg_max - brg_min    # typically ~130°

        # ── Canvas sizing ─────────────────────────────────────────────────
        # ViewPoint fills width with the arc, so we compute canvas dimensions
        # such that the widest point of the arc (at max_range) exactly fills
        # the requested img_w.
        #
        # Half-width of arc at max range:
        #   half_w_m = max_range * sin(fov/2)
        # We want that to map to (img_w/2 - margin) pixels
        margin_px = 30   # pixels of black on each side
        half_w_px = self.img_w / 2 - margin_px

        # Scale: pixels per metre
        half_w_m  = max_range * abs(np.sin(np.radians(fov_deg / 2)))
        if half_w_m < 0.001:
            half_w_m = max_range
        scale = half_w_px / half_w_m

        # Canvas width = img_w (we set it exactly)
        # Canvas height: enough to show full arc depth + apex padding
        arc_height_px = max_range * scale  # pixels from apex to arc top
        apex_pad      = int(arc_height_px * 0.06)  # 6% below apex

        W = self.img_w
        H = int(arc_height_px) + apex_pad + 10
        H = max(H, self.img_h)   # at least requested height

        # Apex position: bottom-centre with apex_pad below
        ox = W / 2.0
        oy = float(H - apex_pad)

        # ── LUT build ─────────────────────────────────────────────────────
        cols = np.arange(W, dtype=np.float32)
        rows = np.arange(H, dtype=np.float32)
        C, R = np.meshgrid(cols, rows)

        x_m = (C - ox) / scale
        y_m = (oy - R) / scale   # positive = upward

        range_m = np.sqrt(x_m**2 + y_m**2)
        brg_d   = np.degrees(np.arctan2(x_m, y_m))

        mask = (
            (y_m     >= 0.0)                          &
            (range_m  > ping.range_resolution * 1.5)  &
            (range_m  < max_range * 0.999)             &
            (brg_d    >= brg_min)                      &
            (brg_d    <= brg_max)
        )

        ri = np.clip(
            (range_m / ping.range_resolution).astype(np.int32),
            0, ping.n_ranges - 1)

        brg_span = max(brg_max - brg_min, 1e-6)
        bi = np.clip(
            ((brg_d - brg_min) / brg_span * (ping.n_beams - 1)).astype(np.int32),
            0, ping.n_beams - 1)

        flat_mask       = mask.ravel()
        self._ri        = ri.ravel()[flat_mask]
        self._bi        = bi.ravel()[flat_mask]
        self._flat_mask = flat_mask
        self._W         = W
        self._H         = H
        self._scale     = scale
        self._ox        = ox
        self._oy        = oy
        self._brg_min   = brg_min
        self._brg_max   = brg_max
        self._max_range = max_range
        self._cfg = (ping.n_ranges, ping.n_beams,
                     ping.range_resolution, brg_min, brg_max)

    def render(self, ping: SonarPing) -> np.ndarray:
        cfg = (ping.n_ranges, ping.n_beams,
               ping.range_resolution,
               float(ping.bearings_deg[0]),
               float(ping.bearings_deg[-1]))
        if cfg != self._cfg:
            self._build_lut(ping)

        img = ping.image   # (n_ranges, n_beams) float32

        # ── Contrast stretch: 2nd–99th percentile of non-zero pixels ──────
        nz = img[img > 0.0]
        if nz.size > 10:
            lo = float(np.percentile(nz, 2))
            hi = float(np.percentile(nz, 99))
        else:
            lo, hi = 0.0, 1.0
        if hi > lo + 1e-6:
            stretched = np.clip((img - lo) / (hi - lo), 0.0, 1.0)
        else:
            stretched = np.zeros_like(img)

        # ── Paint fan pixels ──────────────────────────────────────────────
        raw          = stretched[self._ri, self._bi]
        intensity_u8 = (raw * 255.0).astype(np.uint8)
        canvas       = np.zeros((self._H * self._W, 3), dtype=np.uint8)
        canvas[self._flat_mask] = PALETTE[intensity_u8]
        canvas = canvas.reshape(self._H, self._W, 3)

        # ── Overlay: range rings + labels ─────────────────────────────────
        max_r   = self._max_range
        scale   = self._scale
        ox, oy  = self._ox, self._oy
        bmin    = self._brg_min
        bmax    = self._brg_max

        # Choose ring spacing: aim for ~5 rings regardless of range
        raw_step  = max_r / 5.0
        magnitude = 10 ** np.floor(np.log10(max(raw_step, 1e-9)))
        nice      = [1, 2, 5, 10]
        step = magnitude * min(nice, key=lambda n: abs(n * magnitude - raw_step))

        # Pre-compute label string formatter
        def fmt_label(v):
            if step >= 1.0:
                return f"{v:.0f}m"
            elif step >= 0.1:
                return f"{v:.1f}m"
            else:
                return f"{v:.2f}m"

        # Each label is ~(chars * 12 + 4) px wide at scale=2
        char_w = 12   # pixels per character at scale=2

        # Left edge bearing (most negative angle = left side of cone)
        left_rad  = np.radians(bmin)   # exactly on left edge
        sin_left  = np.sin(left_rad)
        cos_left  = np.cos(left_rad)

        # Right edge bearing
        right_rad = np.radians(bmax)   # exactly on right edge
        sin_right = np.sin(right_rad)
        cos_right = np.cos(right_rad)

        r = step
        while r <= max_r * 1.001:
            r_px = r * scale

            # Draw arc ring
            _draw_thick_arc(canvas, int(ox), int(oy), r_px,
                            bmin, bmax, COL_RING, thickness=1)

            label = fmt_label(r)
            label_w = len(label) * char_w

            # ── LEFT label: just outside the left edge, vertically centred on arc ──
            # Point on left edge of arc
            lx_edge = ox + r_px * sin_left
            ly_edge = oy - r_px * cos_left
            # Place label to the LEFT of this point (offset by label width + gap)
            lx_left = int(lx_edge) - label_w - 4
            ly_left = int(ly_edge) - 7   # vertically centre on arc (7 = half font height)
            _put_text_simple(canvas, label, lx_left, ly_left, COL_LABEL, scale=2)

            # ── RIGHT label: just outside the right edge ──
            rx_edge = ox + r_px * sin_right
            ry_edge = oy - r_px * cos_right
            lx_right = int(rx_edge) + 4   # 4px gap to the right
            ly_right = int(ry_edge) - 7
            _put_text_simple(canvas, label, lx_right, ly_right, COL_LABEL, scale=2)

            r = round(r + step, 10)

        # ── Overlay: bearing lines ────────────────────────────────────────
        # Centre line
        _draw_line(canvas, int(ox), int(oy),
                   int(ox), max(0, int(oy - max_r * scale)),
                   COL_CENTER, thickness=1)

        # Lines at every 30° within FOV
        for angle_d in np.arange(-90, 91, 30):
            if bmin <= angle_d <= bmax:
                rad = np.radians(angle_d)
                ex  = int(ox + max_r * scale * np.sin(rad))
                ey  = int(oy - max_r * scale * np.cos(rad))
                _draw_line(canvas, int(ox), int(oy), ex, ey,
                           COL_BEAM, thickness=1)

        return canvas


# ─────────────────────────────────────────────────────────────────────────────
class SonarNode(Node):

    def __init__(self):
        super().__init__('oculus_sonar_node')

        self.declare_parameter('sonar_ip',           '192.168.2.6')
        self.declare_parameter('sonar_mode',          1)
        self.declare_parameter('range_m',             1.0)
        self.declare_parameter('gain',                50.0)
        self.declare_parameter('speed_of_sound',      1500.0)
        self.declare_parameter('salinity',            0.0)
        self.declare_parameter('ping_rate',          'normal')
        self.declare_parameter('intensity_threshold', 0.01)
        self.declare_parameter('frame_id',           'sonar_link')
        # Wide canvas: ViewPoint is much wider than tall
        self.declare_parameter('fan_width',           1100)
        self.declare_parameter('fan_height',          680)

        ip          = self.get_parameter('sonar_ip').value
        self._thr   = float(self.get_parameter('intensity_threshold').value)
        self._frame = self.get_parameter('frame_id').value
        fw          = int(self.get_parameter('fan_width').value)
        fh          = int(self.get_parameter('fan_height').value)
        pr_str      = self.get_parameter('ping_rate').value
        pr_code     = PING_RATES.get(pr_str, PING_RATES['normal'])

        self._renderer = FanRenderer(fw, fh)

        self._pub_fan = self.create_publisher(Image,       '/sonar/image_fan',  5)
        self._pub_pc  = self.create_publisher(PointCloud2, '/sonar/pointcloud', 10)
        self._pub_ls  = self.create_publisher(LaserScan,   '/sonar/scan',       10)

        self._q = queue.Queue(maxsize=1)
        self._render_thread = threading.Thread(
            target=self._render_loop, daemon=True)
        self._render_thread.start()

        self._driver = OculusDriver(
            host           = ip,
            mode           = int(self.get_parameter('sonar_mode').value),
            range_m        = float(self.get_parameter('range_m').value),
            gain           = float(self.get_parameter('gain').value),
            speed_of_sound = float(self.get_parameter('speed_of_sound').value),
            salinity       = float(self.get_parameter('salinity').value),
            ping_rate      = pr_code,
        )
        self._driver.start(self._on_ping)
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f"Sonar node → {ip}  range={self.get_parameter('range_m').value}m  "
            f"gain={self.get_parameter('gain').value}%  ping_rate={pr_str}  "
            f"canvas={fw}×{fh}")

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'range_m':
                self._driver.set_range(float(p.value))
                self.get_logger().info(f"Range → {p.value} m")
            elif p.name == 'gain':
                self._driver.set_gain(float(p.value))
                self.get_logger().info(f"Gain → {p.value} %")
            elif p.name == 'ping_rate':
                code = PING_RATES.get(p.value, PING_RATES['normal'])
                self._driver.set_ping_rate(code)
                self.get_logger().info(f"Ping rate → {p.value}")
            elif p.name == 'intensity_threshold':
                self._thr = float(p.value)
            elif p.name == 'sonar_mode':
                self._driver.set_mode(int(p.value))
                self.get_logger().info(f"Mode → {p.value} ({'750kHz' if p.value==1 else '1.2MHz'})")    
        return __import__('rcl_interfaces.msg', fromlist=['SetParametersResult']).SetParametersResult(successful=True)

    def _on_ping(self, ping: SonarPing):
        try:
            self._q.put_nowait(ping)
        except queue.Full:
            try:
                self._q.get_nowait()
                self._q.put_nowait(ping)
            except queue.Empty:
                pass

    def _render_loop(self):
        while rclpy.ok():
            try:
                ping = self._q.get(timeout=1.0)
            except queue.Empty:
                continue
            self._process_and_publish(ping)

    def _process_and_publish(self, ping: SonarPing):
        now = self.get_clock().now().to_msg()
        hdr = Header()
        hdr.stamp    = now
        hdr.frame_id = self._frame
        img          = ping.image

        # Dynamic threshold
        data_max = float(img.max())
        dyn_thr  = max(self._thr, data_max * 0.08)

        # ── 1) Fan image ──────────────────────────────────────────────────
        fan_rgb          = self._renderer.render(ping)
        fh, fw           = fan_rgb.shape[:2]
        fan_msg          = Image()
        fan_msg.header   = hdr
        fan_msg.height   = fh
        fan_msg.width    = fw
        fan_msg.encoding = 'rgb8'
        fan_msg.step     = fw * 3
        fan_msg.data     = fan_rgb.tobytes()
        self._pub_fan.publish(fan_msg)

        # ── 2) LaserScan ─────────────────────────────────────────────────
        brg_rad  = np.deg2rad(ping.bearings_deg)
        strong   = img > dyn_thr
        has_ret  = strong.any(axis=0)
        first_i  = np.argmax(strong, axis=0)
        bidx     = np.arange(ping.n_beams)

        hit_rng  = np.where(has_ret, first_i * ping.range_resolution, 0.0).astype(np.float32)
        hit_int  = np.where(has_ret, img[first_i, bidx], 0.0).astype(np.float32)

        ls               = LaserScan()
        ls.header        = hdr
        ls.angle_min     = float(brg_rad[0])
        ls.angle_max     = float(brg_rad[-1])
        ls.angle_increment = float((brg_rad[-1] - brg_rad[0]) / max(ping.n_beams - 1, 1))
        ls.time_increment  = 0.0
        ls.scan_time       = 0.1
        ls.range_min       = float(ping.range_resolution * 2)
        ls.range_max       = float(ping.n_ranges * ping.range_resolution * 0.99)
        ls.ranges          = hit_rng.tolist()
        ls.intensities     = hit_int.tolist()
        self._pub_ls.publish(ls)

        # ── 3) PointCloud2 ────────────────────────────────────────────────
        valid = has_ret
        n     = int(valid.sum())
        if n > 0:
            x_pts = (hit_rng * np.sin(brg_rad))[valid]
            y_pts = (hit_rng * np.cos(brg_rad))[valid]
            i_pts = hit_int[valid]
            data  = np.zeros((n, 4), dtype=np.float32)
            data[:, 0] = x_pts
            data[:, 1] = y_pts
            data[:, 3] = i_pts
            fields = [
                PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            pc              = PointCloud2()
            pc.header       = hdr
            pc.height       = 1
            pc.width        = n
            pc.fields       = fields
            pc.is_bigendian = False
            pc.point_step   = 16
            pc.row_step     = 16 * n
            pc.is_dense     = True
            pc.data         = data.tobytes()
            self._pub_pc.publish(pc)

        self.get_logger().info(
            f"Ping {ping.ping_id}: {n}/{ping.n_beams} hits  "
            f"max={img.max():.4f}  range={ping.n_ranges*ping.range_resolution:.2f}m  "
            f"T={ping.temperature:.1f}C")

    def destroy_node(self):
        self._driver.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()