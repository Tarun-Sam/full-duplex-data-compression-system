import sys
import time
import serial
import serial.tools.list_ports
import tkinter as tk
from queue import Empty, Queue
from threading import Thread, Lock

# ── Protocol constants ─────────────────────────────────────────────────────────
BAUD           = 115200
START_BYTE     = 0xAA
END_BYTE       = 0x55
REV_START_BYTE = 0xC2
REV_END_BYTE   = 0x5A
MODE_BITPACK   = 0x01
MODE_RLE       = 0x02
BUFFER_SIZE    = 50
MAX_DATA_LEN   = 255

# ── ADC / Voltage reference ────────────────────────────────────────────────────
VREF    = 5.0
ADC_MAX = 1023.0

# ── Palette ────────────────────────────────────────────────────────────────────
BG          = "#0a0e14"
BG2         = "#0d1117"
PANEL       = "#131920"
PANEL2      = "#161d26"
BORDER      = "#1e2936"
BORDER2     = "#253040"
ACCENT      = "#4d9ef7"
ACCENT_DIM  = "#1a3a6a"
GREEN       = "#39c96e"
GREEN_DIM   = "#0f3320"
GREEN_BTN   = "#0d2a1a"
RED         = "#f05050"
RED_DIM     = "#3a1010"
RED_BTN     = "#2a0d0d"
ORANGE      = "#e8a020"
ORANGE_DIM  = "#3a2a00"
PURPLE      = "#a57ef5"
TEAL        = "#30c9b0"
CYAN        = "#22d3ee"
YELLOW      = "#f5e642"
TEXT        = "#d8e4f0"
TEXT2       = "#8fa0b8"
TEXT3       = "#4a5a70"
TEXT_BOX    = "#080c10"
CHART_LINE  = "#4d9ef7"
CHART_FILL  = "#1a2a4a"

FONT_MONO   = ("Courier New", 9)
FONT_MONO10 = ("Courier New", 10)
FONT_MONO11 = ("Courier New", 11, "bold")
FONT_MONO13 = ("Courier New", 13, "bold")
FONT_UI     = ("Segoe UI", 9)   if sys.platform == "win32" else ("DejaVu Sans", 9)
FONT_UI10   = ("Segoe UI", 10)  if sys.platform == "win32" else ("DejaVu Sans", 10)
FONT_HEAD   = ("Segoe UI", 8, "bold") if sys.platform == "win32" else ("DejaVu Sans", 8, "bold")
FONT_STAT_V = ("Courier New", 14, "bold")
FONT_STAT_L = ("Segoe UI", 8)   if sys.platform == "win32" else ("DejaVu Sans", 8)

ARDUINO_KEYWORDS = [
    "arduino", "ch340", "ch341", "ftdi", "cp210",
    "usb serial", "usbmodem", "usbserial"
]

# ── Shared state ───────────────────────────────────────────────────────────────
ser                     = None
running                 = False
ser_lock                = Lock()
ui_queue                = Queue()
last_compressed_payload = None
auto_send_enabled       = False

stats = {
    "rx_count":          0,
    "tx_count":          0,
    "checksum_errors":   0,
    "last_packet_size":  0,
    "last_payload_size": 0,
    "last_mode":         "—",
    "total_bytes_rx":    0,
    "total_bytes_tx":    0,
    "avg_payload":       0.0,
    "compression_ratio": 0.0,
    "uptime_start":      None,
    "session_errors":    0,
}

SPARK_MAX      = 120
spark_data     = []
_payload_sizes = []


# ════════════════════════════════════════════════════════════════════════════════
# VOLTAGE HELPER
# ════════════════════════════════════════════════════════════════════════════════

def adc_to_voltage(adc_value, vref=VREF):
    """Convert a raw 10-bit ADC reading to voltage."""
    return adc_value * vref / ADC_MAX


# ════════════════════════════════════════════════════════════════════════════════
# PORT DETECTION
# ════════════════════════════════════════════════════════════════════════════════

def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description  or "").lower()
        mfg  = (p.manufacturer or "").lower()
        if any(kw in desc or kw in mfg for kw in ARDUINO_KEYWORDS):
            return p.device
    if sys.platform != "win32":
        for p in ports:
            if "/dev/ttyACM" in p.device or "/dev/ttyUSB" in p.device:
                return p.device
    return ports[0].device if ports else None


# ════════════════════════════════════════════════════════════════════════════════
# DECODING
# ════════════════════════════════════════════════════════════════════════════════

def nibble_to_delta(nibble):
    nibble = nibble & 0x0F
    return nibble - 16 if nibble >= 8 else nibble

def decode_zero_rle(hi, lo, data):
    base = (hi << 8) | lo
    values, prev = [base], base
    for token in data:
        if token & 0x80:
            for _ in range(token & 0x7F):
                if len(values) >= BUFFER_SIZE: break
                values.append(prev)
        else:
            if len(values) >= BUFFER_SIZE: break
            prev += nibble_to_delta(token)
            values.append(prev)
    return values

def decode_bitpacking(hi, lo, data):
    base = (hi << 8) | lo
    values, prev = [base], base
    for b in data:
        for nibble in ((b >> 4) & 0x0F, b & 0x0F):
            if len(values) >= BUFFER_SIZE: break
            prev += nibble_to_delta(nibble)
            values.append(prev)
        if len(values) >= BUFFER_SIZE: break
    return values

def compute_checksum(mode, hi, lo, data_len, data):
    cs = mode ^ hi ^ lo ^ data_len
    for b in data: cs ^= b
    return cs

def mode_name_from_byte(mode):
    return "ZERO-RLE" if mode == MODE_RLE else ("BIT-PACKING" if mode == MODE_BITPACK else "UNKNOWN")


# ════════════════════════════════════════════════════════════════════════════════
# SEND-BACK
# ════════════════════════════════════════════════════════════════════════════════

def build_reverse_packet(payload):
    mode, hi, lo, data_len = payload[0], payload[1], payload[2], payload[3]
    data = payload[4:]
    cs   = compute_checksum(mode, hi, lo, data_len, data)
    pkt  = bytes([REV_START_BYTE]) + payload + bytes([cs, REV_END_BYTE])
    return pkt, mode_name_from_byte(mode)

def send_payload_back(payload, auto=False):
    global stats
    if not running or ser is None or payload is None:
        return False
    try:
        pkt, mode_name = build_reverse_packet(payload)
        with ser_lock:
            ser.write(pkt)
            ser.flush()
        stats["tx_count"]       += 1
        stats["total_bytes_tx"] += len(pkt)
        kind = "AUTO" if auto else "MANUAL"
        ui_queue.put({
            "type":        "sent",
            "mode_name":   mode_name,
            "packet_size": len(pkt),
            "kind":        kind,
            "status":      f"{'Auto' if auto else 'Manual'} return · {mode_name} · {len(pkt)} B",
        })
        return True
    except Exception as exc:
        ui_queue.put(("error", f"Send error: {exc}"))
        return False


# ════════════════════════════════════════════════════════════════════════════════
# SERIAL READING
# ════════════════════════════════════════════════════════════════════════════════

def sync_to_start():
    while running:
        try:
            with ser_lock:
                raw = ser.read(1)
        except Exception:
            return False
        if raw and raw[0] == START_BYTE:
            return True
    return False

def read_serial():
    global running, last_compressed_payload, auto_send_enabled, stats, spark_data, _payload_sizes
    while running:
        if not sync_to_start(): break
        try:
            with ser_lock: header = ser.read(4)
        except Exception as exc:
            ui_queue.put(("error", f"Header read error: {exc}")); continue
        if len(header) != 4: continue
        mode, hi, lo, data_len = header[0], header[1], header[2], header[3]
        if mode not in (MODE_BITPACK, MODE_RLE): continue
        if data_len > MAX_DATA_LEN: continue
        try:
            with ser_lock: data = ser.read(data_len)
        except Exception as exc:
            ui_queue.put(("error", f"Data read error: {exc}")); continue
        if len(data) != data_len: continue
        try:
            with ser_lock: tail = ser.read(2)
        except Exception as exc:
            ui_queue.put(("error", f"Tail read error: {exc}")); continue
        if len(tail) != 2: continue
        rx_cs, end_byte = tail[0], tail[1]
        if end_byte != END_BYTE: continue
        calc_cs     = compute_checksum(mode, hi, lo, data_len, data)
        checksum_ok = calc_cs == rx_cs
        full_packet = bytes([START_BYTE, mode, hi, lo, data_len]) + bytes(data) + bytes([rx_cs, END_BYTE])
        pkt_size    = len(full_packet)

        if checksum_ok:
            payload = bytes([mode, hi, lo, data_len]) + bytes(data)
            last_compressed_payload = payload
            stats["rx_count"]        += 1
            stats["total_bytes_rx"]  += pkt_size
            _payload_sizes.append(data_len)
            if len(_payload_sizes) > 100:
                _payload_sizes = _payload_sizes[-100:]
            stats["avg_payload"] = sum(_payload_sizes) / len(_payload_sizes)
            uncompressed_est = BUFFER_SIZE * 2
            stats["compression_ratio"] = (1.0 - data_len / uncompressed_est) * 100.0
            if auto_send_enabled:
                send_payload_back(payload, auto=True)
        else:
            stats["checksum_errors"] += 1
            stats["session_errors"]  += 1
            compression_name = mode_name_from_byte(mode)
            raw_hex          = " ".join(f"{b:02X}" for b in full_packet)
            ui_queue.put({
                "type":             "packet_error",
                "raw_hex":          raw_hex,
                "compression_name": compression_name,
                "data_len":         data_len,
                "packet_size":      pkt_size,
                "rx_cs":            rx_cs,
                "calc_cs":          calc_cs,
                "stats":            dict(stats),
            })
            continue

        values           = decode_zero_rle(hi, lo, data) if mode == MODE_RLE else decode_bitpacking(hi, lo, data)
        compression_name = "ZERO-RLE" if mode == MODE_RLE else "BIT-PACKING"
        abs_deltas       = [abs(values[i] - values[i-1]) for i in range(1, len(values))]
        avg_abs          = sum(abs_deltas) / max(1, len(abs_deltas))

        stats["last_packet_size"]  = pkt_size
        stats["last_payload_size"] = data_len
        stats["last_mode"]         = compression_name

        # ── Voltage calculations ──────────────────────────────────────────
        voltage_values = [adc_to_voltage(v) for v in values]
        last_adc       = values[-1]
        last_voltage   = adc_to_voltage(last_adc)
        min_voltage    = min(voltage_values)
        max_voltage    = max(voltage_values)
        avg_voltage    = sum(voltage_values) / len(voltage_values)

        spark_data = (spark_data + voltage_values)[-SPARK_MAX:]

        raw_hex = " ".join(f"{b:02X}" for b in full_packet)

        uptime_str = "—"
        if stats["uptime_start"]:
            elapsed    = int(time.time() - stats["uptime_start"])
            h, rem     = divmod(elapsed, 3600)
            m, s       = divmod(rem, 60)
            uptime_str = f"{h:02d}:{m:02d}:{s:02d}"

        ui_queue.put({
            "type":             "packet",
            "raw_hex":          raw_hex,
            "values":           values,
            "voltage_values":   voltage_values,
            "last_adc":         last_adc,
            "last_voltage":     last_voltage,
            "min_voltage":      min_voltage,
            "max_voltage":      max_voltage,
            "avg_voltage":      avg_voltage,
            "checksum_ok":      checksum_ok,
            "avg_abs":          round(avg_abs, 2),
            "compression_name": compression_name,
            "data_len":         data_len,
            "packet_size":      pkt_size,
            "stats":            dict(stats),
            "spark":            list(spark_data),
            "uptime":           uptime_str,
        })


# ════════════════════════════════════════════════════════════════════════════════
# SPARKLINE CANVAS  — voltage labels
# ════════════════════════════════════════════════════════════════════════════════

def draw_sparkline(canvas, data, w, h):
    canvas.delete("all")
    if len(data) < 2:
        canvas.create_text(w // 2, h // 2, text="AWAITING DATA", fill=TEXT3, font=FONT_MONO)
        return
    mn, mx = min(data), max(data)
    rng    = max(mx - mn, 0.001)
    pad_x, pad_y = 6, 6

    def px(i): return pad_x + (i / (len(data) - 1)) * (w - 2 * pad_x)
    def py(v): return pad_y + (1 - (v - mn) / rng) * (h - 2 * pad_y)

    pts = [(px(i), py(v)) for i, v in enumerate(data)]

    poly = [pad_x, h - pad_y] + [c for p in pts for c in p] + [w - pad_x, h - pad_y]
    canvas.create_polygon(poly, fill=CHART_FILL, outline="")

    for frac in (0.25, 0.5, 0.75):
        gy = pad_y + frac * (h - 2 * pad_y)
        canvas.create_line(pad_x, gy, w - pad_x, gy, fill=BORDER, width=1, dash=(2, 4))

    for i in range(len(pts) - 1):
        canvas.create_line(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1],
                           fill=CHART_LINE, width=1.5, smooth=True)

    cx, cy = pts[-1]
    canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill=ACCENT_DIM, outline="")
    canvas.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=ACCENT, outline=BG, width=1.5)

    canvas.create_text(w - pad_x, h - 2,
                       text=f"min:{mn:.2f}V", fill=TEXT3,
                       font=("Courier New", 7), anchor="se")
    canvas.create_text(w - pad_x, pad_y,
                       text=f"max:{mx:.2f}V", fill=TEXT3,
                       font=("Courier New", 7), anchor="ne")
    canvas.create_text(pad_x, pad_y,
                       text=f"{data[-1]:.2f}V", fill=ACCENT,
                       font=("Courier New", 9, "bold"), anchor="nw")


# ════════════════════════════════════════════════════════════════════════════════
# ════════════════════════════════════════════════════════════════════════════════
# ACTIVITY DOT HELPERS  (RX / TX communication indicators)
# ════════════════════════════════════════════════════════════════════════════════

_blink_after_id = {}

def flash_dot(dot_widget, colour, key):
    dot_widget.config(bg=colour)
    if key in _blink_after_id:
        root.after_cancel(_blink_after_id[key])
    _blink_after_id[key] = root.after(220, lambda: dot_widget.config(bg=BORDER))


# ════════════════════════════════════════════════════════════════════════════════
# TOOLTIP HELPER
# ════════════════════════════════════════════════════════════════════════════════

class Tooltip:
    """Lightweight hover tooltip. text_func is a callable for dynamic text."""
    def __init__(self, widget, text_func):
        self._widget    = widget
        self._text_func = text_func
        self._tip_win   = None
        widget.bind("<Enter>", self._show)
        widget.bind("<Leave>", self._hide)

    def _show(self, event=None):
        if self._tip_win:
            return
        x = self._widget.winfo_rootx()
        y = self._widget.winfo_rooty() + self._widget.winfo_height() + 4
        self._tip_win = tw = tk.Toplevel(self._widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        tw.configure(bg=BORDER2)
        tk.Label(tw, text=self._text_func(),
                 bg=BORDER2, fg=TEXT2,
                 font=("Courier New", 8),
                 padx=8, pady=4,
                 relief=tk.FLAT).pack()

    def _hide(self, event=None):
        if self._tip_win:
            self._tip_win.destroy()
            self._tip_win = None


# ════════════════════════════════════════════════════════════════════════════════
# GUI UPDATE
# ════════════════════════════════════════════════════════════════════════════════

def update_stats_display(s, uptime="—"):
    stat_rx_val.config(text=str(s["rx_count"]))
    stat_tx_val.config(text=str(s["tx_count"]))
    err = s["checksum_errors"]
    stat_err_val.config(text=str(err), fg=RED if err > 0 else GREEN)
    stat_pay_val.config(text=f"{s['last_payload_size']} B")
    stat_total_bytes_val.config(
        text=f"RX {s['total_bytes_rx']:,} / TX {s['total_bytes_tx']:,}"
    )
    ratio = s["compression_ratio"]
    ratio_colour = GREEN if ratio > 50 else (ORANGE if ratio > 20 else RED)
    stat_ratio_val.config(text=f"{ratio:.1f}%", fg=ratio_colour)


def update_gui(pkt):
    # Hex panel
    raw_box.config(state=tk.NORMAL)
    raw_box.delete("1.0", tk.END)
    raw_box.insert(tk.END, pkt["raw_hex"])
    raw_box.config(state=tk.DISABLED)

    # ADC value grid
    vals = pkt["values"]
    for idx, lbl in enumerate(value_labels):
        if idx < len(vals):
            lbl.config(text=str(vals[idx]), fg=TEXT)
        else:
            lbl.config(text="—", fg=TEXT3)

    # Voltage value grid
    vvals = pkt["voltage_values"]
    for idx, lbl in enumerate(voltage_labels):
        if idx < len(vvals):
            lbl.config(text=f"{vvals[idx]:.2f}V", fg=YELLOW)
        else:
            lbl.config(text="—", fg=TEXT3)

    # Voltage monitor cards
    volt_adc_val.config(text=str(pkt["last_adc"]))
    volt_last_val.config(text=f"{pkt['last_voltage']:.3f} V")
    volt_min_val.config(text=f"{pkt['min_voltage']:.3f} V")
    volt_max_val.config(text=f"{pkt['max_voltage']:.3f} V")
    volt_avg_val.config(text=f"{pkt['avg_voltage']:.3f} V")

    # Sparkline
    draw_sparkline(spark_canvas,
                   pkt["spark"],
                   spark_canvas.winfo_width() or 300,
                   spark_canvas.winfo_height() or 80)

    # System log
    cs_ok    = pkt["checksum_ok"]
    cs_tag   = "ok" if cs_ok else "bad"
    cs_str   = "OK" if cs_ok else "BAD"
    ts       = time.strftime("%H:%M:%S")
    log_line = (
        f"{ts}  [RX]  {pkt['compression_name']:<12} "
        f"pkt={pkt['packet_size']:>3}B  pay={pkt['data_len']:>3}B  cs={cs_str}"
    )
    checksum_box.config(state=tk.NORMAL)
    checksum_box.insert(tk.END, log_line + "\n", cs_tag)
    checksum_box.see(tk.END)
    checksum_box.config(state=tk.DISABLED)

    # Mode badge
    mode_colour = PURPLE if pkt["compression_name"] == "ZERO-RLE" else TEAL
    mode_badge.config(text=f" MODE: {pkt['compression_name']}  ", bg=mode_colour, fg=BG)

    # Mode explanation label
    if pkt["compression_name"] == "ZERO-RLE":
        mode_explain_label.config(text="Stable/repeated signal detected", fg=TEXT2)
    elif pkt["compression_name"] == "BIT-PACKING":
        mode_explain_label.config(text="Changing signal, packed deltas used", fg=TEXT2)
    else:
        mode_explain_label.config(text="Compression mode unknown", fg=TEXT3)

    # Waveform info label
    wave_info_label.config(
        text=(
            f"Current: {pkt['last_voltage']:.2f} V   "
            f"Range: {pkt['min_voltage']:.2f} V - {pkt['max_voltage']:.2f} V   "
            f"Samples: {len(pkt['spark'])}"
        )
    )

    # Metric strip
    metric_label.config(
        text=(
            f"  Avg |δ| {pkt['avg_abs']:>6}   "
            f"Payload {pkt['data_len']:>3} B   "
            f"Packet {pkt['packet_size']:>3} B   "
            f"Last V: {pkt['last_voltage']:.2f} V   "
        ),
        fg=TEXT2,
    )

    flash_dot(rx_dot, GREEN, "rx")
    update_stats_display(pkt["stats"], pkt.get("uptime", "—"))


def log_sent_message(item):
    ts       = time.strftime("%H:%M:%S")
    log_line = (
        f"{ts}  [TX]  {item['kind']:<6} {item['mode_name']:<12} pkt={item['packet_size']:>3}B"
    )
    checksum_box.config(state=tk.NORMAL)
    checksum_box.insert(tk.END, log_line + "\n", "sent")
    checksum_box.see(tk.END)
    checksum_box.config(state=tk.DISABLED)
    set_status(item["status"], ACCENT)
    flash_dot(tx_dot, ACCENT, "tx")
    stat_tx_val.config(text=str(stats["tx_count"]))


def log_packet_error(item):
    ts       = time.strftime("%H:%M:%S")
    log_line = (
        f"{ts}  [RX]  {item['compression_name']:<12} "
        f"pkt={item['packet_size']:>3}B  pay={item['data_len']:>3}B  "
        f"cs=BAD rx={item['rx_cs']:02X} calc={item['calc_cs']:02X}"
    )
    checksum_box.config(state=tk.NORMAL)
    checksum_box.insert(tk.END, log_line + "\n", "bad")
    checksum_box.see(tk.END)
    checksum_box.config(state=tk.DISABLED)

    raw_box.config(state=tk.NORMAL)
    raw_box.delete("1.0", tk.END)
    raw_box.insert(tk.END, item["raw_hex"])
    raw_box.config(state=tk.DISABLED)

    set_status("Checksum error received from Arduino", RED)
    flash_dot(rx_dot, RED, "rx")
    update_stats_display(item["stats"], current_uptime())


def process_ui_queue():
    try:
        while True:
            item = ui_queue.get_nowait()
            if isinstance(item, dict):
                if item.get("type") == "packet":
                    update_gui(item)
                elif item.get("type") == "sent":
                    log_sent_message(item)
                elif item.get("type") == "packet_error":
                    log_packet_error(item)
            elif isinstance(item, tuple) and item[0] == "error":
                set_status(item[1], RED)
    except Empty:
        pass
    root.after(40, process_ui_queue)


# ════════════════════════════════════════════════════════════════════════════════
# CLEAR / RESET
# ════════════════════════════════════════════════════════════════════════════════

def clear_display():
    """Reset all display widgets and in-memory buffers. Does NOT disconnect."""
    global last_compressed_payload, spark_data, _payload_sizes, stats

    last_compressed_payload = None
    spark_data              = []
    _payload_sizes          = []

    stats["rx_count"]          = 0
    stats["tx_count"]          = 0
    stats["checksum_errors"]   = 0
    stats["last_packet_size"]  = 0
    stats["last_payload_size"] = 0
    stats["last_mode"]         = "—"
    stats["total_bytes_rx"]    = 0
    stats["total_bytes_tx"]    = 0
    stats["avg_payload"]       = 0.0
    stats["compression_ratio"] = 0.0
    stats["session_errors"]    = 0

    raw_box.config(state=tk.NORMAL)
    raw_box.delete("1.0", tk.END)
    raw_box.config(state=tk.DISABLED)

    checksum_box.config(state=tk.NORMAL)
    checksum_box.delete("1.0", tk.END)
    checksum_box.config(state=tk.DISABLED)

    for lbl in value_labels:
        lbl.config(text="—", fg=TEXT3)
    for lbl in voltage_labels:
        lbl.config(text="—", fg=TEXT3)

    volt_adc_val.config(text="—")
    volt_last_val.config(text="—")
    volt_min_val.config(text="—")
    volt_max_val.config(text="—")
    volt_avg_val.config(text="—")

    update_stats_display(stats, "—")

    spark_canvas.delete("all")
    w = spark_canvas.winfo_width() or 300
    h = spark_canvas.winfo_height() or 80
    spark_canvas.create_text(w // 2, h // 2, text="AWAITING DATA", fill=TEXT3, font=FONT_MONO)

    mode_badge.config(text=" MODE:  —  ", bg=BORDER, fg=TEXT3)
    metric_label.config(text="  Display cleared — awaiting new packets …", fg=TEXT3)
    mode_explain_label.config(text="Waiting for compression mode…", fg=TEXT3)
    wave_info_label.config(text="Current: —   Range: —   Samples: 0")

    set_status("Display cleared", ORANGE)


# ════════════════════════════════════════════════════════════════════════════════
# CONNECT / DISCONNECT
# ════════════════════════════════════════════════════════════════════════════════

def set_status(msg, colour=TEXT2):
    status_label.config(text=msg, fg=colour)


def current_uptime():
    if not stats["uptime_start"]:
        return "—"
    elapsed = int(time.time() - stats["uptime_start"])
    h, rem = divmod(elapsed, 3600)
    m, s = divmod(rem, 60)
    return f"{h:02d}:{m:02d}:{s:02d}"


def connect():
    global ser, running
    if running: return
    port = find_arduino_port()
    if port is None:
        set_status("No serial device found", RED); return
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
        ser.reset_input_buffer(); ser.reset_output_buffer()
        running = True
        stats["uptime_start"] = time.time()
        Thread(target=read_serial, daemon=True).start()
        port_label.config(text=port, fg=GREEN)
        conn_dot.config(bg=GREEN)
        btn_connect.config(state=tk.DISABLED)
        btn_disconnect.config(state=tk.NORMAL)
        # Enable Send Return only when in Manual mode
        if not auto_send_enabled:
            btn_send_return.config(state=tk.NORMAL)
        set_status(f"Connected", GREEN)
    except Exception as exc:
        set_status(f"Error: {exc}", RED)


def disconnect():
    global ser, running
    running = False
    stats["uptime_start"] = None
    if ser:
        try: ser.close()
        except Exception: pass
        ser = None
    port_label.config(text="—", fg=TEXT3)
    conn_dot.config(bg=BORDER)
    btn_connect.config(state=tk.NORMAL)
    btn_disconnect.config(state=tk.DISABLED)
    btn_send_return.config(state=tk.DISABLED)
    mode_badge.config(text=" MODE:  —  ", bg=BORDER, fg=TEXT3)
    set_status("Disconnected", RED)
    mode_explain_label.config(text="Waiting for compression mode…", fg=TEXT3)
    wave_info_label.config(text="Current: —   Range: —   Samples: 0")
    metric_label.config(text="  Awaiting connection …", fg=TEXT3)


def on_close():
    disconnect()
    root.destroy()


# ════════════════════════════════════════════════════════════════════════════════
# SEND RETURN / RETURN MODE TOGGLE
# ════════════════════════════════════════════════════════════════════════════════

def send_back():
    """Manually send the last received packet back to Arduino."""
    global last_compressed_payload
    if not running or ser is None:
        set_status("Not connected", RED); return
    if last_compressed_payload is None:
        set_status("No valid packet received yet", ORANGE); return
    send_payload_back(last_compressed_payload, auto=False)


def toggle_return_mode():
    """Toggle between RETURN: MANUAL and RETURN: AUTO."""
    global auto_send_enabled
    auto_send_enabled = not auto_send_enabled

    if auto_send_enabled:
        # AUTO mode: hide Send Return
        btn_send_return.pack_forget()

        btn_return_mode.config(
            text=" ⟳  RETURN: AUTO ",
            bg=ACCENT_DIM,
            fg=ACCENT,
        )

        set_status(
            "Return Mode: AUTO  ·  every valid packet will be returned automatically",
            ACCENT,
        )

    else:
        # MANUAL mode: show Send Return again
        btn_send_return.pack(side=tk.LEFT, padx=(0, 4))
        btn_send_return.config(state=tk.NORMAL if running else tk.DISABLED)

        btn_return_mode.config(
            text=" ⇌  RETURN: MANUAL ",
            bg=BORDER,
            fg=TEXT2,
        )

        set_status("Return Mode: MANUAL  ·  manual return restored", TEXT2)


def _return_mode_tooltip():
    """Dynamic tooltip text based on current return mode."""
    if auto_send_enabled:
        return "Click to switch back to manual return"
    return "Click to enable automatic return"


# ════════════════════════════════════════════════════════════════════════════════
# WIDGET HELPERS
# ════════════════════════════════════════════════════════════════════════════════

def dark_text(parent, height, width, **kwargs):
    return tk.Text(parent, height=height, width=width,
                   bg=TEXT_BOX, fg=TEXT, insertbackground=ACCENT,
                   selectbackground=ACCENT, selectforeground=BG,
                   relief=tk.FLAT, font=FONT_MONO, padx=8, pady=6, **kwargs)

def dark_scrollbar(parent, orient=tk.VERTICAL):
    return tk.Scrollbar(parent, orient=orient, bg=BORDER, troughcolor=BG2,
                        activebackground=ACCENT, relief=tk.FLAT, width=8)

def mk_button(parent, text, cmd, bg=BORDER, fg=TEXT, state=tk.NORMAL, padx=12):
    btn = tk.Button(parent, text=text, command=cmd,
                    bg=bg, fg=fg, activebackground=ACCENT, activeforeground=BG,
                    relief=tk.FLAT, font=("Courier New", 9, "bold"),
                    state=state, padx=padx, pady=5, bd=0, cursor="hand2")
    _bg_orig = bg
    _fg_orig = fg
    btn.bind("<Enter>", lambda e: btn.config(bg=ACCENT, fg=BG)
             if str(btn["state"]) != tk.DISABLED else None)
    btn.bind("<Leave>", lambda e: btn.config(bg=_bg_orig, fg=_fg_orig)
             if str(btn["state"]) != tk.DISABLED else None)
    return btn

def mk_sep(parent, vertical=True):
    if vertical:
        return tk.Frame(parent, bg=BORDER2, width=1)
    return tk.Frame(parent, bg=BORDER2, height=1)

def panel_frame(parent, title, accent_colour=ACCENT):
    """Returns (outer_frame, body_frame)."""
    outer = tk.Frame(parent, bg=BORDER2, padx=1, pady=1)
    inner = tk.Frame(outer, bg=PANEL)
    inner.pack(fill=tk.BOTH, expand=True)
    hdr = tk.Frame(inner, bg=PANEL2, pady=6)
    hdr.pack(fill=tk.X)
    tk.Frame(hdr, bg=accent_colour, width=3).pack(side=tk.LEFT, fill=tk.Y)
    tk.Label(hdr, text=title.upper(), bg=PANEL2, fg=accent_colour,
             font=("Courier New", 8, "bold")).pack(fill=tk.X, expand=True)
    mk_sep(inner, vertical=False).pack(fill=tk.X)
    body = tk.Frame(inner, bg=PANEL, padx=5, pady=5)
    body.pack(fill=tk.BOTH, expand=True)
    return outer, body

def stat_card(parent, label, colour=TEXT, row=0, col=0, colspan=1):
    card = tk.Frame(parent, bg=PANEL2, padx=8, pady=5)
    card.grid(row=row, column=col, columnspan=colspan, sticky="nsew", padx=2, pady=2)
    tk.Label(card, text=label.upper(), bg=PANEL2, fg=TEXT2,
             font=("Courier New", 8, "bold")).pack(anchor="center")
    val = tk.Label(card, text="0", bg=PANEL2, fg=colour,
                   font=("Courier New", 12, "bold"))
    val.pack(anchor="center")
    return val

def volt_card(parent, label, colour=TEXT, row=0, col=0):
    """Voltage monitor card."""
    card = tk.Frame(parent, bg=PANEL2, padx=8, pady=7)
    card.grid(row=row, column=col, sticky="nsew", padx=2, pady=2)
    tk.Label(card, text=label.upper(), bg=PANEL2, fg=TEXT2,
             font=("Courier New", 8, "bold")).pack(anchor="center")
    val = tk.Label(card, text="—", bg=PANEL2, fg=colour,
                   font=("Courier New", 13, "bold"))
    val.pack(anchor="center")
    return val

def dot_indicator(parent, size=9, colour=BORDER):
    c = tk.Canvas(parent, width=size, height=size, bg=parent["bg"],
                  highlightthickness=0, bd=0)
    oval_id = c.create_oval(1, 1, size - 1, size - 1, fill=colour, outline="")
    def recolor(new_colour):
        c.itemconfig(oval_id, fill=new_colour)
    c.config_ = recolor
    c.bg_colour = colour
    return c


# ════════════════════════════════════════════════════════════════════════════════
# ROOT WINDOW
# ════════════════════════════════════════════════════════════════════════════════

root = tk.Tk()
root.title("Full-Duplex Compression System")
root.configure(bg=BG)
root.resizable(True, True)

# Fit to available screen space
_sw = root.winfo_screenwidth()
_sh = root.winfo_screenheight()
_w  = min(_sw, 1440)
_h  = min(_sh - 60, 860)
root.geometry(f"{_w}x{_h}+{(_sw - _w)//2}+0")
root.minsize(1100, 680)

try:
    root.option_add("*PanedWindow.sashRelief", "flat")
    root.option_add("*PanedWindow.background", BORDER2)
    root.option_add("*PanedWindow.sashWidth",  6)
except Exception:
    pass


# ════════════════════════════════════════════════════════════════════════════════
# TOP BAR
# ════════════════════════════════════════════════════════════════════════════════

top = tk.Frame(root, bg=PANEL2, pady=0)
top.pack(fill=tk.X)
tk.Frame(top, bg=ACCENT, height=2).pack(fill=tk.X, side=tk.TOP)

# Single unified header row
inner_top = tk.Frame(top, bg=PANEL2, pady=6)
inner_top.pack(fill=tk.X, padx=8)

# ── Title ─────────────────────────────────────────────────────────────────────
tk.Label(inner_top, text="FULL-DUPLEX COMPRESSION SYSTEM",
         bg=PANEL2, fg=ACCENT,
         font=("Courier New", 9, "bold")).pack(side=tk.LEFT, padx=(4, 0))

mk_sep(inner_top).pack(side=tk.LEFT, fill=tk.Y, padx=6, pady=3)

# ── Port indicator ─────────────────────────────────────────────────────────────
conn_block = tk.Frame(inner_top, bg=PANEL2)
conn_block.pack(side=tk.LEFT, padx=(0, 4))

conn_dot_canvas = tk.Canvas(conn_block, width=9, height=9, bg=PANEL2,
                             highlightthickness=0, bd=0)
conn_dot_canvas.pack(side=tk.LEFT, padx=(0, 3), pady=1)
_conn_oval = conn_dot_canvas.create_oval(1, 1, 8, 8, fill=BORDER, outline="")

class _DotProxy:
    def config(self, **kwargs):
        if "bg" in kwargs:
            conn_dot_canvas.itemconfig(_conn_oval, fill=kwargs["bg"])
conn_dot = _DotProxy()

tk.Label(conn_block, text="PORT", bg=PANEL2, fg=TEXT3,
         font=("Courier New", 7, "bold")).pack(side=tk.LEFT)
port_label = tk.Label(conn_block, text="—", bg=PANEL2, fg=TEXT3,
                      font=("Courier New", 8, "bold"))
port_label.pack(side=tk.LEFT, padx=(3, 0))

mk_sep(inner_top).pack(side=tk.LEFT, fill=tk.Y, padx=6, pady=3)

# ── Connect / Disconnect buttons ───────────────────────────────────────────────
btn_connect    = mk_button(inner_top, "⏵  CONNECT",    connect,
                            bg=GREEN_BTN, fg=GREEN, state=tk.NORMAL, padx=8)
btn_disconnect = mk_button(inner_top, "⏹  DISCONNECT", disconnect,
                            bg=RED_BTN,   fg=RED,   state=tk.DISABLED, padx=8)

def _bind_themed_btn(btn, hover_bg, hover_fg, normal_bg, normal_fg):
    btn.bind("<Enter>", lambda e: btn.config(bg=hover_bg, fg=hover_fg)
             if str(btn["state"]) != tk.DISABLED else None)
    btn.bind("<Leave>", lambda e: btn.config(bg=normal_bg, fg=normal_fg)
             if str(btn["state"]) != tk.DISABLED else None)

_bind_themed_btn(btn_connect,    GREEN, BG, GREEN_BTN, GREEN)
_bind_themed_btn(btn_disconnect, RED,   BG, RED_BTN,   RED)
btn_connect.config(font=("Courier New", 8, "bold"), padx=8)
btn_disconnect.config(font=("Courier New", 8, "bold"), padx=8)
btn_connect.pack(side=tk.LEFT, padx=(0, 4))
btn_disconnect.pack(side=tk.LEFT, padx=(0, 4))

mk_sep(inner_top).pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=3)

# ── RETURN CONTROLS ────────────────────────────────────────────────────────────
# This frame keeps RETURN MODE and SEND RETURN together.
# So when SEND RETURN is hidden and shown again, it comes back in the correct place.
return_controls = tk.Frame(inner_top, bg=PANEL2)
return_controls.pack(side=tk.LEFT, padx=(0, 4))

# ── RETURN MODE toggle button ──────────────────────────────────────────────────
btn_return_mode = tk.Button(
    return_controls,
    text=" ⇌  RETURN: MANUAL ",
    command=toggle_return_mode,
    bg=BORDER, fg=TEXT2,
    activebackground=ACCENT_DIM, activeforeground=ACCENT,
    relief=tk.FLAT,
    font=("Courier New", 8, "bold"),
    padx=6, pady=4,
    bd=0,
    cursor="hand2",
)
btn_return_mode.pack(side=tk.LEFT, padx=(0, 4))

# Attach dynamic tooltip
Tooltip(btn_return_mode, _return_mode_tooltip)

# ── SEND RETURN button, visible only in Manual mode ────────────────────────────
btn_send_return = mk_button(
    return_controls,
    "⏎  SEND RETURN",
    send_back,
    bg=ACCENT_DIM,
    fg=ACCENT,
    state=tk.DISABLED,
    padx=8
)
btn_send_return.config(font=("Courier New", 8, "bold"), padx=8)
btn_send_return.pack(side=tk.LEFT, padx=(0, 4))

mk_sep(inner_top).pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=3)

# ── Clear button ───────────────────────────────────────────────────────────────
btn_clear = mk_button(inner_top, "✕  CLEAR", clear_display,
                      bg=ORANGE_DIM, fg=ORANGE, state=tk.NORMAL, padx=8)
btn_clear.config(font=("Courier New", 8, "bold"), padx=8)
btn_clear.pack(side=tk.LEFT, padx=(0, 4))
_bind_themed_btn(btn_clear, ORANGE, BG, ORANGE_DIM, ORANGE)

mk_sep(inner_top).pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=3)

# ── RX / TX activity dots ─────────────────────────────────────────────────────
io_frame = tk.Frame(inner_top, bg=PANEL2)
io_frame.pack(side=tk.LEFT, padx=2)

def _make_io_dot(parent, label_text, colour):
    f = tk.Frame(parent, bg=PANEL2)
    f.pack(side=tk.LEFT, padx=4)
    tk.Label(f, text=label_text, bg=PANEL2, fg=TEXT3,
             font=("Courier New", 7, "bold")).pack()
    c   = tk.Canvas(f, width=9, height=9, bg=PANEL2, highlightthickness=0, bd=0)
    oid = c.create_oval(1, 1, 8, 8, fill=BORDER, outline="")
    c.pack()
    class _P:
        def config(self, **kw):
            if "bg" in kw:
                c.itemconfig(oid, fill=kw["bg"])
    return _P()

rx_dot = _make_io_dot(io_frame, "RX", GREEN)
tx_dot = _make_io_dot(io_frame, "TX", ACCENT)

mk_sep(inner_top).pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=3)

# ── Status label (expands to fill remaining space) ────────────────────────────
status_label = tk.Label(inner_top, text="Auto-detect ready  ·  click CONNECT",
                        bg=PANEL2, fg=TEXT3, font=("Courier New", 8), anchor="w")
status_label.pack(side=tk.LEFT, padx=(4, 0), fill=tk.X, expand=True)


# ════════════════════════════════════════════════════════════════════════════════
# MODE / METRIC STRIP
# ════════════════════════════════════════════════════════════════════════════════

strip = tk.Frame(root, bg=BG2, pady=4)
strip.pack(fill=tk.X)
tk.Frame(root, bg=BORDER2, height=1).pack(fill=tk.X)

strip_inner = tk.Frame(strip, bg=BG2)
strip_inner.pack(padx=10, fill=tk.X)

mode_badge = tk.Label(strip_inner, text="  —  ", bg=BORDER, fg=TEXT3,
                      font=("Courier New", 11, "bold"), padx=8, pady=3)
mode_badge.pack(side=tk.LEFT)

mode_explain_label = tk.Label(strip_inner, text="Waiting for compression mode…",
                               bg=BG2, fg=TEXT3, font=("Courier New", 9))
mode_explain_label.pack(side=tk.LEFT, padx=(10, 0))

metric_label = tk.Label(strip_inner, text="  Awaiting connection …",
                        bg=BG2, fg=TEXT3, font=("Courier New", 9))
metric_label.pack(side=tk.LEFT, padx=(14, 0))


# ════════════════════════════════════════════════════════════════════════════════
# MAIN LAYOUT
# ════════════════════════════════════════════════════════════════════════════════

tk.Frame(root, bg=BORDER2, height=1).pack(fill=tk.X)

main = tk.Frame(root, bg=BG)
main.pack(fill=tk.BOTH, expand=True, padx=6, pady=(6, 6))

# ─── Column 1: Compressed Packet HEX (narrow, fixed) ─────────────────────────
col1 = tk.Frame(main, bg=BG)
col1.pack(side=tk.LEFT, fill=tk.BOTH)

p1, b1 = panel_frame(col1, "RECEIVED COMPRESSED PACKET", ACCENT)
p1.pack(fill=tk.BOTH, expand=True)

raw_box = dark_text(b1, height=16, width=24, state=tk.DISABLED, wrap=tk.WORD)
sb1     = dark_scrollbar(b1)
raw_box.config(yscrollcommand=sb1.set)
sb1.config(command=raw_box.yview)
raw_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
sb1.pack(side=tk.LEFT, fill=tk.Y)

# ─── Column 2: ADC Values + Voltage Values grids (stacked) ───────────────────
col2 = tk.Frame(main, bg=BG)
col2.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))

VALUES_COLS = 5
num_rows    = BUFFER_SIZE // VALUES_COLS  # 10

# Panel: Reconstructed ADC Values
p2, b2 = panel_frame(col2, "RECONSTRUCTED VALUES", TEAL)
p2.pack(fill=tk.BOTH, expand=True)

adc_grid = tk.Frame(b2, bg=TEXT_BOX)
adc_grid.pack(fill=tk.BOTH, expand=True)
for c in range(VALUES_COLS): adc_grid.columnconfigure(c, weight=1)
for r in range(num_rows):    adc_grid.rowconfigure(r, weight=1)

value_labels = []
for i in range(BUFFER_SIZE):
    row_i, col_i = divmod(i, VALUES_COLS)
    lbl = tk.Label(adc_grid, text="—", bg=TEXT_BOX, fg=TEXT3,
                   font=FONT_MONO, anchor="center", padx=4, pady=2)
    lbl.grid(row=row_i, column=col_i, sticky="nsew", padx=1, pady=1)
    value_labels.append(lbl)

# Panel: Voltage Values
p2v, b2v = panel_frame(col2, "VOLTAGE VALUES", YELLOW)
p2v.pack(fill=tk.BOTH, expand=True, pady=(6, 0))

volt_grid = tk.Frame(b2v, bg=TEXT_BOX)
volt_grid.pack(fill=tk.BOTH, expand=True)
for c in range(VALUES_COLS): volt_grid.columnconfigure(c, weight=1)
for r in range(num_rows):    volt_grid.rowconfigure(r, weight=1)

voltage_labels = []
for i in range(BUFFER_SIZE):
    row_i, col_i = divmod(i, VALUES_COLS)
    lbl = tk.Label(volt_grid, text="—", bg=TEXT_BOX, fg=TEXT3,
                   font=FONT_MONO, anchor="center", padx=4, pady=2)
    lbl.grid(row=row_i, column=col_i, sticky="nsew", padx=1, pady=1)
    voltage_labels.append(lbl)

# ─── Column 3: Right side — Voltage Monitor + System Log + Stats ──────────────
col3 = tk.Frame(main, bg=BG)
col3.pack(side=tk.LEFT, fill=tk.BOTH, padx=(8, 0))

# Panel: Voltage Monitor
vm_outer, vm_body = panel_frame(col3, "VOLTAGE MONITOR", YELLOW)
vm_outer.pack(fill=tk.X)

for c in range(5): vm_body.columnconfigure(c, weight=1)
volt_adc_val  = volt_card(vm_body, "Last ADC",     TEXT,   0, 0)
volt_last_val = volt_card(vm_body, "Last Voltage", YELLOW, 0, 1)
volt_min_val  = volt_card(vm_body, "Min Voltage",  CYAN,   0, 2)
volt_max_val  = volt_card(vm_body, "Max Voltage",  RED,    0, 3)
volt_avg_val  = volt_card(vm_body, "Avg Voltage",  GREEN,  0, 4)

# Panel: Live Voltage Waveform (directly below Voltage Monitor)
spark_outer, spark_body = panel_frame(col3, "LIVE VOLTAGE WAVEFORM", PURPLE)
spark_outer.pack(fill=tk.X, pady=(6, 0))

wave_info_label = tk.Label(spark_body,
                           text="Current: \u2014   Range: \u2014   Samples: 0",
                           bg=PANEL, fg=TEXT2,
                           font=("Courier New", 8, "bold"),
                           anchor="w")
wave_info_label.pack(fill=tk.X, padx=2, pady=(0, 3))

spark_canvas = tk.Canvas(spark_body, height=110, bg=TEXT_BOX, highlightthickness=0, bd=0)
spark_canvas.pack(fill=tk.X, expand=True)
spark_canvas.create_text(150, 55, text="AWAITING DATA", fill=TEXT3, font=FONT_MONO)

# Panel: System Log
p3, b3 = panel_frame(col3, "SYSTEM LOG", GREEN)
p3.pack(fill=tk.BOTH, expand=True, pady=(6, 0))

checksum_box = dark_text(b3, height=10, width=52, state=tk.DISABLED)
checksum_box.tag_config("ok",   foreground=GREEN)
checksum_box.tag_config("bad",  foreground=RED)
checksum_box.tag_config("sent", foreground=ACCENT)

sb3 = dark_scrollbar(b3)
checksum_box.config(yscrollcommand=sb3.set)
sb3.config(command=checksum_box.yview)
checksum_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
sb3.pack(side=tk.LEFT, fill=tk.Y)

# Panel: Packet Statistics (6 items: 2 rows × 3 cols)
stats_outer, stats_body = panel_frame(col3, "PACKET STATISTICS", ORANGE)
stats_outer.pack(fill=tk.X, pady=(6, 0))

g = stats_body
for c in range(3): g.columnconfigure(c, weight=1)

stat_rx_val         = stat_card(g, "Packets RX",    GREEN,  0, 0)
stat_tx_val         = stat_card(g, "Packets TX",    ACCENT, 0, 1)
stat_err_val        = stat_card(g, "Checksum Errs", GREEN,  0, 2)
stat_pay_val        = stat_card(g, "Last Payload",  TEXT,   1, 0)
stat_total_bytes_val = stat_card(g, "Total Bytes",  TEAL,   1, 1)
stat_ratio_val      = stat_card(g, "Compression %", GREEN,  1, 2)

# Internal-only labels kept for update_stats_display compatibility
# (not displayed, values tracked in stats dict for debugging)
stat_total_bytes_val.config(font=("Courier New", 9, "bold"))  # smaller font for RX/TX text


# ════════════════════════════════════════════════════════════════════════════════
# START
# ════════════════════════════════════════════════════════════════════════════════

root.protocol("WM_DELETE_WINDOW", on_close)
root.after(40, process_ui_queue)
root.mainloop()