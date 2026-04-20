from __future__ import annotations

import argparse
import queue
import re
import threading
import time
import tkinter as tk

import serial


AIR_PATTERN = re.compile(r"air=.*?spd=([0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)
WIND_PATTERN = re.compile(r"wind=.*?spd=([0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)
STALE_TIMEOUT_SEC = 2.0


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, out_queue: queue.Queue[str]) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        try:
            with serial.Serial(self.port, self.baud, timeout=0.2, dsrdtr=False, rtscts=False) as ser:
                ser.setDTR(False)
                ser.setRTS(False)
                while not self._stop_event.is_set():
                    line = ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        self.out_queue.put(line)
        except Exception as exc:
            self.out_queue.put(f"[error] {exc}")


class SpeedPanel:
    def __init__(self, parent: tk.Widget, title: str, color: str, scale_max: float) -> None:
        self.scale_max = scale_max
        self.color = color
        self.last_speed = 0.0
        self.last_update_at = 0.0

        frame = tk.Frame(parent, bg="#ffffff", bd=0, highlightthickness=1, highlightbackground="#d8e0e5")
        frame.pack(fill=tk.X, pady=4)

        tk.Label(
            frame,
            text=title,
            font=("Yu Gothic UI", 10, "bold"),
            bg="#ffffff",
            fg="#17232d",
        ).pack(anchor="w", padx=8, pady=(6, 2))

        self.value_var = tk.StringVar(value="0.0 m/s")
        self.state_var = tk.StringVar(value="STALE")

        tk.Label(
            frame,
            textvariable=self.value_var,
            font=("Consolas", 13, "bold"),
            bg="#ffffff",
            fg="#111111",
        ).pack(anchor="w", padx=8)

        tk.Label(
            frame,
            textvariable=self.state_var,
            font=("Consolas", 8),
            bg="#ffffff",
            fg="#4d5b68",
        ).pack(anchor="w", padx=8, pady=(1, 4))

        canvas = tk.Canvas(frame, width=340, height=20, bg="#eef3f6", highlightthickness=0)
        canvas.pack(padx=8, pady=(0, 8))
        bg_rect = canvas.create_rectangle(0, 4, 340, 16, fill="#dde6ec", outline="")
        self.bar = canvas.create_rectangle(0, 4, 0, 16, fill=color, outline="")
        canvas.create_text(336, 10, text=f"{scale_max:.1f}", anchor="e", fill="#5a6976", font=("Consolas", 8))
        _ = bg_rect

        self.canvas = canvas

    def update(self, speed: float) -> None:
        self.last_speed = speed
        self.last_update_at = time.monotonic()

    def refresh(self) -> None:
        state = "OK" if (time.monotonic() - self.last_update_at) <= STALE_TIMEOUT_SEC else "STALE"
        speed = self.last_speed
        clamped = max(0.0, min(self.scale_max, speed))
        width = 340.0 * (clamped / self.scale_max) if self.scale_max > 0 else 0.0
        self.canvas.coords(self.bar, 0, 4, width, 16)
        self.canvas.itemconfigure(self.bar, fill=self.color if state == "OK" else "#a8b3bc")
        self.value_var.set(f"{speed:.1f} m/s")
        self.state_var.set(state)


class App:
    def __init__(self, root: tk.Tk, port: str, baud: int, max_speed: float) -> None:
        self.root = root
        self.root.title("ESP-NOW Dual Wind Monitor")
        self.root.geometry("380x180")
        self.root.configure(bg="#f3f6f8")

        self.queue: queue.Queue[str] = queue.Queue()
        self.reader = SerialReader(port, baud, self.queue)

        cards = tk.Frame(root, bg="#f3f6f8")
        cards.pack(fill=tk.X, padx=8, pady=(8, 0))

        self.air_panel = SpeedPanel(cards, "main beam", "#2d7ff9", max_speed)
        self.wind_panel = SpeedPanel(cards, "main wing", "#ef7d32", max_speed)

        self.status_var = tk.StringVar(value=f"port={port} baud={baud}")
        self.line_var = tk.StringVar(value="waiting for serial data...")

        tk.Label(
            root,
            textvariable=self.status_var,
            font=("Consolas", 8),
            bg="#f3f6f8",
            fg="#2f3d49",
            anchor="w",
        ).pack(fill=tk.X, padx=8, pady=(4, 2))

        tk.Label(
            root,
            textvariable=self.line_var,
            font=("Consolas", 7),
            bg="#dde6ec",
            fg="#24313c",
            anchor="w",
            padx=6,
            pady=4,
        ).pack(fill=tk.X, padx=8, pady=(0, 6))

        self.reader.start()
        self.root.after(50, self.poll_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def poll_queue(self) -> None:
        while True:
            try:
                line = self.queue.get_nowait()
            except queue.Empty:
                break

            self.line_var.set(line)
            if line.startswith("[error]"):
                self.status_var.set(line)
                continue

            air_match = AIR_PATTERN.search(line)
            wind_match = WIND_PATTERN.search(line)
            if air_match:
                self.air_panel.update(float(air_match.group(1)))
            if wind_match:
                self.wind_panel.update(float(wind_match.group(1)))
            if air_match or wind_match:
                self.status_var.set("receiving")

        self.air_panel.refresh()
        self.wind_panel.refresh()
        self.root.after(50, self.poll_queue)

    def close(self) -> None:
        self.reader.stop()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="ESP-NOW dual monitor visualizer")
    parser.add_argument("--port", default="COM16", help="Serial port name")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--max-speed", type=float, default=15.0, help="Bar graph full-scale speed [m/s]")
    args = parser.parse_args()

    root = tk.Tk()
    App(root, args.port, args.baud, args.max_speed)
    root.mainloop()


if __name__ == "__main__":
    main()
