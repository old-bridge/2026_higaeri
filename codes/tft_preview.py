#!/usr/bin/env python3
"""
TFT Display D1 プレビューツール
================================
display_d1.ino の drawGaugeFrame() + renderDisplay() を PC 上で再現します。
スライダで各センサ値を変えるとリアルタイムに画面が更新されます。

依存ライブラリ: tkinter（Python 標準搭載）のみ
使い方: python tft_preview.py
"""

import math
import tkinter as tk
from tkinter import ttk


# ── ディスプレイ定数（Arduino コードと一致） ──────────────────────────
TFT_W, TFT_H = 240, 320          # ST7789 240×320, setRotation(0)
SCALE = 2                        # PC 表示用の拡大倍率（整数で変更可）

# RGB565 0x5AEB → Python の色文字列 (#RRGGBB)
def _rgb565(c: int) -> str:
    r = ((c >> 11) & 0x1F) * 255 // 31
    g = ((c >>  5) & 0x3F) * 255 // 63
    b = ( c        & 0x1F) * 255 // 31
    return f"#{r:02X}{g:02X}{b:02X}"


# TFT_eSPI 標準色
BG_COLOR  = _rgb565(0x5AEB)   # kDisplayBackground
TFT_WHITE = "#FFFFFF"
TFT_GREEN = "#07E000"          # TFT_GREEN (RGB565 #07E0)
TFT_RED   = "#F80000"          # TFT_RED   (RGB565 #F800)

# ── ゲージ定数（Arduino コードと一致） ──────────────────────────────
GAUGE_CX           = 120
GAUGE_CY           = 125
GAUGE_OUTER_R      = 105
ARC_OUTER_R        = 103
ARC_INNER_R        = 88
TICK_OUTER_R       = 105
TICK_MAJOR_INNER_R = 85
TICK_MINOR_INNER_R = 95
NEEDLE_LEN         = 75
NUMBER_R           = 72
GAUGE_START_DEG    = 30.0
GAUGE_SWEEP_DEG    = 300.0
GAUGE_MAX_SPEED    = 10.0

# フォント
FONT2      = ("Courier", 10 + SCALE * 4, "bold")   # 下部情報パネル
FONT_SPEED = ("Arial Black", 60, "bold")            # 中央デジタル速度
FONT_GAUGE = ("Arial Black", 11, "bold")            # 目盛り数字


def speed_to_angle_deg(speed: float) -> float:
    return GAUGE_START_DEG + (speed / GAUGE_MAX_SPEED) * GAUGE_SWEEP_DEG


def gx(angle_deg: float, r: float) -> float:
    return GAUGE_CX + r * math.sin(math.radians(angle_deg))


def gy(angle_deg: float, r: float) -> float:
    return GAUGE_CY - r * math.cos(math.radians(angle_deg))


class TFTPreviewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("TFT Display D1 プレビュー")
        self.resizable(False, False)
        self._build_ui()
        self.render()

    # ── UI 構築 ──────────────────────────────────────────────────────
    def _build_ui(self):
        # ===== 左列：TFT キャンバス =====
        left = tk.Frame(self)
        left.grid(row=0, column=0, padx=14, pady=14, sticky="n")

        tk.Label(
            left,
            text=f"TFT 240×320  (×{SCALE} 拡大)",
            font=("Segoe UI", 9),
        ).pack(anchor="w")

        # 液晶ベゼル風の外枠
        bezel = tk.Frame(left, bg="#1a1a1a", bd=8, relief=tk.RAISED)
        bezel.pack()

        self.cv = tk.Canvas(
            bezel,
            width=TFT_W * SCALE,
            height=TFT_H * SCALE,
            highlightthickness=0,
        )
        self.cv.pack()

        # ===== 右列：コントロールパネル =====
        right = ttk.Frame(self)
        right.grid(row=0, column=1, padx=14, pady=14, sticky="ns")

        sensor_frame = ttk.LabelFrame(right, text="センサ値の調整")
        sensor_frame.pack(fill=tk.X, pady=(0, 6))

        self._vars: dict[str, tk.Variable] = {}

        # airspeed はModbusレジスタ値（対気速度 m/s × 10）
        # (変数キー, ラベル, 最小, 最大, 初期値)
        sliders = [
            ("airspeed",   "対気速度  [m/s×10 : 0=0.0, 100=10.0]", 0, 100, 50),
            ("baro_alt",   "気圧高度  [×0.1 m]                   ", 0, 5000, 500),
            ("pot1",       "ポテンショ1 [raw ADC 0-1023]          ", 0, 1023, 256),
            ("pot2",       "ポテンショ2 [raw ADC 0-1023]          ", 0, 1023, 256),
            ("battery",    "バッテリ    [raw ADC 0-1023]          ", 0, 1023, 800),
            ("ultrasonic", "超音波高度  [raw ADC 0-1023]          ", 0, 1023,   0),
        ]

        for key, label, lo, hi, init in sliders:
            v = tk.IntVar(value=init)
            self._vars[key] = v
            self._make_slider_row(sensor_frame, label, v, lo, hi)

        # I2C ステータス
        sep = ttk.Separator(sensor_frame, orient="horizontal")
        sep.pack(fill=tk.X, padx=6, pady=4)

        self._i2c_ok = tk.BooleanVar(value=True)
        chk = ttk.Checkbutton(
            sensor_frame,
            text="I2C 接続中 (connected)",
            variable=self._i2c_ok,
            command=self.render,
        )
        chk.pack(anchor="w", padx=10, pady=2)

        for key, label in [("ok_cnt", "OK カウント"), ("err_cnt", "ERR カウント")]:
            v = tk.IntVar(value=0)
            self._vars[key] = v
            row = ttk.Frame(sensor_frame)
            row.pack(fill=tk.X, padx=10, pady=2)
            ttk.Label(row, text=label, width=14, anchor="w").pack(side=tk.LEFT)
            sb = ttk.Spinbox(
                row,
                textvariable=v,
                from_=0,
                to=999999,
                width=8,
                command=self.render,
            )
            sb.pack(side=tk.LEFT)

        # ── 換算メモ ──────────────────────────────────────────────
        note_frame = ttk.LabelFrame(right, text="換算メモ")
        note_frame.pack(fill=tk.X, pady=(4, 0))

        notes = (
            "baroAlt  = altitudeMeters × 10\n"
            "  → 表示値 ÷ 10 = 実際の高度 [m]\n"
            "airspeed = Modbus レジスタ値 (m/s × 10)\n"
            "  → 50 = 5.0 m/s, 73 = 7.3 m/s\n"
            "緑弧: 5.5～8.0 m/s（定常飛行域）"
        )
        tk.Label(
            note_frame,
            text=notes,
            justify=tk.LEFT,
            font=("Consolas", 8),
            fg="#444",
        ).pack(padx=6, pady=4, anchor="w")

    def _make_slider_row(self, parent, label, var: tk.IntVar, lo: int, hi: int):
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, padx=10, pady=2)
        ttk.Label(row, text=label, width=38, anchor="w").pack(side=tk.LEFT)
        ttk.Scale(
            row,
            from_=lo,
            to=hi,
            variable=var,
            orient=tk.HORIZONTAL,
            length=200,
            command=lambda val, v=var: (v.set(int(float(val))), self.render()),
        ).pack(side=tk.LEFT, padx=4)
        ttk.Label(row, textvariable=var, width=6).pack(side=tk.LEFT)

    # ── TFT 描画ヘルパー ─────────────────────────────────────────────
    def _sx(self, x: float) -> int:
        return int(x * SCALE)

    def _sy(self, y: float) -> int:
        return int(y * SCALE)

    def _draw_string(self, text: str, x: int, y: int, color: str = TFT_WHITE):
        """TFT_eSPI の drawString(text, x, y, 2) に対応"""
        self.cv.create_text(
            self._sx(x),
            self._sy(y),
            text=text,
            fill=color,
            font=FONT2,
            anchor="nw",
        )

    def _draw_arc_band(self, start_speed: float, end_speed: float, color: str):
        # 台形ポリゴンで塗りつぶすことで隙間を完全になくす
        a = speed_to_angle_deg(start_speed)
        a_end = speed_to_angle_deg(end_speed)
        step = 2.0  # degrees per segment
        while a < a_end:
            a_next = min(a + step, a_end)
            points = [
                self._sx(gx(a,      ARC_INNER_R)), self._sy(gy(a,      ARC_INNER_R)),
                self._sx(gx(a,      ARC_OUTER_R)), self._sy(gy(a,      ARC_OUTER_R)),
                self._sx(gx(a_next, ARC_OUTER_R)), self._sy(gy(a_next, ARC_OUTER_R)),
                self._sx(gx(a_next, ARC_INNER_R)), self._sy(gy(a_next, ARC_INNER_R)),
            ]
            self.cv.create_polygon(points, fill=color, outline=color)
            a = a_next

    def _draw_needle(self, speed: float, color: str):
        clamped = max(0.0, min(GAUGE_MAX_SPEED, speed))
        rad = math.radians(speed_to_angle_deg(clamped))
        tx = GAUGE_CX + NEEDLE_LEN * math.sin(rad)
        ty = GAUGE_CY - NEEDLE_LEN * math.cos(rad)
        perp = rad + math.pi / 2
        bx1 = GAUGE_CX + 3 * math.sin(perp)
        by1 = GAUGE_CY - 3 * math.cos(perp)
        bx2 = GAUGE_CX - 3 * math.sin(perp)
        by2 = GAUGE_CY + 3 * math.cos(perp)
        self.cv.create_polygon(
            self._sx(tx), self._sy(ty),
            self._sx(bx1), self._sy(by1),
            self._sx(bx2), self._sy(by2),
            fill=color, outline=color,
        )

    def _draw_gauge_frame(self):
        cv = self.cv

        # 外周円（2重線）
        cx, cy, r = GAUGE_CX, GAUGE_CY, GAUGE_OUTER_R
        cv.create_oval(
            self._sx(cx - r), self._sy(cy - r),
            self._sx(cx + r), self._sy(cy + r),
            outline=TFT_WHITE, width=2,
        )
        cv.create_oval(
            self._sx(cx - r + 1), self._sy(cy - r + 1),
            self._sx(cx + r - 1), self._sy(cy + r - 1),
            outline=TFT_WHITE, width=1,
        )

        # 弧帯: 赤 2-5.5 / 緑 5.5-8（定常飛行域）/ 赤 8-10
        self._draw_arc_band(2.0, 5.5, TFT_RED)
        self._draw_arc_band(5.5, 8.0, TFT_GREEN)
        self._draw_arc_band(8.0, 10.0, TFT_RED)

        # 主目盛り + 数字（0〜10）
        for i in range(11):
            angle = speed_to_angle_deg(float(i))
            x1, y1 = gx(angle, TICK_OUTER_R), gy(angle, TICK_OUTER_R)
            x2, y2 = gx(angle, TICK_MAJOR_INNER_R), gy(angle, TICK_MAJOR_INNER_R)
            cv.create_line(
                self._sx(x1), self._sy(y1),
                self._sx(x2), self._sy(y2),
                fill=TFT_WHITE, width=SCALE,
            )
            nx, ny = gx(angle, NUMBER_R), gy(angle, NUMBER_R)
            cv.create_text(
                self._sx(nx), self._sy(ny - 11),
                text=str(i), fill=TFT_WHITE, font=FONT_GAUGE,
            )

        # 副目盛り（0.5 m/s 刻み、奇数ステップのみ）
        for j in range(1, 20, 2):
            angle = speed_to_angle_deg(j * 0.5)
            x1, y1 = gx(angle, TICK_OUTER_R), gy(angle, TICK_OUTER_R)
            x2, y2 = gx(angle, TICK_MINOR_INNER_R), gy(angle, TICK_MINOR_INNER_R)
            cv.create_line(
                self._sx(x1), self._sy(y1),
                self._sx(x2), self._sy(y2),
                fill=TFT_WHITE, width=SCALE,
            )

        # 中心ポイント
        r5 = 5
        cv.create_oval(
            self._sx(cx - r5), self._sy(cy - r5),
            self._sx(cx + r5), self._sy(cy + r5),
            fill=TFT_WHITE, outline=TFT_WHITE,
        )

    # ── メインレンダリング（display_d1.ino の renderDisplay() と同じ） ──
    def render(self, *_):
        cv = self.cv
        cv.delete("all")

        # fillScreen(kDisplayBackground)
        cv.create_rectangle(
            0, 0, TFT_W * SCALE, TFT_H * SCALE,
            fill=BG_COLOR, outline="",
        )

        # センサ値を取得
        airspeed_raw = self._vars["airspeed"].get()
        baro_alt     = self._vars["baro_alt"].get()
        pot1         = self._vars["pot1"].get()
        pot2         = self._vars["pot2"].get()
        battery      = self._vars["battery"].get()
        ultrasonic   = self._vars["ultrasonic"].get()
        i2c_ok       = self._i2c_ok.get()
        ok_cnt       = self._vars["ok_cnt"].get()
        err_cnt      = self._vars["err_cnt"].get()

        # Modbus レジスタ値 → m/s（小数点以下1桁）
        current_speed = airspeed_raw / 10.0

        # ── ゲージフレーム描画 ──
        self._draw_gauge_frame()

        # ── ニードル ──
        self._draw_needle(current_speed, TFT_RED)

        # ── 中央デジタル速度（Arial_Black56 相当: "9.9" 形式）──
        whole = int(current_speed)
        frac  = int(current_speed * 10) % 10
        speed_str = f"{whole}.{frac}"
        cv.create_text(
            self._sx(GAUGE_CX), self._sy(GAUGE_CY - 28),
            text=speed_str,
            fill=TFT_WHITE,
            font=FONT_SPEED,
        )

        # ── 下部情報パネル（Arduino コードと同じ文字列・座標）──
        y = 250
        line = f"B:{baro_alt:<5} Bt:{battery:<5}"
        self._draw_string(line, 4, y)
        y += 18

        line = f"P1:{pot1:<4} P2:{pot2:<4} U:{ultrasonic:<4}"
        self._draw_string(line, 4, y)
        y += 18

        i2c_color = TFT_GREEN if i2c_ok else TFT_RED
        i2c_label = "OK" if i2c_ok else "ERR"
        line = f"I2C:{i2c_label:<3} OK:{ok_cnt:<5} E:{err_cnt:<5}"
        self._draw_string(line, 4, y, color=i2c_color)


if __name__ == "__main__":
    TFTPreviewer().mainloop()
