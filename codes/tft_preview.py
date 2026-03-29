#!/usr/bin/env python3
"""
TFT Display D1 プレビューツール
================================
display_d1.ino の renderDisplay() を PC 上で再現します。
スライダで各センサ値を変えるとリアルタイムに画面が更新されます。

依存ライブラリ: tkinter（Python 標準搭載）のみ
使い方: python tft_preview.py
"""

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

# TFT Font 2 の PC 近似フォント（Courier を使用）
# SCALE=2 のとき 16px フォント高 × 2 = 32px 相当 → 18pt でほぼ一致
FONT2 = ("Courier", 10 + SCALE * 4, "bold")


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

        # (変数キー, ラベル, 最小, 最大, 初期値)
        sliders = [
            ("airspeed",   "対気速度  [raw ADC 0-1023]",  0, 1023, 512),
            ("baro_alt",   "気圧高度  [×0.1 m]          ", 0, 5000, 500),
            ("pot1",       "ポテンショ1 [raw ADC 0-1023]", 0, 1023, 256),
            ("pot2",       "ポテンショ2 [raw ADC 0-1023]", 0, 1023, 256),
            ("battery",    "バッテリ    [raw ADC 0-1023]", 0, 1023, 800),
            ("ultrasonic", "超音波高度  [raw ADC 0-1023]", 0, 1023,   0),
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
            "battery  = analogRead / 1023 × 3.3V\n"
            "  (分圧比に応じて係数を掛ける)\n"
            "airspeed = Modbus 書き込みレジスタ値\n"
            "  (logger → display_d1 に送信)"
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
        ttk.Label(row, text=label, width=28, anchor="w").pack(side=tk.LEFT)
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

    # ── TFT 描画ヘルパー（TFT_eSPI API を近似） ──────────────────────
    def _sx(self, x: int) -> int:
        return x * SCALE

    def _sy(self, y: int) -> int:
        return y * SCALE

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

    def _draw_centre_string(self, text: str, cx: int, y: int, color: str = TFT_WHITE):
        """TFT_eSPI の drawCentreString(text, cx, y, 2) に対応"""
        self.cv.create_text(
            self._sx(cx),
            self._sy(y),
            text=text,
            fill=color,
            font=FONT2,
            anchor="n",
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
        airspeed   = self._vars["airspeed"].get()
        baro_alt   = self._vars["baro_alt"].get()
        pot1       = self._vars["pot1"].get()
        pot2       = self._vars["pot2"].get()
        battery    = self._vars["battery"].get()
        ultrasonic = self._vars["ultrasonic"].get()
        i2c_ok     = self._i2c_ok.get()
        ok_cnt     = self._vars["ok_cnt"].get()
        err_cnt    = self._vars["err_cnt"].get()

        # タイトル
        self._draw_centre_string("DISPLAY D1", 120, 8)

        # ── センサ行（Arduino コードと同じ文字列・座標） ──────────
        y = 30
        self._draw_string(f"Airspeed : {airspeed}", 8, y)
        y += 20

        self._draw_string(f"Baro Alt : {baro_alt}", 8, y)
        y += 20

        self._draw_string(f"Pot1/Pot2: {pot1} / {pot2}", 8, y)
        y += 20

        self._draw_string(f"Battery  : {battery}", 8, y)
        y += 20

        self._draw_string(f"Ultrason.: {ultrasonic}", 8, y)
        y += 26

        # I2C ステータス（緑 or 赤）
        i2c_color = TFT_GREEN if i2c_ok else TFT_RED
        i2c_label = "OK" if i2c_ok else "ERROR"
        self._draw_string(f"I2C {i2c_label}", 8, y, color=i2c_color)
        y += 20

        # OK / ERR カウンタ
        self._draw_string(f"OK:{ok_cnt} ERR:{err_cnt}", 8, y)

        # ── 画面下部の空き領域にグリッド線（改善参考用） ─────────────
        y += 30
        cv.create_line(
            self._sx(0), self._sy(y),
            self._sx(TFT_W), self._sy(y),
            fill="#888888", width=1, dash=(4, 4),
        )
        cv.create_text(
            self._sx(4), self._sy(y + 4),
            text="↑ 空き領域（改善可能）",
            fill="#888888",
            font=("Consolas", 8),
            anchor="nw",
        )


if __name__ == "__main__":
    TFTPreviewer().mainloop()
