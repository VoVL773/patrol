#!/usr/bin/env python3
"""把 maps/room.pgm 渲染成 README 用的预览图（带巡检航点标注）。

用法（在工作空间根目录执行）：
    python3 docs/make_map_preview.py

产出：docs/map_preview.png

说明：这里画的是【航点顺序示意】，不是实际跑出来的轨迹 ——
      真要画轨迹，用 `ros2 bag record /amcl_pose` 或 Nav2 的
      `nav2_route`/`patrol_inspector` 输出的 CSV 再画。
"""
from __future__ import annotations

import os

import yaml
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_YAML = os.path.join(ROOT, "src", "fishbot_navigation2", "maps", "room.yaml")
WAYPOINTS = os.path.join(ROOT, "src", "fishbot_navigation2", "param",
                         "patrol_waypoints.yaml")
OUT = os.path.join(ROOT, "docs", "map_preview.png")

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
SCALE = 3
COLOR = {
    0: (28, 34, 45),        # 障碍
    205: (150, 161, 175),   # 未知
    254: (247, 249, 252),   # 空闲
}
ACCENT = (31, 78, 121)
DEFAULT_COLOR = (200, 200, 200)


def main() -> None:
    meta = yaml.safe_load(open(MAP_YAML, encoding="utf-8"))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

    img = Image.open(os.path.join(os.path.dirname(MAP_YAML), meta["image"]))
    img = img.convert("L")
    w, h = img.size

    # 上色
    rgb = Image.new("RGB", (w, h))
    px = img.load()
    out = rgb.load()
    for y in range(h):
        for x in range(w):
            out[x, y] = COLOR.get(px[x, y], DEFAULT_COLOR)
    # 裁到"有内容的范围"（墙 + 空闲区），去掉外面那圈未知区域
    bbox = img.point(lambda v: 255 if v in (0, 254) else 0).getbbox()
    if bbox:
        pad = 8
        left, top, right, bottom = bbox
        img = img.crop((max(0, left - pad), max(0, top - pad),
                        min(w, right + pad), min(h, bottom + pad)))
        w, h = img.size
        rgb = Image.new("RGB", (w, h))
        px = img.load()
        out = rgb.load()
        for y in range(h):
            for x in range(w):
                out[x, y] = COLOR.get(px[x, y], DEFAULT_COLOR)

    rgb = rgb.resize((w * SCALE, h * SCALE), Image.NEAREST)

    draw = ImageDraw.Draw(rgb)
    font = ImageFont.truetype(FONT, 22)
    font_small = ImageFont.truetype(FONT, 18)

    def to_px(x: float, y: float) -> tuple[float, float]:
        """世界坐标 → 图像坐标（图像 y 轴向下，地图 y 轴向上）"""
        return ((x - ox) / res * SCALE, (h - (y - oy) / res) * SCALE)

    wps = yaml.safe_load(open(WAYPOINTS, encoding="utf-8"))["patrol"]["waypoints"]
    pts = [to_px(p["x"], p["y"]) for p in wps]

    # 航点之间的虚线（示意顺序）
    for a, b in zip(pts, pts[1:]):
        n = max(2, int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) / 14))
        for i in range(n):
            if i % 2:
                continue
            t0, t1 = i / n, (i + 1) / n
            draw.line([(a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0),
                       (a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1)],
                      fill=(120, 150, 180), width=2)

    # 航点圆点 + 编号
    for i, ((cx, cy), wp) in enumerate(zip(pts, wps), start=1):
        r = 13
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(255, 255, 255),
                     outline=ACCENT, width=3)
        draw.text((cx, cy), str(i), font=font, fill=ACCENT, anchor="mm")
        # 标签放在点的左上/右上，靠近边界时翻转，避免被裁掉
        name = wp["name"]
        tw = draw.textlength(name, font=font_small)
        flip = cx + 20 + tw > rgb.width - 6
        tx = cx - 18 if flip else cx + 18
        anchor = "ra" if flip else "la"
        ty = cy - 24
        tb = draw.textbbox((tx, ty), name, font=font_small, anchor=anchor)
        draw.rectangle([tb[0] - 4, tb[1] - 2, tb[2] + 4, tb[3] + 2],
                       fill=(255, 255, 255, 230))
        draw.text((tx, ty), name, font=font_small, fill=(70, 80, 94), anchor=anchor)

    # 右下角标注
    def chip(text: str, xy: tuple[float, float], anchor: str) -> None:
        """带白底的小标注，避免压在深色区域上看不清"""
        tb = draw.textbbox(xy, text, font=font_small, anchor=anchor)
        draw.rectangle([tb[0] - 7, tb[1] - 4, tb[2] + 7, tb[3] + 4],
                       fill=(255, 255, 255))
        draw.text(xy, text, font=font_small, fill=(90, 100, 116), anchor=anchor)

    chip("虚线 = 航点顺序示意，非实际路径", (12, rgb.height - 12), "ls")
    chip(f"{w * res:.1f} m × {h * res:.1f} m @ {res} m/px",
         (rgb.width - 12, rgb.height - 12), "rs")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    rgb.save(OUT)
    print(f"已生成 {OUT}  ({rgb.width}×{rgb.height})")


if __name__ == "__main__":
    main()
