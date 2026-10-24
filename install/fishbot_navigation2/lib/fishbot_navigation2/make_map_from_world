#!/usr/bin/env python3
"""把 Gazebo 世界文件（.sdf）直接渲染成 Nav2 用的占据栅格地图。

为什么需要它
    SLAM 建图会受里程计漂移影响，出现重影/墙体错位；而世界文件里每堵墙、每个
    障碍物的坐标都是精确已知的 —— 直接按几何渲染出来的是一张"真值地图"，
    干净、无重影，Nav2 拿它做 AMCL 定位 + 全局规划完全够用。

用法（在工作空间根目录执行，$WS = 你的工作空间）
    python3 src/fishbot_navigation2/scripts/make_map_from_world.py \
        src/fishbot_description/world/fishbot.sdf \
        src/fishbot_navigation2/maps/room

产出
    <输出前缀>.pgm   占据栅格图像（0=障碍 205=未知 254=空闲）
    <输出前缀>.yaml  Nav2 地图元数据

同时会做两件检查：
    1) 各房间采样点是否落在空闲区
    2) 从出生点(0,0)做洪泛填充，报告能否走到每个房间（门洞没被堵死）
"""
import math
import pathlib
import sys
import xml.etree.ElementTree as ET
from collections import deque

FREE, OCC, UNK = 254, 0, 205

# 户型里各房间的内部采样点（世界坐标），用于验证连通性
ROOM_PROBES = {
    '左上房间': (-3.0, 1.8),
    '左下房间': (-4.0, -2.0),
    '右下房间': (2.5, -2.0),
    '大厅': (0.0, 0.0),
}


def parse_obstacles(sdf_path):
    """从 SDF 里抽出所有静态障碍物的世界坐标包围盒 [(x0,y0,x1,y1), ...]（跳过地面 plane）。"""
    root = ET.parse(sdf_path).getroot()
    boxes = []
    for model in root.iter('model'):
        pose_el = model.find('pose')
        px = py = yaw = 0.0
        if pose_el is not None and pose_el.text:
            vals = [float(v) for v in pose_el.text.split()]
            if len(vals) >= 2:
                px, py = vals[0], vals[1]
            if len(vals) >= 6:
                yaw = vals[5]
        for link in model.iter('link'):
            lpose = link.find('pose')
            lx = ly = lyaw = 0.0
            if lpose is not None and lpose.text:
                vals = [float(v) for v in lpose.text.split()]
                if len(vals) >= 2:
                    lx, ly = vals[0], vals[1]
                if len(vals) >= 6:
                    lyaw = vals[5]
            cx, cy = px + lx, py + ly
            total_yaw = yaw + lyaw
            for geom in link.iter('geometry'):
                box = geom.find('box')
                cyl = geom.find('cylinder')
                if box is not None:
                    size = [float(v) for v in box.find('size').text.split()]
                    hx, hy = size[0] / 2.0, size[1] / 2.0
                    # 有 yaw 就按旋转后的外接框算（本项目 yaw 都是 0，这条是保险）
                    ext_x = abs(hx * math.cos(total_yaw)) + abs(hy * math.sin(total_yaw))
                    ext_y = abs(hx * math.sin(total_yaw)) + abs(hy * math.cos(total_yaw))
                    boxes.append((cx - ext_x, cy - ext_y, cx + ext_x, cy + ext_y))
                elif cyl is not None:
                    r = float(cyl.find('radius').text)
                    boxes.append((cx - r, cy - r, cx + r, cy + r))
                # plane（地面）忽略
    return boxes


def render(boxes, res, margin):
    """渲染成占据栅格。返回 (data, width, height, origin_x, origin_y)。"""
    xs = [b[0] for b in boxes] + [b[2] for b in boxes]
    ys = [b[1] for b in boxes] + [b[3] for b in boxes]
    x0 = math.floor((min(xs) - margin) / res) * res
    y0 = math.floor((min(ys) - margin) / res) * res
    x1 = math.ceil((max(xs) + margin) / res) * res
    y1 = math.ceil((max(ys) + margin) / res) * res
    width = int(round((x1 - x0) / res))
    height = int(round((y1 - y0) / res))
    data = [UNK] * (width * height)

    def cell_of(x, y):
        return int((x - x0) / res), int((y - y0) / res)

    # ① 先标记障碍（所有墙和家具的包围盒）
    for bx0, by0, bx1, by1 in boxes:
        c0, r0 = cell_of(bx0, by0)
        c1, r1 = cell_of(bx1, by1)
        for r in range(max(0, r0), min(height, r1 + 1)):
            base = r * width
            for c in range(max(0, c0), min(width, c1 + 1)):
                data[base + c] = OCC

    # ② 外墙内部填成空闲（外轮廓 = 最外圈的墙包围盒）
    outer_x0 = min(b[0] for b in boxes)
    outer_y0 = min(b[1] for b in boxes)
    outer_x1 = max(b[2] for b in boxes)
    outer_y1 = max(b[3] for b in boxes)
    c0, r0 = cell_of(outer_x0, outer_y0)
    c1, r1 = cell_of(outer_x1, outer_y1)
    for r in range(max(0, r0 + 1), min(height, r1)):
        base = r * width
        y = y0 + (r + 0.5) * res
        if not (outer_y0 < y < outer_y1):
            continue
        for c in range(max(0, c0 + 1), min(width, c1)):
            x = x0 + (c + 0.5) * res
            if outer_x0 < x < outer_x1 and data[base + c] == UNK:
                data[base + c] = FREE
    return data, width, height, x0, y0


def flood_fill(data, width, height, origin_x, origin_y, res, seed):
    """从 seed 世界坐标做洪泛，返回可达的空闲格集合。"""
    c = int((seed[0] - origin_x) / res)
    r = int((seed[1] - origin_y) / res)
    if not (0 <= c < width and 0 <= r < height) or data[r * width + c] != FREE:
        return set()
    seen = {(c, r)}
    q = deque([(c, r)])
    while q:
        cc, rr = q.popleft()
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nc, nr = cc + dc, rr + dr
            if 0 <= nc < width and 0 <= nr < height and (nc, nr) not in seen:
                if data[nr * width + nc] == FREE:
                    seen.add((nc, nr))
                    q.append((nc, nr))
    return seen


def write_pgm(path, data, width, height):
    """PGM(P5)：第一行对应世界 y 最大处，所以行要倒序写。"""
    with open(path, 'wb') as fh:
        fh.write(b'P5\n%d %d\n255\n' % (width, height))
        for r in range(height - 1, -1, -1):
            fh.write(bytes(data[r * width:(r + 1) * width]))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    sdf_path = pathlib.Path(sys.argv[1]).expanduser()
    out_prefix = pathlib.Path(sys.argv[2]).expanduser()
    res = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    margin = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0

    boxes = parse_obstacles(sdf_path)
    print(f'从世界文件读到 {len(boxes)} 个障碍物/墙体包围盒')
    data, width, height, ox, oy = render(boxes, res, margin)
    total = width * height
    free = data.count(FREE)
    occ = data.count(OCC)
    unk = data.count(UNK)
    print(f'栅格 {width} x {height} @ {res} m  origin=({ox:.2f}, {oy:.2f})')
    print(f'空闲 {free*100/total:.1f}%   障碍 {occ*100/total:.1f}%   未知 {unk*100/total:.1f}%')

    # 连通性检查：从出生点(0,0)能不能走到每个房间
    reach = flood_fill(data, width, height, ox, oy, res, (0.0, 0.0))
    print(f'\n从出生点(0,0)洪泛可达空闲格: {len(reach)} 个')
    all_ok = True
    for name, (x, y) in ROOM_PROBES.items():
        c = int((x - ox) / res)
        r = int((y - oy) / res)
        in_free = data[r * width + c] == FREE
        ok = (c, r) in reach
        all_ok &= ok
        tag = '✅ 可达' if ok else ('⚠️ 在该点是空闲但走不到' if in_free else '❌ 不在空闲区')
        print(f'  {name:8s} ({x:>5.1f},{y:>5.1f})  {tag}')

    pgm = out_prefix.with_suffix('.pgm')
    yml = out_prefix.with_suffix('.yaml')
    write_pgm(pgm, data, width, height)
    yml.write_text(
        f'image: {pgm.name}\n'
        f'mode: trinary\n'
        f'resolution: {res}\n'
        f'origin: [{ox:.3f}, {oy:.3f}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.196\n', encoding='utf-8')
    print(f'\n已写出:\n  {pgm}\n  {yml}')
    print('\n结论:', '✅ 地图可用（所有房间都从大厅连通）' if all_ok else '⚠️ 有房间走不到，检查门洞是否被堵')
    return 0


if __name__ == '__main__':
    sys.exit(main())
