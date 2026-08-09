#!/usr/bin/env python3
"""
ABU Robocon 2027 场地 URDF 解析与可视化工具。

用法:
    python3 field_map.py                     # 打印参数表 + 生成场地俯视图
    python3 field_map.py --text-only         # 仅打印参数表
    python3 field_map.py --plot-only         # 仅生成俯视图

不依赖 ROS2，仅需 Python3 + matplotlib。
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


# ── 数据模型 ──────────────────────────────────────────────

@dataclass
class Link:
    name: str
    geometry_type: str       # box / cylinder / sphere
    size: Tuple[float, ...]  # (sx, sy, sz) for box; (r, h) for cylinder; (r,) for sphere
    material: str
    color_rgba: str          # raw rgba from URDF
    parent: str
    xyz: Tuple[float, float, float]   # absolute position in world frame
    rpy: Tuple[float, float, float]   # rotation


# 材料名 → RGB 对应表 (来自 RGBA * 255)
MATERIAL_RGB = {
    "红方地面":   "#F0D2D2",
    "蓝方地面":   "#AAD2E6",
    "共享区米黄": "#F5F0C8",
    "启动重试红": "#DF2222",
    "启动重试蓝": "#3200FF",
    "柱身棕":     "#643E00",
    "红方台面橙粉": "#EBB4A0",
    "蓝方台面青蓝": "#96D7DC",
    "红方斜坡":   "#C8968C",
    "蓝方斜坡":   "#82B4C8",
    "建塔位绿":   "#286432",
    "红方交接橙": "#F5AA3C",
    "蓝方交接蓝": "#3CAAF5",
    "护栏灰":     "#BEBEB9",
    "秘宝金":     "#DAA520",
}

# 区域分组规则
GROUP_RULES = [
    ("地面 — 红方/蓝方地板",        lambda n: n in ("red_ground", "blue_ground")),
    ("地面 — 场地边界木框",          lambda n: n.startswith("frame_")),
    ("地面 — 中央分隔栏",            lambda n: n.startswith("divider_")),
    ("地面 — 启动/重试区",           lambda n: n.startswith("start_")),
    ("地面 — 储存区",                lambda n: n.startswith("storage_")),
    ("地面 — 秘宝柱",                lambda n: n.startswith("mustika_")),
    ("地面 — 天空块区域",            lambda n: n.startswith("sky_")),
    ("L1 — 主体/台面/护栏",          lambda n: n.startswith("l1_")),
    ("L1 — 红方通道 (交接/斜坡/阶梯)", lambda n: n.startswith("transfer_red") or n.startswith("red_")),
    ("L1 — 蓝方通道 (交接/斜坡/阶梯)", lambda n: n.startswith("transfer_blue") or n.startswith("blue_")),
    ("L2 — 主体/台面/护栏",          lambda n: n.startswith("l2_")),
    ("L2 — 中央柱/盘/球",            lambda n: n.startswith("central_")),
]


def parse_rgba_to_hex(rgba_str: str) -> str:
    """将 URDF rgba 字符串转为 #RRGGBB 颜色。"""
    vals = [float(x) for x in rgba_str.split()]
    r, g, b = int(vals[0] * 255), int(vals[1] * 255), int(vals[2] * 255)
    return f"#{r:02X}{g:02X}{b:02X}"


def parse_urdf(filepath: str) -> List[Link]:
    """解析 URDF，返回所有 link 的列表（含绝对坐标）。"""
    tree = ET.parse(filepath)
    root = tree.getroot()

    # 第一步：收集所有 material 定义
    materials: Dict[str, str] = {}  # name → rgba
    for mat in root.iter("material"):
        name = mat.get("name", "")
        color = mat.find("color")
        if name and color is not None:
            rgba = color.get("rgba", "1 1 1 1")
            materials[name] = rgba

    # 第二步：解析 links
    links_raw: Dict[str, dict] = {}
    for link in root.iter("link"):
        name = link.get("name", "")
        visual = link.find("visual")
        if visual is None:
            continue
        geom = visual.find("geometry")
        mat = visual.find("material")
        mat_name = mat.get("name", "") if mat is not None else ""
        color_elem = mat.find("color") if mat is not None else None
        rgba = color_elem.get("rgba", "1 1 1 1") if color_elem is not None else materials.get(mat_name, "1 1 1 1")

        geom_type = ""
        size: Tuple[float, ...] = ()
        if geom is not None:
            box = geom.find("box")
            cyl = geom.find("cylinder")
            sph = geom.find("sphere")
            if box is not None:
                geom_type = "box"
                size = tuple(float(x) for x in box.get("size", "0 0 0").split())
            elif cyl is not None:
                geom_type = "cylinder"
                r = float(cyl.get("radius", 0))
                h = float(cyl.get("length", 0))
                size = (r, h)
            elif sph is not None:
                geom_type = "sphere"
                size = (float(sph.get("radius", 0)),)

        links_raw[name] = {
            "geometry_type": geom_type,
            "size": size,
            "material": mat_name,
            "rgba": rgba,
            "parent": "",
            "xyz": (0.0, 0.0, 0.0),
            "rpy": (0.0, 0.0, 0.0),
        }

    # 第三步：解析 joints，建立父子关系
    joint_map: Dict[str, Tuple[str, Tuple[float, float, float], Tuple[float, float, float]]] = {}
    for joint in root.iter("joint"):
        child = joint.find("child")
        parent = joint.find("parent")
        origin = joint.find("origin")
        if child is None or parent is None:
            continue
        child_name = child.get("link", "")
        parent_name = parent.get("link", "")
        xyz_str = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
        rpy_str = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
        xyz = tuple(float(x) for x in xyz_str.split())
        rpy = tuple(float(x) for x in rpy_str.split())
        joint_map[child_name] = (parent_name, xyz, rpy)

    # 第四步：计算每个 link 的绝对坐标（从 base_link 开始递归）
    def compute_absolute(link_name: str) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
        if link_name == "base_link" or link_name not in joint_map:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        parent, local_xyz, local_rpy = joint_map[link_name]
        parent_xyz, parent_rpy = compute_absolute(parent)
        # 简化：不考虑旋转叠加（场地 URDF 中只有极少数带 rpy，且都是简单绕单轴）
        abs_xyz = (parent_xyz[0] + local_xyz[0],
                   parent_xyz[1] + local_xyz[1],
                   parent_xyz[2] + local_xyz[2])
        abs_rpy = (parent_rpy[0] + local_rpy[0],
                   parent_rpy[1] + local_rpy[1],
                   parent_rpy[2] + local_rpy[2])
        return (abs_xyz, abs_rpy)

    links: List[Link] = []
    for name, info in links_raw.items():
        parent, local_xyz, local_rpy = joint_map.get(name, ("base_link", (0, 0, 0), (0, 0, 0)))
        abs_xyz, abs_rpy = compute_absolute(name)
        links.append(Link(
            name=name,
            geometry_type=info["geometry_type"],
            size=info["size"],
            material=info["material"],
            color_rgba=info["rgba"],
            parent=parent,
            xyz=abs_xyz,
            rpy=abs_rpy,
        ))

    return links


def group_links(links: List[Link]) -> List[Tuple[str, List[Link]]]:
    """将 links 按语义分组。"""
    assigned = set()
    groups: List[Tuple[str, List[Link]]] = []
    for group_name, predicate in GROUP_RULES:
        members = [l for l in links if predicate(l.name) and l.name not in assigned]
        if members:
            groups.append((group_name, members))
            assigned.update(l.name for l in members)
    # 未匹配的放入"其他"
    others = [l for l in links if l.name not in assigned]
    if others:
        groups.append(("其他", others))
    return groups


def print_table(links: List[Link]):
    """打印结构化参数表。"""
    groups = group_links(links)
    print("\n" + "=" * 110)
    print("  ABU Robocon 2027 场地 URDF 参数总览")
    print("  坐标系: 原点=场地正中心 | X-红方 / X+蓝方 / Z+向上")
    print("=" * 110)

    for group_name, members in groups:
        print(f"\n{'─' * 100}")
        print(f"  【{group_name}】  ({len(members)} 个构件)")
        print(f"{'─' * 100}")
        header = f"  {'构件名':<32s} {'几何':8s} {'尺寸':22s} {'中心位置 (x,y,z)':28s} {'旋转 rpy':18s} {'颜色':10s}"
        print(header)
        print("  " + "-" * 105)

        for l in sorted(members, key=lambda x: (round(x.xyz[0], 5), round(x.xyz[1], 5), round(x.xyz[2], 5))):
            if l.geometry_type == "box":
                size_str = f"{l.size[0]:.3f} × {l.size[1]:.3f} × {l.size[2]:.3f}"
            elif l.geometry_type == "cylinder":
                size_str = f"r={l.size[0]:.3f} h={l.size[1]:.3f}"
            elif l.geometry_type == "sphere":
                size_str = f"r={l.size[0]:.3f}"
            else:
                size_str = str(l.size)

            pos_str = f"({l.xyz[0]:+.3f}, {l.xyz[1]:+.3f}, {l.xyz[2]:+.3f})"
            rpy_str = f"({math.degrees(l.rpy[0]):.1f}°, {math.degrees(l.rpy[1]):.1f}°, {math.degrees(l.rpy[2]):.1f}°)"
            color_hex = parse_rgba_to_hex(l.color_rgba) if l.color_rgba else "#??????"

            print(f"  {l.name:<32s} {l.geometry_type:8s} {size_str:22s} {pos_str:28s} {rpy_str:18s} {color_hex:10s}")

    print(f"\n{'=' * 110}")
    print(f"  总计: {len(links)} 个 link + {len(links)} 个 joint")
    print(f"{'=' * 110}\n")


def plot_topdown(links: List[Link], output_path: str):
    """生成场地俯视图 (XY 平面) 和 侧视图 (XZ 平面)。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.patches import FancyBboxPatch, Circle, Rectangle

    fig, (ax_xy, ax_xz) = plt.subplots(1, 2, figsize=(22, 10))
    fig.suptitle("ABU Robocon 2027 场地 URDF 几何总览", fontsize=14, fontweight="bold", y=0.98)

    # 收集图例
    legend_handles: Dict[str, mpatches.Patch] = {}

    for link in links:
        if link.geometry_type not in ("box", "cylinder"):
            continue
        color_hex = parse_rgba_to_hex(link.color_rgba) if link.color_rgba else "#888888"
        alpha = 0.65
        edge = "black"
        lw = 0.4

        cx, cy, cz = link.xyz
        rx, ry, rz = link.rpy

        if link.geometry_type == "cylinder":
            r, h = link.size
            w, d, h_vis = r * 2, r * 2, h
        else:
            w, d, h_vis = link.size

        # ── 俯视图 (XY) ──
        rect = Rectangle(
            (cx - w / 2, cy - d / 2), w, d,
            linewidth=lw, edgecolor=edge, facecolor=color_hex, alpha=alpha,
        )
        ax_xy.add_patch(rect)

        # 标注（仅大于 0.3m 的物体）
        if max(w, d) > 0.3:
            label = link.name.replace("_", "\n").replace("red", "R").replace("blue", "B")
            ax_xy.annotate(
                link.name, (cx, cy), textcoords="offset points",
                xytext=(0, 3), fontsize=3.5, ha="center", va="bottom",
                color="#333333",
            )

        # ── 侧视图 (XZ) ──
        rect_xz = Rectangle(
            (cx - w / 2, cz - h_vis / 2), w, h_vis,
            linewidth=lw, edgecolor=edge, facecolor=color_hex, alpha=alpha,
        )
        ax_xz.add_patch(rect_xz)

        # 图例
        mat_name = link.material or "未命名"
        if mat_name not in legend_handles:
            legend_handles[mat_name] = mpatches.Patch(color=color_hex, alpha=alpha, label=mat_name)

    # ── 俯视图设置 ──
    ax_xy.set_title("俯视图 (XY) — X-红方 / X+蓝方", fontsize=11)
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_xlim(-6.5, 6.5)
    ax_xy.set_ylim(-6.5, 6.5)
    ax_xy.set_aspect("equal")
    ax_xy.grid(True, alpha=0.3, linestyle="--")
    # 画出场地中心十字
    ax_xy.axhline(0, color="black", linewidth=0.5, alpha=0.5)
    ax_xy.axvline(0, color="black", linewidth=0.5, alpha=0.5)
    # 标注红方/蓝方
    ax_xy.text(-3.0, 6.2, "红方 RED", fontsize=11, ha="center", color="#CC3333", fontweight="bold")
    ax_xy.text(3.0, 6.2, "蓝方 BLUE", fontsize=11, ha="center", color="#3355CC", fontweight="bold")

    # ── 侧视图设置 ──
    ax_xz.set_title("侧视图 (XZ) — Z+向上", fontsize=11)
    ax_xz.set_xlabel("X (m)")
    ax_xz.set_ylabel("Z (m)")
    ax_xz.set_xlim(-6.5, 6.5)
    ax_xz.set_ylim(-0.1, 2.3)
    ax_xz.grid(True, alpha=0.3, linestyle="--")
    ax_xz.axhline(0, color="saddlebrown", linewidth=1.5, alpha=0.4, label="地面")
    ax_xz.axhline(0.6, color="gray", linewidth=0.8, alpha=0.4, linestyle="--", label="L1 标高 0.6m")
    ax_xz.axhline(0.9, color="gray", linewidth=0.8, alpha=0.4, linestyle="--", label="L2 标高 0.9m")
    ax_xz.legend(loc="upper right", fontsize=7)

    # ── 图例 ──
    fig.legend(
        handles=list(legend_handles.values()),
        loc="lower center",
        ncol=8,
        fontsize=7,
        frameon=False,
        bbox_to_anchor=(0.5, 0.01),
    )

    plt.tight_layout(rect=[0, 0.06, 1, 0.94])
    plt.savefig(output_path, dpi=200, bbox_inches="tight")
    print(f"\n场地俯视图已保存: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="ABU Robocon 2027 场地 URDF 解析与可视化")
    parser.add_argument("--urdf", default=None, help="URDF 文件路径（默认自动查找）")
    parser.add_argument("--text-only", action="store_true", help="仅打印参数表")
    parser.add_argument("--plot-only", action="store_true", help="仅生成俯视图")
    parser.add_argument("--output", "-o", default=None, help="图片输出路径（默认与 URDF 同目录）")
    args = parser.parse_args()

    # 自动查找 URDF
    if args.urdf:
        urdf_path = args.urdf
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(script_dir, "..", "urdf", "field.urdf")
    urdf_path = os.path.abspath(urdf_path)

    if not os.path.exists(urdf_path):
        print(f"错误: 找不到 URDF 文件: {urdf_path}")
        return 1

    print(f"解析 URDF: {urdf_path}")
    links = parse_urdf(urdf_path)

    if not args.plot_only:
        print_table(links)

    if not args.text_only:
        if args.output:
            plot_path = args.output
        else:
            plot_path = os.path.splitext(urdf_path)[0] + "_map.png"
        plot_topdown(links, plot_path)

    return 0


if __name__ == "__main__":
    exit(main())
