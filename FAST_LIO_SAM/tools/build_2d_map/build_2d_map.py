#!/usr/bin/env python3
"""
build_2d_map — 从 FAST-LIO-SAM 的 SaveMap 输出一条命令做出 nav2 兼容 PGM/YAML.

串了 3 步:
  1. SaveMap 结果检查    (确认 GlobalMap.pcd / trajectory.pcd 存在)
  2. align_floor         (RANSAC 拟合地面, 旋转+平移使 z=0=地面, z 轴=真重力)
  3. pcd_to_occgrid      (z 切片 + 投影 + 沿轨迹 polar raycast 填 free space)

为什么需要这层封装:
  3 步分开跑容易出错.
    - 漏 align: z=0 是 LIO init 位置, 切 [0.2, 1.5] 切到空中, PGM 空白
    - 顺序错: raycast 用的轨迹和点云不在同一坐标系, free 全错
    - 路径乱: 中间产物到处放
  这个 wrapper 把默认值调好, 一条命令出 PGM, 中间产物可选保留供 debug.

依赖: 已安装 align_floor.py 和 pcd_to_occgrid.py (同 repo, 同级目录).

用法:
    # 最简
    build_2d_map.py ~/Downloads/LOAM/ -o ~/maps/airy_room
    # 开 RViz / nav2 用
    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/maps/airy_room.yaml

    # 跳过 align (点云已经对齐了)
    build_2d_map.py ~/Downloads/LOAM/ -o ~/maps/airy_room --no-align

    # 保留中间产物
    build_2d_map.py ~/Downloads/LOAM/ -o ~/maps/airy_room --keep-intermediate

    # 调切片 (机器狗很矮)
    build_2d_map.py ~/Downloads/LOAM/ -o ~/maps/dog_room --z-min 0.10 --z-max 0.40
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import textwrap
import time
from pathlib import Path


# ============================================================
HERE = Path(__file__).resolve().parent
ALIGN_FLOOR  = HERE.parent / "align_floor" / "align_floor.py"
PCD_TO_OCC   = HERE.parent / "pcd_to_occgrid" / "pcd_to_occgrid.py"
FOOTPRINT_FILTER = HERE.parent / "footprint_filter" / "footprint_filter.py"


def run_step(name: str, cmd: list[str], log_path: Path) -> tuple[int, str, str]:
    """跑一步, 返回 (returncode, stdout, stderr); 进度输出实时打到屏幕."""
    print(f"[{name}] 执行: {' '.join(str(c) for c in cmd)}")
    t0 = time.time()
    proc = subprocess.run(cmd, capture_output=True, text=True)
    dur = time.time() - t0
    log_path.write_text(
        f"# {name} ({dur:.1f}s, exit={proc.returncode})\n"
        f"# cmd: {' '.join(cmd)}\n\n"
        f"=== stdout ===\n{proc.stdout}\n"
        f"=== stderr ===\n{proc.stderr}\n"
    )
    if proc.returncode != 0:
        print(f"[{name}] ❌ 失败 (exit={proc.returncode}, {dur:.1f}s)")
        sys.stderr.write(proc.stdout)
        sys.stderr.write(proc.stderr)
    else:
        print(f"[{name}] ✅ 完成 ({dur:.1f}s)")
        # 把关键统计行抽出来
        for line in proc.stdout.splitlines():
            low = line.lower()
            if any(k in low for k in ["tilt", "inliers", "raycast", "尺寸", "occupied", "free", "[ok]"]):
                print(f"[{name}]   {line}")
    return proc.returncode, proc.stdout, proc.stderr


def find_pcd(loam_dir: Path, name: str) -> Path:
    """SaveMap 默认放 $HOME/Downloads/LOAM/, 找 GlobalMap.pcd 等."""
    p = loam_dir / name
    if p.is_file():
        return p
    raise FileNotFoundError(
        f"未找到 {name} 在 {loam_dir}. SaveMap service 是否调用过? 查 launch.log:\n"
        f"  ros2 service call /save_map fast_lio_sam/srv/SaveMap '{{resolution: 0.0, destination: \"\"}}'"
    )


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("loam_dir", type=Path,
                    help="SaveMap service 输出目录 (含 GlobalMap.pcd / trajectory.pcd)")
    ap.add_argument("-o", "--output", type=Path, required=True,
                    help="输出文件前缀, 例: ~/maps/airy_room (会得到 .pgm + .yaml)")

    g_align = ap.add_argument_group("step 2: align_floor")
    g_align.add_argument("--no-align", action="store_true",
                         help="跳过 gravity-align (点云已对齐时用)")
    g_align.add_argument("--floor-z-range", type=float, nargs=2, default=None,
                         metavar=("Z_MIN", "Z_MAX"),
                         help="手动指定地面 z 区间 (默认: align_floor 自动)")

    g_occ = ap.add_argument_group("step 3: pcd_to_occgrid")
    g_occ.add_argument("--profile", choices=["handheld", "dog", "vehicle", "generic"],
                       default="handheld",
                       help="采集场景预设 (默认 handheld). 决定 z 切片 + footprint 半径:\n"
                            "  handheld : z=[0.2, 1.5]  footprint=0.3m  (人手持采集)\n"
                            "  dog      : z=[0.1, 0.5]  footprint=0.4m  (机器狗高度低 + 体型)\n"
                            "  vehicle  : z=[0.3, 2.0]  footprint=1.0m  (车上 LiDAR)\n"
                            "  generic  : z=[0.1, 1.5]  footprint=0    (不知道用啥, 不排除)\n"
                            "可以单独覆盖个别参数 (--z-min / --footprint-radius 等)")
    g_occ.add_argument("--resolution", type=float, default=0.05,
                       help="栅格分辨率 m/cell (默认 0.05)")
    g_occ.add_argument("--z-min", type=float, default=None,
                       help="障碍切片下界 m. 默认按 profile, 注意是 floor=z=0 之上的距离")
    g_occ.add_argument("--z-max", type=float, default=None,
                       help="障碍切片上界 m. 默认按 profile")
    g_occ.add_argument("--footprint-radius", type=float, default=None,
                       metavar="METERS",
                       help="自身排除半径 m. 默认按 profile. 0=关")
    g_occ.add_argument("--floor-z", type=float, default=-0.10,
                       help="free 标注层下界 (默认 -0.10, 给 RANSAC 残差留容差)")
    g_occ.add_argument("--no-raycast", action="store_true",
                       help="跳过沿轨迹 raycast 填 free")
    g_occ.add_argument("--raycast-range", type=float, default=30.0,
                       help="raycast 最大距离 m (默认 30)")
    g_occ.add_argument("--dilate", type=int, default=0,
                       help="障碍膨胀次数 (默认 0, 不膨胀; nav2 inflation cost 由 nav2 自己处理). "
                            "set 1 给视觉/老 nav2 留 5cm 缓冲.")

    g_misc = ap.add_argument_group("misc")
    g_misc.add_argument("--keep-intermediate", action="store_true",
                        help="保留中间产物 (aligned PCD + 各步 log) 到 output_prefix.aligned/")
    g_misc.add_argument("--align-floor-py", type=Path, default=ALIGN_FLOOR,
                        help=f"align_floor.py 路径 (默认 {ALIGN_FLOOR})")
    g_misc.add_argument("--pcd-to-occgrid-py", type=Path, default=PCD_TO_OCC,
                        help=f"pcd_to_occgrid.py 路径 (默认 {PCD_TO_OCC})")

    g_pf = ap.add_argument_group("step 0 (optional): per-scan footprint filter")
    g_pf.add_argument("--per-scan-filter", action="store_true",
                      help="在 align 前先跑 footprint_filter.py: 用每个 KF 的原始 LiDAR-frame "
                           "scan + transformations.pcd 在传感器局部系做几何切除. 比全局空间最近邻 "
                           "(--footprint-radius) 更准, 不依赖 trajectory.pcd 作 KDTree. 需要 "
                           "SaveMap 输出包含 pcd/ 子目录 + 真 IMU↔LiDAR 外参 yaml.")
    g_pf.add_argument("--per-scan-extrinsic-yaml", type=Path, default=None,
                      help="--per-scan-filter 用的外参 yaml (取 mapping.extrinsic_T/R). "
                           "推荐用 tools/airy_extrinsic 出的 examples/<SN>.yaml")
    g_pf.add_argument("--per-scan-radius", type=float, default=0.8,
                      help="--per-scan-filter 的传感器水平半径 m (默认 0.8). "
                           "实测 0.8m 在手持场景是 sweet spot — 切干净支架 / 操作员伸长的手 / "
                           "GPS 天线, 但不吃真墙. 0.5m 不够 (仍有 hairy 边缘), 1.0m+ 开始误伤墙.")
    g_pf.add_argument("--footprint-filter-py", type=Path, default=FOOTPRINT_FILTER,
                      help=f"footprint_filter.py 路径 (默认 {FOOTPRINT_FILTER})")

    args = ap.parse_args()

    # === 应用 profile 默认值 (用户没显式覆盖时) ===
    PROFILES = {
        "handheld": {"z_min": 0.2, "z_max": 1.5, "footprint_radius": 0.3},
        "dog":      {"z_min": 0.1, "z_max": 0.5, "footprint_radius": 0.4},
        "vehicle":  {"z_min": 0.3, "z_max": 2.0, "footprint_radius": 1.0},
        "generic":  {"z_min": 0.1, "z_max": 1.5, "footprint_radius": 0.0},
    }
    prof = PROFILES[args.profile]
    if args.z_min is None:           args.z_min = prof["z_min"]
    if args.z_max is None:           args.z_max = prof["z_max"]
    if args.footprint_radius is None: args.footprint_radius = prof["footprint_radius"]
    print(f"[CFG] profile={args.profile}: z=[{args.z_min}, {args.z_max}]m  "
          f"footprint={args.footprint_radius}m"
          + (" (overridden)" if any([
              args.z_min != prof["z_min"], args.z_max != prof["z_max"],
              args.footprint_radius != prof["footprint_radius"],
          ]) else ""))

    # === 准备路径 ===
    loam_dir = args.loam_dir.expanduser().resolve()
    out_prefix = args.output.expanduser().resolve()
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    if not args.align_floor_py.is_file():
        print(f"[ERR] 找不到 align_floor.py: {args.align_floor_py}", file=sys.stderr)
        return 2
    if not args.pcd_to_occgrid_py.is_file():
        print(f"[ERR] 找不到 pcd_to_occgrid.py: {args.pcd_to_occgrid_py}", file=sys.stderr)
        return 2

    # === step 1: 输入检查 ===
    print(f"[1/3] 检查 SaveMap 输出: {loam_dir}")
    try:
        global_pcd = find_pcd(loam_dir, "GlobalMap.pcd")
        traj_pcd = find_pcd(loam_dir, "trajectory.pcd")
    except FileNotFoundError as e:
        print(f"[ERR] {e}", file=sys.stderr)
        return 2
    transforms_pcd = loam_dir / "transformations.pcd"
    if not transforms_pcd.is_file():
        transforms_pcd = None
    size_mb = global_pcd.stat().st_size / 1024 / 1024
    print(f"      GlobalMap.pcd ({size_mb:.1f} MB), trajectory.pcd ({traj_pcd.stat().st_size/1024:.1f} KB)"
          + (", transformations.pcd" if transforms_pcd else ""))

    # === 多楼层检测 (issue #26) ===
    # 物理事实: 操作员/机器狗不可能飞起来, trajectory z 必然贴着地面. 所以 trajectory z
    # 是 "楼层指示器". 单纯 range 不够 (单层操作员蹲下+伸手可达 ~1.8m),
    # 关键是 **z 是否多峰**: 单层 = 单峰 (跨度内连续分布), 多层 = 多峰 (cluster 间有 gap).
    try:
        import numpy as np
        with open(traj_pcd, "rb") as f:
            while True:
                line = f.readline()
                if line.startswith(b"DATA"):
                    break
            arr = np.frombuffer(f.read(), dtype=np.float32).reshape(-1, 8)
            z = arr[:, 2]
        z_range = float(z.ptp())
        z_min, z_max = float(z.min()), float(z.max())
        print(f"      trajectory z: range=[{z_min:.2f}, {z_max:.2f}]m  span={z_range:.2f}m")

        # 多楼层检测: 分级 z range 警告.
        #
        # 物理事实: 操作员/狗不可能飞. 单层场景 z range 受身体尺度约束:
        #   - 完全静立 + 蹲下: ~0.5m
        #   - 单手伸举 + 蹲到地: ~1.2m (极限, 几乎不可能维持 1+ 分钟)
        #   - z range > 1.0m → 几乎肯定走过楼梯/坡道/下沉空间, 即使是 \"半层\"
        #   - z range > 2.5m → 至少跨一层全高
        #
        # 注意: 上下楼梯过程中 trajectory z 是 **连续过渡** 的, KMeans / 1D gap 检测
        # 不一定能切出离散 cluster. 所以纯 z range 反而是更可靠的指标.
        if z_range >= 2.5:
            level = "ERR"
            msg = (
                f"[ERR] 轨迹 z range = {z_range:.2f}m, **必然跨多层楼**. "
                f"单张 2D 投影把所有层墙壁叠在一起, 给 nav2 用 = 错的. "
                f"用 --z-min / --z-max 限到单层, 或者用 RViz 看 GlobalMap.pcd 3D. "
                f"分层支持在 issue #26 跟踪."
            )
        elif z_range >= 1.0:
            level = "WARN"
            msg = (
                f"[WARN] 轨迹 z range = {z_range:.2f}m. "
                f">= 1m 时几乎肯定走过楼梯 / 半层下沉 / 坡道, "
                f"建议核实是不是单层. 如果是多层, 单张 2D 投影会失真. "
                f"建议: 用 RViz 加载 GlobalMap.pcd, 拖 z-axis 颜色看实际是 1 层 还是 N 层. "
                f"或手动 --z-min/--z-max 限单层. 分层支持在 issue #26 跟踪."
            )
        else:
            level = "OK"
            msg = f"      → 单层 (z range {z_range:.2f}m < 1m, 操作员身体抖动范围)"
        if level == "OK":
            print(msg)
        else:
            print(msg, file=sys.stderr)
    except Exception as e:
        print(f"[WARN] 跳过多楼层检测: {e}", file=sys.stderr)

    # 中间产物目录
    if args.keep_intermediate:
        scratch = out_prefix.parent / (out_prefix.name + ".aligned")
        scratch.mkdir(parents=True, exist_ok=True)
        cleanup = False
    else:
        scratch_obj = tempfile.TemporaryDirectory(prefix="build_2d_map_")
        scratch = Path(scratch_obj.name)
        cleanup = True

    # === step 0 (optional): per-scan footprint filter ===
    if args.per_scan_filter:
        if not args.per_scan_extrinsic_yaml:
            print(f"[ERR] --per-scan-filter 需要 --per-scan-extrinsic-yaml", file=sys.stderr)
            return 2
        if not args.per_scan_extrinsic_yaml.is_file():
            print(f"[ERR] 外参 yaml 不存在: {args.per_scan_extrinsic_yaml}", file=sys.stderr)
            return 2
        if not args.footprint_filter_py.is_file():
            print(f"[ERR] 找不到 footprint_filter.py: {args.footprint_filter_py}", file=sys.stderr)
            return 2
        print(f"[0/3] per-scan footprint filter (LiDAR-frame, r={args.per_scan_radius}m)")
        filtered_pcd = scratch / "GlobalMap.pcd"
        cmd0 = [
            sys.executable, str(args.footprint_filter_py),
            str(loam_dir),
            "--extrinsic-yaml", str(args.per_scan_extrinsic_yaml),
            "-o", str(filtered_pcd),
            "--sensor-radius", str(args.per_scan_radius),
        ]
        rc, _, _ = run_step("filter", cmd0, scratch / "step0_filter.log")
        if rc != 0:
            print(f"[ERR] footprint_filter 失败. log: {scratch}/step0_filter.log", file=sys.stderr)
            return rc
        # 后续把 filter 输出当 GlobalMap 用
        # 注意 align_floor 需要 trajectory.pcd / transformations.pcd 在同目录, copy 过来
        import shutil
        shutil.copy2(loam_dir / "trajectory.pcd", scratch / "trajectory.pcd")
        if (loam_dir / "transformations.pcd").is_file():
            shutil.copy2(loam_dir / "transformations.pcd", scratch / "transformations.pcd")
        global_pcd = filtered_pcd
        traj_pcd = scratch / "trajectory.pcd"

    # === step 2: align_floor ===
    if args.no_align:
        print(f"[2/3] 跳过 align_floor (--no-align)")
        cloud_for_occ = global_pcd
        traj_for_occ = traj_pcd
    else:
        print(f"[2/3] align_floor: gravity-align cloud + trajectory")
        # main 版 align_floor 接口: 吃"输入目录"(自动找里面的 GlobalMap.pcd / trajectory.pcd),
        # -o 指定输出目录, 输出固定名 GlobalMap.pcd / trajectory.pcd. 不再传单个 pcd 文件,
        # 也不需要 --transformations (那是 feat 旧接口). 用独立 aligned/ 子目录避免覆盖输入.
        align_in = global_pcd.parent      # 正常 = loam_dir; per-scan 时 = scratch (已含过滤后 GlobalMap+traj)
        align_out = scratch / "aligned"
        cmd = [
            sys.executable, str(args.align_floor_py),
            str(align_in),
            "-o", str(align_out),
        ]
        if args.floor_z_range:
            cmd.extend(["--floor-z-range", str(args.floor_z_range[0]), str(args.floor_z_range[1])])
        rc, _, _ = run_step("align", cmd, scratch / "step2_align.log")
        if rc != 0:
            print(f"[ERR] align_floor 失败. log: {scratch}/step2_align.log", file=sys.stderr)
            return rc
        cloud_for_occ = align_out / "GlobalMap.pcd"
        traj_for_occ = align_out / "trajectory.pcd"
        if not cloud_for_occ.is_file():
            print(f"[ERR] align_floor 没生成 {cloud_for_occ}", file=sys.stderr)
            return 2

    # === step 3: pcd_to_occgrid ===
    print(f"[3/3] pcd_to_occgrid: 投影 + 切片"
          + (" + raycast" if not args.no_raycast else ""))
    cmd = [
        sys.executable, str(args.pcd_to_occgrid_py),
        str(cloud_for_occ),
        "-o", str(out_prefix),
        "--resolution", str(args.resolution),
        "--z-min", str(args.z_min),
        "--z-max", str(args.z_max),
        "--floor-z", str(args.floor_z),
        "--dilate", str(args.dilate),
    ]
    if not args.no_raycast:
        cmd.extend([
            "--raycast", str(traj_for_occ),
            "--raycast-max-range", str(args.raycast_range),   # main pcd_to_occgrid 接口名
        ])
    if args.footprint_radius > 0:
        cmd.extend([
            "--footprint-radius", str(args.footprint_radius),
        ])
        if args.no_raycast:
            # raycast 关掉时还要单独传 trajectory 给 footprint 用
            cmd.extend(["--trajectory", str(traj_for_occ)])
    rc, _, _ = run_step("occgrid", cmd, scratch / "step3_occgrid.log")
    if rc != 0:
        print(f"[ERR] pcd_to_occgrid 失败. log: {scratch}/step3_occgrid.log", file=sys.stderr)
        # 常见 hint
        if "裁剪后没有点" in (scratch / "step3_occgrid.log").read_text():
            print("[HINT] 切片为空, 试 --floor-z-range 手动指定地面 z 区间, "
                  "或者点云没 align 时用 --no-align + 手动 --z-min/--z-max", file=sys.stderr)
        return rc

    pgm = Path(str(out_prefix) + ".pgm")
    yaml = Path(str(out_prefix) + ".yaml")
    if not pgm.is_file() or not yaml.is_file():
        print(f"[ERR] 输出文件没生成: {pgm} / {yaml}", file=sys.stderr)
        return 2

    # === summary ===
    summary_lines = [
        f"build_2d_map summary",
        f"=" * 40,
        f"input    : {loam_dir.name}",
        f"output   : {pgm.name} + {yaml.name}",
        f"profile  : {args.profile}",
        f"per-scan : {'ON (r=' + str(args.per_scan_radius) + 'm)' if args.per_scan_filter else 'OFF'}",
        f"align    : {'OFF' if args.no_align else 'ON'}",
        f"raycast  : {'OFF' if args.no_raycast else 'ON (range ' + str(args.raycast_range) + 'm)'}",
        f"footprint: {'OFF' if args.footprint_radius == 0 else 'ON (r=' + str(args.footprint_radius) + 'm)'}",
        f"resolution: {args.resolution} m/px",
        f"slice    : z ∈ [{args.z_min}, {args.z_max}]m  (above floor=0)",
        f"floor-z  : {args.floor_z} m (free 标注层下界)",
    ]
    print()
    for line in summary_lines:
        print(line)
    summary_path = Path(str(out_prefix) + ".summary.txt")
    summary_path.write_text("\n".join(summary_lines) + "\n")
    print(f"\n  summary: {summary_path}")
    if args.keep_intermediate:
        print(f"  中间产物: {scratch}/")
    else:
        print("  (中间产物已清理, 用 --keep-intermediate 保留)")
        scratch_obj.cleanup()  # 显式清, 避免 TemporaryDirectory 析构警告

    print(f"\n  下一步: ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={yaml}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
