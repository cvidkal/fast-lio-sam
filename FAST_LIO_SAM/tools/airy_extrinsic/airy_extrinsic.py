#!/usr/bin/env python3
"""
airy_extrinsic — 从 RoboSense Airy 的 DIFOP 包里抽取 IMU↔LiDAR 出厂外参

Airy 每台出厂标定的 IMU 外参 (qx, qy, qz, qw, x, y, z) 存在 DIFOP 包字节
偏移 1092 起 28 字节 (7 个 IEEE-754 float, 大端). rslidar_sdk 解析了但不
应用, 用户需要把它喂到 LIO config 里, 否则只能依赖 extrinsic_est_en 在线收敛.

支持三种输入:
  1) live   : 直接从雷达网口抓 DIFOP UDP 包
  2) pcap   : 解析 Wireshark / tcpdump 抓到的 .pcap (libpcap 格式)
  3) bag    : 从 rosbag2 (sqlite3) 里读 /rslidar_packets 话题

输出: 一段 YAML, 直接粘到 mapping_airy.yaml 的 mapping.extrinsic_T / R 即可.

用法:
  airy_extrinsic.py live  --port 7788 [--bind 0.0.0.0] [--timeout 10]
  airy_extrinsic.py pcap  --file dump.pcap  [--port 7788]
  airy_extrinsic.py bag   --bag <rosbag2_dir> [--topic /rslidar_packets]

依赖:
  必需: 仅 Python 3.8+ stdlib
  bag 模式可选: pip install rosbags  (如果系统没装 ROS2 也能解 sqlite3 bag)
"""
from __future__ import annotations

import argparse
import io
import math
import os
import socket
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator


# ============================================================
# DIFOP 字节布局常量 (Airy)
# ============================================================
DIFOP_LEN     = 1248
DIFOP_ID      = bytes([0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55])
EXT_OFFSET    = 1092           # qx 起始字节偏移
EXT_BYTES     = 28             # qx,qy,qz,qw,x,y,z 共 7 个大端 float
SN_OFFSET     = 292
SN_BYTES      = 6


# ============================================================
@dataclass
class Extrinsic:
    qx: float
    qy: float
    qz: float
    qw: float
    tx: float
    ty: float
    tz: float
    sn: str = ""

    def quat_norm(self) -> float:
        return math.sqrt(self.qx**2 + self.qy**2 + self.qz**2 + self.qw**2)

    def rotation_matrix(self) -> list[list[float]]:
        """四元数 (qx,qy,qz,qw) -> 3x3 旋转矩阵, 假设左手系正交."""
        # 归一化, 防止数值漂移
        n = self.quat_norm()
        if n < 1e-9:
            raise ValueError("退化四元数 (norm≈0), DIFOP 数据可能未填")
        x, y, z, w = self.qx / n, self.qy / n, self.qz / n, self.qw / n
        return [
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ]


def parse_difop(buf: bytes) -> Extrinsic:
    """从一个 1248B DIFOP 包字节流里解外参."""
    if len(buf) < EXT_OFFSET + EXT_BYTES:
        raise ValueError(f"包长不足: {len(buf)} < {EXT_OFFSET + EXT_BYTES}")
    if buf[:8] != DIFOP_ID:
        raise ValueError(f"DIFOP id 不匹配: 期望 {DIFOP_ID.hex()}, 拿到 {buf[:8].hex()}")
    # 大端 float
    qx, qy, qz, qw, tx, ty, tz = struct.unpack(
        ">7f", buf[EXT_OFFSET:EXT_OFFSET + EXT_BYTES]
    )
    sn_bytes = buf[SN_OFFSET:SN_OFFSET + SN_BYTES]
    sn = sn_bytes.hex().upper()
    return Extrinsic(qx, qy, qz, qw, tx, ty, tz, sn)


# ============================================================
# 输入源 1: 在线 UDP 抓包
# ============================================================
def collect_live(port: int, bind: str, timeout: float) -> bytes:
    print(f"[INFO] 监听 {bind}:{port}, 等待 DIFOP (id={DIFOP_ID.hex()})")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind, port))
    sock.settimeout(timeout)
    try:
        deadline = (
            None if timeout <= 0 else
            __import__("time").monotonic() + timeout
        )
        while True:
            data, _ = sock.recvfrom(4096)
            if len(data) >= EXT_OFFSET + EXT_BYTES and data[:8] == DIFOP_ID:
                print(f"[INFO] 抓到 DIFOP 包 ({len(data)} B)")
                return data
            if deadline and __import__("time").monotonic() > deadline:
                raise TimeoutError(f"超时未收到 DIFOP 包 ({timeout}s)")
    finally:
        sock.close()


# ============================================================
# 输入源 2: pcap (libpcap 经典格式, 不支持 pcapng)
# ============================================================
_PCAP_MAGIC_LE = 0xA1B2C3D4   # microsecond ts, little-endian
_PCAP_MAGIC_BE = 0xD4C3B2A1
_PCAPNG_MAGIC  = 0x0A0D0D0A   # 提示用户用 -F libpcap


def _iter_pcap_payloads(path: str) -> Iterator[bytes]:
    """从 pcap 里逐包提取 UDP payload. 支持 Ethernet + IPv4 + UDP."""
    with open(path, "rb") as f:
        magic = struct.unpack("<I", f.read(4))[0]
        if magic == _PCAPNG_MAGIC:
            raise SystemExit(
                "[ERR] 这是 pcapng 格式, 请用经典 pcap. 在 Wireshark 里"
                "另存为 'Wireshark/tcpdump/... - pcap' 即可."
            )
        if magic == _PCAP_MAGIC_LE:
            endian = "<"
        elif magic == _PCAP_MAGIC_BE:
            endian = ">"
        else:
            raise SystemExit(f"[ERR] 未识别的 pcap magic: {magic:#x}")
        # 跳过余下文件头 (24 字节总长)
        f.read(20)
        rec_hdr = struct.Struct(endian + "IIII")  # ts_sec, ts_usec, incl_len, orig_len
        while True:
            hdr = f.read(rec_hdr.size)
            if len(hdr) < rec_hdr.size:
                return
            _, _, incl_len, _ = rec_hdr.unpack(hdr)
            data = f.read(incl_len)
            if len(data) < 14:
                continue
            # Ethernet
            ethertype = struct.unpack(">H", data[12:14])[0]
            if ethertype == 0x8100:  # 802.1Q VLAN
                ethertype = struct.unpack(">H", data[16:18])[0]
                ip_off = 18
            else:
                ip_off = 14
            if ethertype != 0x0800:  # IPv4
                continue
            if len(data) < ip_off + 20:
                continue
            ihl = (data[ip_off] & 0x0F) * 4
            proto = data[ip_off + 9]
            if proto != 17:  # UDP
                continue
            udp_off = ip_off + ihl
            if len(data) < udp_off + 8:
                continue
            payload = data[udp_off + 8:]
            yield payload


def collect_pcap(path: str, port_filter: int | None) -> bytes:
    print(f"[INFO] 解析 pcap: {path}")
    for payload in _iter_pcap_payloads(path):
        # 端口过滤已隐含在 UDP payload 长度里; 我们直接看 DIFOP id
        if len(payload) >= EXT_OFFSET + EXT_BYTES and payload[:8] == DIFOP_ID:
            return payload
    raise SystemExit("[ERR] pcap 里没找到任何 Airy DIFOP 包")


# ============================================================
# 输入源 3: rosbag2 (sqlite3) - 用 rosbags 库做最少依赖解码
# ============================================================
def collect_bag(bag_path: str, topic: str) -> bytes:
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as e:
        raise SystemExit(
            "[ERR] 需要 rosbags 库: pip install rosbags"
        ) from e

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    print(f"[INFO] 读取 rosbag2: {bag_path}, 话题: {topic}")
    with Reader(bag_path) as reader:
        # 找话题
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            avail = ", ".join(c.topic for c in reader.connections)
            raise SystemExit(
                f"[ERR] bag 内没有 {topic} 话题. 已有: {avail}\n"
                f"      请录制时加上 /rslidar_packets, 或用 live/pcap 模式."
            )
        for conn, _, raw in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            # rslidar_msgs/msg/RslidarPacket 的 data 字段是 1248B
            data = bytes(msg.data) if hasattr(msg, "data") else bytes(msg.bytes)
            if len(data) >= EXT_OFFSET + EXT_BYTES and data[:8] == DIFOP_ID:
                return data
    raise SystemExit(
        f"[ERR] bag 内 {topic} 没有任何 DIFOP 包. 录制时是否启用 send_packet_ros?"
    )


# ============================================================
# YAML 输出
# ============================================================
def render_yaml(ext: Extrinsic) -> str:
    R = ext.rotation_matrix()
    norm = ext.quat_norm()
    sn_pretty = ext.sn if ext.sn else "(unknown)"
    return (
        "# 由 airy_extrinsic.py 自动生成 - 请勿手动编辑\n"
        f"# 雷达 SN (DIFOP 字节 292..297): {sn_pretty}\n"
        f"# 原始四元数 [qx, qy, qz, qw] = "
        f"[{ext.qx: .6f}, {ext.qy: .6f}, {ext.qz: .6f}, {ext.qw: .6f}]\n"
        f"# 四元数模长: {norm:.6f} (理想 = 1)\n"
        "\n"
        "mapping:\n"
        f"  extrinsic_T: [{ext.tx: .6f}, {ext.ty: .6f}, {ext.tz: .6f}]\n"
        "  extrinsic_R: [\n"
        f"    {R[0][0]: .6f}, {R[0][1]: .6f}, {R[0][2]: .6f},\n"
        f"    {R[1][0]: .6f}, {R[1][1]: .6f}, {R[1][2]: .6f},\n"
        f"    {R[2][0]: .6f}, {R[2][1]: .6f}, {R[2][2]: .6f}\n"
        "  ]\n"
    )


# ============================================================
# CLI
# ============================================================
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="mode", required=True)

    p_live = sub.add_parser("live", help="实时抓 UDP DIFOP 包")
    p_live.add_argument("--port", type=int, default=7788, help="DIFOP 端口 (默认 7788)")
    p_live.add_argument("--bind", default="0.0.0.0", help="绑定网卡 IP (默认 0.0.0.0)")
    p_live.add_argument("--timeout", type=float, default=15.0, help="超时秒 (默认 15)")

    p_pcap = sub.add_parser("pcap", help="离线解析 pcap")
    p_pcap.add_argument("--file", required=True, help="pcap 路径")
    p_pcap.add_argument("--port", type=int, default=None, help="(可选) 端口过滤, 默认按 DIFOP id 匹配")

    p_bag = sub.add_parser("bag", help="从 rosbag2 (sqlite3) 提取")
    p_bag.add_argument("--bag", required=True, help="rosbag2 目录")
    p_bag.add_argument("--topic", default="/rslidar_packets", help="话题名 (默认 /rslidar_packets)")

    p.add_argument("-o", "--output", help="输出 YAML 文件 (默认打印到 stdout)")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.mode == "live":
        buf = collect_live(args.port, args.bind, args.timeout)
    elif args.mode == "pcap":
        buf = collect_pcap(args.file, args.port)
    elif args.mode == "bag":
        buf = collect_bag(args.bag, args.topic)
    else:
        raise SystemExit("unreachable")

    ext = parse_difop(buf)
    yaml_text = render_yaml(ext)

    if args.output:
        Path(args.output).write_text(yaml_text)
        print(f"[OK] 写出: {args.output}")
    print(yaml_text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
