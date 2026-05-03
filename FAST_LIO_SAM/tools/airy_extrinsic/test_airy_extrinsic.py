#!/usr/bin/env python3
"""
单元测试: 用合成 DIFOP 包验证 parse_difop 和 render_yaml 正确.

跑法:
  python3 test_airy_extrinsic.py
"""
import math
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from airy_extrinsic import (
    DIFOP_ID, DIFOP_LEN, EXT_OFFSET, EXT_BYTES,
    SN_OFFSET, SN_BYTES,
    parse_difop, render_yaml, Extrinsic,
)


def craft_difop(qx, qy, qz, qw, tx, ty, tz, sn=b'\xAB\xCD\x12\x34\x56\x78') -> bytes:
    buf = bytearray(DIFOP_LEN)
    buf[0:8] = DIFOP_ID
    buf[SN_OFFSET:SN_OFFSET+SN_BYTES] = sn
    # 大端 IEEE-754 7 个 float
    struct.pack_into(">7f", buf, EXT_OFFSET, qx, qy, qz, qw, tx, ty, tz)
    return bytes(buf)


class TestParseDifop(unittest.TestCase):
    def test_identity_quat_zero_translation(self):
        buf = craft_difop(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
        ext = parse_difop(buf)
        self.assertAlmostEqual(ext.qw, 1.0, places=6)
        self.assertAlmostEqual(ext.tx, 0.0, places=6)
        R = ext.rotation_matrix()
        # 单位四元数 -> 单位旋转
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(R[i][j], 1.0 if i == j else 0.0, places=6)

    def test_realistic_airy_extrinsic(self):
        # 真实 Airy 量级: IMU 几乎和 LiDAR 重合, 小偏移 + 接近单位四元数
        buf = craft_difop(
            qx=0.00271, qy=0.00108, qz=0.00386, qw=0.99999,
            tx=0.0123,  ty=-0.0008,  tz=0.0421,
        )
        ext = parse_difop(buf)
        self.assertAlmostEqual(ext.qw, 0.99999, places=5)
        self.assertAlmostEqual(ext.tx, 0.0123,  places=5)
        # 模长应近似 1
        self.assertAlmostEqual(ext.quat_norm(), 1.0, places=4)
        # SN 取出来应能 hex
        self.assertEqual(ext.sn, "ABCD12345678")

    def test_yaw_90_rotation(self):
        # 绕 z 轴 90°: q = (0, 0, sin45°, cos45°)
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        buf = craft_difop(0.0, 0.0, s, c, 0.1, 0.2, 0.3)
        ext = parse_difop(buf)
        R = ext.rotation_matrix()
        # 期望 R = [[0,-1,0],[1,0,0],[0,0,1]]
        self.assertAlmostEqual(R[0][0], 0.0, places=5)
        self.assertAlmostEqual(R[0][1], -1.0, places=5)
        self.assertAlmostEqual(R[1][0], 1.0, places=5)
        self.assertAlmostEqual(R[1][1], 0.0, places=5)
        self.assertAlmostEqual(R[2][2], 1.0, places=5)

    def test_bad_id_raises(self):
        buf = bytearray(DIFOP_LEN)
        buf[0:8] = b'\x00' * 8  # 错的 id
        with self.assertRaises(ValueError):
            parse_difop(bytes(buf))

    def test_short_packet_raises(self):
        with self.assertRaises(ValueError):
            parse_difop(b'\x00' * 100)

    def test_zero_quat_raises(self):
        # qw=0 也不要紧, 但 qx=qy=qz=qw=0 -> norm=0 应抛
        ext = Extrinsic(0, 0, 0, 0, 0, 0, 0)
        with self.assertRaises(ValueError):
            ext.rotation_matrix()


class TestYamlOutput(unittest.TestCase):
    def test_render_round_trip_via_eval(self):
        buf = craft_difop(0.001, 0.002, 0.003, 0.99999, 0.01, 0.02, 0.03)
        ext = parse_difop(buf)
        text = render_yaml(ext)
        self.assertIn("extrinsic_T:", text)
        self.assertIn("extrinsic_R:", text)
        self.assertIn("ABCD12345678", text)


if __name__ == "__main__":
    unittest.main()
