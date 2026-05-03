# airy_extrinsic — Airy DIFOP 出厂外参解析

从 RoboSense Airy 的 DIFOP 包里抽取 IMU↔LiDAR **出厂标定**外参 (`qx, qy, qz, qw, x, y, z`)，自动算出 3×3 旋转矩阵 + 3×1 平移，输出可直接粘到 LIO config 的 YAML 片段。

> 解决 issue #3.

## 为什么需要

RoboSense Airy 内置 IMU，但 IMU 的位置/朝向相对 LiDAR 是**每台单独标定的**：
- 标定值出厂烧进固件，从 DIFOP 包字节 `1092..1119` 输出 (7 个大端 IEEE-754 float)
- `rslidar_sdk` 解析了但**不应用**到点云/IMU 输出
- 所以 LIO (FAST-LIO / FAST-LIO-SAM 等) **必须知道这个外参**才能正确融合 IMU
- 不知道就只能依赖 `extrinsic_est_en: true` 在线收敛，对快速运动的机器狗来说不够稳

官方 `Airy IMU外参解析.pdf` 的方法是：用 Wireshark 抓 → 找字节 → 复制粘贴到 IEEE-754 转换网站。这个工具把这一切自动化。

## 安装

```bash
# 必需依赖: 仅 Python 3.8+ stdlib, 不用装别的就能跑 live / pcap
# bag 模式可选:
pip install rosbags
```

## 用法

### 1) 在线抓包 (live)

雷达接好、网卡能 ping 通时直接跑：

```bash
sudo python3 airy_extrinsic.py live --port 7788 --bind 0.0.0.0 --timeout 15
```

> `sudo` 是因为绑定 < 1024 端口要 root；7788 不需要，但有些系统对原始 UDP 也限制。
> 如果 host 上有 rslidar_sdk 在跑，会跟它抢 7788 端口，先 stop 一下：
> `sudo systemctl stop airy-lidar.service`

### 2) Wireshark / tcpdump 抓的 pcap (pcap)

```bash
# 抓个 5 秒的 pcap (任意工具)
sudo tcpdump -i eno1 -nn -w /tmp/airy.pcap udp port 7788 -G 5 -W 1

# 离线提取
python3 airy_extrinsic.py pcap --file /tmp/airy.pcap
```

⚠️ 必须是经典 pcap（不是 pcapng）。Wireshark 另存时选 `Wireshark/tcpdump - pcap`.

### 3) 从 rosbag2 提取 (bag)

前提是你录 bag 时打开了 `send_packet_ros: true`，bag 里有 `/rslidar_packets`：

```bash
python3 airy_extrinsic.py bag --bag /path/to/airy_xxx --topic /rslidar_packets
```

## 输出

```yaml
# 由 airy_extrinsic.py 自动生成 - 请勿手动编辑
# 雷达 SN (DIFOP 字节 292..297): ABCD12345678
# 原始四元数 [qx, qy, qz, qw] = [ 0.002710,  0.001080,  0.003860,  0.999990]
# 四元数模长: 1.000002 (理想 = 1)

mapping:
  extrinsic_T: [ 0.012300, -0.000800,  0.042100]
  extrinsic_R: [
     0.999968, -0.007714,  0.002181,
     0.007726,  0.999956, -0.005412,
    -0.002139,  0.005428,  0.999983
  ]
```

把 `mapping:` 块下两条直接覆盖到 `airy.yaml` 即可。

## 写到文件

```bash
python3 airy_extrinsic.py live -o airy_extrinsic.yaml
```

## DIFOP 字节布局参考

```
RSAIRYDifopPkt (#pragma pack(1), 总 1248 字节):
  offset    size  field
  0         8     id[8]                   = A5 FF 00 5A 11 11 55 55  (DIFOP 标识)
  ...
  292       6     SN                      雷达序列号
  ...
  1092      4     qx (大端 float)
  1096      4     qy
  1100      4     qz
  1104      4     qw
  1108      4     x  (米)
  1112      4     y
  1116      4     z
  ...
  1246      2     tail
```

字段定义见 `rslidar_sdk/.../decoder_RSAIRY.hpp` 中 `RSAIRYDifopPkt`.

## 自测

```bash
python3 test_airy_extrinsic.py -v
```
