#!/usr/bin/env python3
"""
map_editor — 在浏览器里手动擦 / 补 nav2 占据栅格图 (.pgm + .yaml).

为什么需要:
  pcd_to_occgrid / build_2d_map 投影出的 2D 图常残留动态物体 (走动的人 / 机身)
  打出的散斑, 或缺一段墙. 自动阈值滤不干净又怕误删真结构. 这个工具起一个本地
  web server, 在浏览器 canvas 上用画笔手动擦噪点 / 补墙, 直接覆盖写回源 PGM.

笔刷 (canvas 上点 1/2/3 或工具栏切换):
  - 擦除 → free      (白, 可通行, 像素值 254)
  - 障碍 → occupied  (黑, 像素值 0)
  - 未知 → unknown   (灰, 像素值 205)
  画笔大小可调, Ctrl+Z undo, 鼠标滚轮缩放, 空格 / 中键拖动平移, Ctrl+S 保存.

保存: 直接覆盖源 .pgm (原子写: 先写同目录临时文件再 os.replace, 写一半也不会
      损坏原图). .yaml 不动 —— 只改像素值, origin / resolution / 尺寸都不变.
      想留底用 --backup, 第一次保存前把源 .pgm 复制成 .pgm.bak.

依赖: Pillow numpy (已是其它 tools 的依赖); 其余全用标准库, 无新增第三方包.

用法:
  python3 tools/map_editor/map_editor.py ~/maps/airy_room.pgm
  # 脚本会尝试自动打开浏览器; 没打开就手动访问打印出的 URL
  python3 tools/map_editor/map_editor.py ~/maps/airy_room.pgm --port 8123 --no-browser --backup
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import shutil
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
EDITOR_HTML = HERE / "editor.html"

# nav2 占据栅格约定 (跟 pcd_to_occgrid.save_map 一致)
VAL_OCC = 0      # 占据 (黑)
VAL_UNK = 205    # 未知 (灰)
VAL_FREE = 254   # 自由 (白)

# /download 允许的文件类型 (地图目录里的产物; 不开放任意文件)
DOWNLOAD_EXTS = {".pcd", ".zip", ".pgm", ".yaml", ".png"}

# 按源文件后缀决定保存格式, 避免把 PPM 内容写进 .png 名字 (会让 map_server 读到名实不符的图).
# 缺省走 PPM (P5 PGM), 跟历史行为一致.
SUFFIX_TO_FORMAT = {".png": "PNG", ".pgm": "PPM", ".ppm": "PPM", ".pbm": "PPM"}


class MapState:
    """服务端持有的地图状态; handler 通过它读写."""

    def __init__(self, pgm_path: Path, backup: bool):
        # 故意不在这里 resolve(): src_path 可能是 /opt/dog/map/current_map/map.pgm 这种
        # 软链路径, 保留它才能在每次算图后 current_map 重指时自动跟随到最新图 (maybe_reload).
        self.src_path = pgm_path.expanduser()
        self.backup = backup
        self._backed_up = False
        self.lock = threading.Lock()
        self._sig = None
        self._load()  # 设定 pgm_path / arr / height / width / yaml / resolution / origin / _sig

    def _load(self) -> None:
        """从 src_path 解析到的当前真实 PGM 读入内存。调用方需持锁 (或在单线程 __init__ 里)。"""
        resolved = self.src_path.resolve()
        img = Image.open(resolved).convert("L")
        self.arr = np.array(img, dtype=np.uint8)   # (H, W), row 0 = 图像上边
        self.height, self.width = self.arr.shape
        self.pgm_path = resolved                   # 保存仍写回这张具体图
        self.yaml_path = resolved.with_suffix(".yaml")
        self._backed_up = False                    # 换图了, 备份标记重置

        # 从 yaml 读 resolution / origin (仅用于前端坐标读数; 缺了也能跑)
        self.resolution = 0.05
        self.origin = [0.0, 0.0, 0.0]
        if self.yaml_path.is_file():
            try:
                self._parse_yaml(self.yaml_path)
            except Exception as e:  # noqa: BLE001
                print(f"[WARN] 解析 {self.yaml_path.name} 失败 ({e}); 用默认 res/origin",
                      file=sys.stderr)
        try:
            self._sig = (str(resolved), resolved.stat().st_mtime)
        except OSError:
            self._sig = (str(resolved), None)

    def maybe_reload(self) -> None:
        """current_map 软链重指 / 源 PGM 被重建时自动重载, 免手动重启 editor。
        在每个 GET 前调用; 没变化就是一次 stat, 开销可忽略。"""
        try:
            resolved = self.src_path.resolve()
            sig = (str(resolved), resolved.stat().st_mtime)
        except OSError:
            return  # 源暂时不可达 (例如部署换链的瞬间), 沿用旧图
        if sig == self._sig:
            return
        with self.lock:
            if sig == self._sig:   # 双检, 防并发重复重载
                return
            try:
                self._load()
            except Exception as e:  # noqa: BLE001
                print(f"[WARN] 自动重载地图失败 ({e}); 沿用旧图", file=sys.stderr)
                return
        print(f"[INFO] 检测到地图更新, 已重载 -> {self.pgm_path}")

    def _parse_yaml(self, path: Path) -> None:
        # map.yaml 很简单, 手解析避免引入 PyYAML 依赖.
        for raw in path.read_text().splitlines():
            line = raw.split("#", 1)[0].strip()
            if line.startswith("resolution:"):
                self.resolution = float(line.split(":", 1)[1])
            elif line.startswith("origin:"):
                vals = line.split(":", 1)[1].strip().strip("[]")
                parts = [p for p in vals.replace(",", " ").split() if p]
                self.origin = [float(p) for p in parts[:3]] + [0.0] * (3 - len(parts[:3]))

    def list_downloads(self) -> list[dict]:
        """地图同目录下可下载的产物 (PCD / 压缩包 / 栅格图), 按文件名排序."""
        out = []
        map_dir = self.pgm_path.parent
        for p in sorted(map_dir.iterdir()):
            if p.is_file() and p.suffix.lower() in DOWNLOAD_EXTS:
                out.append({"name": p.name, "size": p.stat().st_size})
        return out

    def resolve_download(self, name: str) -> Path | None:
        """把 /download/<name> 解析成地图目录内的真实文件; 防穿越, 白名单后缀."""
        if "/" in name or "\\" in name or name.startswith("."):
            return None
        map_dir = self.pgm_path.parent.resolve()
        p = (map_dir / name).resolve()
        if p.parent != map_dir or not p.is_file():
            return None
        if p.suffix.lower() not in DOWNLOAD_EXTS:
            return None
        return p

    def to_payload(self) -> dict:
        with self.lock:
            data_b64 = base64.b64encode(self.arr.tobytes()).decode("ascii")
        return {
            "ok": True,
            "filename": self.pgm_path.name,
            "path": str(self.pgm_path),
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": self.origin,
            "data": data_b64,
        }

    def save(self, raw: bytes) -> dict:
        expected = self.width * self.height
        if len(raw) != expected:
            return {"ok": False, "error": f"数据长度 {len(raw)} != W*H {expected}"}
        with self.lock:
            new_arr = np.frombuffer(raw, dtype=np.uint8).reshape(self.height, self.width)
            if self.backup and not self._backed_up:
                bak = self.pgm_path.with_suffix(self.pgm_path.suffix + ".bak")
                shutil.copy2(self.pgm_path, bak)
                self._backed_up = True
                print(f"[INFO] 已备份原图 -> {bak}")
            # 原子写: 临时文件同目录, 再 os.replace; 保存格式跟源文件后缀一致
            fmt = SUFFIX_TO_FORMAT.get(self.pgm_path.suffix.lower(), "PPM")
            tmp = self.pgm_path.with_suffix(self.pgm_path.suffix + ".tmp")
            Image.fromarray(new_arr, mode="L").save(tmp, format=fmt)
            os.replace(tmp, self.pgm_path)
            self.arr = new_arr.copy()
        n_occ = int((new_arr == VAL_OCC).sum())
        n_free = int((new_arr == VAL_FREE).sum())
        n_unk = int((new_arr == VAL_UNK).sum())
        print(f"[OK] 已覆盖保存 {self.pgm_path.name}  "
              f"occ={n_occ:,} free={n_free:,} unk={n_unk:,}")
        return {"ok": True, "occ": n_occ, "free": n_free, "unknown": n_unk}


def make_handler(state: MapState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # 静音默认每请求日志
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, obj: dict, code: int = 200) -> None:
            self._send(code, json.dumps(obj).encode("utf-8"), "application/json")

        def do_GET(self):  # noqa: N802
            state.maybe_reload()  # 跟随 current_map: 算图后浏览器刷新即见最新图, 无需重启
            if self.path in ("/", "/index.html"):
                try:
                    html = EDITOR_HTML.read_bytes()
                except FileNotFoundError:
                    self._send(500, b"editor.html missing next to map_editor.py",
                               "text/plain; charset=utf-8")
                    return
                self._send(200, html, "text/html; charset=utf-8")
            elif self.path == "/api/map":
                self._send_json(state.to_payload())
            elif self.path == "/api/files":
                self._send_json({"ok": True, "files": state.list_downloads()})
            elif self.path.startswith("/download/"):
                self._serve_download(self.path[len("/download/"):])
            else:
                self._send(404, b"not found", "text/plain; charset=utf-8")

        def _serve_download(self, raw_name: str) -> None:
            from urllib.parse import unquote
            p = state.resolve_download(unquote(raw_name))
            if p is None:
                self._send(404, b"no such downloadable file", "text/plain; charset=utf-8")
                return
            size = p.stat().st_size
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(size))
            self.send_header("Content-Disposition", f'attachment; filename="{p.name}"')
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            # GlobalMap.pcd 可达上百 MB, 分块流式发, 不整块进内存
            with p.open("rb") as f:
                while True:
                    chunk = f.read(1024 * 1024)
                    if not chunk:
                        break
                    try:
                        self.wfile.write(chunk)
                    except (BrokenPipeError, ConnectionResetError):
                        return

        def do_POST(self):  # noqa: N802
            if self.path != "/api/save":
                self._send(404, b"not found", "text/plain; charset=utf-8")
                return
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length else b""
            try:
                result = state.save(raw)
            except Exception as e:  # noqa: BLE001
                self._send_json({"ok": False, "error": str(e)}, code=500)
                return
            self._send_json(result, code=200 if result.get("ok") else 400)

    return Handler


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pgm", type=Path, help="要编辑的 nav2 占据栅格图 (.pgm 或 .png)")
    ap.add_argument("--port", type=int, default=8000, help="本地端口 (默认 8000)")
    ap.add_argument("--host", default="127.0.0.1", help="绑定地址 (默认 127.0.0.1)")
    ap.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    ap.add_argument("--backup", action="store_true",
                    help="第一次保存前把源 .pgm 复制成 .pgm.bak")
    args = ap.parse_args()

    # 不 resolve(): 传软链路径 (如 .../current_map/map.pgm) 给 MapState, 让它能跟随重指。
    pgm = args.pgm.expanduser()
    if not pgm.is_file():   # is_file() 会跟随软链, 目标在就为真
        print(f"[ERR] 找不到 PGM: {pgm}", file=sys.stderr)
        return 2
    if pgm.suffix.lower() not in SUFFIX_TO_FORMAT:
        print(f"[WARN] {pgm.name} 后缀不在 {sorted(SUFFIX_TO_FORMAT)} 内, "
              f"仍按灰度图加载, 保存回退成 PPM 格式", file=sys.stderr)

    try:
        state = MapState(pgm, backup=args.backup)
    except Exception as e:  # noqa: BLE001
        print(f"[ERR] 加载地图失败: {e}", file=sys.stderr)
        return 2

    print(f"[INFO] 地图: {state.pgm_path}")
    print(f"       尺寸 {state.width} x {state.height} px, "
          f"res {state.resolution} m/px, origin {state.origin}")
    if state.backup:
        print("       --backup: 首次保存前会写 .pgm.bak")
    else:
        print("       保存将直接覆盖源 .pgm (无备份; 想留底加 --backup)")

    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    url = f"http://{args.host}:{args.port}"
    print(f"\n[READY] 擦图工具已启动: {url}")
    print("        浏览器里: 1/2/3 切笔刷, 滚轮缩放, 空格+拖动平移, Ctrl+Z 撤销, Ctrl+S 保存")
    print("        Ctrl+C 关闭服务\n")

    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] 关闭服务")
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
