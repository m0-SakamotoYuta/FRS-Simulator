"""アプリ本体の _AnkleRamRecorder を、実物の条件で通しで検証する。

  ・1280x720@30 (111 MB/s) を 90 秒 = 10 GB 流す (固定バッファ方式では絶対に無理な長さ)
  ・書き出し中に別の大きなファイルも書いて、わざとディスクを詰まらせる
  ・取り込み側が止まらないか / 1枚も落ちないか / 中身が保たれるか
"""
import importlib.util
import os
import sqlite3
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import pyrealsense2 as rs

ROOT = Path(r"C:\Users\yutas\Desktop\FRS-Simulator")
OUT = Path(r"C:\Users\yutas\Desktop\FRS-Simulator\cache\ringapp.db3")
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location("frs", str(ROOT / "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(frs)
Rec = frs._AnkleRamRecorder

W, H, FPS = 1280, 720, 30
SECONDS = 90
N = SECONDS * FPS
RING = 8 * FPS          # 既定の 8 秒
DU = 9.999999747378752e-05

ci = rs.intrinsics(); ci.width, ci.height = W, H
ci.ppx, ci.ppy, ci.fx, ci.fy = 643.21, 359.87, 645.12, 645.30
ci.model = rs.distortion.inverse_brown_conrady
ci.coeffs = [-0.0551, 0.0653, 0.0002, -0.0007, -0.0210]
di = rs.intrinsics(); di.width, di.height = W, H
di.ppx, di.ppy, di.fx, di.fy = 643.21, 359.87, 645.12, 645.30
di.model = rs.distortion.brown_conrady; di.coeffs = [0.0] * 5
ext = rs.extrinsics(); ext.rotation = [1, 0, 0, 0, 1, 0, 0, 0, 1]
ext.translation = [0.0, 0.0, 0.0]
meta = {"w": W, "h": H, "fps": FPS, "color_bpp": 2, "color_fmt": rs.format.yuyv,
        "color_intr": ci, "depth_intr": di, "extrinsics": ext, "depth_units": DU}

if OUT.exists():
	OUT.unlink()
rng = np.random.default_rng(51)
src_c = rng.integers(0, 255, (H, W), dtype=np.uint16)     # カメラが返す形 (YUYV)
src_d = rng.integers(0, 4000, (H, W), dtype=np.uint16)

# --- 妨害役: 同じディスクに別の大ファイルを全速で書き続ける ---
noise_stop = threading.Event()


def noise():
	buf = np.random.default_rng(7).integers(0, 255, 8 * 1024 * 1024, np.uint8).tobytes()
	while not noise_stop.is_set():
		fd, tmp = tempfile.mkstemp(suffix=".noise", dir=str(OUT.parent))
		os.close(fd)
		try:
			with open(tmp, "wb", buffering=0) as f:
				for _ in range(256):          # 2 GB
					if noise_stop.is_set():
						break
					f.write(buf)
				f.flush()
				os.fsync(f.fileno())          # わざと強制フラッシュ
		except Exception:
			pass
		finally:
			try:
				os.remove(tmp)
			except Exception:
				pass


nt = threading.Thread(target=noise, daemon=True)
nt.start()
print(f"妨害あり: 同じディスクに 2GB の書き込み+fsync を延々と並行実行")

rec = Rec(str(OUT), meta, RING, log=lambda *a: None)
rec.start()
lat = np.zeros(N)
t_start = time.perf_counter()
for i in range(N):
	want = t_start + i / FPS
	d = want - time.perf_counter()
	if d > 0:
		time.sleep(d)
	t0 = time.perf_counter()
	rec.push(src_c, src_d, 1000.0 + i * (1000.0 / FPS), i)
	lat[i] = time.perf_counter() - t0
cap_wall = time.perf_counter() - t_start
st = rec.stop()
noise_stop.set()
nt.join(timeout=10)
time.sleep(1.0)

print(f"\n--- 取り込み側 ({SECONDS} 秒 / {N * W * H * 4 / 1e9:.1f} GB) ---")
print(f"  実時間 {cap_wall:.1f} s (狙い {SECONDS} s)")
print(f"  1フレームの所要: 中央 {np.median(lat) * 1000:.2f} ms / 最大 {lat.max() * 1000:.1f} ms")
for thr in (33.3, 100):
	k = int((lat * 1000 > thr).sum())
	print(f"    {thr:5.1f} ms 超え: {k} 回")
print(f"\n--- 書き出し側 ---")
print(f"  記録 {st['n']} / 投入 {st['pushed']} / 取りこぼし {st['dropped']}")
print(f"  リング最大使用 {st['max_used']}/{st['cap']} "
      f"({st['max_used'] / FPS:.1f} 秒ぶん / {st['cap'] / FPS:.0f} 秒)")
print(f"  50ms を超えた書き出し {len(st['stalls'])} 回"
      + (f" / 最大 {max(x[1] for x in st['stalls']):.0f} ms" if st["stalls"] else ""))
print(f"  停止後の残り処理 {st['drain_s']:.2f} s / err={st['err']}")

con = sqlite3.connect(str(OUT))
rows = dict(con.execute(
	"SELECT t.name, COUNT(*) FROM messages m JOIN topics t ON m.topic_id=t.id "
	"WHERE t.name LIKE '%image/data' GROUP BY t.name").fetchall())
con.close()
nc = [v for k, v in rows.items() if "Color" in k]
nd = [v for k, v in rows.items() if "Depth" in k]
print(f"\n--- 出来たファイル ---")
print(f"  {OUT.stat().st_size / 1e9:.2f} GB / color {nc} / depth {nd} (期待 {N})")

# 中身の確認 (別プロセスではないが、内部パラメータと画素を突き合わせる)
p = rs.pipeline(); c = rs.config()
rs.config.enable_device_from_file(c, str(OUT), repeat_playback=False)
c.enable_stream(rs.stream.color); c.enable_stream(rs.stream.depth)
pr = p.start(c)
try:
	pr.get_device().as_playback().set_real_time(False)
except Exception:
	pass
intr = pr.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
ds = pr.get_device().first_depth_sensor().get_depth_scale()
n_read = 0
match_c = match_d = 0
while True:
	try:
		fs = p.wait_for_frames(timeout_ms=3000)
	except RuntimeError:
		break
	cf = fs.get_color_frame(); df = fs.get_depth_frame()
	if not cf or not df:
		break
	if n_read < 3:
		match_c += int(np.array_equal(np.asanyarray(cf.get_data()), src_c))
		match_d += int(np.array_equal(np.asanyarray(df.get_data()), src_d))
	n_read += 1
p.stop()
print(f"  読み直し {n_read} 枚 / 先頭3枚の画素一致 color {match_c}/3 depth {match_d}/3")
print(f"  fx={intr.fx:.2f} depth_scale={ds}")

ok = (st["dropped"] == 0 and st["n"] == N and nc and nc[0] == N
      and n_read == N and match_c == 3 and match_d == 3
      and abs(ds - DU) < 1e-9 and lat.max() < 0.0333)
print("\n=> 上限なし・妨害下でも成立" if ok else "\n=> 不成立")
try:
	OUT.unlink()
except Exception:
	pass
sys.exit(0 if ok else 1)
