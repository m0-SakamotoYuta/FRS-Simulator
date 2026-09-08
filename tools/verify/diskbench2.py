"""111 MB/s を長く流したときに、書いた量に応じて詰まるかを確かめる。

録画で観測された欠落は 1.3〜2.9 GB 書いた時点に集中していた。
経過時間ではなく「書いた量」で決まるなら、ストレージ側の挙動 (SLCキャッシュ/GC/
ダーティページのしきい値) が原因ということになる。
"""
import os
import tempfile
import time
from pathlib import Path

import numpy as np

TARGET_DIR = Path(r"C:\Users\yutas\Desktop\FRS-Simulator\cache")
FRAME_MB = 3.69
CHUNK = int(FRAME_MB * 1024 * 1024)
RATE = 111.0
BUDGET = 1 / 30.0


def run(label, total_gb, fsync_every=0, prefile_gb=0.0):
	buf = np.random.default_rng(2).integers(0, 255, CHUNK, dtype=np.uint8).tobytes()
	tmp_pre = None
	if prefile_gb > 0:
		# 直前に別の録画を1本書いた状態を作る (撮り直しの再現)
		fd, tmp_pre = tempfile.mkstemp(suffix=".pre", dir=str(TARGET_DIR))
		os.close(fd)
		with open(tmp_pre, "wb", buffering=0) as f:
			for _ in range(int(prefile_gb * 1024 / FRAME_MB)):
				f.write(buf)
	n = int(total_gb * 1024 / FRAME_MB)
	lat = np.zeros(n)
	fd, tmp = tempfile.mkstemp(suffix=".bench", dir=str(TARGET_DIR))
	os.close(fd)
	t_start = time.perf_counter()
	try:
		with open(tmp, "wb", buffering=0) as f:
			for i in range(n):
				want = t_start + (i * FRAME_MB) / RATE
				dt = want - time.perf_counter()
				if dt > 0:
					time.sleep(dt)
				t0 = time.perf_counter()
				f.write(buf)
				if fsync_every and (i + 1) % fsync_every == 0:
					f.flush()
					os.fsync(f.fileno())
				lat[i] = time.perf_counter() - t0
	finally:
		for q in (tmp, tmp_pre):
			if q:
				try:
					os.remove(q)
				except Exception:
					pass
	over = np.maximum(lat - BUDGET, 0).sum()
	print(f"\n--- {label} ---")
	print(f"  {n * FRAME_MB / 1024:.2f} GB / 中央 {np.median(lat) * 1000:.2f} ms / "
	      f"最大 {lat.max() * 1000:.1f} ms")
	for thr in (33.3, 100, 300, 1000):
		k = int((lat * 1000 > thr).sum())
		if k:
			print(f"    {thr:6.1f} ms 超え: {k:4d} 回")
	big = np.where(lat * 1000 > 60)[0]
	if len(big):
		print("    60ms 超えの発生位置 (書いた量, 停止時間):")
		for j in big[np.argsort(-lat[big])][:10]:
			print(f"      {int(j) * FRAME_MB / 1024:5.2f} GB   {lat[int(j)] * 1000:7.1f} ms "
			      f"(≒{lat[int(j)] * 30:.0f} フレーム)")
	else:
		print("    60ms を超える停止なし")
	print(f"  コマ落ち相当 {over * 30:.0f} フレーム")


run("(A) 111MB/s で 8 GB 連続 (fsyncなし)", 8.0)
run("(B) 111MB/s で 8 GB / 2秒ごとに fsync", 8.0, fsync_every=60)
run("(C) 直前に 3GB 書いた直後に 3 GB (撮り直しの再現)", 3.0, prefile_gb=3.0)
