"""取り込みで落ちる原因を実機で切り分ける。

仮説: pipeline.wait_for_frames の内部キューは容量1しかなく、
      呼び出しが1回でも 33ms を超えるとその間のフレームが捨てられる。
      GUI は表示処理(imshow×3 + Tk update)を挟むので、たまにそれが起きる。

比べる:
  1) wait_for_frames のループ                      … 現状
  2) wait_for_frames + rs.frame_queue を明示的に大きく
  3) pipeline.start(cfg, callback) のコールバック方式 … 本命の対策案
それぞれに「30フレームごとに 150ms 止まる」という妨害を入れて、
カメラのフレーム番号がどれだけ飛ぶかを数える。
"""
import sys
import threading
import time

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 1280, 720, 30
SEC = 25
STALL_EVERY = 30
STALL_MS = 150


def cfg_of():
	c = rs.config()
	c.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
	c.enable_stream(rs.stream.color, W, H, rs.format.yuyv, FPS)
	return c


def maximize_queues(prof):
	for s in prof.get_device().query_sensors():
		try:
			if s.supports(rs.option.frames_queue_size):
				r = s.get_option_range(rs.option.frames_queue_size)
				s.set_option(rs.option.frames_queue_size, float(r.max))
		except Exception:
			pass


def gaps_of(fns):
	f = np.asarray(fns, dtype=np.int64)
	if len(f) < 3:
		return 0
	d = np.diff(f)
	return int(np.sum(d[d > 1] - 1))


def m_wait(stall):
	pipe = rs.pipeline()
	prof = pipe.start(cfg_of())
	maximize_queues(prof)
	for _ in range(45):
		pipe.wait_for_frames(5000)
	fns = []
	t0 = time.perf_counter()
	k = 0
	while time.perf_counter() - t0 < SEC:
		fs = pipe.wait_for_frames(3000)
		c = fs.get_color_frame()
		if not c:
			continue
		fns.append(int(c.get_frame_number()))
		k += 1
		if stall and k % STALL_EVERY == 0:
			time.sleep(STALL_MS / 1000.0)
	pipe.stop()
	return len(fns), gaps_of(fns)


def m_queue(stall):
	"""pipeline を明示的な frame_queue に流し込む。"""
	pipe = rs.pipeline()
	q = rs.frame_queue(120, keep_frames=True)
	prof = pipe.start(cfg_of(), q)
	maximize_queues(prof)
	time.sleep(1.5)
	while True:                      # 露光待ちのぶんを捨てる
		try:
			q.poll_for_frame()
		except Exception:
			break
		if q.poll_for_frame():
			continue
		break
	fns = []
	t0 = time.perf_counter()
	k = 0
	while time.perf_counter() - t0 < SEC:
		try:
			fs = q.wait_for_frame(3000).as_frameset()
		except Exception:
			continue
		c = fs.get_color_frame()
		if not c:
			continue
		fns.append(int(c.get_frame_number()))
		k += 1
		if stall and k % STALL_EVERY == 0:
			time.sleep(STALL_MS / 1000.0)
	pipe.stop()
	return len(fns), gaps_of(fns)


def m_callback(stall):
	"""コールバック方式: librealsense のスレッドが直接呼んでくる。"""
	fns = []
	lock = threading.Lock()

	def cb(frame):
		try:
			fs = frame.as_frameset()
			if not fs:
				return
			c = fs.get_color_frame()
			if not c:
				return
			with lock:
				fns.append(int(c.get_frame_number()))
		except Exception:
			pass

	pipe = rs.pipeline()
	prof = pipe.start(cfg_of(), cb)
	maximize_queues(prof)
	time.sleep(1.5)
	with lock:
		fns.clear()
	t0 = time.perf_counter()
	k = 0
	while time.perf_counter() - t0 < SEC:
		time.sleep(1.0 / FPS)
		k += 1
		if stall and k % STALL_EVERY == 0:
			time.sleep(STALL_MS / 1000.0)     # メインスレッドを止める
	pipe.stop()
	time.sleep(0.5)
	with lock:
		return len(fns), gaps_of(fns)


print(f"実機 {SEC} 秒ずつ。妨害 = {STALL_EVERY} フレームごとに {STALL_MS} ms 止める")
print(f"（{SEC}秒なので妨害は約 {int(SEC * FPS / STALL_EVERY)} 回入る）\n")
print(f"{'方式':44s} {'妨害':>4s} {'受信':>6s} {'番号の飛び':>9s}")
for name, fn in (("1) wait_for_frames (現状)", m_wait),
                  ("2) 明示的な frame_queue(120)", m_queue),
                  ("3) コールバック方式", m_callback)):
	for stall in (False, True):
		try:
			n, g = fn(stall)
			print(f"{name:44s} {'あり' if stall else 'なし':>4s} {n:6d} {g:9d}")
		except Exception as e:
			print(f"{name:44s} {'あり' if stall else 'なし':>4s}  例外: {e}")
		time.sleep(2)
