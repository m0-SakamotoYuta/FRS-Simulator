"""実機での最終検証: アプリと同じ負荷をかけて、現行方式とリング方式を比べる。

アプリの録画ループが実際にやっていることを再現する:
  ・align.process → colorize → cvtColor
  ・ArUco 検出 (プレビュー用の軽い検出器)
  ・MP4 3本の書き出し
  ・これらを preview_every で間引く
これを両方式に同じだけ掛けて、記録がどうなるかを見る。
"""
import importlib.util
import sqlite3
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

ROOT = Path(r"C:\Users\yutas\Desktop\FRS-Simulator")
CACHE = ROOT / "cache"
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location("frs", str(ROOT / "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(frs)
Rec = frs._AnkleRamRecorder
G = frs.MainMenuGUI

W, H, FPS = 1280, 720, 30
SEC = int(sys.argv[1]) if len(sys.argv) > 1 else 45
EVERY = 3
FOURCC = cv2.VideoWriter_fourcc(*"mp4v")

# アプリと同じ「プレビュー用の軽い検出器」(GUIを作らずにメソッドだけ借りる)
class _Sh:
	_ankle_make_preview_detector = G._ankle_make_preview_detector
	_ankle_resolve_aruco_dict = G._ankle_resolve_aruco_dict


det, dic, prm, newapi = _Sh()._ankle_make_preview_detector("DICT_4X4_100")


def to_gray_bgr(a):
	a = np.asanyarray(a)
	if a.ndim == 2 and a.dtype == np.uint16:
		a = a.view(np.uint8).reshape(a.shape[0], a.shape[1], 2)
		return (cv2.cvtColor(a, cv2.COLOR_YUV2GRAY_YUY2),
		        cv2.cvtColor(a, cv2.COLOR_YUV2BGR_YUY2))
	return cv2.cvtColor(a, cv2.COLOR_RGB2GRAY), cv2.cvtColor(a, cv2.COLOR_RGB2BGR)


def bag_stats(p):
	con = sqlite3.connect(str(p))
	try:
		ts = [r[0] for r in con.execute(
			"SELECT m.timestamp FROM messages m JOIN topics t ON m.topic_id=t.id "
			"WHERE t.name LIKE '%Color%image/data' ORDER BY m.timestamp")]
	finally:
		con.close()
	n = len(ts)
	if n < 3:
		return n, 0.0, 0
	a = np.asarray(ts, float) / 1e9
	d = np.diff(a)
	nom = float(np.median(d))
	return n, float(a[-1] - a[0]), max(0, int(np.sum(np.round(d / nom) - 1)))


def run(label, use_ring, name):
	out = CACHE / name
	for suf in ("", ".capture.json", "_c.mp4", "_d.mp4", "_a.mp4"):
		try:
			Path(str(out) + suf).unlink()
		except Exception:
			pass
	pipe = rs.pipeline()
	cfg = rs.config()
	cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
	cfg.enable_stream(rs.stream.color, W, H, rs.format.yuyv, FPS)
	if not use_ring:
		cfg.enable_record_to_file(str(out))
	prof = pipe.start(cfg)
	for sens in prof.get_device().query_sensors():
		try:
			if sens.supports(rs.option.frames_queue_size):
				r_ = sens.get_option_range(rs.option.frames_queue_size)
				sens.set_option(rs.option.frames_queue_size, float(r_.max))
		except Exception:
			pass
	recorder = None
	if not use_ring:
		recorder = prof.get_device().as_recorder()
		recorder.pause()
	for _ in range(45):
		pipe.wait_for_frames(5000)

	# MP4 writer 3本 (アプリと同じく録画開始前に作る)
	vw = [cv2.VideoWriter(str(out) + s, FOURCC, FPS / EVERY, (W, H))
	      for s in ("_c.mp4", "_d.mp4", "_a.mp4")]
	align = rs.align(rs.stream.color)
	colorizer = rs.colorizer()
	ram = None
	if use_ring:
		cvp = prof.get_stream(rs.stream.color).as_video_stream_profile()
		dvp = prof.get_stream(rs.stream.depth).as_video_stream_profile()
		meta = {"w": W, "h": H, "fps": FPS, "color_bpp": 2, "color_fmt": cvp.format(),
		        "color_intr": cvp.get_intrinsics(), "depth_intr": dvp.get_intrinsics(),
		        "extrinsics": dvp.get_extrinsics_to(cvp),
		        "depth_units": float(prof.get_device().first_depth_sensor().get_depth_scale())}
		ram = Rec(str(out), meta, 8 * FPS, log=lambda *a: None)
		ram.start()
	else:
		recorder.resume()

	n = 0
	prev_fn = None
	fn_gaps = 0
	lat = []
	ev = EVERY * (2 if use_ring else 1)      # RAM録画では更に間引く (アプリと同じ)
	t0 = time.perf_counter()
	while time.perf_counter() - t0 < SEC:
		try:
			fs = pipe.wait_for_frames(3000)
		except RuntimeError:
			print("  タイムアウト")
			break
		c = fs.get_color_frame()
		d0 = fs.get_depth_frame()
		if not c or not d0:
			continue
		ta = time.perf_counter()
		fn = int(c.get_frame_number())
		if prev_fn is not None and fn > prev_fn + 1:
			fn_gaps += fn - prev_fn - 1
		prev_fn = fn
		n += 1
		if use_ring:
			ram.push(c.get_data(), d0.get_data(), c.get_timestamp(), fn)
		if n % ev == 0:
			al = align.process(fs)
			cc = al.get_color_frame(); dd = al.get_depth_frame()
			if cc and dd:
				gray, bgr = to_gray_bgr(cc.get_data())
				dbgr = cv2.cvtColor(np.asanyarray(colorizer.colorize(dd).get_data()),
				                     cv2.COLOR_RGB2BGR)
				if newapi and det is not None:
					det.detectMarkers(gray)
				else:
					cv2.aruco.detectMarkers(gray, dic, parameters=prm)
				for k, img in enumerate((bgr, dbgr, bgr)):
					if vw[k] is not None and vw[k].isOpened():
						vw[k].write(img)
		lat.append(time.perf_counter() - ta)
	wall = time.perf_counter() - t0
	st = ram.stop() if use_ring else None
	if not use_ring:
		try:
			recorder.pause()
		except Exception:
			pass
	for v in vw:
		try:
			v.release()
		except Exception:
			pass
	pipe.stop()
	time.sleep(1.5)

	lat = np.asarray(lat) * 1000.0
	bn, bd, bg = bag_stats(out)
	print(f"\n===== {label} =====")
	print(f"  アプリが受けた   : {n} 枚 / {wall:.1f}s = {n / wall:.2f} fps"
	      f"（期待 {SEC * FPS}）")
	print(f"  フレーム番号の飛び: {fn_gaps} 枚")
	print(f"  ループ1周        : 中央 {np.median(lat):.2f} ms / 最大 {lat.max():.1f} ms / "
	      f"33.3ms超え {int((lat > 33.3).sum())} 回")
	if st:
		print(f"  リング           : 取りこぼし {st['dropped']} / 最大使用 "
		      f"{st['max_used']}/{st['cap']} / 詰まり {len(st['stalls'])} 回"
		      + (f" (最大 {max(x[1] for x in st['stalls']):.0f} ms)" if st["stalls"] else "")
		      + f" / 残り処理 {st['drain_s']:.2f}s")
	print(f"  ★ .db3          : {bn} 枚 / {bd:.2f}s = {bn / max(bd, 1e-9):.2f} fps / "
	      f"内部の空白 {bg} 枚 / {out.stat().st_size / 1e9:.2f} GB")
	# 再生で枚数が合うか
	pp = rs.pipeline(); cc2 = rs.config()
	rs.config.enable_device_from_file(cc2, str(out), repeat_playback=False)
	cc2.enable_stream(rs.stream.color); cc2.enable_stream(rs.stream.depth)
	pr = pp.start(cc2)
	try:
		pr.get_device().as_playback().set_real_time(False)
	except Exception:
		pass
	al2 = rs.align(rs.stream.color)
	m = 0
	seen = set()
	dup = 0
	while True:
		try:
			f2 = pp.wait_for_frames(3000)
		except RuntimeError:
			break
		a2 = al2.process(f2)
		c2 = a2.get_color_frame(); d2 = a2.get_depth_frame()
		if not c2 or not d2:
			break
		k = int(c2.get_frame_number())
		if k in seen:
			dup += 1
		seen.add(k)
		m += 1
	pp.stop()
	print(f"  ④と同じ手順で再生: {m} 枚 / 重複 {dup} 枚  → "
	      f"{'一致' if (m == bn and dup == 0) else '不一致'}")
	for suf in ("", ".capture.json", "_c.mp4", "_d.mp4", "_a.mp4"):
		try:
			Path(str(out) + suf).unlink()
		except Exception:
			pass
	return {"app": n, "bag": bn, "gaps": bg, "play": m, "dup": dup,
	        "exp": SEC * FPS, "st": st}


print(f"実機 最終検証: アプリと同じ負荷 (align+colorize+ArUco+MP4×3) を掛けて {SEC} 秒ずつ")
ONLY = sys.argv[2] if len(sys.argv) > 2 else "both"
if ONLY in ("both", "A"):
	a = run("A) 現行方式 (SDKに直接書かせる)", False, "fin_old.db3")
	time.sleep(3)
else:
	a = {"app": 0, "bag": 0, "gaps": 0, "play": 0, "dup": 0, "exp": SEC * FPS, "st": None}
b = run("B) リング方式 (RAMリング+別スレッド)", True, "fin_ring.db3")
print("\n" + "=" * 66)
print(f"{'方式':30s} {'アプリ受信':>9s} {'記録':>7s} {'内部欠落':>8s} {'再生':>6s} {'重複':>5s}")
print(f"{'A) 現行方式':30s} {a['app']:9d} {a['bag']:7d} {a['gaps']:8d} {a['play']:6d} {a['dup']:5d}")
print(f"{'B) リング方式':30s} {b['app']:9d} {b['bag']:7d} {b['gaps']:8d} {b['play']:6d} {b['dup']:5d}")
print(f"{'期待':30s} {a['exp']:9d} {a['exp']:7d} {0:8d} {a['exp']:6d} {0:5d}")
