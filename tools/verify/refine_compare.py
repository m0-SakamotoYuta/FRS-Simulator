"""コーナー精緻化の方式を変えると、実際にどれだけ精度が変わるかを実測する。

【何を確かめたいのか】
「ArUco は輪郭近似だからコーナーがバタつく／AprilTag は勾配から直線を当てるので
安定する」という主張が、この装置の実データで本当に効くのかを見る。

本体 (FRS-SIMULATOR.py の _ankle_make_detector) は既に
CORNER_REFINE_APRILTAG を使っている。これは AprilTag のコーナー精緻化そのもので、
OpenCV の ArUco 検出器に移植されたもの。つまり「輪郭近似のまま」ではない。

そこで NONE (精緻化なし=輪郭そのまま) から APRILTAG までを同じ録画で比べれば、
  ・コーナー精度を上げると姿勢がどれだけ良くなるのか (伸びしろの大きさ)
  ・現行設定が既に頭打ちなのか
が数字で出る。頭打ちなら、検出器を AprilTag に替えても得られるものは小さい。

【測り方】
静止区間が無くても測れるように、時系列に Savitzky-Golay (2次) を掛け、
その残差 (=高周波成分) を「ジッタ」とする。ロボットの動きは滑らかなので
残差にはほぼ載らない。コーナー(px) と 姿勢(deg/mm) の両方で測る。

使い方:
    set PYTHONIOENCODING=utf-8
    python tools/verify/refine_compare.py <録画.db3> [--id 1] [--size 20] [--max 600]
    python tools/verify/refine_compare.py --tab "260904"      # 状態ファイルから U/W を拾う
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.signal import savgol_filter

ROOT = Path(__file__).resolve().parents[2]


# ---------------------------------------------------------------- 検出器
def make_params(mode: str):
	"""比較する4通りの検出パラメータ。精緻化以外は本体と同じにする。"""
	p = cv2.aruco.DetectorParameters()
	# --- 本体 _ankle_make_detector と同じ高精度寄り設定 ---
	p.adaptiveThreshWinSizeMin = 3
	p.adaptiveThreshWinSizeMax = 23
	p.adaptiveThreshWinSizeStep = 4
	p.adaptiveThreshConstant = 7
	p.minMarkerPerimeterRate = 0.02
	p.maxMarkerPerimeterRate = 4.0
	p.polygonalApproxAccuracyRate = 0.03
	p.maxErroneousBitsInBorderRate = 0.15
	p.errorCorrectionRate = 0.6
	# --- ここだけを変える ---
	if mode == "NONE":
		p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
	elif mode == "CONTOUR":
		p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
	elif mode == "SUBPIX":
		p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
		p.cornerRefinementWinSize = 7
		p.cornerRefinementMaxIterations = 50
		p.cornerRefinementMinAccuracy = 0.001
	elif mode == "APRILTAG":
		p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
		p.aprilTagQuadDecimate = 1.0
		p.aprilTagQuadSigma = 0.0
		p.aprilTagMinClusterPixels = 5
		p.aprilTagMaxNmaxima = 10
		p.aprilTagCriticalRad = 10.0 * np.pi / 180.0
		p.aprilTagMaxLineFitMse = 10.0
		p.aprilTagMinWhiteBlackDiff = 5
		p.aprilTagDeglitch = 0
	else:
		raise ValueError(mode)
	return p


def to_gray(a):
	"""カラーフレームをグレースケールにする (本体 _ankle_color_to_gray_bgr と同じ扱い)。

	pyrealsense2 は YUYV を (H, W) の uint16 で返す (1画素2バイトが1要素)。
	OpenCV は (H, W, 2) の uint8 を要求するのでビューを張り替える。
	"""
	a = np.asanyarray(a)
	if a.ndim == 2 and a.dtype == np.uint16:
		a = a.view(np.uint8).reshape(a.shape[0], a.shape[1], 2)
		return cv2.cvtColor(a, cv2.COLOR_YUV2GRAY_YUY2)
	if a.ndim == 2:
		return a
	ch = a.shape[2]
	if ch == 2:
		return cv2.cvtColor(a, cv2.COLOR_YUV2GRAY_YUY2)
	if ch == 4:
		return cv2.cvtColor(a, cv2.COLOR_RGBA2GRAY)
	return cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)


def obj_points(size_mm: float):
	s = float(size_mm) / 2.0
	return np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]],
	                 dtype=np.float64)


# ---------------------------------------------------------------- 録画の読み込み
def load_gray_frames(bag: str, max_frames: int):
	"""色フレームをグレースケールでまとめて読み出す (検出は後でまとめて回す)。"""
	pipeline = rs.pipeline()
	config = rs.config()
	rs.config.enable_device_from_file(config, str(bag), repeat_playback=False)
	config.enable_stream(rs.stream.color)
	profile = pipeline.start(config)
	try:
		profile.get_device().as_playback().set_real_time(False)
	except Exception:
		pass
	vsp = profile.get_stream(rs.stream.color).as_video_stream_profile()
	intr = vsp.get_intrinsics()
	K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], float)
	dist = np.array(intr.coeffs, float)
	grays, ts = [], []
	try:
		while len(grays) < max_frames:
			try:
				fs = pipeline.wait_for_frames(2000)
			except Exception:
				break
			cf = fs.get_color_frame()
			if not cf:
				continue
			grays.append(to_gray(np.asanyarray(cf.get_data())).copy())
			ts.append(float(cf.get_timestamp()) / 1000.0)
	finally:
		pipeline.stop()
	return grays, np.asarray(ts, float), K, dist, (intr.width, intr.height, intr.fx)


# ---------------------------------------------------------------- 指標
def highpass_rms(y, t, win_s=0.25, order=2):
	"""Savitzky-Golay で滑らかな成分を抜き、残差 (=ジッタ) の RMS を返す。"""
	y = np.asarray(y, float)
	m = np.isfinite(y)
	if m.sum() < 15:
		return float("nan")
	yy, tt = y[m], t[m]
	dt = float(np.median(np.diff(tt))) if len(tt) > 1 else 0.0
	if dt <= 0:
		return float("nan")
	win = max(order + 2, int(round(win_s / dt)) | 1)
	win = min(win, len(yy) if len(yy) % 2 == 1 else len(yy) - 1)
	if win <= order + 1:
		return float("nan")
	return float(np.sqrt(np.mean((yy - savgol_filter(yy, win, order)) ** 2)))


def accuracy(rot, tra, ts, kind, feed, band_ref=None):
	"""ロボットの指令と突き合わせた精度。AV タブの k と同じ考え方。

	並進試験: |p(t)-p(t0)| / 回転試験: ∠(R(t0)^T R(t)) を主軸へ投影し、
	移動量の中央50%(等速区間)で 位置 vs 時刻 を回帰した傾きを送り速度と比べる。
	直交残差 (真直度 / 回転軸のブレ) も返す。
	"""
	from scipy.spatial.transform import Rotation as Rot
	m = np.all(np.isfinite(tra), axis=1) & np.all(np.isfinite(rot), axis=1)
	if m.sum() < 30:
		return float("nan"), float("nan"), None
	idx = np.where(m)[0]
	i0 = idx[0]
	if kind == "lin":
		vec = tra[idx] - tra[i0]
	else:
		R = Rot.from_rotvec(np.deg2rad(rot[idx])).as_matrix()
		R0T = R[0].T
		vec = Rot.from_matrix(np.einsum("ij,mjk->mik", R0T, R)).as_rotvec() * 180.0 / np.pi
	_u, _s, Vt = np.linalg.svd(vec, full_matrices=False)
	d = Vt[0] / max(np.linalg.norm(Vt[0]), 1e-12)
	a = vec @ d
	if abs(a.min()) > abs(a.max()):
		d, a = -d, -a
	perp = float(np.sqrt(np.mean(np.linalg.norm(vec - np.outer(a, d), axis=1) ** 2)))
	if band_ref is not None:
		# 窓を固定する。方式ごとに窓が動くと、検出器の差なのか窓の差なのか
		# 分からなくなる (実際に V軸で k が +6.8% と大きく外れて見えた)。
		band = np.asarray(band_ref, bool)[idx]
	else:
		rng = a.max() - a.min()
		band = (a >= a.min() + 0.25 * rng) & (a <= a.min() + 0.75 * rng)
		# 往復に備えて最長の連続区間だけ採る (AV タブと同じ)
		pad = np.concatenate(([0], band.astype(np.int8), [0]))
		dd = np.diff(pad); st = np.where(dd == 1)[0]; en = np.where(dd == -1)[0]
		if len(st):
			bi = int(np.argmax(en - st))
			band = np.zeros_like(band); band[st[bi]:en[bi]] = True
	if band.sum() < 10:
		return float("nan"), perp, None
	tt, aa = ts[idx][band], a[band]
	A = np.vstack([tt, np.ones_like(tt)]).T
	sol, *_ = np.linalg.lstsq(A, aa, rcond=None)
	k = float(sol[0]) / feed if feed else float("nan")
	full = np.zeros(len(ts), bool); full[idx[band]] = True
	return k, perp, full


def run_variant(mode, grays, ts, K, dist, target_id, size_mm):
	"""1つの精緻化方式で全フレームを検出し、コーナーと姿勢を返す。"""
	params = make_params(mode)
	dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
	detector = cv2.aruco.ArucoDetector(dictionary, params)
	op = obj_points(size_mm)
	n = len(grays)
	corners = np.full((n, 4, 2), np.nan)
	rot = np.full((n, 3), np.nan)      # 回転ベクトル (deg)
	tra = np.full((n, 3), np.nan)      # 並進 (mm)
	t0 = time.perf_counter()
	prev_R = None
	for i, g in enumerate(grays):
		cs, ids, _ = detector.detectMarkers(g)
		if ids is None:
			continue
		for c, mid in zip(cs, np.asarray(ids).reshape(-1)):
			if int(mid) != target_id:
				continue
			ip = c.reshape(-1, 2).astype(np.float64)
			corners[i] = ip
			try:
				retval, rvecs, tvecs, _e = cv2.solvePnPGeneric(
					op, ip, K, dist, flags=cv2.SOLVEPNP_IPPE)
				ns = int(retval) if retval else len(rvecs)
			except Exception:
				continue
			# 表裏2解のうち、直前フレームに近い方を採る (深度を使わずに一貫させる)
			best, best_d = 0, np.inf
			for si in range(ns):
				R, _ = cv2.Rodrigues(rvecs[si])
				d = 0.0 if prev_R is None else float(
					np.linalg.norm(cv2.Rodrigues(prev_R.T @ R)[0]))
				if d < best_d:
					best, best_d = si, d
			R, _ = cv2.Rodrigues(rvecs[best])
			prev_R = R
			rot[i] = cv2.Rodrigues(R)[0].flatten() * 180.0 / np.pi
			tra[i] = np.asarray(tvecs[best], float).flatten()
			break
	ms = (time.perf_counter() - t0) / max(n, 1) * 1000.0
	det = np.isfinite(corners[:, 0, 0])
	return {"mode": mode, "corners": corners, "rot": rot, "tra": tra,
	        "det_rate": 100.0 * det.mean(), "ms_per_frame": ms}


def summarize(r, ts):
	"""コーナー・姿勢のジッタをまとめる。"""
	c = r["corners"]
	# 4隅 × (x,y) の 8 系列それぞれのジッタを RMS でまとめる
	px = [highpass_rms(c[:, k, ax], ts) for k in range(4) for ax in range(2)]
	px = np.asarray(px, float)
	rotj = np.asarray([highpass_rms(r["rot"][:, i], ts) for i in range(3)], float)
	traj = np.asarray([highpass_rms(r["tra"][:, i], ts) for i in range(3)], float)
	return {
		"検出率": r["det_rate"],
		"コーナー px": float(np.sqrt(np.nanmean(px ** 2))),
		"回転 deg": float(np.sqrt(np.nanmean(rotj ** 2))),
		"並進 mm": float(np.sqrt(np.nanmean(traj ** 2))),
		"ms/枚": r["ms_per_frame"],
	}


# ---------------------------------------------------------------- 本体
def compare(bag, target_id, size_mm, max_frames, modes, kind="rot", feed=0.0):
	print(f"\n{'=' * 78}")
	print(f"録画: {Path(bag).name}   ID={target_id}  実寸={size_mm}mm")
	grays, ts, K, dist, meta = load_gray_frames(bag, max_frames)
	if not grays:
		print("  フレームを読めませんでした")
		return None
	w, h, fx = meta
	print(f"  {len(grays)} 枚 / {w}x{h} / fx={fx:.1f}")
	unit = "mm" if kind == "lin" else "deg"
	rows = {}
	# 窓は最初の方式で決めて、以降は同じ窓を使い回す (公平に比べるため)
	band_ref = None
	for mode in modes:
		r = run_variant(mode, grays, ts, K, dist, target_id, size_mm)
		s = summarize(r, ts)
		s["k"], s["残差"], _b = accuracy(r["rot"], r["tra"], ts, kind, feed, band_ref)
		if band_ref is None and _b is not None:
			band_ref = _b
		rows[mode] = s
		print(f"  {mode:9s} 検出{s['検出率']:5.1f}%  "
		      f"ジッタ: コーナー {s['コーナー px']:6.4f}px / "
		      f"回転 {s['回転 deg']:7.5f}deg / 並進 {s['並進 mm']:6.4f}mm  "
		      f"| 精度: k={s['k']:.5f} ({(s['k'] - 1) * 100:+6.2f}%) "
		      f"残差 {s['残差']:7.4f}{unit}  {s['ms/枚']:6.1f}ms")
	base = rows.get("APRILTAG")
	if base:
		print(f"  --- APRILTAG (現行) を 1.00 としたときの比 ---")
		for mode, s in rows.items():
			print(f"  {mode:9s} コーナー x{s['コーナー px'] / base['コーナー px']:5.2f}  "
			      f"回転 x{s['回転 deg'] / base['回転 deg']:5.2f}  "
			      f"並進 x{s['並進 mm'] / base['並進 mm']:5.2f}  "
			      f"残差 x{s['残差'] / base['残差']:5.2f}")
	return rows


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("bag", nargs="?", help="録画 .db3")
	ap.add_argument("--tab", help="状態ファイルのタブ名 (部分一致) から録画を拾う")
	ap.add_argument("--axes", default="U,W", help="--tab のときに見る軸")
	ap.add_argument("--id", type=int, default=None, help="対象マーカーID")
	ap.add_argument("--size", type=float, default=None, help="マーカー実寸 mm")
	ap.add_argument("--max", type=int, default=600, help="読むフレーム数の上限")
	ap.add_argument("--modes", default="NONE,CONTOUR,SUBPIX,APRILTAG")
	ap.add_argument("--kind", default="rot", choices=["lin", "rot"])
	ap.add_argument("--feed", type=float, default=0.0, help="送り速度 (mm/s or deg/s)")
	a = ap.parse_args()
	modes = [m.strip() for m in a.modes.split(",") if m.strip()]

	jobs = []
	if a.tab:
		st = json.load((ROOT / "frs2015_gui_state_aruco_verify.json").open(encoding="utf-8"))
		for t in st.get("tabs", []):
			if a.tab not in str(t.get("name", "")):
				continue
			s = t.get("settings") or {}
			size = float(s.get("marker_size_mm", 20.0))
			for ax in [x.strip() for x in a.axes.split(",")]:
				row = (s.get("rows") or {}).get(ax) or {}
				bag = row.get("bag") or ""
				if not (bag and Path(bag).exists()):
					print(f"[skip] {t.get('name')} {ax}軸: 録画が見つかりません")
					continue
				mid = a.id if a.id is not None else int(
					s.get("id_rot" if ax in "UVW" else "id_lin", 1))
				jobs.append((bag, mid, a.size or size,
				              "rot" if ax in "UVW" else "lin",
				              float(row.get("feed") or 0.0)))
	elif a.bag:
		jobs.append((a.bag, a.id if a.id is not None else 1, a.size or 20.0,
		              a.kind, a.feed))
	else:
		ap.error("録画か --tab を指定してください")

	for bag, mid, size, kind, feed in jobs:
		compare(bag, mid, size, a.max, modes, kind, feed)


if __name__ == "__main__":
	main()
