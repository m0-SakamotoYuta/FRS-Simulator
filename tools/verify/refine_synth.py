"""合成画像で、コーナー精緻化の「かたより(bias)」と「ばらつき(jitter)」を分けて測る。

【なぜ要るのか】
実録画で測れるのは「滑らかさ」だけ。滑らかでも系統的にズレていれば意味がない
(平滑化しすぎて動かない、など)。合成画像なら真のコーナー位置が分かるので、
  ・かたより = 平均誤差 (真値からのズレ)
  ・ばらつき = 標準偏差
を分けて出せる。

マーカーをサブピクセル単位でずらしながら多数枚作り、
ぼかし・ノイズ・コントラスト低下を実機相当に加えて検出する。
"""
import argparse

import cv2
import numpy as np

MODES = ["NONE", "CONTOUR", "SUBPIX", "APRILTAG"]


def make_params(mode):
	p = cv2.aruco.DetectorParameters()
	p.adaptiveThreshWinSizeMin = 3
	p.adaptiveThreshWinSizeMax = 23
	p.adaptiveThreshWinSizeStep = 4
	p.adaptiveThreshConstant = 7
	p.minMarkerPerimeterRate = 0.02
	p.maxMarkerPerimeterRate = 4.0
	p.polygonalApproxAccuracyRate = 0.03
	p.maxErroneousBitsInBorderRate = 0.15
	p.errorCorrectionRate = 0.6
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
	return p


def render(marker_px, tilt_deg, dx, dy, blur, noise, contrast, W=640, H=480,
            dict_id=cv2.aruco.DICT_4X4_100, mid=2, ss=8):
	"""マーカーを1枚描く。ss倍で描いてから縮小し、サブピクセル位置を正しく作る。

	Returns: (画像, 真のコーナー4点)
	"""
	dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
	# 余白1セルぶんを含むタグ画像 (境界の黒枠は generateImageMarker が付ける)
	side = 400
	tag = cv2.aruco.generateImageMarker(dictionary, mid, side)
	tag = cv2.copyMakeBorder(tag, side // 6, side // 6, side // 6, side // 6,
	                          cv2.BORDER_CONSTANT, value=255)
	src = np.array([[0, 0], [tag.shape[1] - 1, 0],
	                 [tag.shape[1] - 1, tag.shape[0] - 1], [0, tag.shape[0] - 1]], np.float32)
	# マーカー本体 (黒枠の外側) が marker_px になるように、余白ぶんを外に出す
	half = marker_px / 2.0
	pad = marker_px / 6.0
	cx, cy = W / 2.0 + dx, H / 2.0 + dy
	th = np.deg2rad(tilt_deg)
	# 面外の傾き: x 方向を cos(th) に縮め、奥行きで遠近を付ける
	def proj(ux, uy):
		X, Y, Z = ux * np.cos(th), uy, 1000.0 + ux * np.sin(th)
		f = 1000.0
		return cx + f * X / Z, cy + f * Y / Z
	outer = [(-half - pad, -half - pad), (half + pad, -half - pad),
	          (half + pad, half + pad), (-half - pad, half + pad)]
	inner = [(-half, -half), (half, -half), (half, half), (-half, half)]
	dst = np.array([proj(*q) for q in outer], np.float32)
	truth = np.array([proj(*q) for q in inner], np.float64)
	Msup = np.float32([[ss, 0, 0], [0, ss, 0], [0, 0, 1]])
	Hm = cv2.getPerspectiveTransform(src, dst)
	img = cv2.warpPerspective(tag, Msup @ Hm, (W * ss, H * ss),
	                           flags=cv2.INTER_NEAREST, borderValue=255)
	img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
	if blur > 0:
		img = cv2.GaussianBlur(img, (0, 0), blur)
	img = (img.astype(np.float32) - 128.0) * contrast + 128.0
	if noise > 0:
		img += np.random.normal(0, noise, img.shape).astype(np.float32)
	return np.clip(img, 0, 255).astype(np.uint8), truth


def order_like(det, truth):
	"""検出コーナーを真値と同じ並びに合わせる (回転のずれを吸収)。"""
	best, bd = det, np.inf
	for k in range(4):
		cand = np.roll(det, k, axis=0)
		d = float(np.sum(np.linalg.norm(cand - truth, axis=1)))
		if d < bd:
			best, bd = cand, d
	return best


def run(n, marker_px, tilt, blur, noise, contrast, seed=0):
	rng = np.random.default_rng(seed)
	dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
	dets = {m: cv2.aruco.ArucoDetector(dictionary, make_params(m)) for m in MODES}
	err = {m: [] for m in MODES}
	np.random.seed(seed)
	for _ in range(n):
		dx, dy = rng.uniform(-1, 1), rng.uniform(-1, 1)   # サブピクセルのずらし
		img, truth = render(marker_px, tilt, dx, dy, blur, noise, contrast)
		for m in MODES:
			cs, ids, _ = dets[m].detectMarkers(img)
			if ids is None or len(cs) == 0:
				continue
			c = order_like(np.asarray(cs[0]).reshape(-1, 2).astype(np.float64), truth)
			err[m].append(c - truth)
	out = {}
	for m in MODES:
		if not err[m]:
			out[m] = None
			continue
		e = np.concatenate(err[m], axis=0)          # (N*4, 2)
		out[m] = {
			"n": len(err[m]),
			"かたより": float(np.linalg.norm(e.mean(axis=0))),
			"ばらつき": float(np.sqrt(np.mean(e.std(axis=0) ** 2))),
			"全誤差RMS": float(np.sqrt(np.mean(np.sum(e ** 2, axis=1)))),
		}
	return out


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("--n", type=int, default=60)
	ap.add_argument("--blur", type=float, default=0.8)
	ap.add_argument("--noise", type=float, default=2.0)
	ap.add_argument("--contrast", type=float, default=0.8)
	a = ap.parse_args()
	print("合成マーカーでのコーナー誤差 [px]")
	print(f"  ぼかしσ={a.blur} ノイズσ={a.noise} コントラスト={a.contrast}  各{a.n}枚\n")
	for marker_px, tilt, label in ((100, 0, "100px 正対 (実機: 20mm@130mm 相当)"),
	                                (100, 60, "100px 面外60° (U/V 相当)"),
	                                (50, 0, "50px 正対 (10mm マーカー相当)")):
		print(f"--- {label} ---")
		res = run(a.n, marker_px, tilt, a.blur, a.noise, a.contrast)
		for m in MODES:
			r = res[m]
			if r is None:
				print(f"  {m:9s} 検出できず")
				continue
			print(f"  {m:9s} かたより {r['かたより']:6.4f}  "
			      f"ばらつき {r['ばらつき']:6.4f}  全誤差RMS {r['全誤差RMS']:6.4f}  "
			      f"({r['n']}/{a.n}枚)")
		print()


if __name__ == "__main__":
	main()
