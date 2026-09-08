"""U/V/W のばらつきの正体を調べる。

知りたいこと:
  1) 残差(直線からのズレ)の大きさは軸ごとにどれだけ違うか
  2) それは白色ノイズか、低周波のドリフトか
     → 白色なら平滑化が効く。低周波なら平滑化では消えず、撮影条件を変えるしかない
  3) 平滑化(ローパス)を掛けると残差と k がどう変わるか
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy import signal

ROOT = Path(r"C:\Users\yutas\Desktop\FRS-Simulator")
D = ROOT / "cache" / "av_result"


def unflatten(path, arrays, meta):
	e = meta.get(path)
	if e is None:
		return None
	if e["t"] == "dict":
		return {k: unflatten(f"{path}.{k}", arrays, meta) for k in e["keys"]}
	if e["t"] == "arr":
		return arrays[int(e["i"])]
	return e["v"]


def load(ax):
	p = D / f"av_result_{ax}.npz"
	with np.load(str(p), allow_pickle=True) as z:
		blob = json.loads(str(z["meta_json"].item()))
		n_arr = sum(1 for k in z.files if k.startswith("a") and k[1:].isdigit())
		arrays = [z[f"a{i}"] for i in range(n_arr)]
		return unflatten("root", arrays, blob["meta"])


CMD = {"X": (50.0, 5.0), "Y": (50.0, 5.0), "Z": (50.0, 5.0),
       "U": (20.0, 1.0), "V": (20.0, 1.0), "W": (20.0, 1.0)}

print(f"{'軸':3s} {'幾何角':>6s} {'距離':>6s} {'残差RMS':>9s} {'白色成分':>9s} "
      f"{'低周波成分':>10s} {'白色の割合':>9s}")
print("-" * 68)
rows = {}
for ax in "XYZUVW":
	res = load(ax)
	kind = res["kind"]
	prim = res["series"]["rgb"][kind]
	ts = np.asarray(res["timestamps"], float)
	a = np.asarray(prim["along"], float)
	pm = np.asarray(prim["plot_mask"], bool)
	m = pm & np.isfinite(a)
	t = ts[m]; y = a[m]
	# 直線を引いて残差を見る
	A = np.vstack([t, np.ones_like(t)]).T
	sol, *_ = np.linalg.lstsq(A, y, rcond=None)
	r = y - A @ sol
	dt = float(np.median(np.diff(t)))
	fs = 1.0 / dt
	# 1 Hz でローパス → 低周波成分 / 残りが白色成分
	b, aa = signal.butter(2, 1.0 / (fs / 2), btype="low")
	lo = signal.filtfilt(b, aa, r)
	hi = r - lo
	rows[ax] = dict(t=t, y=y, r=r, fs=fs, slope=sol[0], kind=kind,
	                geom=res.get("geom_deg", float("nan")),
	                dist=res.get("dist_mm", float("nan")),
	                feed=CMD[ax][1])
	unit = "mm" if kind == "lin" else "°"
	print(f"{ax:3s} {res.get('geom_deg', float('nan')):6.0f} "
	      f"{res.get('dist_mm', float('nan')):6.0f} "
	      f"{r.std():7.4f}{unit:2s} {hi.std():7.4f}{unit:2s} {lo.std():8.4f}{unit:2s} "
	      f"{100 * hi.var() / max(r.var(), 1e-12):8.1f}%")

print()
print("=== ローパスの効き方 (残差RMS と k の変化) ===")
print(f"{'軸':3s} {'カットオフ':>9s} {'残差RMS':>9s} {'低減':>6s} {'k':>9s} {'誤差':>8s}")
print("-" * 54)
for ax in "UVW":
	d = rows[ax]
	t, y, fs, feed = d["t"], d["y"], d["fs"], d["feed"]
	A = np.vstack([t, np.ones_like(t)]).T
	base_r = (y - A @ np.linalg.lstsq(A, y, rcond=None)[0]).std()
	for fc in (None, 3.0, 1.5, 1.0, 0.5):
		if fc is None:
			yy = y
			lab = "なし"
		else:
			b, aa = signal.butter(2, fc / (fs / 2), btype="low")
			yy = signal.filtfilt(b, aa, y)
			lab = f"{fc:.1f} Hz"
		sol, *_ = np.linalg.lstsq(A, yy, rcond=None)
		r = yy - A @ sol
		k = sol[0] / feed
		print(f"{ax:3s} {lab:>9s} {r.std():8.4f}° "
		      f"{100 * r.std() / base_r:5.0f}% {k:9.5f} {100 * (k - 1):+7.3f}%")
	print()

print("=== 残差のスペクトル (どの周波数にエネルギーがあるか) ===")
for ax in "UVW":
	d = rows[ax]
	r = d["r"]; fs = d["fs"]
	f, P = signal.welch(r, fs=fs, nperseg=min(256, len(r)))
	tot = P.sum()
	bands = [(0, 0.5), (0.5, 1.5), (1.5, 3), (3, 6), (6, 15)]
	txt = "  ".join(f"{lo}-{hi}Hz:{100 * P[(f >= lo) & (f < hi)].sum() / tot:4.1f}%"
	                 for lo, hi in bands)
	print(f"  {ax}: {txt}")
print()
print("※ 低い周波数にエネルギーが集中していれば、ローパスでは消せない")
