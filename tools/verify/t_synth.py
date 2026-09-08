"""合成データ（真値が分かっている）で、等速区間の切り出し精度を検証する。

比較するのは 3 通り:
  A) 従来: 移動量の中央 50% (25〜75%) の帯で回帰
  B) 新規: ロボット指令 (v, a, D) から窓を決めて回帰
  C) 真値: 本当の等速区間だけで回帰
故障モードも入れる:
  ・区間の途中でトラッキングが外れる (欠測)
  ・姿勢が飛ぶ (外れ値)
  ・録画の末尾が切れる
"""
import sys
import importlib.util
from pathlib import Path

import numpy as np

ROOT = Path(r"C:\Users\yutas\Desktop\FRS-Simulator")
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location("frs", str(ROOT / "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(frs)
G = frs.MainMenuGUI


def make(v, a, D, sigma, fps=30.0, pre=3.0, post=3.0, seed=0,
         dropout=None, outliers=None, cut_tail=0.0):
	rng = np.random.default_rng(seed)
	prof = G._av_profile_times(v, a, D)
	total = pre + prof["t3"] + post - cut_tail
	t = np.arange(0.0, total, 1.0 / fps)
	p = G._av_profile_position(t - pre, v, a, D, prof)
	p = p + rng.normal(0.0, sigma, size=len(t))
	keep = np.ones(len(t), dtype=bool)
	if dropout:
		lo, hi = dropout
		keep &= ~((t >= pre + lo) & (t <= pre + hi))
	if outliers:
		for (tt, amp) in outliers:
			j = int(np.argmin(np.abs(t - (pre + tt))))
			p[j] += amp
	return t[keep], p[keep], prof


def fit_band(t, a, lo_f, hi_f):
	"""従来方式: 移動量の中央 50% (最長連続区間) で回帰。"""
	rng_a = float(a.max() - a.min())
	mm = (a >= a.min() + lo_f * rng_a) & (a <= a.min() + hi_f * rng_a)
	pad = np.concatenate(([0], mm.astype(np.int8), [0]))
	dd = np.diff(pad)
	st = np.where(dd == 1)[0]; en = np.where(dd == -1)[0]
	if len(st):
		bi = int(np.argmax(en - st))
		mm = np.zeros(len(a), dtype=bool)
		mm[st[bi]:en[bi]] = True
	if mm.sum() < 6:
		return float("nan")
	A = np.vstack([t[mm], np.ones(int(mm.sum()))]).T
	sol, *_ = np.linalg.lstsq(A, a[mm], rcond=None)
	return float(sol[0])


def fit_true(t, a, prof, pre):
	m = (t >= pre + prof["t1"]) & (t <= pre + prof["t2"])
	A = np.vstack([t[m], np.ones(int(m.sum()))]).T
	sol, *_ = np.linalg.lstsq(A, a[m], rcond=None)
	return float(sol[0])


CASES = [
	("理想 (直動 50mm 5mm/s a=10)", dict(v=5.0, a=10.0, D=50.0, sigma=0.05), {}),
	("ノイズ大 sigma=0.3mm", dict(v=5.0, a=10.0, D=50.0, sigma=0.3), {}),
	("加速がゆるい a=1", dict(v=5.0, a=1.0, D=50.0, sigma=0.1), {}),
	("等速区間の真ん中で 2 秒トラッキング落ち",
	 dict(v=5.0, a=10.0, D=50.0, sigma=0.1), dict(dropout=(4.0, 6.0))),
	("等速の前半 3 秒がごっそり欠測",
	 dict(v=5.0, a=10.0, D=50.0, sigma=0.1), dict(dropout=(0.6, 3.6))),
	("姿勢が2回飛ぶ (+3mm, -3mm)",
	 dict(v=5.0, a=10.0, D=50.0, sigma=0.1), dict(outliers=[(3.0, 3.0), (7.0, -3.0)])),
	("V軸再現: 飛び -2.5 と 大スパイク",
	 dict(v=1.0, a=1.0, D=20.0, sigma=0.15), dict(outliers=[(15.0, 1.5), (17.0, -2.5)])),
	("末尾が 2 秒切れている",
	 dict(v=5.0, a=10.0, D=50.0, sigma=0.1), dict(cut_tail=2.0)),
	("回転 20deg 1deg/s a=0.5", dict(v=1.0, a=0.5, D=20.0, sigma=0.15), {}),
]

print(f"{'ケース':38s} {'真値窓':>9s} {'従来25-75%':>11s} {'指令から':>10s}  判定")
print("-" * 90)
for name, mk, extra in CASES:
	errs_a, errs_b, errs_c = [], [], []
	for seed in range(12):
		t, p, prof = make(seed=seed, **mk, **extra)
		v = mk["v"]
		ka = fit_band(t, p, 0.25, 0.75) / v
		rob = G._av_fit_robot_window(t, p, 1.0 / 30.0, v, mk["a"], mk["D"])
		kb = (rob["v_const"] / v) if rob.get("ok") else float("nan")
		kc = fit_true(t, p, prof, 3.0) / v
		errs_a.append(100 * (ka - 1)); errs_b.append(100 * (kb - 1)); errs_c.append(100 * (kc - 1))
	ma = float(np.nanmean(errs_a)); mb = float(np.nanmean(errs_b)); mc = float(np.nanmean(errs_c))
	sa = float(np.nanstd(errs_a)); sb = float(np.nanstd(errs_b))
	verdict = "新方式が良い" if abs(mb) < abs(ma) - 1e-9 else ("同等" if abs(mb) - abs(ma) < 0.02 else "従来が良い")
	print(f"{name:38s} {mc:+8.3f}% {ma:+8.3f}%±{sa:.3f} {mb:+8.3f}%±{sb:.3f}  {verdict}")
