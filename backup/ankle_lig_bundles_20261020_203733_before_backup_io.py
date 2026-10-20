# -*- coding: utf-8 -*-
"""段階2: 靭帯の束（付着部を楕円の面にした、柔らかい線維の束）。2026-10-16

FRS-SIMULATOR.py の段階1（点と点の靭帯・骨に沿わせる）とは別のファイルにしてある。
本体からは attach() だけを呼び、ここで起きた例外は本体側で捕まえる（段階1に影響させない）。
段階1のコード（_ankle_lig_wrap_* など）は呼ばない（同じ考え方の計算もここに別に持つ）。

■ 付着部の面
  骨の上で、付着部の外周を右クリックで 5 点以上打つ → 楕円を当てはめる（打ち直し・取り消しができる）。
  段階1の付着部の点（ファイル）を読み込んで、目安として表示できる。
■ 線維
  楕円の中に「ひまわりの並び」で線維の端を置く（1本目は楕円の中心＝中心の線維）。
  2つの面の線維は、楕円の中の同じ位置どうしをつなぐ（ねじれは足さない。楕円の向きの ± は、自然長を
  決めるコマで線維の長さの合計が最小になる向き）。
■ 動き（位置の補正をくり返す方法。XPBD の考え方を、速度を持たない形にしたもの）
  コマごとに、付着部（線維の両端）を骨と一緒に動かし、
    ・線維は伸びない（区間の長さ ≤ 自然長/区間数。縮むのは自由＝たるむ）
    ・少し曲がりにくい
    ・骨の中に入らない（骨から線維の半径だけ離す）
    ・ほかの線維と重ならない（半径の和だけ離す）
  を何回かくり返す。前のコマの形から始めるので、たるみの形がつながる。弱い重さ（下向き）でたわむ向きを決める。
  自然長は「表示の始まりのコマ」で骨を避けて張ったときの長さ × 係数。
■ 長さ（グラフ）
  束ごとに「中心の線維」の、骨を避けて張った経路の長さ（たるんでいても、付着部どうしの最短の経路の長さ）。
"""
import json
import math
import os
import time
import hashlib
import traceback
from pathlib import Path

import numpy as np

VERSION = 2          # 2: 付着部の楕円を、楕円の面の法線の向きに骨の面へ落とす（2026-10-20。線維の端が変わる）
ELL_V = 2            # 保存してある楕円の当てはめ方の版（古いものは外周の点から当てはめ直す）
PALETTE = ("#FF8800", "#D62728", "#1F77B4", "#2CA02C", "#9467BD", "#8C564B", "#E377C2", "#17BECF")


# ======================================================================== 骨の形（ぶつからないため）
class _Bones:
	"""骨のメッシュ（モデル座標）と、いちばん近い点を探す仕組み。"""

	def __init__(self, app, bones_full):
		import open3d as o3d
		self.o3d = o3d
		self.obs = {}
		for (idx, name, mesh, Ts) in bones_full:
			dm = app._sim_simplify_for_heatmap(str(name), mesh, 15000)
			dm = dm.extract_surface().triangulate()
			F = np.asarray(dm.faces).reshape(-1, 4)[:, 1:].astype(np.int32)
			Pv = np.asarray(dm.points, dtype=float)
			sc = o3d.t.geometry.RaycastingScene()
			sc.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(Pv.astype(np.float32)), o3d.core.Tensor(F)))
			nm = dm.compute_normals(cell_normals=True, point_normals=True, consistent_normals=True,
			                        auto_orient_normals=True, split_vertices=False)
			self.obs[int(idx)] = dict(sc=sc, tn=np.asarray(nm.cell_data["Normals"], float),
			                          vn=np.asarray(nm.point_data["Normals"], float), F=F,
			                          lo=Pv.min(axis=0), hi=Pv.max(axis=0))

	def closest(self, i, q):
		"""骨 i のモデル座標の点 q (n,3) → (いちばん近い面の点, なめらかな外向きの法線, 符号つき距離)"""
		o = self.obs[int(i)]
		q = np.asarray(q, dtype=np.float32).reshape(-1, 3)
		cp = o["sc"].compute_closest_points(self.o3d.core.Tensor(q))
		cpp = cp["points"].numpy().astype(float)
		pid = cp["primitive_ids"].numpy().astype(np.int64)
		uv = cp["primitive_uvs"].numpy().astype(float)
		Fi = o["F"][pid]
		ns = ((1.0 - uv[:, :1] - uv[:, 1:]) * o["vn"][Fi[:, 0]] + uv[:, :1] * o["vn"][Fi[:, 1]] + uv[:, 1:] * o["vn"][Fi[:, 2]])
		ns /= np.maximum(np.linalg.norm(ns, axis=1, keepdims=True), 1e-9)
		dq = q - cpp
		d = np.linalg.norm(dq, axis=1)
		sd = np.where(np.einsum("ij,ij->i", dq, o["tn"][pid]) >= 0, d, -d)
		return cpp, ns, sd

	def onto(self, i, P, n, R=15.0):
		"""点 P (m,3) を、向き n の直線に沿って骨 i の面へ落とす（その直線と面の交点のうち P にいちばん近いもの）。
		±R mm の中に交点が無い点は、いちばん近い面の点。返り値: (面の上の点, なめらかな外向きの法線)。
		（いちばん近い面の点へ落とすと、面が傾いた所・凹凸のある所で横へずれた。2026-10-20）"""
		o = self.obs[int(i)]
		P = np.asarray(P, dtype=float).reshape(-1, 3)
		n = np.asarray(n, dtype=float)
		n = n / max(float(np.linalg.norm(n)), 1e-12)
		out = self.closest(i, P)[0].copy()
		best = np.full(len(P), np.inf)
		for sgn in (1.0, -1.0):
			org = P + sgn * R * n
			d = np.tile(-sgn * n, (len(P), 1))
			rays = np.c_[org, d].astype(np.float32)
			t = o["sc"].cast_rays(self.o3d.core.Tensor(rays))["t_hit"].numpy().astype(float)
			dd = np.abs(t - R)
			ok = np.isfinite(t) & (dd <= R) & (dd < best)
			out[ok] = org[ok] + t[ok, None] * d[ok]
			best[ok] = dd[ok]
		_cp, ns, _sd = self.closest(i, out)
		return out, ns

	def push_out(self, P, Ts_f, clr):
		"""世界座標の点 P (n,3) を、どの骨からも clr (n,) 以上離す（その場で書き換え）。"""
		for i, o in self.obs.items():
			T = Ts_f[i]
			R, t = T[:3, :3], T[:3, 3]
			q = (P - t) @ R
			near = np.all((q > o["lo"] - 3.0) & (q < o["hi"] + 3.0), axis=1)
			if not near.any():
				continue
			cpp, ns, sd = self.closest(i, q[near])
			c = clr[near]
			bad = sd < c
			if bad.any():
				ii = np.where(near)[0][bad]
				P[ii] = (cpp[bad] + c[bad, None] * ns[bad]) @ R.T + t
		return P


# ======================================================================== 楕円・線維の端
def fit_ellipse(points):
	"""骨の上の外周の点 (n≥5, モデル座標) に楕円を当てはめる。返り値 dict(center,u,v,ra,rb) か例外。"""
	import cv2
	P = np.asarray(points, dtype=float)
	if len(P) < 5:
		raise ValueError("外周の点が 5 点より少ない")
	c = P.mean(axis=0)
	_u, _s, vt = np.linalg.svd(P - c, full_matrices=False)
	e1, e2 = vt[0], vt[1]
	xy = np.c_[(P - c) @ e1, (P - c) @ e2]
	sc = 100.0                                         # cv2 は整数寄りの座標が得意なので拡大
	(cx, cy), (w, h), ang = cv2.fitEllipse((xy * sc).astype(np.float32))
	cx, cy, w, h = cx / sc, cy / sc, w / sc, h / sc
	a = math.radians(ang)
	d1 = math.cos(a) * e1 + math.sin(a) * e2           # 幅 w の向き
	d2 = -math.sin(a) * e1 + math.cos(a) * e2          # 高さ h の向き
	center = c + cx * e1 + cy * e2
	if w >= h:
		u, ra, v, rb = d1, w / 2.0, d2, h / 2.0
	else:
		u, ra, v, rb = d2, h / 2.0, d1, w / 2.0
	if not (np.isfinite(ra) and np.isfinite(rb)) or ra <= 0 or rb <= 0 or ra > 50:
		raise ValueError("楕円をうまく当てはめられません（点の並びを確かめてください）")
	return {"center": center.tolist(), "u": (u / np.linalg.norm(u)).tolist(), "v": (v / np.linalg.norm(v)).tolist(),
	        "ra": float(ra), "rb": float(rb)}


PACKING = 0.8          # 断面の楕円に線維を詰める割合（円をばらばらに詰めると約 0.8）


def auto_fiber_radius(n, A, B, packing=PACKING):
	"""断面の楕円（長径 A・短径 B, mm）に n 本の線維を詰めたときの線維の半径（mm）。
	n × π r² = 詰め具合 × π (A/2)(B/2) から r = √(詰め具合 × A × B / (4n))。"""
	return math.sqrt(float(packing) * float(A) * float(B) / (4.0 * max(1, int(n))))


def _sheath(X, foot_r, AB, rf):
	"""束 X (F,K,3) を、断面の楕円の中に収める（靭帯を包む膜の代わり。その場で書き換え）。
	楕円の大きさは、中央で計った長径・短径（AB）、両端で付着部の楕円（foot_r）。間は sin でなめらかに。
	楕円の向きは、その場所の線維の広がりの向き（主成分）。"""
	F, K, _ = X.shape
	if F < 2 or K < 3:
		return
	C = X.mean(axis=0)
	t = np.linspace(0.0, 1.0, K)
	w = np.sin(np.pi * t)
	(raa, rba), (rab, rbb) = foot_r
	fa = (1.0 - t)[:, None] * np.array([raa, rba]) + t[:, None] * np.array([rab, rbb])
	R = (1.0 - w)[:, None] * fa + w[:, None] * np.array([AB[0] / 2.0, AB[1] / 2.0])
	R = np.maximum(R - float(rf), 0.05)                  # 線維の中心が入ってよい楕円
	for k in range(1, K - 1):
		tg = C[k + 1] - C[k - 1]
		nt = np.linalg.norm(tg)
		if nt < 1e-9:
			continue
		tg = tg / nt
		Q = X[:, k] - C[k]
		along = Q @ tg
		Qp = Q - np.outer(along, tg)
		_u, _s, vt = np.linalg.svd(Qp, full_matrices=False)
		e1 = vt[0] - (vt[0] @ tg) * tg
		n1 = np.linalg.norm(e1)
		if n1 < 1e-9:
			continue
		e1 = e1 / n1
		e2 = np.cross(tg, e1)
		q1, q2 = Qp @ e1, Qp @ e2
		sc = np.sqrt((q1 / R[k, 0]) ** 2 + (q2 / R[k, 1]) ** 2)
		out = sc > 1.0
		if out.any():
			X[out, k] = (C[k] + along[out, None] * tg + (q1[out] / sc[out])[:, None] * e1
			             + (q2[out] / sc[out])[:, None] * e2)


def sunflower(n):
	"""楕円（単位円）の中に均一に n 点。1点目は中心（中心の線維）。"""
	n = max(1, int(n))
	out = [(0.0, 0.0)]
	m = n - 1
	for k in range(m):
		r = math.sqrt((k + 0.5) / m)
		th = (k + 0.5) * math.pi * (3.0 - math.sqrt(5.0))
		out.append((r * math.cos(th), r * math.sin(th)))
	return np.array(out[:n], dtype=float)


def ell_normal(bones, bone_idx, ell):
	"""楕円の面の法線 u × v（骨の外向きにそろえる）。"""
	c = np.asarray(ell["center"], float)
	n = np.cross(np.asarray(ell["u"], float), np.asarray(ell["v"], float))
	n /= max(float(np.linalg.norm(n)), 1e-12)
	_cp, ns, _ = bones.closest(bone_idx, c[None])
	return n if float(n @ ns[0]) >= 0 else -n


def fiber_ends(bones, bone_idx, ell, uv):
	"""楕円の中の位置 uv (n,2) → 骨の面の上の点（モデル座標）。楕円の面の法線の向きに落とす。"""
	c = np.asarray(ell["center"], float)
	u = np.asarray(ell["u"], float)
	v = np.asarray(ell["v"], float)
	P = c + uv[:, :1] * float(ell["ra"]) * u + uv[:, 1:] * float(ell["rb"]) * v
	return bones.onto(bone_idx, P, ell_normal(bones, bone_idx, ell))[0]


# ======================================================================== 計算の本体
def _taut_paths(bones, Ts, f, A, B, K, clr, X=None, maxit=20, tol=1e-2, taper=0.18):
	"""骨を避けて張った最短の経路（M 本まとめて）。A,B (M,3) 世界座標。返り値 X (M,K,3)。"""
	M = len(A)
	if X is None:
		X = A[:, None, :] + (B - A)[:, None, :] * np.linspace(0.0, 1.0, K)[None, :, None]
	X[:, 0], X[:, -1] = A, B
	odd, even = np.arange(1, K - 1, 2), np.arange(2, K - 1, 2)
	Ts_f = {i: T[f] for i, T in Ts.items()}
	Lp, calm = None, 0
	for _it in range(maxit):
		for idx in (odd, even):
			X[:, idx] += 1.6 * (0.5 * (X[:, idx - 1] + X[:, idx + 1]) - X[:, idx])
		seg = np.linalg.norm(np.diff(X, axis=1), axis=2)
		sa = np.concatenate([np.zeros((M, 1)), np.cumsum(seg, axis=1)], axis=1)
		sb = sa[:, -1:] - sa
		cm = np.minimum(clr[:, None], taper * np.minimum(sa, sb))[:, 1:-1].reshape(-1)
		P = X[:, 1:-1].reshape(-1, 3).copy()
		bones.push_out(P, Ts_f, cm)
		X[:, 1:-1] = P.reshape(M, K - 2, 3)
		Lc = np.linalg.norm(np.diff(X, axis=1), axis=2).sum(axis=1)
		if Lp is not None and float(np.abs(Lc - Lp).max()) < tol:
			calm += 1
			if calm >= 2:
				break
		else:
			calm = 0
		Lp = Lc
	return X


def _resample(X, K):
	"""折れ線 X (M,k,3) を、道のりで等間隔の K 点に。"""
	out = np.zeros((X.shape[0], K, 3))
	for m in range(X.shape[0]):
		seg = np.linalg.norm(np.diff(X[m], axis=0), axis=1)
		s = np.concatenate([[0.0], np.cumsum(seg)])
		t = np.linspace(0.0, s[-1], K)
		for c in range(3):
			out[m, :, c] = np.interp(t, s, X[m, :, c])
	return out


def simulate(app, bones_full, bundles, s0, params, progress=None, cancelled=None, bones=None):
	"""束をまとめて動かす。bundles: [{"a": 骨番号, "b": 骨番号, "pa": (F,3), "pb": (F,3), "rf": 半径}]
	返り値: [{"frames", "local": (nf,F,K,3) a の骨の座標, "center_len": (nf,), "L0": (F,)}] か None（キャンセル）。"""
	if bones is None:
		bones = _Bones(app, bones_full)
	Ts = {int(i): np.asarray(T, dtype=float) for (i, n, m, T) in bones_full}
	N = min(len(T) for T in Ts.values())
	s0 = int(max(0, min(int(s0), N - 1)))
	stride = int(params.get("stride", 3))
	frames = list(range(s0, N, stride))
	if frames[-1] != N - 1:
		frames.append(N - 1)
	scale = float(params.get("rest_scale", 1.0))
	gstep = float(params.get("gravity", 0.02))
	gdir = np.asarray(params.get("gravity_dir", (0.0, 1.0, 0.0)), dtype=float)
	gdir = gdir / max(np.linalg.norm(gdir), 1e-9)
	iters = int(params.get("iters", 12))
	inner = int(params.get("inner", 6))
	bend = float(params.get("bend", 0.05))
	nb = len(bundles)

	def world(i, P, f):
		T = Ts[int(i)][f]
		return np.asarray(P, float) @ T[:3, :3].T + T[:3, 3]

	# --- 自然長: 表示の始まりのコマで、骨を避けて張った長さ × 係数。形もそこから始める ---
	Xs, seg0, Ks, rfs = [], [], [], []
	f0 = frames[0]
	for b in bundles:
		A, B = world(b["a"], b["pa"], f0), world(b["b"], b["pb"], f0)
		F = len(A)
		Kt = 33
		Xt = _taut_paths(bones, Ts, f0, A, B, Kt, np.full(F, float(b["rf"])), maxit=3000)
		Lt = np.linalg.norm(np.diff(Xt, axis=1), axis=2).sum(axis=1)
		L0 = Lt * scale
		K = int(np.clip(round(float(L0.max()) / max(1.2, 2.5 * float(b["rf"]))) + 1, 15, 33))
		Xs.append(_resample(Xt, K))
		seg0.append((L0 / (K - 1))[:, None])
		Ks.append(K)
		rfs.append(float(b["rf"]))
		b["_L0"] = L0
	# 線維どうしの当たり判定用に、全部の点を1列に並べる
	sizes = [len(b["pa"]) * Ks[j] for j, b in enumerate(bundles)]
	offs = np.concatenate([[0], np.cumsum(sizes)])
	fib_id = np.concatenate([np.repeat(np.arange(len(b["pa"])) + 1000 * j, Ks[j]) for j, b in enumerate(bundles)])
	pos_id = np.concatenate([np.tile(np.arange(Ks[j]), len(b["pa"])) for j, b in enumerate(bundles)])
	rad = np.concatenate([np.full(sizes[j], rfs[j]) for j in range(nb)])
	is_end = np.concatenate([np.tile((np.arange(Ks[j]) == 0) | (np.arange(Ks[j]) == Ks[j] - 1), len(b["pa"]))
	                         for j, b in enumerate(bundles)])
	from scipy.spatial import cKDTree
	rmax = float(rad.max()) if len(rad) else 0.5

	def step(f):
		Ts_f = {i: T[f] for i, T in Ts.items()}
		for j, b in enumerate(bundles):
			Xs[j][:, 0], Xs[j][:, -1] = world(b["a"], b["pa"], f), world(b["b"], b["pb"], f)
			Xs[j][:, 1:-1] += gstep * gdir
		for _it in range(iters):
			for j in range(nb):
				X, K, s_ = Xs[j], Ks[j], seg0[j]
				Fn = X.shape[0]
				for par in (0, 1) * inner:
					idx = np.arange(par, K - 1, 2)
					d = X[:, idx + 1] - X[:, idx]
					L = np.linalg.norm(d, axis=2, keepdims=True)
					corr = (np.maximum(L - s_[:, :, None], 0.0) / np.maximum(L, 1e-9)) * d
					wi = np.ones((1, len(idx), 1))
					wj = np.ones((1, len(idx), 1))
					wi[:, idx == 0] = 0.0
					wj[:, idx + 1 == K - 1] = 0.0
					sw = np.maximum(wi + wj, 1e-9)
					X[:, idx] += (wi / sw) * corr
					X[:, idx + 1] -= (wj / sw) * corr
				X[:, 1:-1] += bend * (0.5 * (X[:, :-2] + X[:, 2:]) - X[:, 1:-1])
				if bundles[j].get("sheath"):
					_sheath(X, bundles[j]["foot_r"], bundles[j]["sheath"], rfs[j])
			# 線維どうし
			P = np.concatenate([X.reshape(-1, 3) for X in Xs], axis=0)
			pr = cKDTree(P).query_pairs(2.0 * rmax, output_type="ndarray")
			if len(pr):
				ok = (fib_id[pr[:, 0]] != fib_id[pr[:, 1]]) | (np.abs(pos_id[pr[:, 0]] - pos_id[pr[:, 1]]) > 2)
				pr = pr[ok]
			if len(pr):
				d = P[pr[:, 1]] - P[pr[:, 0]]
				L = np.linalg.norm(d, axis=1, keepdims=True)
				need = (rad[pr[:, 0]] + rad[pr[:, 1]])[:, None]
				pen = np.maximum(need - L, 0.0)
				c = 0.5 * pen / np.maximum(L, 1e-9) * d
				mv = np.zeros_like(P)
				np.add.at(mv, pr[:, 0], -c)
				np.add.at(mv, pr[:, 1], c)
				mv[is_end] = 0.0
				P = P + mv
			# 骨に入らない（両端を除く）
			inner_pts = ~is_end
			Q = P[inner_pts].copy()
			bones.push_out(Q, Ts_f, rad[inner_pts])
			P[inner_pts] = Q
			for j in range(nb):
				Xs[j] = P[offs[j]:offs[j + 1]].reshape(Xs[j].shape)

	nf = len(frames)
	out_local = [np.zeros((nf,) + Xs[j].shape, dtype=np.float32) for j in range(nb)]
	center_len = [np.zeros(nf, dtype=np.float32) for _ in range(nb)]
	# 中心の線維（1本目）の、骨を避けて張った長さ
	Xc = [None] * nb
	t0 = time.time()
	last = 0.0
	for k, f in enumerate(frames):
		step(f)
		for j, b in enumerate(bundles):
			T = Ts[int(b["a"])][f]
			out_local[j][k] = ((Xs[j] - T[:3, 3]) @ T[:3, :3]).astype(np.float32)
			A = world(b["a"], b["pa"][:1], f)
			B = world(b["b"], b["pb"][:1], f)
			Xc[j] = _taut_paths(bones, Ts, f, A, B, 33, np.array([float(b["rf"])]), X=Xc[j],
			                    maxit=(3000 if k == 0 else 20))
			center_len[j][k] = float(np.linalg.norm(np.diff(Xc[j][0], axis=0), axis=1).sum())
		now = time.time()
		if progress is not None and now - last > 0.25:
			last = now
			el = now - t0
			eta = el / (k + 1) * (nf - k - 1)
			progress((k + 1) / nf, f"{nb} 束 ・ コマ {f + 1:,} / {N:,}（残り 約 {eta:.0f} 秒）")
			if cancelled is not None and cancelled():
				return None
	print(f"[靭帯の束] {nb} 束・{nf:,} コマを {time.time() - t0:.1f} 秒で計算")
	fr = np.asarray(frames, dtype=np.int32)
	return [{"frames": fr, "local": out_local[j], "center_len": center_len[j], "L0": np.asarray(bundles[j]["_L0"], float)}
	        for j in range(nb)]


# ======================================================================== 可視化への組み込み
class Stage2:
	"""可視化ウィンドウ1つ分の「靭帯の束」。"""

	def __init__(self, app, scene, bones_full, scene_bones, s0, store_key, base_dir):
		self.app = app
		self.scene = scene
		self.bones_full = bones_full
		self.sb = scene_bones
		self.s0 = int(s0)
		self.names = [str(b.get("name", "")) for b in scene_bones]
		self.idx_of = [int(b.get("_idx", -1)) for b in scene_bones]
		self.poses = [np.asarray(b["poses"], dtype=float) for b in scene_bones]
		self.n_view = min(len(p) for p in self.poses) if self.poses else 0
		self.store = Path(base_dir) / "cache" / "ankle_lig_bundles" / f"{store_key}.json"
		self.res_dir = self.store.parent / "results"
		self.data = self._load()
		self._refit_old()
		self.bones = None
		self.results = {}            # 束の id → 結果
		self.listeners = []          # 右のグラフの描き直し
		self.plotter = None
		self.fi = 0
		self.actors = {}             # 束の id → {...}
		self.guides = []             # 目安の点のアクター
		self.pend = []               # 外周を打っている点のアクター
		self.win = None
		self.w = {}
		# clicks: ("R"|"L", x, y)。右 = 点を打つ、左 = 打った点を選ぶ。sel = 選んでいる点の番号
		self.st = {"mode": None, "clicks": [], "press": None, "lpress": None, "poll": None, "obs": [], "replace": None,
		           "sel": None}
		self._load_cached_results()

	# ---------------------------------------------------------------- 保存
	def _load(self):
		try:
			d = json.loads(self.store.read_text(encoding="utf-8"))
			if isinstance(d, dict):
				d.setdefault("bundles", [])
				d.setdefault("params", {})
				return d
		except Exception:
			pass
		return {"version": VERSION, "bundles": [], "params": {}, "guide_file": ""}

	def _refit_old(self):
		"""古いやり方で決めた楕円（中心をいちばん近い面の点へ動かしていた）を、保存してある外周の点から当てはめ直す。"""
		n = 0
		for b in self.data.get("bundles", []):
			for end in ("a", "b"):
				e = b.get(end) or {}
				if e.get("ellipse") and e.get("outline") and int(e.get("ell_v", 1)) < ELL_V:
					try:
						e["ellipse"] = fit_ellipse(e["outline"])
						e["ell_v"] = ELL_V
						n += 1
					except Exception as ex:
						print(f"[靭帯の束] {b.get('name', '')} の楕円を当てはめ直せません: {ex}")
		if n:
			self._save()
			print(f"[靭帯の束] 古いやり方の楕円 {n} 個を、保存してある外周の点から当てはめ直しました（中心のずれを直す）")

	def _save(self):
		try:
			self.store.parent.mkdir(parents=True, exist_ok=True)
			tmp = self.store.with_name(self.store.stem + ".tmp.json")
			tmp.write_text(json.dumps(self.data, ensure_ascii=False, indent=1), encoding="utf-8")
			os.replace(str(tmp), str(self.store))
		except Exception as e:
			print(f"[靭帯の束] 保存に失敗: {e}")

	def params(self):
		p = {"rest_scale": 1.0, "gravity": 0.02, "iters": 12, "inner": 6, "bend": 0.05, "stride": 3, "view": "both",
		     "lig_opacity": 1.0}
		p.update(self.data.get("params") or {})
		return p

	# ---------------------------------------------------------------- 骨・束の対応
	def _si_of_name(self, nm):
		hits = [si for si, n in enumerate(self.names) if n == str(nm)]
		return hits[0] if len(hits) == 1 else None

	def _ready(self, b):
		return bool(b.get("a", {}).get("ellipse")) and bool(b.get("b", {}).get("ellipse")) \
			and self._si_of_name(b["a"].get("bone_name")) is not None and self._si_of_name(b["b"].get("bone_name")) is not None

	def _ensure_bones(self):
		if self.bones is None:
			self.bones = _Bones(self.app, self.bones_full)
		return self.bones

	def _runtime(self, b):
		"""束の定義 → 計算に渡す形（線維の端の点・対応の向き）。"""
		bones = self._ensure_bones()
		sia, sib = self._si_of_name(b["a"]["bone_name"]), self._si_of_name(b["b"]["bone_name"])
		ia, ib = self.idx_of[sia], self.idx_of[sib]
		n = int(b.get("n_fibers", 10))
		uv = sunflower(n)
		pa = fiber_ends(bones, ia, b["a"]["ellipse"], uv)
		Ts = {int(i): np.asarray(T, dtype=float) for (i, nm, m, T) in self.bones_full}
		f0 = max(0, min(self.s0, min(len(T) for T in Ts.values()) - 1))

		def world(i, P):
			T = Ts[int(i)][f0]
			return P @ T[:3, :3].T + T[:3, 3]
		best = None
		for su in (1.0, -1.0):
			for sv in (1.0, -1.0):
				pb = fiber_ends(bones, ib, b["b"]["ellipse"], uv * [su, sv])
				L = float(np.linalg.norm(world(ia, pa) - world(ib, pb), axis=1).sum())
				if best is None or L < best[0]:
					best = (L, pb)
		AB = self._xsec(b)
		return {"a": ia, "b": ib, "pa": pa, "pb": best[1], "rf": self._rf(b), "_sia": sia,
		        "sheath": AB,
		        "foot_r": ((float(b["a"]["ellipse"]["ra"]), float(b["a"]["ellipse"]["rb"])),
		                   (float(b["b"]["ellipse"]["ra"]), float(b["b"]["ellipse"]["rb"])))}

	@staticmethod
	def _xsec(b):
		"""束の断面（長径, 短径）mm。入っていなければ None（線維の太さは手で入れたもの）。"""
		try:
			A, B = float(b.get("xsec_a", 0) or 0), float(b.get("xsec_b", 0) or 0)
		except (TypeError, ValueError):
			return None
		if A > 0 and B > 0:
			return (max(A, B), min(A, B))
		return None

	def _rf(self, b):
		"""線維の半径。断面があれば、本数と断面から自動。無ければ手で入れた値。"""
		AB = self._xsec(b)
		if AB is not None:
			return auto_fiber_radius(int(b.get("n_fibers", 10)), AB[0], AB[1])
		return float(b.get("fiber_radius", 0.4))

	def _key(self, ready):
		h = hashlib.sha1()
		for (idx, name, mesh, Ts) in sorted(self.bones_full, key=lambda x: int(x[0])):
			h.update(f"bone{int(idx)}|".encode())
			h.update(np.round(np.asarray(mesh.points, dtype=float), 3).astype(np.float32).tobytes())
			h.update(np.round(np.asarray(Ts, dtype=float), 4).tobytes())
		h.update(f"|s0{self.s0}|V{VERSION}|".encode())
		pp = self.params()
		h.update(json.dumps({k: pp[k] for k in ("rest_scale", "gravity", "iters", "inner", "bend", "stride")},
		                    sort_keys=True).encode())
		for b in ready:
			h.update(json.dumps({"a": b["a"], "b": b["b"], "n": int(b.get("n_fibers", 10)),
			                     "r": round(self._rf(b), 6), "xs": self._xsec(b)}, sort_keys=True).encode())
		return h.hexdigest()[:24]

	def _load_cached_results(self):
		ready = [b for b in self.data["bundles"] if self._ready(b)]
		self.results = {}
		if not ready:
			return
		try:
			fp = self.res_dir / f"lb_{self._key(ready)}.npz"
			if not fp.exists():
				return
			with np.load(str(fp), allow_pickle=False) as z:
				for j, b in enumerate(ready):
					self.results[b["id"]] = {"frames": np.asarray(z[f"frames{j}"]), "local": np.asarray(z[f"local{j}"]),
					                         "center_len": np.asarray(z[f"clen{j}"]), "L0": np.asarray(z[f"L0{j}"]),
					                         "sia": self._si_of_name(b["a"]["bone_name"])}
		except Exception as e:
			print(f"[靭帯の束] 保存した結果を読めません: {e}")
			self.results = {}

	def compute(self, parent=None):
		ready = [b for b in self.data["bundles"] if self._ready(b)]
		if not ready:
			self._say("付着部の面が両方ある束がありません。", "#a06000")
			return False
		upd, close, cancelled = _progress(self.app, "靭帯の束を計算しています（段階2）", parent)
		try:
			upd(0.0, "骨の形と線維の端を準備しています…")
			rts = [self._runtime(b) for b in ready]
			res = simulate(self.app, self.bones_full, rts, self.s0, self.params(), progress=upd, cancelled=cancelled,
			               bones=self._ensure_bones())
		finally:
			close()
		if res is None:
			self._say("キャンセルしました。", "#a06000")
			return False
		try:
			self.res_dir.mkdir(parents=True, exist_ok=True)
			arrs = {}
			for j, r in enumerate(res):
				arrs[f"frames{j}"] = r["frames"]
				arrs[f"local{j}"] = r["local"]
				arrs[f"clen{j}"] = r["center_len"]
				arrs[f"L0{j}"] = r["L0"].astype(np.float32)
			tmp = self.res_dir / "lb_tmp.npz"
			np.savez_compressed(str(tmp), **arrs)
			os.replace(str(tmp), str(self.res_dir / f"lb_{self._key(ready)}.npz"))
		except Exception as e:
			print(f"[靭帯の束] 結果を保存できませんでした: {e}")
		self.results = {b["id"]: dict(r, sia=rt["_sia"]) for b, r, rt in zip(ready, res, rts)}
		self._rebuild_actors()
		self._notify()
		self._say(f"{len(ready)} 束を計算しました。", "#1b5e20")
		return True

	# ---------------------------------------------------------------- 表示の時刻
	def _local_at(self, r, fi):
		fr = r["frames"].astype(float)
		g = float(self.s0 + min(max(int(fi), 0), self.n_view - 1))
		j = int(np.searchsorted(fr, g, side="right")) - 1
		j = max(0, min(j, len(fr) - 1))
		if j >= len(fr) - 1 or fr[j + 1] <= fr[j]:
			return r["local"][j].astype(float)
		w = (g - fr[j]) / (fr[j + 1] - fr[j])
		return (1.0 - w) * r["local"][j].astype(float) + w * r["local"][j + 1].astype(float)

	def series(self):
		"""右のグラフ: [(時刻, 中心の線維の長さ, 色, 名前)]"""
		times = np.asarray(self.scene.get("frame_times") or [], dtype=float)
		out = []
		for b in self.data["bundles"]:
			r = self.results.get(b.get("id"))
			if r is None or not b.get("chart", True):
				continue
			g = self.s0 + np.arange(len(times))
			L = np.interp(g, r["frames"].astype(float), r["center_len"].astype(float))
			out.append((times, L, str(b.get("color", "#FF8800")), str(b.get("name", ""))))
		return out

	def _notify(self):
		for fn in list(self.listeners):
			try:
				fn()
			except Exception as e:
				print(f"[靭帯の束] グラフの描き直しに失敗: {e}")

	# ---------------------------------------------------------------- 描画
	def make(self, plotter):
		"""scene の追加描画の口。返り値は毎コマの更新。"""
		self.plotter = plotter
		try:
			self._rebuild_actors()
			self._load_guides()
		except Exception as e:
			print(f"[靭帯の束] 表示の準備に失敗: {e}")
			traceback.print_exc()
		return self.update

	def _vtk(self):
		import vtk
		return vtk

	def _mat(self, T):
		vtk = self._vtk()
		m = vtk.vtkMatrix4x4()
		for i in range(4):
			for j in range(4):
				m.SetElement(i, j, float(T[i, j]))
		return m

	def _rgb(self, h):
		return self.app._sim_hex_to_rgb01(h)

	def _clear_actors(self):
		if self.plotter is None:
			return
		for a in self.actors.values():
			for key in ("fib_act", "skin_act", "ea_act", "eb_act"):
				act = a.get(key)
				if act is not None:
					try:
						self.plotter.renderer.RemoveActor(act)
					except Exception:
						pass
		self.actors = {}

	def _ellipse_poly(self, ell, n=48):
		vtk = self._vtk()
		c = np.asarray(ell["center"], float)
		u, v = np.asarray(ell["u"], float), np.asarray(ell["v"], float)
		th = np.linspace(0, 2 * np.pi, n, endpoint=False)
		P = c + np.outer(np.cos(th) * float(ell["ra"]), u) + np.outer(np.sin(th) * float(ell["rb"]), v)
		pts = vtk.vtkPoints()
		for p in P:
			pts.InsertNextPoint(*[float(x) for x in p])
		ca = vtk.vtkCellArray()
		ca.InsertNextCell(n + 1)
		for k in range(n):
			ca.InsertCellPoint(k)
		ca.InsertCellPoint(0)
		pd = vtk.vtkPolyData()
		pd.SetPoints(pts)
		pd.SetLines(ca)
		return pd

	def _ring_poly(self, P):
		"""閉じた輪の折れ線。"""
		vtk = self._vtk()
		pts = vtk.vtkPoints()
		for p in P:
			pts.InsertNextPoint(*[float(x) for x in p])
		n = len(P)
		ca = vtk.vtkCellArray()
		ca.InsertNextCell(n + 1)
		for k in range(n):
			ca.InsertCellPoint(k)
		ca.InsertCellPoint(0)
		pd = vtk.vtkPolyData()
		pd.SetPoints(pts)
		pd.SetLines(ca)
		return pd

	def _line_actor(self, pd, color, width=3.0):
		vtk = self._vtk()
		mp = vtk.vtkPolyDataMapper()
		mp.SetInputData(pd)
		act = vtk.vtkActor()
		act.SetMapper(mp)
		act.PickableOff()
		act.GetProperty().SetColor(*self._rgb(color))
		act.GetProperty().SetLineWidth(width)
		act.GetProperty().SetRenderLinesAsTubes(True)
		self.plotter.renderer.AddActor(act)
		return act

	def _rebuild_actors(self):
		if self.plotter is None:
			return
		vtk = self._vtk()
		self._clear_actors()
		view = self.params().get("view", "both")
		for b in self.data["bundles"]:
			a = {}
			col = str(b.get("color", "#FF8800"))
			vis = bool(b.get("visible", True))
			for end in ("a", "b"):
				e = b.get(end) or {}
				si = self._si_of_name(e.get("bone_name"))
				if e.get("ellipse") and si is not None:
					ring = None
					try:
						ring, _si = self._foot_ring(e, 48)
					except Exception as ex:
						print(f"[靭帯の束] 付着部の輪郭を骨の面に落とせません（平らな楕円で描きます）: {ex}")
					pd_e = self._ring_poly(ring["out_calc"]) if ring is not None else self._ellipse_poly(e["ellipse"])
					act = self._line_actor(pd_e, col, 3.0)
					act.SetVisibility(vis and self._outline_on() and not self._drawing(b, end))
					a[f"e{end}_act"] = act
					a[f"e{end}_si"] = si
					a[f"e{end}_ring"] = ring
					a[f"e{end}_pd"] = pd_e
					a[f"e{end}_kind"] = "calc"
			if self._xsec(b) is not None and self._ready(b):
				xpd = self._ring_poly(np.zeros((48, 3)))
				xact = self._line_actor(xpd, col, 2.0)
				xact.SetVisibility(vis and bool(self.params().get("show_xsec", True)) and view != "foot")
				a.update(xs_act=xact, xs_pd=xpd)
			r = self.results.get(b.get("id"))
			if r is not None:
				Fn, K = r["local"].shape[1], r["local"].shape[2]
				# 線維: F 本の折れ線をチューブに
				pd = vtk.vtkPolyData()
				pts = vtk.vtkPoints()
				pts.SetNumberOfPoints(Fn * K)
				ca = vtk.vtkCellArray()
				for f_ in range(Fn):
					ca.InsertNextCell(K)
					for k in range(K):
						ca.InsertCellPoint(f_ * K + k)
				pd.SetPoints(pts)
				pd.SetLines(ca)
				tube = vtk.vtkTubeFilter()
				tube.SetInputData(pd)
				tube.SetRadius(max(0.05, self._rf(b)))
				tube.SetNumberOfSides(8)
				mp = vtk.vtkPolyDataMapper()
				mp.SetInputConnection(tube.GetOutputPort())
				fact = vtk.vtkActor()
				fact.SetMapper(mp)
				fact.PickableOff()
				fact.GetProperty().SetColor(*self._rgb(col))
				fact.GetProperty().SetOpacity(self._lig_opacity())
				fact.SetVisibility(vis and view in ("fibers", "both"))
				self.plotter.renderer.AddActor(fact)
				# 皮: 経路に沿って楕円の輪をつなぐ
				M = 24
				spd = vtk.vtkPolyData()
				spts = vtk.vtkPoints()
				NS = len(self.SKIN_CAP_SCALES)
				spts.SetNumberOfPoints(K * M + 2 * NS * M)
				polys = vtk.vtkCellArray()

				def _band(r0, r1):
					"""点の番号 r0〜r0+M-1 の輪と r1〜 の輪を四角でつなぐ。"""
					for m in range(M):
						q = vtk.vtkQuad()
						q.GetPointIds().SetId(0, r0 + m)
						q.GetPointIds().SetId(1, r0 + (m + 1) % M)
						q.GetPointIds().SetId(2, r1 + (m + 1) % M)
						q.GetPointIds().SetId(3, r1 + m)
						polys.InsertNextCell(q)
				for k in range(K - 1):
					_band(k * M, (k + 1) * M)
				# 両端のふた: 付着部の楕円を同心の輪に分けて骨の面へ落としたもの（骨の面に沿う。平らな板だと、
				# 凹んだ付着部（顆間窩の壁など）で板が骨から浮いて見えた）
				for kk, base in ((0, K * M), (K - 1, K * M + NS * M)):
					_band(kk * M, base)
					for j in range(NS - 1):
						_band(base + j * M, base + (j + 1) * M)
				spd.SetPoints(spts)
				spd.SetPolys(polys)
				nrm = vtk.vtkPolyDataNormals()
				nrm.SetInputData(spd)
				nrm.SplittingOff()
				smp = vtk.vtkPolyDataMapper()
				smp.SetInputConnection(nrm.GetOutputPort())
				sact = vtk.vtkActor()
				sact.SetMapper(smp)
				sact.PickableOff()
				sp = sact.GetProperty()
				sp.SetColor(*self._rgb(col))
				# 皮だけのときは不透明（少しでも透明だと、骨に入っている端の奥が透けて、端が骨から浮いて見えた）。
				# 「両方」のときは中の線維が見えるように半透明
				sp.SetOpacity((0.55 if view == "both" else 1.0) * self._lig_opacity())
				sp.SetInterpolationToPhong()
				sact.SetVisibility(vis and view in ("skin", "both"))
				self.plotter.renderer.AddActor(sact)
				a.update(fib_act=fact, fib_pd=pd, fib_pts=pts, skin_act=sact, skin_pd=spd, skin_pts=spts,
				         F=Fn, K=K, M=M, r=r, rf=self._rf(b))
				try:
					a["foot"] = (self._foot_ring(b["a"], M), self._foot_ring(b["b"], M))
				except Exception as e:
					a["foot"] = None
					print(f"[靭帯の束] 付着部の輪を作れません（皮の端は合わせません）: {e}")
				# 線維の端（モデル座標）と、表示変更用モデルの面へ落とした端
				try:
					Tpre = {int(i): np.asarray(T, dtype=float) for (i, nm_, m_, T) in self.bones_full}
					f0 = int(r["frames"][0])
					sia_, sib_ = r["sia"], self._si_of_name(b["b"]["bone_name"])
					ia_, ib_ = self.idx_of[sia_], self.idx_of[sib_]
					La = r["local"][0][:, 0].astype(float)
					Wb = r["local"][0][:, -1].astype(float) @ Tpre[ia_][f0][:3, :3].T + Tpre[ia_][f0][:3, 3]
					Lb = (Wb - Tpre[ib_][f0][:3, 3]) @ Tpre[ib_][f0][:3, :3]
					a["fend"] = ((sia_, self._onto_alt(sia_, La)), (sib_, self._onto_alt(sib_, Lb)))
				except Exception as e:
					a["fend"] = None
					print(f"[靭帯の束] 線維の端を表示変更用モデルに合わせられません: {e}")
			self.actors[b["id"]] = a
		self.update(self.fi, render=True)

	# 皮の端は付着部の楕円そのもの（骨の面に落として、少しだけ骨の中へ）。端から長さの BLEND の範囲で、
	# 経路に垂直な断面の楕円から付着部の楕円へなめらかに移す（靭帯が付着部で広がる形）。
	# 以前は経路に垂直な断面だけで皮を作っていたので、靭帯が骨に斜めに付く所で、皮の端が付着部の楕円と
	# ずれ、骨との間にすき間が見えた（2026-10-16 ユーザー指摘）。
	SKIN_SINK = 0.3        # mm。付着部の輪を骨の中へ入れる量（輪の間で骨の面が曲がっていてもすき間を見せない）
	SKIN_BLEND = 0.2       # 長さに対する割合
	SKIN_CAP_SCALES = (0.7, 0.4, 0.0)    # ふたの同心の輪（付着部の楕円に対する大きさ。0 は中心）

	def _shown(self, si):
		"""骨 si でいま見えているモデル: "alt"（表示変更用モデル）か "calc"（計算に使ったモデル・関節面）。"""
		if self.plotter is None:
			return "calc"
		try:
			act = self.plotter.renderer.actors.get(f"bone_{si}_alt")
			if act is not None and act.GetVisibility():
				return "alt"
		except Exception:
			pass
		return "calc"

	def _alt_scene(self, si):
		"""表示変更用モデル（モデル座標）の、いちばん近い点を探す仕組み。無ければ None。"""
		cache = self.__dict__.setdefault("_alt_sc", {})
		if si in cache:
			return cache[si]
		sc = None
		try:
			act = self.plotter.renderer.actors.get(f"bone_{si}_alt") if self.plotter is not None else None
			if act is not None:
				import pyvista as pv
				import open3d as o3d
				tri = pv.wrap(act.GetMapper().GetInput()).triangulate()
				T0 = self.poses[si][0]                      # 表示の骨 = UserMatrix × 最初のコマの姿勢 × モデル座標
				P = (np.asarray(tri.points, float) - T0[:3, 3]) @ T0[:3, :3]
				F = np.asarray(tri.faces).reshape(-1, 4)[:, 1:].astype(np.int32)
				sc = o3d.t.geometry.RaycastingScene()
				sc.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(P.astype(np.float32)), o3d.core.Tensor(F)))
		except Exception as e:
			print(f"[靭帯の束] 表示変更用モデルの面を使えません: {e}")
			sc = None
		cache[si] = sc
		return sc

	def _onto_alt(self, si, P):
		"""モデル座標の点 P を、表示変更用モデルの面の上へ。無ければ None。"""
		sc = self._alt_scene(si)
		if sc is None:
			return None
		import open3d as o3d
		cp = sc.compute_closest_points(o3d.core.Tensor(np.asarray(P, np.float32)))
		return cp["points"].numpy().astype(float)

	def _foot_ring(self, end, M):
		"""付着部の楕円を骨の面へ落とした輪（M 点・その骨のモデル座標）。
		返り値: ({"calc": 骨の少し中, "alt": 表示変更用モデルの少し中 or None, "out_calc"/"out_alt": 面の少し外（輪郭の線）}, 骨の番号)"""
		bones = self._ensure_bones()
		ell = end["ellipse"]
		si = self._si_of_name(end.get("bone_name"))
		idx = self.idx_of[si]
		c = np.asarray(ell["center"], float)
		u, v = np.asarray(ell["u"], float), np.asarray(ell["v"], float)
		nE = ell_normal(bones, idx, ell)                 # 楕円の面の法線の向きに骨の面へ落とす
		th = np.linspace(0.0, 2.0 * np.pi, M, endpoint=False)
		P = c + np.outer(np.cos(th) * float(ell["ra"]), u) + np.outer(np.sin(th) * float(ell["rb"]), v)
		cpp, ns2 = bones.onto(idx, P, nE)
		out = {"calc": cpp - self.SKIN_SINK * ns2, "out_calc": cpp + 0.05 * ns2, "alt": None, "out_alt": None}
		ca = self._onto_alt(si, cpp)
		if ca is not None:
			out["alt"] = ca - self.SKIN_SINK * ns2
			out["out_alt"] = ca + 0.05 * ns2
		# ふたの同心の輪（骨の面へ落として、少し中へ）
		caps_c, caps_a = [], []
		for sc_ in self.SKIN_CAP_SCALES:
			Ps = c + sc_ * (np.outer(np.cos(th) * float(ell["ra"]), u) + np.outer(np.sin(th) * float(ell["rb"]), v))
			cps, nss = bones.onto(idx, Ps, nE)
			caps_c.append(cps - self.SKIN_SINK * nss)
			if ca is not None:
				cas = self._onto_alt(si, cps)
				caps_a.append(cas - self.SKIN_SINK * nss)
		out["cap_calc"] = np.asarray(caps_c)
		out["cap_alt"] = np.asarray(caps_a) if ca is not None else None
		return out, si

	@staticmethod
	def _match_ring(ref, ring):
		"""輪 ring (M,3) の点の順番（始まりと回る向き）を、ref (M,3) にいちばん近くなるようにそろえる（皮がねじれない）。"""
		M = len(ring)
		idx = (np.arange(M)[None, :] + np.arange(M)[:, None]) % M
		best, out, perm = None, ring, np.arange(M)
		for base in (np.arange(M), np.arange(M)[::-1]):
			cand = ring[base][idx]
			err = ((cand - ref[None]) ** 2).sum(axis=(1, 2))
			j = int(np.argmin(err))
			if best is None or err[j] < best:
				best, out, perm = float(err[j]), cand[j], base[idx[j]]
		return out, perm

	def _skin_points(self, W, rf, M, foot=None):
		"""線維の点 W (F,K,3) → 皮の点 (K*M + 両端のふた, 3)。各所で楕円（向きはなめらかにつなぐ）。
		foot=({"ring": 1点目の付着部の輪, "cap": ふたの輪}, {2点目…})（世界座標）があれば、皮の両端をそれに合わせる。"""
		Fn, K, _ = W.shape
		C = W.mean(axis=0)
		out = np.zeros((K, M, 3))
		th = np.linspace(0, 2 * np.pi, M, endpoint=False)
		prev = None
		for k in range(K):
			t = C[min(k + 1, K - 1)] - C[max(k - 1, 0)]
			t /= max(np.linalg.norm(t), 1e-9)
			Q = W[:, k] - C[k]
			Q = Q - np.outer(Q @ t, t)
			if Fn >= 2:
				_u, s, vt = np.linalg.svd(Q, full_matrices=False)
				e1 = vt[0] - (vt[0] @ t) * t
			else:
				e1 = np.cross(t, [1.0, 0.0, 0.0])
			if np.linalg.norm(e1) < 1e-6:
				e1 = np.cross(t, [0.0, 1.0, 0.0])
			e1 /= np.linalg.norm(e1)
			if prev is not None and e1 @ prev < 0:
				e1 = -e1
			prev = e1
			e2 = np.cross(t, e1)
			r1 = float(np.abs(Q @ e1).max()) + rf if Fn else rf
			r2 = float(np.abs(Q @ e2).max()) + rf if Fn else rf
			out[k] = C[k] + np.outer(np.cos(th) * r1, e1) + np.outer(np.sin(th) * r2, e2)
		NS = len(self.SKIN_CAP_SCALES)
		caps = []
		tk = np.linspace(0.0, 1.0, K)
		z = float(self.SKIN_BLEND)
		for j_, (kend, tt) in enumerate(((0, tk), (K - 1, 1.0 - tk))):
			f_ = foot[j_] if foot is not None else None
			if f_ is None or f_.get("ring") is None:
				cap = np.repeat(np.repeat(out[kend].mean(axis=0)[None, None], M, axis=1), NS, axis=0)
				caps.append(cap)
				continue
			ring, perm = self._match_ring(out[kend], np.asarray(f_["ring"], float))
			w = np.clip((z - tt) / z, 0.0, 1.0)
			w = w * w * (3.0 - 2.0 * w)
			for k in np.where(w > 0)[0]:
				out[k] = (1.0 - w[k]) * out[k] + w[k] * ring
			cap = f_.get("cap")
			if cap is None:
				cap = np.repeat(np.repeat(ring.mean(axis=0)[None, None], M, axis=1), NS, axis=0)
			else:
				cap = np.asarray(cap, float)[:, perm]          # 端の輪と同じ並べ替え（ふたがねじれない）
			caps.append(cap)
		return np.concatenate([out.reshape(-1, 3), caps[0].reshape(-1, 3), caps[1].reshape(-1, 3)], axis=0)

	def _xsec_ring(self, b, W, fi, M=48):
		"""断面のめやす: 束の中ほどに、計った長径・短径の楕円（世界座標 M 点）。
		計算済みなら線維の中ほどの位置と広がりの向き、まだなら 2つの付着部の中心を結ぶ線の真ん中。"""
		AB = self._xsec(b)
		if AB is None:
			return None
		try:
			if W is not None:
				K = W.shape[1]
				C = W.mean(axis=0)
				k = K // 2
				c = C[k]
				tg = C[min(k + 1, K - 1)] - C[max(k - 1, 0)]
				tg /= max(np.linalg.norm(tg), 1e-9)
				Q = W[:, k] - c
				Q = Q - np.outer(Q @ tg, tg)
				e1 = np.linalg.svd(Q, full_matrices=False)[2][0] if len(Q) >= 2 else np.cross(tg, [1.0, 0, 0])
			else:
				ends = []
				for end in ("a", "b"):
					si = self._si_of_name(b[end]["bone_name"])
					T = self.poses[si][fi]
					e = b[end]["ellipse"]
					ends.append((T[:3, :3] @ np.asarray(e["center"], float) + T[:3, 3], T[:3, :3] @ np.asarray(e["u"], float)))
				c = 0.5 * (ends[0][0] + ends[1][0])
				tg = ends[1][0] - ends[0][0]
				tg /= max(np.linalg.norm(tg), 1e-9)
				e1 = ends[0][1] + ends[1][1]
			e1 = e1 - (e1 @ tg) * tg
			e1 /= max(np.linalg.norm(e1), 1e-9)
			e2 = np.cross(tg, e1)
			th = np.linspace(0.0, 2.0 * np.pi, M, endpoint=False)
			return c + np.outer(np.cos(th) * AB[0] / 2.0, e1) + np.outer(np.sin(th) * AB[1] / 2.0, e2)
		except Exception:
			return None

	def update(self, fi, render=False):
		try:
			self.fi = int(fi)
			fi = max(0, min(int(fi), self.n_view - 1)) if self.n_view else 0
			for b in self.data["bundles"]:
				a = self.actors.get(b.get("id"))
				if not a:
					continue
				for end in ("a", "b"):
					act = a.get(f"e{end}_act")
					if act is not None:
						si_e = a[f"e{end}_si"]
						act.SetUserMatrix(self._mat(self.poses[si_e][fi]))
						ring = a.get(f"e{end}_ring")
						kind = self._shown(si_e)
						if ring is not None and a.get(f"e{end}_kind") != kind:
							P_ = ring["out_alt"] if (kind == "alt" and ring.get("out_alt") is not None) else ring["out_calc"]
							from vtk.util.numpy_support import vtk_to_numpy as _v2n
							_v2n(a[f"e{end}_pd"].GetPoints().GetData())[:] = P_
							a[f"e{end}_pd"].GetPoints().Modified()
							a[f"e{end}_pd"].Modified()
							a[f"e{end}_kind"] = kind
				r = a.get("r")
				W = None
				if r is not None:
					T = self.poses[r["sia"]][fi]
					W = self._local_at(r, fi) @ T[:3, :3].T + T[:3, 3]
				if a.get("xs_act") is not None and a["xs_act"].GetVisibility():
					P_ = self._xsec_ring(b, W, fi)
					if P_ is not None:
						from vtk.util.numpy_support import vtk_to_numpy as _v2n
						_v2n(a["xs_pd"].GetPoints().GetData())[:] = P_
						a["xs_pd"].GetPoints().Modified()
						a["xs_pd"].Modified()
				if r is None:
					continue
				# 表示変更用モデルが見えている骨では、線維の端をその面の上へ
				if a.get("fend"):
					for col_, (si_e, Palt) in zip((0, -1), a["fend"]):
						if Palt is not None and self._shown(si_e) == "alt":
							Te = self.poses[si_e][fi]
							W[:, col_] = Palt @ Te[:3, :3].T + Te[:3, 3]
				from vtk.util.numpy_support import vtk_to_numpy
				if a["fib_act"].GetVisibility():
					vtk_to_numpy(a["fib_pts"].GetData())[:] = W.reshape(-1, 3)
					a["fib_pts"].Modified()
					a["fib_pd"].Modified()
				if a["skin_act"].GetVisibility():
					foot = None
					if a.get("foot"):
						foot = []
						for f_, si_f in a["foot"]:
							alt_ = self._shown(si_f) == "alt" and f_.get("alt") is not None
							Tf = self.poses[si_f][fi]
							ring_ = f_["alt"] if alt_ else f_["calc"]
							cap_ = f_.get("cap_alt") if alt_ else f_.get("cap_calc")
							foot.append({"ring": ring_ @ Tf[:3, :3].T + Tf[:3, 3],
							             "cap": (None if cap_ is None else np.asarray(cap_) @ Tf[:3, :3].T + Tf[:3, 3])})
						foot = tuple(foot)
					vtk_to_numpy(a["skin_pts"].GetData())[:] = self._skin_points(W, a["rf"], a["M"], foot)
					a["skin_pts"].Modified()
					a["skin_pd"].Modified()
			for act, si in self.guides:
				act.SetUserMatrix(self._mat(self.poses[si][fi]))
			for act, si in self.pend:
				act.SetUserMatrix(self._mat(self.poses[si][fi]))
			if render and self.plotter is not None:
				try:
					self.plotter.render()
				except Exception:
					pass
		except Exception as e:
			print(f"[靭帯の束] 更新に失敗: {e}")

	# ---------------------------------------------------------------- 目安の点（段階1の付着部の点のファイル）
	def _load_guides(self):
		if self.plotter is None:
			return
		vtk = self._vtk()
		for act, _si in self.guides:
			try:
				self.plotter.renderer.RemoveActor(getattr(act, "lab", act))
			except Exception:
				pass
		self.guides = []
		fp = str(self.data.get("guide_file") or "")
		if not fp or not os.path.exists(fp) or not self.data.get("guide_show", True):
			return
		try:
			d = json.loads(Path(fp).read_text(encoding="utf-8"))
			items = d.get("ligaments", []) if isinstance(d, dict) else list(d)
		except Exception as e:
			self._say(f"付着部の点のファイルを読めません: {e}", "red")
			return
		font = None
		try:
			font = self.app._sim_ligament_font_file()
		except Exception:
			pass
		for it in items:
			for key, tag in (("a", "1"), ("b", "2")):
				try:
					ep = it[key]
					si = self._si_of_name(ep.get("bone_name"))
					if si is None:
						continue
					p = [float(x) for x in ep["point"]]
					src = vtk.vtkSphereSource()
					src.SetCenter(*p)
					src.SetRadius(0.7)
					src.SetThetaResolution(12)
					src.SetPhiResolution(8)
					mp = vtk.vtkPolyDataMapper()
					mp.SetInputConnection(src.GetOutputPort())
					act = vtk.vtkActor()
					act.SetMapper(mp)
					act.PickableOff()
					act.GetProperty().SetColor(*self._rgb(str(it.get("color", "#ff00ff"))))
					self.plotter.renderer.AddActor(act)
					self.guides.append((act, si))
					lab = vtk.vtkTextActor()
					lab.SetInput(f"{it.get('name', '')}-{tag}")
					lab.PickableOff()
					lab.GetPositionCoordinate().SetCoordinateSystemToWorld()
					tp = lab.GetTextProperty()
					tp.SetFontSize(13)
					tp.SetColor(*self._rgb(str(it.get("color", "#ff00ff"))))
					tp.SetBackgroundColor(1, 1, 1)
					tp.SetBackgroundOpacity(0.6)
					if font:
						tp.SetFontFamily(vtk.VTK_FONT_FILE)
						tp.SetFontFile(font)
					self.plotter.renderer.AddActor(lab)
					self.guides.append((_LabelAt(lab, p), si))
				except Exception:
					continue
		self.update(self.fi, render=True)

	# ---------------------------------------------------------------- 管理の画面
	def _say(self, text, color="gray"):
		for key in ("status", "ol_msg"):
			lbl = self.w.get(key)
			try:
				if lbl is not None and lbl.winfo_exists():
					lbl.config(text=str(text), foreground=color)
			except Exception:
				pass
		print(f"[靭帯の束] {text}")

	def open_window(self):
		import tkinter as tk
		from tkinter import ttk
		try:
			if self.win is not None and self.win.winfo_exists():
				self.win.deiconify()
				self.win.lift()
				return
		except Exception:
			pass
		win = tk.Toplevel(self.app)
		win.title("靭帯の束（段階2）")
		win.geometry("1040x560")
		try:
			win.attributes("-topmost", True)
		except Exception:
			pass
		self.win = win
		win.protocol("WM_DELETE_WINDOW", self._close_window)
		top = ttk.LabelFrame(win, text="段階1の付着部の点（目安）")
		top.pack(fill="x", padx=8, pady=(8, 4))
		ttk.Button(top, text="付着部の点のファイルを読み込む…", command=self._choose_guide).pack(side="left", padx=6, pady=4)
		self.w["guide_var"] = tk.BooleanVar(win, value=bool(self.data.get("guide_show", True)))
		ttk.Checkbutton(top, text="点を表示", variable=self.w["guide_var"], command=self._toggle_guide).pack(side="left", padx=6)
		self.w["guide_lbl"] = ttk.Label(top, text=os.path.basename(str(self.data.get("guide_file") or "（まだ読み込んでいません）")),
		                                foreground="gray")
		self.w["guide_lbl"].pack(side="left", padx=6)

		mid = ttk.LabelFrame(win, text="束（例: ACL を AM・PL の2束、PCL を AL・PM の2束）")
		mid.pack(fill="both", expand=True, padx=8, pady=4)
		self.w["rows"] = ttk.Frame(mid)
		self.w["rows"].pack(fill="both", expand=True, padx=4, pady=4)
		ttk.Button(mid, text="＋ 束を追加", command=self._add_bundle).pack(anchor="w", padx=6, pady=(0, 6))

		ol = ttk.LabelFrame(win, text="付着部の面を描く（骨の上で外周を右クリック。5 点以上。打った点は左クリックで選べます。右ドラッグのズームは従来どおり）")
		ol.pack(fill="x", padx=8, pady=4)
		self.w["ol_lbl"] = ttk.Label(ol, text="（束の行の「面を描く」を押すと始まります）", foreground="gray")
		self.w["ol_lbl"].pack(anchor="w", padx=6)
		self.w["ol_msg"] = ttk.Label(ol, text="", foreground="gray", wraplength=980, justify="left")
		self.w["ol_msg"].pack(anchor="w", padx=6)
		olr = ttk.Frame(ol)
		olr.pack(fill="x", padx=6, pady=4)
		self.w["ol_list"] = tk.Listbox(olr, height=4, width=46, exportselection=False)
		self.w["ol_list"].pack(side="left")
		self.w["ol_list"].bind("<<ListboxSelect>>", lambda e: self._ol_on_list_select())
		bf = ttk.Frame(olr)
		bf.pack(side="left", padx=8)
		ttk.Button(bf, text="最後の点を取り消す", command=self._ol_undo).grid(row=0, column=0, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="選んだ点を打ち直す", command=self._ol_replace).grid(row=0, column=1, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="選んだ点を消す", command=self._ol_delete).grid(row=1, column=0, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="すべて消す", command=self._ol_clear).grid(row=1, column=1, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="① 楕円を当てはめる", command=self._ol_fit).grid(row=2, column=0, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="② この楕円で決める（保存）", command=self._ol_commit).grid(row=2, column=1, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="前の点を読み込む（直すとき）", command=self._ol_load_prev).grid(row=3, column=0, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="やめる", command=self._ol_cancel).grid(row=3, column=1, sticky="ew", padx=2, pady=1)
		cf = ttk.Frame(ol)
		cf.pack(fill="x", padx=6, pady=(0, 2))
		ttk.Label(cf, text="ノギスで測った値（端から端）: 長径").pack(side="left")
		for k, lab_ in (("cal_L", "mm　短径"), ("cal_S", "mm（入れると①の結果との差を出します。空でも可）")):
			e_ = ttk.Entry(cf, width=7)
			e_.pack(side="left", padx=(2, 2))
			for sq_ in ("<KeyRelease>", "<FocusOut>", "<Return>"):
				e_.bind(sq_, lambda ev: self._ol_dims())
			self.w[k] = e_
			ttk.Label(cf, text=lab_).pack(side="left")
		self.w["ol_dims"] = ttk.Label(ol, text="", font=("", 11, "bold"))
		self.w["ol_dims"].pack(anchor="w", padx=6, pady=(0, 4))

		# 付着部の楕円を動かす（平行移動。2026-10-20）
		mv = ttk.LabelFrame(win, text="付着部の楕円を動かす（平行移動。いまの視点で見た上下左右に、楕円の面の中で）")
		mv.pack(fill="x", padx=8, pady=4)
		mvr = ttk.Frame(mv)
		mvr.pack(fill="x", padx=6, pady=4)
		ttk.Label(mvr, text="動かす楕円").pack(side="left")
		self.w["mv_target"] = tk.StringVar(win, value="")
		self.w["mv_combo"] = ttk.Combobox(mvr, textvariable=self.w["mv_target"], state="readonly", width=34)
		self.w["mv_combo"].pack(side="left", padx=(4, 10))
		self.w["mv_combo"].bind("<Button-1>", lambda e: self._mv_refresh_targets())
		ttk.Label(mvr, text="1 回").pack(side="left")
		self.w["mv_step"] = tk.StringVar(win, value="0.5")
		ttk.Spinbox(mvr, from_=0.1, to=10.0, increment=0.1, width=5, textvariable=self.w["mv_step"]).pack(side="left", padx=2)
		ttk.Label(mvr, text="mm").pack(side="left", padx=(0, 8))
		for lab_, dx_, dy_ in (("↑", 0, 1), ("↓", 0, -1), ("←", -1, 0), ("→", 1, 0)):
			ttk.Button(mvr, text=lab_, width=3, command=lambda dx=dx_, dy=dy_: self._mv_click(dx, dy)).pack(side="left", padx=1)
		ttk.Button(mvr, text="動かす前に戻す", command=self._mv_undo).pack(side="left", padx=(10, 0))
		self.w["mv_lbl"] = ttk.Label(mv, text="（束の面を決めると選べます。動かすと計算の結果は外れるので、もう一度「▶ 計算する」）",
		                             foreground="gray")
		self.w["mv_lbl"].pack(anchor="w", padx=6, pady=(0, 4))
		self._mv_refresh_targets()

		bot = ttk.Frame(win)
		bot.pack(fill="x", padx=8, pady=(4, 8))
		pp = self.params()
		ttk.Label(bot, text="表示:").pack(side="left")
		self.w["view_var"] = tk.StringVar(win, value=str(pp.get("view", "both")))
		for v_, t_ in (("fibers", "線維"), ("skin", "楕円の皮"), ("both", "両方"), ("foot", "付着部の楕円だけ")):
			ttk.Radiobutton(bot, text=t_, value=v_, variable=self.w["view_var"], command=self._set_view).pack(side="left")
		ttk.Label(bot, text="　自然長 ×").pack(side="left")
		self.w["scale_var"] = tk.StringVar(win, value=f"{100.0 * float(pp.get('rest_scale', 1.0)):g}")
		ttk.Spinbox(bot, from_=80, to=120, increment=1, width=5, textvariable=self.w["scale_var"]).pack(side="left")
		ttk.Label(bot, text="%（表示の始まりのコマで張った長さに対して）").pack(side="left")
		self.w["grav_var"] = tk.BooleanVar(win, value=float(pp.get("gravity", 0.02)) > 0)
		ttk.Checkbutton(bot, text="たるみを下へ（弱い重さ）", variable=self.w["grav_var"]).pack(side="left", padx=8)
		self.w["outline_var"] = tk.BooleanVar(win, value=bool(pp.get("show_outline", True)))
		ttk.Checkbutton(bot, text="付着部の楕円", variable=self.w["outline_var"],
		                command=lambda: self._set_param_view("show_outline", bool(self.w["outline_var"].get()))).pack(side="left", padx=4)
		self.w["xsec_var"] = tk.BooleanVar(win, value=bool(pp.get("show_xsec", True)))
		ttk.Checkbutton(bot, text="断面のめやす", variable=self.w["xsec_var"],
		                command=lambda: self._set_param_view("show_xsec", bool(self.w["xsec_var"].get()))).pack(side="left", padx=4)
		ttk.Button(bot, text="▶ 計算する", command=self._compute_clicked).pack(side="right", padx=4)
		# 靭帯（線維・皮）だけの不透明度（骨・付着部の楕円は変えない。2026-10-20）
		bot2 = ttk.Frame(win)
		bot2.pack(fill="x", padx=8, pady=(0, 6))
		ttk.Label(bot2, text="靭帯の不透明度（線維・皮だけ）").pack(side="left")
		self.w["op_var"] = tk.DoubleVar(win, value=round(100.0 * self._lig_opacity()))
		self.w["op_lbl"] = ttk.Label(bot2, text=f"{int(round(100.0 * self._lig_opacity()))} %", width=6)

		def _op_move(v):
			try:
				val = float(v) / 100.0
			except ValueError:
				return
			self.data.setdefault("params", {})["lig_opacity"] = max(0.0, min(1.0, val))
			self.w["op_lbl"].config(text=f"{int(round(100.0 * self._lig_opacity()))} %")
			self._apply_lig_opacity()
		sc_ = ttk.Scale(bot2, from_=0, to=100, orient="horizontal", length=220, variable=self.w["op_var"], command=_op_move)
		sc_.pack(side="left", padx=6)
		sc_.bind("<ButtonRelease-1>", lambda e: self._save())          # 離したときに覚える
		self.w["op_scale"] = sc_
		self.w["op_lbl"].pack(side="left")
		ttk.Label(bot2, text="（「付着部の楕円だけ」で靭帯を隠し、付着部の楕円だけを出せます）", foreground="gray").pack(side="left", padx=6)
		self.w["status"] = ttk.Label(win, text="", foreground="gray", wraplength=1000, justify="left")
		self.w["status"].pack(fill="x", padx=10, pady=(0, 6))
		self._rows()
		self._start_poll()
		n_ok = sum(1 for b in self.data["bundles"] if self.results.get(b.get("id")) is not None)
		self._say(f"束 {len(self.data['bundles'])} 個（計算済み {n_ok}）。" if self.data["bundles"]
		          else "まず「付着部の点のファイルを読み込む…」で目安の点を出し、「＋ 束を追加」→「面を描く」で付着部の面を作ってください。")

	def _close_window(self):
		self._ol_cancel(quiet=True)
		self._stop_poll()
		try:
			if self.win is not None and self.win.winfo_exists():
				self.win.destroy()
		except Exception:
			pass
		self.win = None
		self.w = {}

	def _rows(self):
		import tkinter as tk
		from tkinter import ttk
		fr = self.w.get("rows")
		if fr is None:
			return
		for c in fr.winfo_children():
			c.destroy()
		hdr = ("表示", "グラフ", "名前", "色", "線維の本数", "断面の長径 mm", "断面の短径 mm", "線維の太さ(半径) mm",
		       "1点目の骨の面", "2点目の骨の面", "計算", "")
		for c, h in enumerate(hdr):
			ttk.Label(fr, text=h, font=("", 9, "bold")).grid(row=0, column=c, padx=4, sticky="w")
		self.w["row_w"] = {}
		for r_, b in enumerate(self.data["bundles"], start=1):
			rw = {}
			vv = tk.BooleanVar(fr, value=bool(b.get("visible", True)))
			ttk.Checkbutton(fr, variable=vv, command=lambda b=b, v=vv: self._set(b, "visible", bool(v.get()), rebuild=True)).grid(row=r_, column=0)
			cv = tk.BooleanVar(fr, value=bool(b.get("chart", True)))
			ttk.Checkbutton(fr, variable=cv, command=lambda b=b, v=cv: self._set(b, "chart", bool(v.get()), notify=True)).grid(row=r_, column=1)
			nv = tk.StringVar(fr, value=str(b.get("name", "")))
			ttk.Entry(fr, textvariable=nv, width=14).grid(row=r_, column=2, sticky="w")
			nv.trace_add("write", lambda *_a, b=b, v=nv: self._set(b, "name", v.get(), notify=True))
			tk.Button(fr, text="    ", bg=b.get("color", "#FF8800"), command=lambda b=b: self._pick_color(b)).grid(row=r_, column=3)
			# 数の欄: 打っている間は何もしない。Enter・欄を離れたとき・▲▼で反映（以前は1文字ごとに一覧を
			# 作り直していたので、欄そのものが作り直されて入力が終わってしまった。2026-10-16 ユーザー指摘）
			for col_, key, typ, lo, hi, inc, w_ in ((4, "n_fibers", int, 1, 300, 1, 5), (5, "xsec_a", float, 0, 30, 0.5, 6),
			                                       (6, "xsec_b", float, 0, 30, 0.5, 6)):
				dv = b.get(key, 10 if key == "n_fibers" else 0)
				sv = tk.StringVar(fr, value=(str(int(dv)) if typ is int else f"{float(dv or 0):g}"))
				sp_ = ttk.Spinbox(fr, from_=lo, to=hi, increment=inc, width=w_, textvariable=sv)
				sp_.configure(command=lambda b=b, k=key, v=sv, t=typ: self._commit_num(b, k, v, t))
				sp_.grid(row=r_, column=col_)
				for ev in ("<Return>", "<FocusOut>", "<KP_Enter>"):
					sp_.bind(ev, lambda _e, b=b, k=key, v=sv, t=typ: self._commit_num(b, k, v, t))
				rw[key] = sp_
			# 線維の太さ: 断面があれば自動（表示だけ）、無ければ手で入れる
			rf_fr = ttk.Frame(fr)
			rf_fr.grid(row=r_, column=7, sticky="w")
			rv = tk.StringVar(fr, value=f"{float(b.get('fiber_radius', 0.4)):g}")
			rsp = ttk.Spinbox(rf_fr, from_=0.05, to=3.0, increment=0.05, width=5, textvariable=rv)
			rsp.configure(command=lambda b=b, v=rv: self._commit_num(b, "fiber_radius", v, float))
			for ev in ("<Return>", "<FocusOut>", "<KP_Enter>"):
				rsp.bind(ev, lambda _e, b=b, v=rv: self._commit_num(b, "fiber_radius", v, float))
			rlb = ttk.Label(rf_fr, text="", foreground="#1b5e20")
			rw.update(rsp=rsp, rlb=rlb)
			for c_, end in ((8, "a"), (9, "b")):
				e = b.get(end) or {}
				f_ = ttk.Frame(fr)
				f_.grid(row=r_, column=c_, sticky="w")
				txt = (f"{e.get('bone_name', '')} {2 * float(e['ellipse']['ra']):.1f}×{2 * float(e['ellipse']['rb']):.1f}mm"
				       if e.get("ellipse") else "まだ")
				ttk.Label(f_, text=txt, foreground=("black" if e.get("ellipse") else "#a06000")).pack(side="left")
				ttk.Button(f_, text="面を描く", command=lambda b=b, end=end: self._ol_start(b, end)).pack(side="left", padx=2)
			st_ = ttk.Label(fr, text="")
			st_.grid(row=r_, column=10)
			rw["st"] = st_
			ttk.Button(fr, text="削除", width=6, command=lambda b=b: self._delete(b)).grid(row=r_, column=11)
			self.w["row_w"][b["id"]] = rw
		self._refresh_row_labels()
		ttk.Label(fr, text=("※ 断面の長径・短径（束の中ほどで計った太さ）を入れると、線維の太さは本数から自動で決まります"
		                    f"（本数 × 線維の断面積 ＝ 楕円の断面積 × {PACKING:g}）。計算では、束の中ほどの線維がこの楕円の中に収まります。"
		                    "空（0）にすると、線維の太さを手で入れます。数の欄は Enter・欄を離れたとき・▲▼で反映します。"),
		          foreground="gray", wraplength=980, justify="left").grid(row=len(self.data["bundles"]) + 1, column=0,
		                                                                  columnspan=len(hdr), sticky="w", pady=(6, 0))

	def _refresh_row_labels(self):
		"""一覧の「線維の太さ」と「計算」の欄を、一覧を作り直さずに更新する（入力中の欄を壊さない）。"""
		for b in self.data["bundles"]:
			rw = (self.w.get("row_w") or {}).get(b.get("id"))
			if not rw:
				continue
			try:
				if self._xsec(b) is not None:
					rw["rsp"].pack_forget()
					rw["rlb"].config(text=f"自動 {self._rf(b):.3f}")
					rw["rlb"].pack(side="left")
				else:
					rw["rlb"].pack_forget()
					rw["rsp"].pack(side="left")
				done = self.results.get(b.get("id")) is not None
				rw["st"].config(text=("済み" if done else "要計算"), foreground=("#1b5e20" if done else "#a06000"))
			except Exception:
				pass

	def _commit_num(self, b, key, var, typ):
		"""数の欄の値を反映する（変わったときだけ。計算し直しが要る）。おかしな値なら元に戻す。"""
		cur = b.get(key, 10 if key == "n_fibers" else (0.4 if key == "fiber_radius" else 0))
		try:
			x = int(float(var.get())) if typ is int else float(var.get())
		except (TypeError, ValueError):
			var.set(str(int(cur)) if typ is int else f"{float(cur or 0):g}")
			return
		lo = 1 if key == "n_fibers" else (0.05 if key == "fiber_radius" else 0.0)
		if x < lo:
			var.set(str(int(cur)) if typ is int else f"{float(cur or 0):g}")
			return
		if float(cur or 0) == float(x):
			return
		b[key] = x
		self._save()
		self._invalidate(rows=False)
		self._refresh_row_labels()

	def _set(self, b, key, val, rebuild=False, notify=False):
		b[key] = val
		self._save()
		if rebuild:
			self._rebuild_actors()
		if notify:
			self._notify()

	def _invalidate(self, rows=True):
		"""束の形・本数などが変わった → 計算し直しが要る（結果を外す）。
		rows=False: 一覧は作り直さない（入力中の欄を壊さない）。"""
		self.results = {}
		self._rebuild_actors()
		self._notify()
		if rows:
			self._rows()

	def _pick_color(self, b):
		from tkinter import colorchooser
		res = colorchooser.askcolor(color=b.get("color", "#FF8800"), parent=self.win)
		if res and res[1]:
			b["color"] = str(res[1])
			self._save()
			self._rebuild_actors()
			self._notify()
			self._rows()

	def _add_bundle(self):
		used = {b.get("name") for b in self.data["bundles"]}
		k = len(self.data["bundles"]) + 1
		while f"束{k}" in used:
			k += 1
		b = {"id": hashlib.sha1(f"{time.time()}{k}".encode()).hexdigest()[:10], "name": f"束{k}",
		     "color": PALETTE[len(self.data["bundles"]) % len(PALETTE)], "visible": True, "chart": True,
		     "n_fibers": 10, "fiber_radius": 0.4, "a": {}, "b": {}}
		self.data["bundles"].append(b)
		self._save()
		self._rows()
		self._say(f"「{b['name']}」を足しました。名前の欄で変えられます（例: ACL(AM)）。次に「面を描く」で付着部の面を作ります。", "blue")

	def _delete(self, b):
		from tkinter import messagebox
		if not messagebox.askyesno("束の削除", f"「{b.get('name', '')}」を削除しますか？", parent=self.win):
			return
		self.data["bundles"] = [x for x in self.data["bundles"] if x is not b]
		self._save()
		self._invalidate()

	def _set_param_view(self, key, val):
		"""表示だけの設定（計算し直しは要らない。計算の鍵には入れていない）。"""
		self.data.setdefault("params", {})[key] = val
		self._save()
		self._rebuild_actors()

	def _set_view(self):
		self.data.setdefault("params", {})["view"] = str(self.w["view_var"].get())
		self._save()
		self._rebuild_actors()

	def _compute_clicked(self):
		try:
			sc = float(self.w["scale_var"].get()) / 100.0
		except Exception:
			sc = 1.0
		p = self.data.setdefault("params", {})
		p["rest_scale"] = max(0.5, min(1.5, sc))
		p["gravity"] = 0.02 if bool(self.w["grav_var"].get()) else 0.0
		self._save()
		self._load_cached_results()
		ready = [b for b in self.data["bundles"] if self._ready(b)]
		if ready and all(self.results.get(b["id"]) is not None for b in ready):
			self._rebuild_actors()
			self._notify()
			self._rows()
			self._say("保存した結果を使いました（計算し直しはしていません）。", "#1b5e20")
			return
		self.compute(parent=self.win)
		self._rows()

	def _choose_guide(self):
		from tkinter import filedialog
		fp = filedialog.askopenfilename(parent=self.win, title="付着部の点のファイル（段階1の靭帯のファイル）",
		                                filetypes=[("靭帯ファイル (JSON)", "*.json"), ("すべてのファイル", "*.*")])
		if not fp:
			return
		self.data["guide_file"] = fp
		self.data["guide_show"] = True
		try:
			self.w["guide_var"].set(True)
			self.w["guide_lbl"].config(text=os.path.basename(fp))
		except Exception:
			pass
		self._save()
		self._load_guides()
		self._say(f"付着部の点を表示しました（{os.path.basename(fp)}）。点のまわりを右クリックして面を作ってください。", "blue")

	def _toggle_guide(self):
		self.data["guide_show"] = bool(self.w["guide_var"].get())
		self._save()
		self._load_guides()

	# ---------------------------------------------------------------- 外周を打つ（右クリック）
	def _ol_start(self, b, end):
		self._ol_cancel(quiet=True)
		self.st["mode"] = {"b": b, "end": end, "si": None, "pts": [], "prev": None}
		# 新しく描くときは空から。前に描いた外周は「前の点を読み込む」で読み込める（直すとき）。
		# （以前は自動で読み込んでいたので、新しく打った点が前の点と混ざって楕円が当てはめられ、思った楕円にならなかった。2026-10-19）
		e = b.get(end) or {}
		si = self._si_of_name(e.get("bone_name"))
		if e.get("outline") and si is not None:
			self.st["mode"]["prev"] = (si, [np.asarray(p, float) for p in e["outline"]])
		cal = e.get("caliper") or [None, None]
		for k, v in zip(("cal_L", "cal_S"), cal):
			try:
				self.w[k].delete(0, "end")
				if v is not None:
					self.w[k].insert(0, f"{float(v):g}")
			except Exception:
				pass
		self._install_observers()
		self._ol_refresh()
		nm = f"「{b.get('name', '')}」の{'1' if end == 'a' else '2'}点目の骨の面"
		steps = "外周を右クリックで 5 点以上 → ①「楕円を当てはめる」で長径・短径を確かめる → よければ ②「この楕円で決める」。"
		if self.st["mode"]["prev"] is not None:
			self._say(f"{nm}: {steps}前の面（赤い輪）は①を押すまで出したまま、①で黄色の輪に替わります（やめると戻ります）。"
			          f"前の {len(self.st['mode']['prev'][1])} 点を直したいときは「前の点を読み込む（直すとき）」。", "blue")
		else:
			self._say(f"{nm}: {steps}", "blue")

	def _ol_load_prev(self):
		"""前に描いた外周の点を読み込む（いま打っている点とは入れ替える）。"""
		m = self.st.get("mode")
		if not m:
			self._say("先に束の行の「面を描く」を押してください。", "#a06000")
			return
		if m.get("prev") is None:
			self._say("前に描いた外周はありません。", "#a06000")
			return
		si, pts = m["prev"]
		m["si"] = si
		m["pts"] = [np.asarray(p, float).copy() for p in pts]
		self.st["replace"] = None
		self.st["sel"] = None
		self._ol_refresh()
		self._say(f"前の外周の {len(pts)} 点を読み込みました。点は 3D の画面で左クリック（または一覧）で選んで、消す・打ち直すができます。", "blue")

	def _ol_set_sel(self, k):
		"""点 k を選ぶ（一覧も選び、3D の点の色を変える）。None で選びを外す。"""
		m = self.st.get("mode")
		if not m or k is None or not (0 <= int(k) < len(m["pts"])):
			k = None
		self.st["sel"] = None if k is None else int(k)
		lb = self.w.get("ol_list")
		try:
			if lb is not None and lb.winfo_exists():
				lb.selection_clear(0, "end")
				if k is not None:
					lb.selection_set(int(k))
					lb.see(int(k))
		except Exception:
			pass
		self._draw_pending()

	def _ol_on_list_select(self):
		lb = self.w.get("ol_list")
		try:
			cur = lb.curselection()
		except Exception:
			cur = ()
		if cur and int(cur[0]) != self.st.get("sel"):
			self.st["sel"] = int(cur[0])
			self._draw_pending()

	def _ol_sel_index(self):
		"""選んでいる点の番号（一覧の選び、無ければ 3D で選んだ点）。"""
		lb = self.w.get("ol_list")
		try:
			if lb is not None and lb.curselection():
				return int(lb.curselection()[0])
		except Exception:
			pass
		return self.st.get("sel")

	def _ol_refresh(self):
		m = self.st.get("mode")
		lb = self.w.get("ol_list")
		if lb is not None:
			lb.delete(0, "end")
			if m:
				for k, p in enumerate(m["pts"]):
					lb.insert("end", f"{k + 1}: ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})")
			sel = self.st.get("sel")
			if m and sel is not None and 0 <= sel < len(m["pts"]):
				lb.selection_set(sel)
				lb.see(sel)
			else:
				self.st["sel"] = None
		if self.w.get("ol_lbl") is not None:
			if m:
				bone = self.names[m["si"]] if m["si"] is not None else "（最初の点の骨）"
				rp = self.st.get("replace")
				self.w["ol_lbl"].config(text=(f"「{m['b'].get('name', '')}」{'1' if m['end'] == 'a' else '2'}点目・{bone}: {len(m['pts'])} 点"
				                               + (f"　→ 次の右クリックで {rp + 1} 番目を打ち直します" if rp is not None else "")
				                               + ("（5 点以上で決められます）" if len(m["pts"]) < 5 else "")),
				                         foreground="blue")
			else:
				self.w["ol_lbl"].config(text="（束の行の「面を描く」を押すと始まります）", foreground="gray")
		self._draw_pending()

	def _drawing(self, b, end):
		"""いまこの束のこの端の面を描いていて、①「楕円を当てはめる」を押した後か（そのときだけ前の面の輪を隠す）。
		（描いている間は前の輪を見せたまま、当てはめたら黄色の輪に替える。2026-10-20 ユーザーの要望）"""
		m = self.st.get("mode") if isinstance(getattr(self, "st", None), dict) else None
		return bool(m) and m.get("b") is b and m.get("end") == end and bool(m.get("fitted"))

	# ---------------------------------------------------------------- 付着部の楕円を動かす（平行移動）
	def _mv_targets(self):
		"""動かせる楕円: [(表示名, 束, 端)]。"""
		out = []
		for b in self.data.get("bundles", []):
			for end in ("a", "b"):
				e = b.get(end) or {}
				if e.get("ellipse") and self._si_of_name(e.get("bone_name")) is not None:
					out.append((f"{b.get('name', '')}・{'1' if end == 'a' else '2'}点目（{e.get('bone_name', '')}）", b, end))
		return out

	def _mv_refresh_targets(self):
		cb = self.w.get("mv_combo")
		if cb is None:
			return
		try:
			labs = [t[0] for t in self._mv_targets()]
			cb.configure(values=labs)
			if labs and self.w["mv_target"].get() not in labs:
				self.w["mv_target"].set(labs[0])
			elif not labs:
				self.w["mv_target"].set("")
		except Exception:
			pass

	def _mv_selected(self):
		lab = str(self.w["mv_target"].get()) if self.w.get("mv_target") is not None else ""
		return next(((b, end) for (l_, b, end) in self._mv_targets() if l_ == lab), (None, None))

	def _screen_dirs(self):
		"""いまの視点の、画面の右向き・上向き（世界座標の単位ベクトル）。"""
		cam = self.plotter.camera
		dop = np.asarray(cam.GetDirectionOfProjection(), float)
		up = np.asarray(cam.GetViewUp(), float)
		right = np.cross(dop, up)
		right /= max(float(np.linalg.norm(right)), 1e-12)
		up2 = np.cross(right, dop)
		up2 /= max(float(np.linalg.norm(up2)), 1e-12)
		return right, up2

	def move_ellipse(self, b, end, dx, dy, step):
		"""束 b の端 end の楕円を、画面の右 dx・上 dy の向きに step mm 動かす（楕円の面の中で）。返り値: 動かした量（mm, 骨の座標）。"""
		e = b.get(end) or {}
		ell = e.get("ellipse")
		si = self._si_of_name(e.get("bone_name"))
		if not ell or si is None or self.plotter is None:
			return None
		bones = self._ensure_bones()
		idx = self.idx_of[si]
		right, up = self._screen_dirs()
		w = float(dx) * right + float(dy) * up
		fi = max(0, min(int(self.fi), self.n_view - 1)) if self.n_view else 0
		T = self.poses[si][fi]
		lv = T[:3, :3].T @ w                                   # 骨の座標へ
		n = ell_normal(bones, idx, ell)
		lv = lv - float(lv @ n) * n                            # 楕円の面の中へ
		ln = float(np.linalg.norm(lv))
		if ln < 1e-9:
			return None
		d = lv / ln * float(step)
		key = (id(b), end)
		und = self.__dict__.setdefault("_mv_undo_store", {})
		if key not in und:
			und[key] = json.loads(json.dumps(e))               # 動かす前（この画面を開いている間の最初）
		ell["center"] = (np.asarray(ell["center"], float) + d).tolist()
		if e.get("outline"):
			O = np.asarray(e["outline"], float) + d
			e["outline"] = bones.onto(idx, O, n)[0].tolist()
		self._save()
		self._invalidate()
		self._mv_refresh_targets()
		return d

	def _mv_click(self, dx, dy):
		b, end = self._mv_selected()
		if b is None:
			self._say("動かす楕円を選んでください（束の面を決めると選べます）。", "#a06000")
			return
		try:
			step = float(self.w["mv_step"].get())
		except ValueError:
			step = 0.5
		if self.st.get("mode"):
			self._ol_cancel(quiet=True)
		d = self.move_ellipse(b, end, dx, dy, step)
		if d is None:
			self._say("動かせませんでした（いまの視点では、その向きは楕円の面に垂直です）。", "#a06000")
			return
		e0 = (self.__dict__.get("_mv_undo_store") or {}).get((id(b), end))
		tot = float(np.linalg.norm(np.asarray(b[end]["ellipse"]["center"], float) - np.asarray(e0["ellipse"]["center"], float))) if e0 else 0.0
		msg = f"{self.w['mv_target'].get()} を {step:g} mm 動かしました（動かす前から {tot:.1f} mm）。計算の結果は外れました。"
		self.w["mv_lbl"].config(text=msg, foreground="#0b5394")
		self._say(msg, "blue")

	def _mv_undo(self):
		b, end = self._mv_selected()
		und = self.__dict__.get("_mv_undo_store") or {}
		if b is None or (id(b), end) not in und:
			self._say("戻す楕円がありません（この画面で動かした楕円だけ戻せます）。", "#a06000")
			return
		b[end] = und.pop((id(b), end))
		self._save()
		self._invalidate()
		self._mv_refresh_targets()
		self.w["mv_lbl"].config(text=f"{self.w['mv_target'].get()} を動かす前に戻しました。", foreground="#1b5e20")

	def _outline_on(self):
		"""付着部の楕円を出すか（「付着部の楕円」のチェック、または表示が「付着部の楕円だけ」）。"""
		pp = self.params()
		return bool(pp.get("show_outline", True)) or str(pp.get("view", "both")) == "foot"

	def _lig_opacity(self):
		try:
			return float(min(1.0, max(0.0, float(self.params().get("lig_opacity", 1.0)))))
		except Exception:
			return 1.0

	def _apply_lig_opacity(self, render=True):
		"""線維と皮の不透明度だけを変える（作り直さない）。"""
		op = self._lig_opacity()
		view = str(self.params().get("view", "both"))
		for a in self.actors.values():
			if a.get("fib_act") is not None:
				a["fib_act"].GetProperty().SetOpacity(op)
			if a.get("skin_act") is not None:
				a["skin_act"].GetProperty().SetOpacity((0.55 if view == "both" else 1.0) * op)
		if render and self.plotter is not None:
			try:
				self.plotter.render()
			except Exception:
				pass

	def _old_ring_vis(self):
		"""前の面の輪の見え方を、いまの状態（表示・付着部の楕円・描いている端）に合わせる。"""
		show_ol = self._outline_on()
		for b in self.data["bundles"]:
			a = self.actors.get(b.get("id")) or {}
			for end in ("a", "b"):
				act = a.get(f"e{end}_act")
				if act is not None:
					act.SetVisibility(bool(b.get("visible", True)) and show_ol and not self._drawing(b, end))

	def _preview_ring(self, si, pts):
		"""打った点に当てはめた楕円を、決めたあとと同じく骨の面へ落とした輪（モデル座標）。できなければ平らな楕円。"""
		ell = fit_ellipse(pts)
		try:
			# 中心は打った点の平面の上のまま（いちばん近い面の点へ動かすと横へずれた。2026-10-20）
			ring, _si = self._foot_ring({"ellipse": ell, "bone_name": self.names[si]}, 48)
			P_ = ring["out_alt"] if (self._shown(si) == "alt" and ring.get("out_alt") is not None) else ring["out_calc"]
			return self._ring_poly(P_)
		except Exception as e:
			print(f"[靭帯の束] 見本の輪を骨の面に落とせません（平らな楕円で描きます）: {e}")
			return self._ellipse_poly(ell)

	def _draw_pending(self):
		if self.plotter is None:
			return
		self._old_ring_vis()
		vtk = self._vtk()
		for act, _si in self.pend:
			try:
				self.plotter.renderer.RemoveActor(act)
			except Exception:
				pass
		self.pend = []
		m = self.st.get("mode")
		if m and m["si"] is not None:
			sel = self.st.get("sel")
			for k_, p in enumerate(m["pts"]):
				is_sel = (k_ == sel)
				src = vtk.vtkSphereSource()
				src.SetCenter(*[float(x) for x in p])
				src.SetRadius(0.9 if is_sel else 0.5)
				mp = vtk.vtkPolyDataMapper()
				mp.SetInputConnection(src.GetOutputPort())
				act = vtk.vtkActor()
				act.SetMapper(mp)
				act.PickableOff()
				# 選んだ点 = 赤紫、ほか = 黄
				act.GetProperty().SetColor(*((1.0, 0.0, 1.0) if is_sel else (1.0, 0.83, 0.0)))
				self.plotter.renderer.AddActor(act)
				self.pend.append((act, m["si"]))
			if m.get("fitted") and len(m["pts"]) >= 5:
				try:
					# 決めたあとの輪と同じ形（骨の面へ落とした輪）で見せる。点を直すとその場で当てはめ直す
					act = self._line_actor(self._preview_ring(m["si"], m["pts"]), "#FFD400", 3.0)
					self.pend.append((act, m["si"]))
				except Exception:
					pass
		self._ol_dims()
		self.update(self.fi, render=True)

	def _caliper_vals(self):
		"""ノギスの長径・短径（mm）。入っていない・数でなければ None。"""
		out = []
		for k in ("cal_L", "cal_S"):
			try:
				v = float(str(self.w[k].get()).strip())
				out.append(v if v > 0 else None)
			except Exception:
				out.append(None)
		return out

	def _ol_dims(self):
		"""①で当てはめた楕円の長径・短径（端から端 = 2 × 半径）を出す。ノギスの値があれば差も。"""
		lbl = self.w.get("ol_dims")
		try:
			if lbl is None or not lbl.winfo_exists():
				return
		except Exception:
			return
		m = self.st.get("mode")
		if not m or not m.get("fitted"):
			lbl.config(text="")
			return
		if len(m["pts"]) < 5:
			lbl.config(text="当てはめた楕円: 点が 5 点より少ないので当てはめられません", foreground="#a06000")
			return
		try:
			ell = fit_ellipse(m["pts"])
		except Exception as e:
			lbl.config(text=f"当てはめた楕円: 当てはめられません（{e}）", foreground="red")
			return
		L, S = 2.0 * float(ell["ra"]), 2.0 * float(ell["rb"])
		cL, cS = self._caliper_vals()

		def _cmp(v, c):
			return f"{v:.1f} mm" + (f"（ノギス {c:g} mm、差 {v - c:+.1f}）" if c is not None else "")
		lbl.config(text=f"当てはめた楕円（{len(m['pts'])} 点）: 長径 {_cmp(L, cL)}　短径 {_cmp(S, cS)}"
		                + "　→ よければ ②「この楕円で決める」", foreground="#0b5394")

	def _ol_undo(self):
		m = self.st.get("mode")
		if m and m["pts"]:
			m["pts"].pop()
			if not m["pts"]:
				m["si"] = None
			self.st["replace"] = None
			if self.st.get("sel") is not None and self.st["sel"] >= len(m["pts"]):
				self.st["sel"] = None
			self._ol_refresh()

	def _ol_delete(self):
		m = self.st.get("mode")
		k = self._ol_sel_index()
		if not m or k is None or not (0 <= k < len(m["pts"])):
			self._say("消す点を、3D の画面で左クリック（または一覧）で選んでください。", "#a06000")
			return
		del m["pts"][k]
		if not m["pts"]:
			m["si"] = None
		self.st["replace"] = None
		self.st["sel"] = None
		self._ol_refresh()
		self._say(f"{k + 1} 番目の点を消しました（残り {len(m['pts'])} 点）。", "blue")

	def _ol_replace(self):
		m = self.st.get("mode")
		k = self._ol_sel_index()
		if not m or k is None or not (0 <= k < len(m["pts"])):
			self._say("打ち直す点を、3D の画面で左クリック（または一覧）で選んでください。", "#a06000")
			return
		self.st["replace"] = k
		self.st["sel"] = k
		self._ol_refresh()

	def _ol_clear(self):
		m = self.st.get("mode")
		if m:
			m["pts"] = []
			m["si"] = None
			self.st["replace"] = None
			self.st["sel"] = None
			self._ol_refresh()

	def _ol_fit(self):
		"""① 楕円を当てはめる: 黄色の輪（骨の面へ落とした形）を出し、前の赤い輪を隠し、長径・短径を出す。まだ保存しない。"""
		m = self.st.get("mode")
		if not m:
			self._say("先に束の行の「面を描く」を押してください。", "#a06000")
			return
		if len(m["pts"]) < 5:
			self._say("外周の点が 5 点より少ないので、楕円を当てはめられません。", "#a06000")
			return
		m["fitted"] = True
		self._draw_pending()
		self._say("楕円を当てはめました（黄色の輪）。長径・短径をノギスの値と比べて、よければ ②「この楕円で決める」。"
		          "違えば、点を 3D の画面で左クリックして選び、「選んだ点を消す」（または Delete キー）・「選んだ点を打ち直す」"
		          "で直すと、楕円と長径・短径がその場で変わります。", "blue")

	def _ol_commit(self):
		m = self.st.get("mode")
		if not m or len(m["pts"]) < 5:
			self._say("外周の点が 5 点より少ないので、楕円を決められません。", "#a06000")
			return
		try:
			ell = fit_ellipse(m["pts"])
			# 中心は打った点の平面の上のまま（骨の面へは、輪・線維の端を描くときに楕円の法線の向きに落とす）
		except Exception as e:
			self._say(f"楕円を当てはめられません: {e}", "red")
			return
		b, end = m["b"], m["end"]
		same = False
		try:
			old = (b.get(end) or {}).get("ellipse")
			if old and (b.get(end) or {}).get("bone_name") == self.names[m["si"]]:
				same = (float(np.linalg.norm(np.asarray(old["center"], float) - np.asarray(ell["center"], float))) < 0.3
				        and abs(float(old["ra"]) - float(ell["ra"])) < 0.3 and abs(float(old["rb"]) - float(ell["rb"])) < 0.3)
		except Exception:
			same = False
		b[end] = {"bone_name": self.names[m["si"]], "bone_index": self.idx_of[m["si"]],
		          "outline": [list(map(float, p)) for p in m["pts"]], "ellipse": ell, "ell_v": ELL_V}
		cL, cS = self._caliper_vals()
		if cL is not None or cS is not None:
			b[end]["caliper"] = [cL, cS]
		self._save()
		self._ol_cancel(quiet=True)
		self._invalidate()
		try:
			self._mv_refresh_targets()
		except Exception:
			pass
		self._say(f"「{b.get('name', '')}」の{'1' if end == 'a' else '2'}点目の面を決めました（{self.names[self._si_of_name(b[end]['bone_name'])]}・"
		          f"長径 {2 * ell['ra']:.1f} × 短径 {2 * ell['rb']:.1f} mm）。"
		          + ("※ 前の面とほぼ同じ楕円です（点が前とほぼ同じため。描き直すなら「面を描く」のあと、前の点は読み込まずに打ってください）。"
		             if same else "")
		          + "両方の面ができたら「▶ 計算する」。", "#a06000" if same else "#1b5e20")

	def _ol_cancel(self, quiet=False):
		self.st["mode"] = None
		self.st["replace"] = None
		self.st["sel"] = None
		self._remove_observers()
		try:
			self._old_ring_vis()            # ①で隠した前の輪を戻す
		except Exception:
			pass
		try:
			self._ol_dims()
		except Exception:
			pass
		try:
			self._ol_refresh()
		except Exception:
			pass

	# VTK のコールバックでは Tk を呼ばない（呼ぶとアプリごと落ちる。段階1の靭帯と同じ理由）。
	# クリックの位置を積むだけにして、Tk の after で処理する。
	def _install_observers(self):
		if self.plotter is None or self.st["obs"]:
			return
		try:
			it = self.plotter.iren.interactor
			for ev, fn in (("RightButtonPressEvent", self._on_press), ("RightButtonReleaseEvent", self._on_release),
			               ("LeftButtonPressEvent", self._on_lpress), ("LeftButtonReleaseEvent", self._on_lrelease),
			               ("KeyPressEvent", self._on_key)):
				oid = it.AddObserver(ev, fn, 10.0)
				try:
					it.GetCommand(oid).SetPassiveObserver(True)
				except Exception:
					pass
				self.st["obs"].append(oid)
		except Exception as e:
			print(f"[靭帯の束] 右クリックの登録に失敗: {e}")

	def _remove_observers(self):
		try:
			for oid in self.st["obs"]:
				self.plotter.iren.interactor.RemoveObserver(oid)
		except Exception:
			pass
		self.st["obs"] = []
		self.st["clicks"] = []

	def _pos(self, obj):
		try:
			return tuple(self.plotter.iren.interactor.GetEventPosition())
		except Exception:
			return tuple(obj.GetEventPosition())

	def _on_press(self, obj, ev):
		self.st["press"] = self._pos(obj) if self.st.get("mode") else None

	def _on_release(self, obj, ev):
		press, self.st["press"] = self.st["press"], None
		if press is None or not self.st.get("mode"):
			return
		x, y = self._pos(obj)
		if abs(x - press[0]) > 4 or abs(y - press[1]) > 4:
			return
		self.st["clicks"].append(("R", int(x), int(y)))

	def _on_key(self, obj, ev):
		"""3D の画面で Delete キー → 選んだ点を消す（VTK の中では積むだけ）。"""
		if not self.st.get("mode"):
			return
		try:
			key = str(self.plotter.iren.interactor.GetKeySym())
		except Exception:
			return
		if key in ("Delete", "BackSpace"):
			self.st["clicks"].append(("D", 0, 0))

	def _on_lpress(self, obj, ev):
		self.st["lpress"] = self._pos(obj) if self.st.get("mode") else None

	def _on_lrelease(self, obj, ev):
		"""左クリック（ドラッグで回したときは除く）→ 打った点を選ぶ。"""
		press, self.st["lpress"] = self.st.get("lpress"), None
		if press is None or not self.st.get("mode"):
			return
		x, y = self._pos(obj)
		if abs(x - press[0]) > 4 or abs(y - press[1]) > 4:
			return
		self.st["clicks"].append(("L", int(x), int(y)))

	def _start_poll(self):
		self._stop_poll()
		self.st["poll"] = self.app.after(30, self._poll)

	def _stop_poll(self):
		if self.st.get("poll") is not None:
			try:
				self.app.after_cancel(self.st["poll"])
			except Exception:
				pass
		self.st["poll"] = None

	def _poll(self):
		self.st["poll"] = None
		if self.win is None:
			return
		while self.st["clicks"]:
			btn, x, y = self.st["clicks"].pop(0)
			try:
				if btn == "L":
					self._select_near(x, y)
				elif btn == "D":
					if self.st.get("sel") is not None:
						self._ol_delete()
				else:
					self._pick(x, y)
			except Exception as e:
				print(f"[靭帯の束] ピックに失敗: {e}")
		try:
			self.st["poll"] = self.app.after(30, self._poll)
		except Exception:
			self.st["poll"] = None

	def _pick(self, x, y):
		m = self.st.get("mode")
		if not m or self.plotter is None:
			return
		vtk = self._vtk()
		picker = vtk.vtkCellPicker()
		picker.SetTolerance(0.005)
		picker.SetPickFromList(True)
		picker.InitializePickList()
		amap = {}
		for nm, act in self.plotter.renderer.actors.items():
			parts = str(nm).split("_")
			if parts[0] == "bone" and len(parts) >= 2 and parts[1].isdigit() and act.GetVisibility():
				picker.AddPickList(act)
				amap[id(act)] = int(parts[1])
		picker.Pick(x, y, 0, self.plotter.renderer)
		if picker.GetCellId() < 0:
			self._say("骨に当たりませんでした。骨の表面を右クリックしてください。", "#a06000")
			return
		hit = picker.GetActor()
		si = amap.get(id(hit))
		if si is None:
			return
		world = np.array(picker.GetPickPosition(), dtype=float)
		M = np.eye(4)
		um = hit.GetUserMatrix()
		if um is not None:
			M = np.array([[um.GetElement(r, c) for c in range(4)] for r in range(4)])
		local = (np.linalg.inv(M @ self.poses[si][0]) @ np.append(world, 1.0))[:3]
		self.add_point(si, local)

	SELECT_PX = 12          # 左クリックで点を選ぶときの、画面の上での近さ（px）

	def _select_near(self, x, y):
		"""画面の (x, y) にいちばん近い、打った点を選ぶ（SELECT_PX 以内）。返り値: 選んだ番号 or None。"""
		m = self.st.get("mode")
		if not m or m["si"] is None or not m["pts"] or self.plotter is None:
			return None
		ren = self.plotter.renderer
		T = self.poses[m["si"]][max(0, min(int(self.fi), self.n_view - 1)) if self.n_view else 0]
		best, bk = None, None
		for k, p in enumerate(m["pts"]):
			w = T[:3, :3] @ np.asarray(p, float) + T[:3, 3]
			ren.SetWorldPoint(float(w[0]), float(w[1]), float(w[2]), 1.0)
			ren.WorldToDisplay()
			dx, dy, _dz = ren.GetDisplayPoint()
			d = float(np.hypot(dx - x, dy - y))
			if best is None or d < best:
				best, bk = d, k
		if best is None or best > self.SELECT_PX:
			return None
		self._ol_set_sel(bk)
		p = m["pts"][bk]
		self._say(f"{bk + 1} 番目の点を選びました（{p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}）。「選んだ点を消す」「選んだ点を打ち直す」が使えます。", "blue")
		return bk

	def add_point(self, si, local):
		"""外周の点を足す（テストからも呼ぶ）。最初の点の骨と違う骨なら断る。"""
		m = self.st.get("mode")
		if not m:
			return False
		if m["si"] is not None and si != m["si"]:
			self._say(f"同じ骨（{self.names[m['si']]}）の上を右クリックしてください（いまは {self.names[si]}）。", "#a06000")
			return False
		m["si"] = si
		rp = self.st.get("replace")
		if rp is not None and 0 <= rp < len(m["pts"]):
			m["pts"][rp] = np.asarray(local, float)
			self.st["replace"] = None
			self.st["sel"] = rp
		else:
			m["pts"].append(np.asarray(local, float))
		self._ol_refresh()
		return True


class _LabelAt:
	"""世界座標の文字を、骨と一緒に動かすための小さな入れ物（SetUserMatrix の代わり）。"""

	def __init__(self, lab, p):
		self.lab = lab
		self.p = np.asarray(p, float)

	def SetUserMatrix(self, m):
		T = np.array([[m.GetElement(r, c) for c in range(4)] for r in range(4)])
		q = T[:3, :3] @ self.p + T[:3, 3]
		self.lab.GetPositionCoordinate().SetValue(float(q[0]), float(q[1]), float(q[2]))

	def __getattr__(self, k):
		return getattr(self.lab, k)


def _progress(app, title, parent=None):
	"""計算の進み具合の小さな窓（段階1とは別に持つ）。返り値: (update(割合, 文), close(), cancelled())。"""
	import tkinter as tk
	from tkinter import ttk
	win = tk.Toplevel(parent if parent is not None else app)
	win.title(title)
	try:
		win.attributes("-topmost", True)
	except Exception:
		pass
	win.geometry(f"520x150+{app.winfo_rootx() + 140}+{app.winfo_rooty() + 140}")
	ttk.Label(win, text=title, font=("", 10, "bold")).pack(padx=12, pady=(10, 4), anchor="w")
	msg = tk.StringVar(win, value="準備しています…")
	ttk.Label(win, textvariable=msg).pack(padx=12, anchor="w")
	bar = ttk.Progressbar(win, mode="determinate", maximum=1000, length=490)
	bar.pack(padx=12, pady=6)
	flag = {"cancel": False, "closed": False}
	ttk.Button(win, text="キャンセル", command=lambda: flag.update(cancel=True)).pack(pady=(0, 8))
	win.protocol("WM_DELETE_WINDOW", lambda: flag.update(cancel=True))

	def _upd(frac, text=None):
		if flag["closed"]:
			return
		try:
			bar["value"] = int(1000 * max(0.0, min(1.0, float(frac))))
			if text is not None:
				msg.set(str(text))
			win.update()
		except Exception:
			pass

	def _close():
		flag["closed"] = True
		try:
			win.destroy()
		except Exception:
			pass
	return _upd, _close, (lambda: bool(flag["cancel"]))


def attach(app, scene, bones_full, scene_bones, s0, store_key, base_dir):
	"""本体から呼ぶ口。scene に「靭帯の束」のボタン・描画・グラフの口を付ける。返り値は Stage2。
	store_key: 束の定義を保存するファイルの名前（骨リストを共有していればその共有の番号）。
	base_dir: FRS-SIMULATOR.py のフォルダ（cache/ankle_lig_bundles/ に保存する）。"""
	ctx = Stage2(app, scene, bones_full, scene_bones, s0, store_key, base_dir)
	scene.setdefault("panel_buttons", []).append(("靭帯の束（段階2）…", ctx.open_window))
	scene["_stage2_make"] = ctx.make
	scene["_lig_bundles"] = ctx
	return ctx
