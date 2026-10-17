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

VERSION = 1
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


def fiber_ends(bones, bone_idx, ell, uv):
	"""楕円の中の位置 uv (n,2) → 骨の面の上の点（モデル座標）。"""
	c = np.asarray(ell["center"], float)
	u = np.asarray(ell["u"], float)
	v = np.asarray(ell["v"], float)
	cpp, ns, _ = bones.closest(bone_idx, c[None])
	nrm = ns[0]
	P = cpp[0] + uv[:, :1] * float(ell["ra"]) * u + uv[:, 1:] * float(ell["rb"]) * v + 2.0 * nrm
	return bones.closest(bone_idx, P)[0]


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
		self.st = {"mode": None, "clicks": [], "press": None, "poll": None, "obs": [], "replace": None}
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

	def _save(self):
		try:
			self.store.parent.mkdir(parents=True, exist_ok=True)
			tmp = self.store.with_name(self.store.stem + ".tmp.json")
			tmp.write_text(json.dumps(self.data, ensure_ascii=False, indent=1), encoding="utf-8")
			os.replace(str(tmp), str(self.store))
		except Exception as e:
			print(f"[靭帯の束] 保存に失敗: {e}")

	def params(self):
		p = {"rest_scale": 1.0, "gravity": 0.02, "iters": 12, "inner": 6, "bend": 0.05, "stride": 3, "view": "both"}
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
		return {"a": ia, "b": ib, "pa": pa, "pb": best[1], "rf": float(b.get("fiber_radius", 0.4)), "_sia": sia}

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
			                     "r": float(b.get("fiber_radius", 0.4))}, sort_keys=True).encode())
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
					act = self._line_actor(self._ellipse_poly(e["ellipse"]), col, 3.0)
					act.SetVisibility(vis)
					a[f"e{end}_act"] = act
					a[f"e{end}_si"] = si
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
				tube.SetRadius(max(0.05, float(b.get("fiber_radius", 0.4))))
				tube.SetNumberOfSides(8)
				mp = vtk.vtkPolyDataMapper()
				mp.SetInputConnection(tube.GetOutputPort())
				fact = vtk.vtkActor()
				fact.SetMapper(mp)
				fact.PickableOff()
				fact.GetProperty().SetColor(*self._rgb(col))
				fact.SetVisibility(vis and view in ("fibers", "both"))
				self.plotter.renderer.AddActor(fact)
				# 皮: 経路に沿って楕円の輪をつなぐ
				M = 24
				spd = vtk.vtkPolyData()
				spts = vtk.vtkPoints()
				spts.SetNumberOfPoints(K * M)
				polys = vtk.vtkCellArray()
				for k in range(K - 1):
					for m in range(M):
						q = vtk.vtkQuad()
						q.GetPointIds().SetId(0, k * M + m)
						q.GetPointIds().SetId(1, k * M + (m + 1) % M)
						q.GetPointIds().SetId(2, (k + 1) * M + (m + 1) % M)
						q.GetPointIds().SetId(3, (k + 1) * M + m)
						polys.InsertNextCell(q)
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
				sp.SetOpacity(0.55 if view == "both" else 0.9)
				sp.SetInterpolationToPhong()
				sact.SetVisibility(vis and view in ("skin", "both"))
				self.plotter.renderer.AddActor(sact)
				a.update(fib_act=fact, fib_pd=pd, fib_pts=pts, skin_act=sact, skin_pd=spd, skin_pts=spts,
				         F=Fn, K=K, M=M, r=r, rf=float(b.get("fiber_radius", 0.4)))
			self.actors[b["id"]] = a
		self.update(self.fi, render=True)

	def _skin_points(self, W, rf, M):
		"""線維の点 W (F,K,3) → 皮の点 (K*M,3)。各所で楕円（向きはなめらかにつなぐ）。"""
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
		return out.reshape(-1, 3)

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
						act.SetUserMatrix(self._mat(self.poses[a[f"e{end}_si"]][fi]))
				r = a.get("r")
				if r is None:
					continue
				T = self.poses[r["sia"]][fi]
				L = self._local_at(r, fi)
				W = L @ T[:3, :3].T + T[:3, 3]
				from vtk.util.numpy_support import vtk_to_numpy
				if a["fib_act"].GetVisibility():
					vtk_to_numpy(a["fib_pts"].GetData())[:] = W.reshape(-1, 3)
					a["fib_pts"].Modified()
					a["fib_pd"].Modified()
				if a["skin_act"].GetVisibility():
					vtk_to_numpy(a["skin_pts"].GetData())[:] = self._skin_points(W, a["rf"], a["M"])
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
		lbl = self.w.get("status")
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

		ol = ttk.LabelFrame(win, text="付着部の面を描く（骨の上で外周を右クリック。5 点以上。右ドラッグのズームは従来どおり）")
		ol.pack(fill="x", padx=8, pady=4)
		self.w["ol_lbl"] = ttk.Label(ol, text="（束の行の「面を描く」を押すと始まります）", foreground="gray")
		self.w["ol_lbl"].pack(anchor="w", padx=6)
		olr = ttk.Frame(ol)
		olr.pack(fill="x", padx=6, pady=4)
		self.w["ol_list"] = tk.Listbox(olr, height=4, width=46, exportselection=False)
		self.w["ol_list"].pack(side="left")
		bf = ttk.Frame(olr)
		bf.pack(side="left", padx=8)
		ttk.Button(bf, text="最後の点を取り消す", command=self._ol_undo).grid(row=0, column=0, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="選んだ点を打ち直す", command=self._ol_replace).grid(row=0, column=1, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="選んだ点を消す", command=self._ol_delete).grid(row=1, column=0, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="すべて消す", command=self._ol_clear).grid(row=1, column=1, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="✅ 楕円を当てはめて決める", command=self._ol_commit).grid(row=2, column=0, sticky="ew", padx=2, pady=1)
		ttk.Button(bf, text="やめる", command=self._ol_cancel).grid(row=2, column=1, sticky="ew", padx=2, pady=1)

		bot = ttk.Frame(win)
		bot.pack(fill="x", padx=8, pady=(4, 8))
		pp = self.params()
		ttk.Label(bot, text="表示:").pack(side="left")
		self.w["view_var"] = tk.StringVar(win, value=str(pp.get("view", "both")))
		for v_, t_ in (("fibers", "線維"), ("skin", "楕円の皮"), ("both", "両方")):
			ttk.Radiobutton(bot, text=t_, value=v_, variable=self.w["view_var"], command=self._set_view).pack(side="left")
		ttk.Label(bot, text="　自然長 ×").pack(side="left")
		self.w["scale_var"] = tk.StringVar(win, value=f"{100.0 * float(pp.get('rest_scale', 1.0)):g}")
		ttk.Spinbox(bot, from_=80, to=120, increment=1, width=5, textvariable=self.w["scale_var"]).pack(side="left")
		ttk.Label(bot, text="%（表示の始まりのコマで張った長さに対して）").pack(side="left")
		self.w["grav_var"] = tk.BooleanVar(win, value=float(pp.get("gravity", 0.02)) > 0)
		ttk.Checkbutton(bot, text="たるみを下へ（弱い重さ）", variable=self.w["grav_var"]).pack(side="left", padx=8)
		ttk.Button(bot, text="▶ 計算する", command=self._compute_clicked).pack(side="right", padx=4)
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
		hdr = ("表示", "グラフ", "名前", "色", "線維の本数", "線維の太さ(半径) mm", "1点目の骨の面", "2点目の骨の面", "計算", "")
		for c, h in enumerate(hdr):
			ttk.Label(fr, text=h, font=("", 9, "bold")).grid(row=0, column=c, padx=4, sticky="w")
		for r_, b in enumerate(self.data["bundles"], start=1):
			vv = tk.BooleanVar(fr, value=bool(b.get("visible", True)))
			ttk.Checkbutton(fr, variable=vv, command=lambda b=b, v=vv: self._set(b, "visible", bool(v.get()), rebuild=True)).grid(row=r_, column=0)
			cv = tk.BooleanVar(fr, value=bool(b.get("chart", True)))
			ttk.Checkbutton(fr, variable=cv, command=lambda b=b, v=cv: self._set(b, "chart", bool(v.get()), notify=True)).grid(row=r_, column=1)
			nv = tk.StringVar(fr, value=str(b.get("name", "")))
			ttk.Entry(fr, textvariable=nv, width=14).grid(row=r_, column=2, sticky="w")
			nv.trace_add("write", lambda *_a, b=b, v=nv: self._set(b, "name", v.get(), notify=True))
			tk.Button(fr, text="    ", bg=b.get("color", "#FF8800"), command=lambda b=b: self._pick_color(b)).grid(row=r_, column=3)
			fv = tk.StringVar(fr, value=str(int(b.get("n_fibers", 10))))
			ttk.Spinbox(fr, from_=1, to=40, increment=1, width=5, textvariable=fv).grid(row=r_, column=4)
			fv.trace_add("write", lambda *_a, b=b, v=fv: self._set_num(b, "n_fibers", v.get(), int))
			rv = tk.StringVar(fr, value=f"{float(b.get('fiber_radius', 0.4)):g}")
			ttk.Spinbox(fr, from_=0.1, to=3.0, increment=0.05, width=5, textvariable=rv).grid(row=r_, column=5)
			rv.trace_add("write", lambda *_a, b=b, v=rv: self._set_num(b, "fiber_radius", v.get(), float))
			for c_, end in ((6, "a"), (7, "b")):
				e = b.get(end) or {}
				f_ = ttk.Frame(fr)
				f_.grid(row=r_, column=c_, sticky="w")
				txt = (f"{e.get('bone_name', '')} {2 * float(e['ellipse']['ra']):.1f}×{2 * float(e['ellipse']['rb']):.1f}mm"
				       if e.get("ellipse") else "まだ")
				ttk.Label(f_, text=txt, foreground=("black" if e.get("ellipse") else "#a06000")).pack(side="left")
				ttk.Button(f_, text="面を描く", command=lambda b=b, end=end: self._ol_start(b, end)).pack(side="left", padx=2)
			ttk.Label(fr, text=("済み" if self.results.get(b.get("id")) is not None else "要計算"),
			          foreground=("#1b5e20" if self.results.get(b.get("id")) is not None else "#a06000")).grid(row=r_, column=8)
			ttk.Button(fr, text="削除", width=6, command=lambda b=b: self._delete(b)).grid(row=r_, column=9)

	def _set(self, b, key, val, rebuild=False, notify=False):
		b[key] = val
		self._save()
		if rebuild:
			self._rebuild_actors()
		if notify:
			self._notify()

	def _set_num(self, b, key, val, typ):
		try:
			x = typ(val)
		except (TypeError, ValueError):
			return
		if x <= 0 or b.get(key) == x:
			return
		b[key] = x
		self._save()
		self._invalidate()

	def _invalidate(self):
		"""束の形・本数などが変わった → 計算し直しが要る（結果を外す）。"""
		self.results = {}
		self._rebuild_actors()
		self._notify()
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
		self.st["mode"] = {"b": b, "end": end, "si": None, "pts": []}
		# 前に描いた外周があれば、そこから直せるようにする
		e = b.get(end) or {}
		si = self._si_of_name(e.get("bone_name"))
		if e.get("outline") and si is not None:
			self.st["mode"]["si"] = si
			self.st["mode"]["pts"] = [np.asarray(p, float) for p in e["outline"]]
		self._install_observers()
		self._ol_refresh()
		self._say(f"「{b.get('name', '')}」の{'1' if end == 'a' else '2'}点目の骨の面: 付着部の外周を右クリックで 5 点以上打ってください。", "blue")

	def _ol_refresh(self):
		m = self.st.get("mode")
		lb = self.w.get("ol_list")
		if lb is not None:
			lb.delete(0, "end")
			if m:
				for k, p in enumerate(m["pts"]):
					lb.insert("end", f"{k + 1}: ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})")
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

	def _draw_pending(self):
		if self.plotter is None:
			return
		vtk = self._vtk()
		for act, _si in self.pend:
			try:
				self.plotter.renderer.RemoveActor(act)
			except Exception:
				pass
		self.pend = []
		m = self.st.get("mode")
		if m and m["si"] is not None:
			for p in m["pts"]:
				src = vtk.vtkSphereSource()
				src.SetCenter(*[float(x) for x in p])
				src.SetRadius(0.5)
				mp = vtk.vtkPolyDataMapper()
				mp.SetInputConnection(src.GetOutputPort())
				act = vtk.vtkActor()
				act.SetMapper(mp)
				act.PickableOff()
				act.GetProperty().SetColor(1.0, 0.83, 0.0)
				self.plotter.renderer.AddActor(act)
				self.pend.append((act, m["si"]))
			if len(m["pts"]) >= 5:
				try:
					ell = fit_ellipse(m["pts"])
					act = self._line_actor(self._ellipse_poly(ell), "#FFD400", 2.0)
					self.pend.append((act, m["si"]))
				except Exception:
					pass
		self.update(self.fi, render=True)

	def _ol_undo(self):
		m = self.st.get("mode")
		if m and m["pts"]:
			m["pts"].pop()
			if not m["pts"]:
				m["si"] = None
			self.st["replace"] = None
			self._ol_refresh()

	def _ol_delete(self):
		m = self.st.get("mode")
		lb = self.w.get("ol_list")
		if not m or lb is None or not lb.curselection():
			return
		del m["pts"][int(lb.curselection()[0])]
		if not m["pts"]:
			m["si"] = None
		self.st["replace"] = None
		self._ol_refresh()

	def _ol_replace(self):
		m = self.st.get("mode")
		lb = self.w.get("ol_list")
		if not m or lb is None or not lb.curselection():
			self._say("打ち直す点を一覧で選んでください。", "#a06000")
			return
		self.st["replace"] = int(lb.curselection()[0])
		self._ol_refresh()

	def _ol_clear(self):
		m = self.st.get("mode")
		if m:
			m["pts"] = []
			m["si"] = None
			self.st["replace"] = None
			self._ol_refresh()

	def _ol_commit(self):
		m = self.st.get("mode")
		if not m or len(m["pts"]) < 5:
			self._say("外周の点が 5 点より少ないので、楕円を決められません。", "#a06000")
			return
		try:
			ell = fit_ellipse(m["pts"])
			# 中心は骨の面の上へ
			bones = self._ensure_bones()
			cpp, _ns, _sd = bones.closest(self.idx_of[m["si"]], np.asarray(ell["center"], float)[None])
			ell["center"] = cpp[0].tolist()
		except Exception as e:
			self._say(f"楕円を当てはめられません: {e}", "red")
			return
		b, end = m["b"], m["end"]
		b[end] = {"bone_name": self.names[m["si"]], "bone_index": self.idx_of[m["si"]],
		          "outline": [list(map(float, p)) for p in m["pts"]], "ellipse": ell}
		self._save()
		self._ol_cancel(quiet=True)
		self._invalidate()
		self._say(f"「{b.get('name', '')}」の{'1' if end == 'a' else '2'}点目の面を決めました（{self.names[self._si_of_name(b[end]['bone_name'])]}・"
		          f"{2 * ell['ra']:.1f}×{2 * ell['rb']:.1f}mm）。両方の面ができたら「▶ 計算する」。", "#1b5e20")

	def _ol_cancel(self, quiet=False):
		self.st["mode"] = None
		self.st["replace"] = None
		self._remove_observers()
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
			for ev, fn in (("RightButtonPressEvent", self._on_press), ("RightButtonReleaseEvent", self._on_release)):
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
		self.st["clicks"].append((int(x), int(y)))

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
			x, y = self.st["clicks"].pop(0)
			try:
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
