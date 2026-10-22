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

VERSION = 4          # 2: 付着部の楕円を、楕円の面の法線の向きに骨の面へ落とす（2026-10-20。線維の端が変わる）
                     # 3: 束どうしの当たり（外形どうし）と、自然長の基準のコマ（2026-10-20）
                     # 4: 束どうしの当たりを中心線どうしの距離で（跳ばない。2026-10-21）
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


def _envelopes(X, rf):
	"""束 X (F,K,3) の外形: 各所の楕円の断面（皮と同じ作り方）。返り値: C, t, e1, e2 (K,3), r1, r2 (K,)。"""
	F, K, _ = X.shape
	C = X.mean(axis=0)
	kk = np.arange(K)
	t = C[np.minimum(kk + 1, K - 1)] - C[np.maximum(kk - 1, 0)]
	t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
	Q = X - C[None]
	Q = Q - np.einsum("fki,ki->fk", Q, t)[:, :, None] * t[None]
	if F >= 2:
		_w, V = np.linalg.eigh(np.einsum("fki,fkj->kij", Q, Q))
		e1 = V[:, :, -1]
	else:
		e1 = np.cross(t, [1.0, 0.0, 0.0])
	e1 = e1 - np.einsum("ki,ki->k", e1, t)[:, None] * t
	bad = np.linalg.norm(e1, axis=1) < 1e-6
	if bad.any():
		e1[bad] = np.cross(t[bad], [0.0, 1.0, 0.0])
	e1 /= np.maximum(np.linalg.norm(e1, axis=1, keepdims=True), 1e-12)
	e2 = np.cross(t, e1)
	r1 = np.abs(np.einsum("fki,ki->fk", Q, e1)).max(axis=0) + rf
	r2 = np.abs(np.einsum("fki,ki->fk", Q, e2)).max(axis=0) + rf
	return C, t, e1, e2, r1, r2


def _closest_on_polyline(C, P):
	"""点 P (n,3) に、折れ線 C (K,3) の上でいちばん近い点。返り値: 点 (n,3)・区間の番号 i (n,)・区間の中の割合 u (n,)。"""
	A, B = C[:-1], C[1:]
	AB = B - A
	L2 = np.maximum((AB ** 2).sum(1), 1e-12)
	u = np.clip(np.einsum("nsi,si->ns", P[:, None, :] - A[None], AB) / L2[None], 0.0, 1.0)
	Q = A[None] + u[:, :, None] * AB[None]
	i = np.argmin(((P[:, None, :] - Q) ** 2).sum(2), axis=1)
	r = np.arange(len(P))
	return Q[r, i], i, u[r, i]


def _support(dirv, t, e1, e2, r1, r2):
	"""楕円の断面（e1・e2 の向きに半径 r1・r2）の、向き dirv（断面の面へ落とす）の半径。"""
	dp = dirv - np.einsum("ni,ni->n", dirv, t)[:, None] * t
	dp = dp / np.maximum(np.linalg.norm(dp, axis=1), 1e-9)[:, None]
	c1, c2 = np.einsum("ni,ni->n", dp, e1), np.einsum("ni,ni->n", dp, e2)
	return 1.0 / np.sqrt((c1 / r1) ** 2 + (c2 / r2) ** 2)


def _ramp(x, w):
	"""0 → 1 のなめらかな立ち上がり（x ≤ 0 で 0、x ≥ w で 1）。"""
	t = np.clip(np.asarray(x, float) / max(float(w), 1e-9), 0.0, 1.0)
	return t * t * (3.0 - 2.0 * t)


def _bundle_contact(Xs, rfs, end_frac=0.08, gain=0.5, margin=1.1, ramp=3.0):
	"""束どうしの当たり（その場で書き換え）: 中心線どうしの距離で離す（カプセル同士の当たり）。
	束 B の各断面の中心から束 A の中心線へのいちばん近い点を結ぶ向きで、両方の外形（皮と同じ楕円の断面）のその向きの半径の和 × margin
	より近ければ、その差の gain/2 ずつ離す（A の側は近い区間の両端に割合で分ける。A・B を入れ替えても行う）。
	両端の end_frac の長さは除き、そこから ramp 区間かけてなめらかに効かせる（付着部は動かない・隣り合う付着部の近くは重なってよい）。
	以前（10/20）は外形の縁のいちばん深い点で断面ごと押していたので、選ばれる点がコマごとに入れ替わって外形が跳んだ。
	返り値: 近すぎた断面の数。"""
	nb = len(Xs)
	if nb < 2:
		return 0
	env = [_envelopes(X, rf) for X, rf in zip(Xs, rfs)]
	moves = [np.zeros((X.shape[1], 3)) for X in Xs]
	cnt = 0
	for a in range(nb):
		CA, tA, e1A, e2A, r1A, r2A = env[a]
		KA = len(CA)
		ka0 = max(1, int(round(end_frac * (KA - 1))))
		for b in range(nb):
			if b == a:
				continue
			CB, tB, e1B, e2B, r1B, r2B = env[b]
			KB = len(CB)
			kb0 = max(1, int(round(end_frac * (KB - 1))))
			kbs = np.arange(kb0, KB - kb0)
			if len(kbs) == 0:
				continue
			Pb = CB[kbs]
			Q, i, u = _closest_on_polyline(CA, Pb)
			ia0, ia1 = i, i + 1
			# 端の近くは、当たりをなめらかに弱める（急に入ったり切れたりすると跳ぶ）
			sa = ia0 + u
			wa = _ramp(sa - ka0, ramp) * _ramp((KA - 1 - ka0) - sa, ramp)
			wb = _ramp(kbs - kb0 + 1.0, ramp) * _ramp((KB - 1 - kb0) - kbs + 1.0, ramp)
			ok = wa > 0
			d = Pb - Q
			dist = np.linalg.norm(d, axis=1)
			dirv = d / np.maximum(dist, 1e-9)[:, None]
			w = u[:, None]
			tA_ = (1 - w) * tA[ia0] + w * tA[ia1]
			tA_ /= np.maximum(np.linalg.norm(tA_, axis=1, keepdims=True), 1e-9)
			e1A_ = (1 - w) * e1A[ia0] + w * e1A[ia1]
			e1A_ = e1A_ - np.einsum("ni,ni->n", e1A_, tA_)[:, None] * tA_
			e1A_ /= np.maximum(np.linalg.norm(e1A_, axis=1, keepdims=True), 1e-9)
			e2A_ = np.cross(tA_, e1A_)
			ra = _support(dirv, tA_, e1A_, e2A_, (1 - u) * r1A[ia0] + u * r1A[ia1], (1 - u) * r2A[ia0] + u * r2A[ia1])
			rb = _support(-dirv, tB[kbs], e1B[kbs], e2B[kbs], r1B[kbs], r2B[kbs])
			pen = (margin * (ra + rb) - dist) * wa * wb
			hit = ok & (pen > 0) & (dist > 1e-6)
			if not hit.any():
				continue
			mv = (0.5 * gain * pen * hit)[:, None] * dirv          # B は離れる向き、A は逆向き（半分ずつ）
			moves[b][kbs] += mv
			np.add.at(moves[a], ia0, -mv * (1 - u)[:, None])
			np.add.at(moves[a], ia1, -mv * u[:, None])
			cnt += int(hit.sum())
	for X, m in zip(Xs, moves):
		ms = m.copy()
		ms[1:-1] = 0.25 * m[:-2] + 0.5 * m[1:-1] + 0.25 * m[2:]         # 長さの向きに少しならす（折れを作らない）
		X[:, 1:-1] += ms[None, 1:-1]
	return cnt


STUMP_BAND = (1.1, 1.35)   # 付着部の楕円に対する、基準の膜の縁（この輪の骨の高さを使う）
STUMP_RAMP = (1.0, 1.1)    # 楕円の中（〜1.0）は全部、1.0〜1.1 でなめらかに弱める
STUMP_HMIN = 0.3           # mm。膜よりこれだけ高い所から下げる
STUMP_ZLIM = 8.0           # mm。楕円の面からこの範囲の点だけ（反対側の骨の面を拾わない）


def stump_membrane(Pm, ell, n, g=0.4, iters=600):
	"""付着部の楕円のまわりの骨の高さから、楕円の内側へなめらかに張った膜（ラプラス方程式）。
	Pm: 骨のメッシュの点（モデル座標）、ell: 楕円、n: 楕円の面の外向きの法線。返り値: 高さ f(x, y) を返す関数（x・y は楕円の u・v の向き）
	か None（縁の点が足りない）。"""
	from scipy.interpolate import RegularGridInterpolator
	c = np.asarray(ell["center"], float); u = np.asarray(ell["u"], float); v = np.asarray(ell["v"], float)
	ra, rb = float(ell["ra"]), float(ell["rb"])
	d = Pm - c
	x, y, z = d @ u, d @ v, d @ n
	sr = np.sqrt((x / ra) ** 2 + (y / rb) ** 2)
	bnd = (sr >= STUMP_BAND[0]) & (sr <= STUMP_BAND[1]) & (np.abs(z) < STUMP_ZLIM)
	if int(bnd.sum()) < 12:
		return None
	xm, ym = ra * STUMP_BAND[1] + g, rb * STUMP_BAND[1] + g
	gx = np.arange(-xm, xm + g * 0.5, g)
	gy = np.arange(-ym, ym + g * 0.5, g)
	GX, GY = np.meshgrid(gx, gy, indexing="ij")
	GS = np.sqrt((GX / ra) ** 2 + (GY / rb) ** 2)
	valid = GS <= STUMP_BAND[1] + 0.05
	# 縁の格子: そこに落ちる点の高さの中央値
	ix = np.clip(np.round((x[bnd] + xm) / g).astype(int), 0, len(gx) - 1)
	iy = np.clip(np.round((y[bnd] + ym) / g).astype(int), 0, len(gy) - 1)
	Z = np.zeros(GX.shape)
	fixed = np.zeros(GX.shape, bool)
	key = ix * len(gy) + iy
	order = np.argsort(key)
	ks, zs = key[order], z[bnd][order]
	starts = np.r_[0, np.where(np.diff(ks) > 0)[0] + 1]
	for a_, b_ in zip(starts, np.r_[starts[1:], len(ks)]):
		k_ = ks[a_]
		Z[k_ // len(gy), k_ % len(gy)] = float(np.median(zs[a_:b_]))
		fixed[k_ // len(gy), k_ % len(gy)] = True
	fixed &= valid
	if int(fixed.sum()) < 8:
		return None
	unk = valid & ~fixed
	Z[unk] = float(Z[fixed].mean())
	vm = valid.astype(float)
	for _ in range(int(iters)):
		acc = np.zeros_like(Z)
		cnt = np.zeros_like(Z)
		for sh, ax in ((1, 0), (-1, 0), (1, 1), (-1, 1)):
			acc += np.roll(Z * vm, sh, axis=ax)
			cnt += np.roll(vm, sh, axis=ax)
		Znew = np.where(cnt > 0, acc / np.maximum(cnt, 1e-9), Z)
		Z = np.where(unk, Znew, Z)
	itp = RegularGridInterpolator((gx, gy), Z, bounds_error=False, fill_value=None)
	return lambda xx, yy: itp(np.c_[xx, yy])


def stump_lower(Pm, ell, n, f, offset=0.0, hmin=STUMP_HMIN):
	"""点 Pm（モデル座標）のうち、楕円の中で膜 f + offset より hmin 以上高い所を、膜 + offset まで下げる（縁 1.0〜1.1 はなめらかに）。
	返り値: (新しい点, 下げた量の最大 mm, 下げた点の数)。低い所は動かさない。"""
	c = np.asarray(ell["center"], float); u = np.asarray(ell["u"], float); v = np.asarray(ell["v"], float)
	ra, rb = float(ell["ra"]), float(ell["rb"])
	Pm = np.asarray(Pm, float)
	d = Pm - c
	x, y, z = d @ u, d @ v, d @ n
	sr = np.sqrt((x / ra) ** 2 + (y / rb) ** 2)
	cand = (sr <= STUMP_RAMP[1]) & (np.abs(z) < STUMP_ZLIM + 6.0)
	out = Pm.copy()
	if not cand.any():
		return out, 0.0, 0
	ii = np.where(cand)[0]
	h = z[ii] - (np.asarray(f(x[ii], y[ii]), float) + float(offset))
	t_ = np.clip((STUMP_RAMP[1] - sr[ii]) / max(STUMP_RAMP[1] - STUMP_RAMP[0], 1e-9), 0.0, 1.0)
	w = t_ * t_ * (3.0 - 2.0 * t_)
	low = (h > hmin) & (w > 0)
	if not low.any():
		return out, 0.0, 0
	dz = w[low] * h[low]
	out[ii[low]] = Pm[ii[low]] - dz[:, None] * n[None]
	return out, float(dz.max()), int(low.sum())


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
	contact = bool(params.get("bundle_contact", True))
	nb = len(bundles)

	def world(i, P, f):
		T = Ts[int(i)][f]
		return np.asarray(P, float) @ T[:3, :3].T + T[:3, 3]

	# --- 自然長: 基準のコマ（指定が無ければ表示の始まりのコマ）で、骨を避けて張った長さ × 係数。形は表示の始まりから ---
	Xs, seg0, Ks, rfs = [], [], [], []
	f0 = frames[0]
	fref = params.get("rest_frame")
	fref = int(fref) if fref is not None and 0 <= int(fref) < N else f0
	for b in bundles:
		A, B = world(b["a"], b["pa"], f0), world(b["b"], b["pb"], f0)
		F = len(A)
		Kt = 33
		Xt = _taut_paths(bones, Ts, f0, A, B, Kt, np.full(F, float(b["rf"])), maxit=3000)
		if fref != f0:
			Ar, Br = world(b["a"], b["pa"], fref), world(b["b"], b["pb"], fref)
			Xr = _taut_paths(bones, Ts, fref, Ar, Br, Kt, np.full(F, float(b["rf"])), maxit=3000)
			Lt = np.linalg.norm(np.diff(Xr, axis=1), axis=2).sum(axis=1)
		else:
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
			# 束どうし（外形どうし。線維の点どうしだけだと、束の外形が重なった。2026-10-20）
			if contact:
				_bundle_contact(Xs, rfs)
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
		# 仕上げ: 束どうしの当たりだけを数回（線維は少し伸びてよける。伸びない糸のままだと、張った束どうしは長さの制約に
		# まっすぐ戻されてよけられなかった）。そのあと骨の外へ
		if contact:
			for _k in range(6):
				if _bundle_contact(Xs, rfs) == 0:
					break
			P = np.concatenate([X.reshape(-1, 3) for X in Xs], axis=0)
			Q = P[~is_end].copy()
			bones.push_out(Q, Ts_f, rad[~is_end])
			P[~is_end] = Q
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
		self._flat_fits = {}         # (骨, メッシュの種類) → [(楕円, 法線, 膜)]（切り残しを消した付着部）
		self._flat_info = {}         # (束の名前, 端) → (下げた量の最大 mm, 下げた点の数)
		try:
			self._flatten_scene_meshes()
		except Exception as e:
			traceback.print_exc()
			print(f"[靭帯の束] 付着部の切り残しを消せません（そのまま表示します）: {e}")
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
		h.update(json.dumps({"rest_frame": self._rest_frame(), "contact": bool(pp.get("bundle_contact", True))},
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
			res = simulate(self.app, self.bones_full, rts, self.s0, dict(self.params(), rest_frame=self._rest_frame()),
			               progress=upd, cancelled=cancelled,
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
				M = int(self.SKIN_M)
				Ks = (K - 1) * int(self.SKIN_UP) + 1          # 皮の輪の数（線維の点の間に補間）
				spd = vtk.vtkPolyData()
				spts = vtk.vtkPoints()
				NS = len(self.SKIN_CAP_SCALES)
				spts.SetNumberOfPoints(Ks * M + 2 * NS * M)
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
				for k in range(Ks - 1):
					_band(k * M, (k + 1) * M)
				# 両端のふた: 付着部の楕円を同心の輪に分けて骨の面へ落としたもの（骨の面に沿う。平らな板だと、
				# 凹んだ付着部（顆間窩の壁など）で板が骨から浮いて見えた）
				for kk, base in ((0, Ks * M), (Ks - 1, Ks * M + NS * M)):
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
				         F=Fn, K=K, Ks=Ks, M=M, r=r, rf=self._rf(b))
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
	SKIN_CLR = 0.25        # mm。見えている骨の面から皮をこれだけ外へ（骨が皮を突き抜けて見えないように。2026-10-20）
	SKIN_SINK_LEN = 1.5    # mm。付く骨の端から、皮が骨の中へ入るのを許す長さ（以前は長さの 20%＝こぶが突き抜けた）
	FIB_SINK_LEN = 3.0     # mm。線維で同じ
	SKIN_UP = 2            # 皮の輪を、線維の点の間に補間して何倍にするか（骨の細かい凹凸を点の間から出さない）
	SKIN_M = 24            # 皮の輪のまわりの点の数
	# 入力した断面の皮（2026-10-21 午後。_skin_shape）
	SKIN_N_SIG = 0.10      # 輪の面の向き = 中心の線の接線を、輪の数のこの割合でならしたもの（輪どうしが交差しない）
	SKIN_C_SMOOTH = 0.25   # 中心の線（線維の重心）のうねりを取る強さ（輪の数の割合。両端は動かさない。10/22 に 0.06 から: 0.06 では波が残った）
	SKIN_LIFT = (0.5, 2.0)  # 輪の中心を骨から離す: その向きの輪の半径のこの割合（mm の上限）＋ 許す距離
	SKIN_LIFT_IT = 3
	SKIN_LIFT_END = 2.0    # mm。両端からこの長さは中心の線を動かさない（付着部につながったまま）
	SKIN_MOVE_SIG = 0.15   # 中心の線を動かす量を、輪の数のこの割合でならす（10/22 に 0.04 から: 短い幅だと S 字の波になった）
	SKIN_CLIP_EXTRA = 0.15  # mm。骨に当たる側を縮めるとき、許す距離にさらに足す
	SKIN_CLIP_MIN = 0.3    # mm。縮めても、輪の中心からこれより内へは入れない
	SKIN_CLIP_SIG = (1.5, 0.04)  # 縮める量のならし（輪のまわり: 点の数、長さ: 輪の数の割合）
	SKIN_CLIP_END_POW = 2.0  # 付着部の近くは縮めるのを (1 − 端の輪へ寄せる重み)^これ で弱める（付着部の広がりを残す）
	SKIN_SEP_IT = 0        # 皮どうしを中心線ごと離す回数（10/22 に 2 → 0: AM束と PL束の間にすき間と S 字の波ができた。重なりは平らにするだけ）
	SKIN_SEP_GAIN = 0.5
	SKIN_MUT_FRAC = 0.5    # 皮どうしの重なりは、それぞれ重なりのこの割合だけ縮める（押し合って平らに）
	SKIN_CONTAIN = False   # 入力した断面の皮を、線維の広がりまで広げるか（10/22 に False: PCL の厚みが 5 → 7.5mm になった）
	SKIN_ORIENT_SIG = 0.12  # 骨に挟まれる所で短径をすき間の向きへそろえるとき、向きを長さ方向にならす幅（輪の数の割合）

	def _shown(self, si):
		"""骨 si でいま見えているモデル: "alt"（表示変更用モデル）か "calc"（計算に使ったモデル・関節面）。"""
		if self.plotter is None:
			return "calc"
		try:
			act = self._actors().get(f"bone_{si}_alt")
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

	# ---------------------------------------------------------------- 付着部の切り残しを消す（表示だけ）
	def _stump_on(self, b, end):
		return bool((b.get("stump") or {}).get(end, False))

	def _flatten_scene_meshes(self):
		"""印のある付着部の、表示に使う骨のメッシュの写しで、切り残し（膜より高い所）を消す。可視化を開く前（エンジンが骨を作る前）に呼ぶ。"""
		todo = [(b, end) for b in self.data.get("bundles", []) for end in ("a", "b")
		        if self._stump_on(b, end) and (b.get(end) or {}).get("ellipse") and self._si_of_name(b[end].get("bone_name")) is not None]
		if not todo:
			return
		bones = self._ensure_bones()
		for b, end in todo:
			e = b[end]
			si = self._si_of_name(e["bone_name"])
			n = ell_normal(bones, self.idx_of[si], e["ellipse"])
			for key in ("mesh", "alt_mesh", "joint_mesh"):
				m = self.sb[si].get(key)
				if m is None:
					continue
				try:
					Pm = np.asarray(m.points, float)
					f = stump_membrane(Pm, e["ellipse"], n)
					if f is None:
						continue
					newP, dmax, cnt = stump_lower(Pm, e["ellipse"], n, f)
					if cnt:
						m2 = m.copy(deep=True)
						m2.points = newP.astype(np.asarray(m.points).dtype)
						self.sb[si][key] = m2
					self._flat_fits.setdefault((si, key), []).append((e["ellipse"], n, f))
					if key == "mesh" or (b.get("name"), end) not in self._flat_info:
						self._flat_info[(b.get("name"), end)] = (dmax, cnt)
					print(f"[靭帯の束] 付着部の切り残しを消した: {b.get('name', '')}・{e['bone_name']}側（{key}）最大 {dmax:.1f} mm・{cnt} 点")
				except Exception as ex:
					print(f"[靭帯の束] 切り残しを消せません（{b.get('name', '')}・{key}）: {ex}")

	def _flat_map(self, si, P, offset=0.0):
		"""モデル座標の点 P を、切り残しを消した面（計算のモデルの膜）に合わせて下げる（印の無い骨はそのまま）。"""
		fits = self._flat_fits.get((si, "mesh")) or self._flat_fits.get((si, "alt_mesh")) or []
		P = np.asarray(P, float)
		for ell, n, f in fits:
			P = stump_lower(P, ell, n, f, offset=offset, hmin=0.02)[0]
		return P

	# ---------------------------------------------------------------- 見えている骨から皮・線維を押し出す（表示だけ）
	def _skin_push_on(self):
		return bool(self.params().get("skin_push", True))

	def _disp_ob(self, si):
		"""いま見えている骨 si の形（モデル座標）と、いちばん近い点を探す仕組み。見えていなければ None。
		表示変更用モデルが見えていればそれ、そうでなければ画面に出ている骨（関節面だけの表示でも骨全体）。"""
		if self.plotter is None:
			return None
		acts = self._actors()

		def _vis(nm):
			a_ = acts.get(nm)
			try:
				return a_ is not None and bool(a_.GetVisibility()) and float(a_.GetProperty().GetOpacity()) > 0.02
			except Exception:
				return False
		if _vis(f"bone_{si}_alt"):
			kind = "alt"
		elif _vis(f"bone_{si}") or _vis(f"bone_{si}_joint"):
			kind = "calc"
		else:
			return None
		cache = self.__dict__.setdefault("_disp_obs", {})
		key = (int(si), kind)
		if key not in cache:
			cache[key] = None
			try:
				import pyvista as pv
				import open3d as o3d
				act = acts.get(f"bone_{si}_alt" if kind == "alt" else f"bone_{si}")
				tri = pv.wrap(act.GetMapper().GetInput()).extract_surface().triangulate()
				T0 = self.poses[si][0]                      # 表示の骨 = UserMatrix × 最初のコマの姿勢 × モデル座標
				Pv = (np.asarray(tri.points, float) - T0[:3, 3]) @ T0[:3, :3]
				F = np.asarray(tri.faces).reshape(-1, 4)[:, 1:].astype(np.int32)
				mm = pv.PolyData(Pv, np.c_[np.full(len(F), 3), F].ravel())
				nm = mm.compute_normals(cell_normals=True, point_normals=True, consistent_normals=True,
				                        auto_orient_normals=True, split_vertices=False)
				F2 = np.asarray(nm.faces).reshape(-1, 4)[:, 1:].astype(np.int32)
				sc = o3d.t.geometry.RaycastingScene()
				sc.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(Pv.astype(np.float32)), o3d.core.Tensor(F2)))
				cache[key] = dict(sc=sc, F=F2, tn=np.asarray(nm.cell_data["Normals"], float),
				                  vn=np.asarray(nm.point_data["Normals"], float), lo=Pv.min(axis=0), hi=Pv.max(axis=0))
			except Exception as e:
				print(f"[靭帯の束] 見えている骨の形を使えません（{self.names[si] if si < len(self.names) else si}）: {e}")
		return cache[key]

	@staticmethod
	def _ob_closest(o, q):
		"""_Bones.closest と同じ（いちばん近い面の点, なめらかな外向きの法線, 符号つき距離）。"""
		import open3d as o3d
		q = np.asarray(q, dtype=np.float32).reshape(-1, 3)
		cp = o["sc"].compute_closest_points(o3d.core.Tensor(q))
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

	def _push_disp(self, P, c_of, fi, groups=None):
		"""世界座標の点 P (n,3) を、見えている骨 si の面から c_of(si) (n,) mm 以上外へ（その場で書き換え。nan は動かさない）。
		groups (n,) があれば、同じ組（皮の輪など）の点は、その組の骨の法線の平均の向きへ、骨の外へ出るまで持ち上げる
		（いちばん近い面へ出すと、骨のこぶに当たった点がこぶの横へ逃げて、点の間の面がこぶを貫いた。2026-10-20）。"""
		if groups is not None:
			return self._push_disp_dir(P, c_of, fi, np.asarray(groups, int))
		for si in range(len(self.poses)):
			o = self._disp_ob(si)
			if o is None:
				continue
			c = np.asarray(c_of(si), float)
			T = self.poses[si][fi]
			R, t = T[:3, :3], T[:3, 3]
			q = (P - t) @ R
			near = np.all((q > o["lo"] - 3.0) & (q < o["hi"] + 3.0), axis=1) & np.isfinite(c)
			if not near.any():
				continue
			cpp, ns, sd = self._ob_closest(o, q[near])
			cc = c[near]
			bad = sd < cc
			if bad.any():
				ii = np.where(near)[0][bad]
				P[ii] = (cpp[bad] + cc[bad, None] * ns[bad]) @ R.T + t
		return P

	def _push_disp_dir(self, P, c_of, fi, groups):
		import open3d as o3d
		ng = int(groups.max()) + 1 if len(groups) else 0
		self._psd = {}                                          # 骨 → 押し出した後の距離（おおよそ。遠い点は inf）
		for si in range(len(self.poses)):
			o = self._disp_ob(si)
			if o is None:
				continue
			c = np.asarray(c_of(si), float)
			T = self.poses[si][fi]
			R, t = T[:3, :3], T[:3, 3]
			q = (P - t) @ R
			near = np.all((q > o["lo"] - 3.0) & (q < o["hi"] + 3.0), axis=1) & np.isfinite(c)
			if not near.any():
				continue
			ii = np.where(near)[0]
			cpp, ns, sd = self._ob_closest(o, q[ii])
			cc = c[ii]
			bad = sd < cc
			sda = np.full(len(P), np.inf)
			sda[ii] = np.maximum(sd, cc)
			self._psd[si] = sda
			if not bad.any():
				continue
			# 組ごとの向き（その組の、骨の近く（5mm 以内）の点の法線の平均）
			g = groups[ii]
			wgt = (np.abs(sd) < 5.0).astype(float)[:, None]
			acc = np.zeros((ng, 3))
			np.add.at(acc, g, ns * wgt)
			nrm = np.linalg.norm(acc, axis=1, keepdims=True)
			dg = acc / np.maximum(nrm, 1e-9)
			d = dg[g]
			okd = (nrm[g, 0] > 1e-6) & (np.einsum("ij,ij->i", d, ns) > -0.2)
			jj = np.where(bad)[0]
			newq = cpp[jj] + cc[jj, None] * ns[jj]                       # 向きが決まらない点は、いちばん近い面へ
			use = okd[jj]
			if use.any():
				k_ = jj[use]
				org = q[ii][k_]
				dd = d[k_]
				inside = sd[k_] < 0.0
				r = np.zeros(len(k_))
				if inside.any():
					rays = np.c_[org[inside], dd[inside]].astype(np.float32)
					th = o["sc"].cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy().astype(float)
					r[inside] = np.where(np.isfinite(th) & (th < 15.0), th, np.nan)
				# 骨の外（0 ≤ 距離 < c）は、足りない分だけ（向きが面に斜めなら、そのぶん長く）。面に沿う向きなら使わない
				cos_ = np.einsum("ij,ij->i", dd, ns[k_])
				r[~inside] = np.where(cos_[~inside] > 0.2, 0.0, np.nan)
				step = np.where(inside, cc[k_], (cc[k_] - sd[k_]) / np.maximum(cos_, 0.3))
				cand = org + (r + step)[:, None] * dd
				ok = np.isfinite(r)
				newq[np.where(use)[0][ok]] = cand[ok]
			# 確かめ: まだ c に足りない点（向きが面とほぼ平行・薄い所を抜けた など）は、その場所からいちばん近い面の外へ
			cp2, ns2, sd2 = self._ob_closest(o, newq)
			short = sd2 < cc[jj] - 0.02
			if short.any():
				newq[short] = cp2[short] + cc[jj][short, None] * ns2[short]
			P[ii[jj]] = newq @ R.T + t
		return P

	@staticmethod
	def _end_ramp(Cl, lo, hi, L):
		"""中心の線 Cl (K,3) に沿った、1点目側・2点目側それぞれの「許す距離」: 端で lo、端から長さ L mm から先は hi（なめらか）。"""
		sl = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(Cl, axis=0), axis=1))]

		def _r(d):
			w = np.clip(d / max(float(L), 1e-6), 0.0, 1.0)
			w = w * w * (3.0 - 2.0 * w)
			return lo + (hi - lo) * w
		return _r(sl), _r(sl[-1] - sl)

	@staticmethod
	def _upsample(W, Ks):
		"""線維 W (F,K,3) を長さ方向に Ks 点へ（Catmull-Rom。元の点を通る）。"""
		F, K, _ = W.shape
		if Ks <= K or K < 2:
			return W
		t = np.linspace(0.0, K - 1.0, Ks)
		i1 = np.clip(np.floor(t).astype(int), 0, K - 2)
		u = (t - i1)[None, :, None]
		i0, i2, i3 = np.clip(i1 - 1, 0, K - 1), i1 + 1, np.clip(i1 + 2, 0, K - 1)
		p0, p1, p2, p3 = W[:, i0], W[:, i1], W[:, i2], W[:, i3]
		return 0.5 * ((2.0 * p1) + (-p0 + p2) * u + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u ** 2
		              + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u ** 3)

	QUAD_EXTRA = 0.3       # mm。皮の四角形の中心を、骨の面から SKIN_CLR よりこれだけ余分に離す

	def _update_lig(self, jobs, fi):
		"""束の線維と皮を描き直す。骨の外へ出すのは、全部の束をまとめて骨ごとに1回ずつ問い合わせる
		（1回に渡す点が多いほど1点あたりが速い。束ごと・段階ごとに分けると1コマ36回で 40ms 以上かかった）。"""
		from vtk.util.numpy_support import vtk_to_numpy
		push = self._skin_push_on()
		Ws = [W for (_b, _a, W) in jobs]
		fj = [j for j, (_b, a, _W) in enumerate(jobs) if a["fib_act"].GetVisibility()]   # 線維が見えている束だけ
		if push and fj:
			try:
				Wp = self._push_many([Ws[j] for j in fj], [self._fib_spec(jobs[j][2], jobs[j][1], jobs[j][0]) for j in fj], fi)
				for j, W in zip(fj, Wp):
					Ws[j] = W
			except Exception as e:
				traceback.print_exc()
				print(f"[靭帯の束] 線維を骨の外へ出せません: {e}")
				push = False
		skins, rxi = [], []
		for (b, a, _W), W in zip(jobs, Ws):
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
				Wu = self._upsample(W, int(a.get("Ks", W.shape[1])))
				rx = self._skin_rx(b, fi)
				fr = None
				if rx is not None:
					try:
						fr = self._skin_frame(Wu, a["rf"], rx)
					except Exception as ex:
						print(f"[靭帯の束] 入力した断面で皮を作れません（線維の広がりで描きます）: {ex}")
				if fr is None:
					skins.append((b, a, self._skin_points(Wu, a["rf"], a["M"], foot)))
				else:
					rxi.append({"b": b, "a": a, "fr": fr, "foot": foot, "X": fr["C"].copy(), "j": len(skins)})
					skins.append((b, a, None))
		# 入力した断面の皮: 骨・となりの皮から離す（輪の点を外へは動かさない）
		if rxi:
			try:
				self._skin_shape(rxi, fi, push, bool(self.params().get("skin_separate", True)))
			except Exception as e:
				traceback.print_exc()
				print(f"[靭帯の束] 入力した断面の皮を、骨・となりの皮から離せません: {e}")
				for it in rxi:
					M_ = int(it["a"]["M"])
					it["S"] = self._skin_build(self._skin_rings(it["fr"], it["fr"]["C"], M_)[0], M_, it["foot"])
			for it in rxi:
				skins[it["j"]] = (it["b"], it["a"], it["S"])
		# 線維の広がりで描く皮は、これまでどおり点ごとに骨の外へ
		js = [j for j in range(len(skins)) if j not in {it["j"] for it in rxi}]
		if push and js:
			try:
				specs = [self._skin_spec(skins[j][2], skins[j][1], skins[j][0]) for j in js]
				Ss = self._push_many([skins[j][2] for j in js], specs, fi)
				Ss = self._lift_many(Ss, specs, fi)
				for j, S in zip(js, Ss):
					skins[j] = (skins[j][0], skins[j][1], S)
			except Exception as e:
				traceback.print_exc()
				print(f"[靭帯の束] 皮を骨の外へ出せません: {e}")
		for (b, a, S) in skins:
			vtk_to_numpy(a["skin_pts"].GetData())[:] = S
			a["skin_pts"].Modified()
			a["skin_pd"].Modified()

	def _fib_spec(self, W, a, b):
		"""線維の押し出しの条件: {"c": 骨 → (n,) 許す距離（nan は動かさない）, "g": 組（線維の同じ番号の点）}。両端の点は動かさない。"""
		Fn, K, _ = W.shape
		rf = float(a.get("rf", 0.4))
		ra, rb = self._end_ramp(W.mean(axis=0), -rf, rf, self.FIB_SINK_LEN)
		sia, sib = a["r"]["sia"], self._si_of_name(b["b"]["bone_name"])
		kk = np.tile(np.arange(K), Fn)
		cs = {}
		for si in range(len(self.poses)):
			c = np.full(K, rf)
			if si == sia:
				c = np.minimum(c, ra)
			if si == sib:
				c = np.minimum(c, rb)
			c = c.copy()
			c[0] = c[-1] = np.nan
			cs[si] = c[kk]
		return {"c": cs, "g": kk}

	def _skin_spec(self, S, a, b):
		"""皮の押し出しの条件（筒だけ。ふたは動かさない）。K（輪の数）・M・輪ごとの c も。"""
		K, M = int(a.get("Ks", a["K"])), int(a["M"])
		clr = float(self.SKIN_CLR)
		ra, rb = self._end_ramp(S[:K * M].reshape(K, M, 3).mean(axis=1), -float(self.SKIN_SINK), clr, self.SKIN_SINK_LEN)
		sia, sib = a["r"]["sia"], self._si_of_name(b["b"]["bone_name"])
		n_cap = len(S) - K * M
		cs, cr = {}, {}
		for si in range(len(self.poses)):
			c = np.full(K, clr)
			if si == sia:
				c = np.minimum(c, ra)
			if si == sib:
				c = np.minimum(c, rb)
			cr[si] = c
			cs[si] = np.concatenate([np.repeat(c, M), np.full(n_cap, np.nan)])
		g = np.concatenate([np.repeat(np.arange(K), M), np.zeros(n_cap, int)])
		return {"c": cs, "g": g, "K": K, "M": M, "c_ring": cr}

	def _push_many(self, Ps, specs, fi):
		"""いくつもの点の組 Ps を、まとめて骨の外へ（骨ごとに1回ずつ問い合わせる）。返り値: 押し出した Ps（形はそのまま）。"""
		flat = [np.asarray(P, float).reshape(-1, 3).copy() for P in Ps]
		off = np.r_[0, np.cumsum([len(f) for f in flat])]
		goff = np.r_[0, np.cumsum([int(sp["g"].max()) + 1 if len(sp["g"]) else 0 for sp in specs])]
		Pall = np.concatenate(flat, axis=0)
		gall = np.concatenate([sp["g"] + goff[j] for j, sp in enumerate(specs)])
		call = {si: np.concatenate([sp["c"][si] for sp in specs]) for si in range(len(self.poses))}
		self._push_disp(Pall, lambda si: call[si], fi, groups=gall)
		psd = self.__dict__.get("_psd") or {}
		for j, sp in enumerate(specs):
			sp["psd"] = {si: v[off[j]:off[j + 1]] for si, v in psd.items()}
		return [Pall[off[j]:off[j + 1]].reshape(np.shape(P)) for j, P in enumerate(Ps)]

	def _lift_many(self, Ss, specs, fi, iters=3):
		"""皮の筒の四角形の中心が骨の面から c より近ければ、その四隅を筒の外向きに持ち上げる（全部の束をまとめて）。
		点がすべて骨の外でも、とがったこぶの先が四角形の真ん中から出た（PCL の脛骨の付着部の上）。下側は動かさない。"""
		Ss = [np.asarray(S, float).copy() for S in Ss]
		active = None                                 # 2回目からは、前に持ち上げた四角形だけを確かめ直す
		for _it in range(iters):
			Qs, Rqs = [], []
			for S, sp in zip(Ss, specs):
				K, M = sp["K"], sp["M"]
				tube = S[:K * M].reshape(K, M, 3)
				Cl = tube.mean(axis=1)
				A_, B_ = tube[:-1], tube[1:]
				Q = 0.25 * (A_ + B_ + np.roll(A_, -1, axis=1) + np.roll(B_, -1, axis=1))
				rq = Q - (0.5 * (Cl[:-1] + Cl[1:]))[:, None]
				rq /= np.maximum(np.linalg.norm(rq, axis=2, keepdims=True), 1e-9)
				Qs.append(Q.reshape(-1, 3))
				Rqs.append(rq.reshape(-1, 3))
			off = np.r_[0, np.cumsum([len(q) for q in Qs])]
			Qa, Ra = np.concatenate(Qs), np.concatenate(Rqs)
			lift = np.zeros(len(Qa))
			for si in range(len(self.poses)):
				o = self._disp_ob(si)
				if o is None:
					continue
				cq_l, cand_l = [], []
				for sp in specs:
					K, M = sp["K"], sp["M"]
					cr = np.asarray(sp["c_ring"][si], float)
					crq = 0.5 * (cr[:-1] + cr[1:])
					# 四角形の中心は少し余分に離す（中心だけ確かめるので、とがったこぶの先が中心と角の間から出ないように）。
					# 付着部の近く（骨の中へ入るのを許す所）では、その割合に合わせて小さく
					crq = crq + self.QUAD_EXTRA * np.clip((crq + self.SKIN_SINK) / (self.SKIN_CLR + self.SKIN_SINK), 0.0, 1.0)
					cand = np.repeat(np.isfinite(crq), M)
					psd = (sp.get("psd") or {}).get(si)
					if psd is not None and len(psd) >= K * M:
						sc_ = psd[:K * M].reshape(K, M)
						qmin = np.minimum(np.minimum(sc_[:-1], sc_[1:]),
						                  np.minimum(np.roll(sc_[:-1], -1, axis=1), np.roll(sc_[1:], -1, axis=1)))
						cand &= (qmin < crq[:, None] + 1.5).reshape(-1)      # 四隅のどれかが骨に近い四角形だけ
					cq_l.append(np.repeat(crq, M))
					cand_l.append(cand)
				cqa, ca = np.concatenate(cq_l), np.concatenate(cand_l)
				if active is not None:
					ca &= active
				T = self.poses[si][fi]
				R, t = T[:3, :3], T[:3, 3]
				q = (Qa - t) @ R
				ca &= np.all((q > o["lo"] - 3.0) & (q < o["hi"] + 3.0), axis=1)
				if not ca.any():
					continue
				ii = np.where(ca)[0]
				_cp, ns, sd = self._ob_closest(o, q[ii])
				cos_ = np.einsum("ij,ij->i", Ra[ii], ns @ R.T)
				dfc = cqa[ii] - sd
				ok = (dfc > 0) & (cos_ > 0.2) & (dfc < 6.0)
				if ok.any():
					v = np.zeros(len(Qa))
					v[ii[ok]] = dfc[ok] / np.maximum(cos_[ok], 0.3)
					lift = np.maximum(lift, v)
			if not (lift > 1e-4).any():
				break
			active = lift > 1e-4
			for j, (S, sp) in enumerate(zip(Ss, specs)):
				K, M = sp["K"], sp["M"]
				lj = lift[off[j]:off[j + 1]].reshape(K - 1, M)
				if not (lj > 1e-4).any():
					continue
				tube = S[:K * M].reshape(K, M, 3)
				Cl = tube.mean(axis=1)
				lv = np.zeros((K, M))
				for dk, dm in ((0, 0), (1, 0), (0, 1), (1, 1)):
					lv[dk:K - 1 + dk] = np.maximum(lv[dk:K - 1 + dk], np.roll(lj, dm, axis=1))
				rv = tube - Cl[:, None]
				rv /= np.maximum(np.linalg.norm(rv, axis=2, keepdims=True), 1e-9)
				S[:K * M] = (tube + lv[:, :, None] * rv).reshape(-1, 3)
				mv = sp.setdefault("moved", np.zeros(K * M, bool))
				mv |= (lv > 1e-4).reshape(-1)
		# 確かめ: 持ち上げで動いた点（四隅は筒の外向きに動かすので、骨に接している側の角が骨へ入ることがある）を、骨の外へ
		for si in range(len(self.poses)):
			o = self._disp_ob(si)
			if o is None:
				continue
			sel = [(j, np.where(sp.get("moved", np.zeros(0, bool)))[0]) for j, sp in enumerate(specs)]
			sel = [(j, ix) for j, ix in sel if len(ix)]
			if not sel:
				continue
			Pm = np.concatenate([Ss[j][ix] for j, ix in sel])
			cm = np.concatenate([np.asarray(specs[j]["c"][si], float)[ix] for j, ix in sel])
			T = self.poses[si][fi]
			R, t = T[:3, :3], T[:3, 3]
			q = (Pm - t) @ R
			okm = np.isfinite(cm) & np.all((q > o["lo"] - 3.0) & (q < o["hi"] + 3.0), axis=1)
			if not okm.any():
				continue
			kk = np.where(okm)[0]
			cp_, ns_, sd_ = self._ob_closest(o, q[kk])
			short = sd_ < cm[kk] - 0.02
			if not short.any():
				continue
			q[kk[short]] = cp_[short] + cm[kk[short], None] * ns_[short]
			Pm2 = q @ R.T + t
			o_ = 0
			for j, ix in sel:
				Ss[j][ix] = Pm2[o_:o_ + len(ix)]
				o_ += len(ix)
		return Ss

	def _push_fibers(self, W, a, b, fi):
		"""線維 W (F,K,3)（世界座標）を見えている骨の外へ。両端の点は動かさない。"""
		Fn, K, _ = W.shape
		rf = float(a.get("rf", 0.4))
		ra, rb = self._end_ramp(W.mean(axis=0), -rf, rf, self.FIB_SINK_LEN)
		sia, sib = a["r"]["sia"], self._si_of_name(b["b"]["bone_name"])
		kk = np.tile(np.arange(K), Fn)

		def c_of(si):
			c = np.full(K, rf)
			if si == sia:
				c = np.minimum(c, ra)
			if si == sib:
				c = np.minimum(c, rb)
			c = c.copy()
			c[0] = c[-1] = np.nan
			return c[kk]
		P = W.reshape(-1, 3).copy()
		self._push_disp(P, c_of, fi, groups=kk)
		return P.reshape(W.shape)

	def _push_skin(self, S, a, b, fi):
		"""皮の点 S（K*M の筒 + 両端のふた）を見えている骨の外へ。ふたは動かさない。"""
		K, M = int(a.get("Ks", a["K"])), int(a["M"])
		clr = float(self.SKIN_CLR)
		ra, rb = self._end_ramp(S[:K * M].reshape(K, M, 3).mean(axis=1), -float(self.SKIN_SINK), clr, self.SKIN_SINK_LEN)
		sia, sib = a["r"]["sia"], self._si_of_name(b["b"]["bone_name"])
		n_cap = len(S) - K * M

		def c_of(si):
			c = np.full(K, clr)
			if si == sia:
				c = np.minimum(c, ra)
			if si == sib:
				c = np.minimum(c, rb)
			return np.concatenate([np.repeat(c, M), np.full(n_cap, np.nan)])
		grp = np.concatenate([np.repeat(np.arange(K), M), np.zeros(n_cap, int)])
		self._push_disp(S, c_of, fi, groups=grp)
		self._lift_quads(S, K, M, {si: c_of(si)[:K * M:M] for si in range(len(self.poses))}, fi)
		return S

	def _lift_quads(self, S, K, M, c_ring, fi, iters=2):
		"""皮の筒の四角形の中心が骨の面から c（輪ごと）より近ければ、その四隅を筒の外向きに持ち上げる（その場で書き換え）。
		点の間から骨のとがった所が出ないように。下側（筒の外向きが骨の中を向く所）は動かさない。"""
		for _it in range(iters):
			tube = S[:K * M].reshape(K, M, 3)
			Cl = tube.mean(axis=1)
			A_, B_ = tube[:-1], tube[1:]
			Q = 0.25 * (A_ + B_ + np.roll(A_, -1, axis=1) + np.roll(B_, -1, axis=1))          # (K-1, M, 3)
			Cq = 0.5 * (Cl[:-1] + Cl[1:])
			rq = Q - Cq[:, None]
			rq /= np.maximum(np.linalg.norm(rq, axis=2, keepdims=True), 1e-9)
			lift = np.zeros((K - 1, M))
			for si in range(len(self.poses)):
				o = self._disp_ob(si)
				if o is None:
					continue
				cr = np.asarray(c_ring[si], float)
				cq = 0.5 * (cr[:-1] + cr[1:])
				T = self.poses[si][fi]
				R, t = T[:3, :3], T[:3, 3]
				q = (Q.reshape(-1, 3) - t) @ R
				near = np.all((q > o["lo"] - 3.0) & (q < o["hi"] + 3.0), axis=1) & np.repeat(np.isfinite(cq), M)
				psd = (self.__dict__.get("_psd") or {}).get(si)
				if psd is not None and len(psd) >= K * M:
					sc_ = psd[:K * M].reshape(K, M)
					qmin = np.minimum(np.minimum(sc_[:-1], sc_[1:]), np.minimum(np.roll(sc_[:-1], -1, axis=1), np.roll(sc_[1:], -1, axis=1)))
					near &= (qmin < cq[:, None] + 1.5).reshape(-1)          # 四隅のどれかが骨に近い四角形だけ
				if not near.any():
					continue
				ii = np.where(near)[0]
				_cp, ns, sd = self._ob_closest(o, q[ii])
				nsw = ns @ R.T
				cos_ = np.einsum("ij,ij->i", rq.reshape(-1, 3)[ii], nsw)
				dfc = np.repeat(cq, M)[ii] - sd
				ok = (dfc > 0) & (cos_ > 0.2) & (dfc < 6.0)
				if ok.any():
					v = np.zeros(len(Q.reshape(-1, 3)))
					v[ii[ok]] = dfc[ok] / np.maximum(cos_[ok], 0.3)
					lift = np.maximum(lift, v.reshape(K - 1, M))
			if not (lift > 1e-4).any():
				break
			lv = np.zeros((K, M))
			for dk, dm in ((0, 0), (1, 0), (0, 1), (1, 1)):
				lv[dk:K - 1 + dk] = np.maximum(lv[dk:K - 1 + dk], np.roll(lift, dm, axis=1))
			rv = tube - Cl[:, None]
			rv /= np.maximum(np.linalg.norm(rv, axis=2, keepdims=True), 1e-9)
			S[:K * M] = (tube + lv[:, :, None] * rv).reshape(-1, 3)
		return S

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
		if self._flat_fits.get((si, "mesh")):
			cpp = self._flat_map(si, cpp)                    # 切り残しを消した付着部（計算のモデルの表示）
			out["calc"] = self._flat_map(si, out["calc"], offset=-self.SKIN_SINK)
			out["out_calc"] = self._flat_map(si, out["out_calc"], offset=0.05)
		ca = self._onto_alt(si, cpp)
		if ca is not None:
			out["alt"] = ca - self.SKIN_SINK * ns2
			out["out_alt"] = ca + 0.05 * ns2
		# ふたの同心の輪（骨の面へ落として、少し中へ）
		caps_c, caps_a = [], []
		for sc_ in self.SKIN_CAP_SCALES:
			Ps = c + sc_ * (np.outer(np.cos(th) * float(ell["ra"]), u) + np.outer(np.sin(th) * float(ell["rb"]), v))
			cps, nss = bones.onto(idx, Ps, nE)
			if self._flat_fits.get((si, "mesh")):
				cps = self._flat_map(si, cps)
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

	def _skin_rx(self, b, fi):
		"""皮を入力した断面で描くための値（入力が無い・切っているときは None）。"""
		if not bool(self.params().get("skin_input_dims", True)):
			return None
		AB = self._xsec(b)
		if AB is None:
			return None
		try:
			ea, eb = b["a"]["ellipse"], b["b"]["ellipse"]
			sa, sb = self._si_of_name(b["a"]["bone_name"]), self._si_of_name(b["b"]["bone_name"])
			Ta, Tb = self.poses[sa][fi], self.poses[sb][fi]
			return {"AB": AB, "foot_r": ((float(ea["ra"]), float(ea["rb"])), (float(eb["ra"]), float(eb["rb"]))),
			        "u_a": Ta[:3, :3] @ np.asarray(ea["u"], float), "u_b": Tb[:3, :3] @ np.asarray(eb["u"], float)}
		except Exception:
			return None

	@staticmethod
	def _rx_rings(C, t, Q, rf, rx, Qc=None, contain=True):
		"""入力した断面の皮の輪: 向き e1 (K,3) と半径 r1・r2 (K,)。
		向き: 1点目の付着部の長径の向きから 2点目の長径の向きへ、いちばん小さいねじれでなめらかに回す（中心の線に沿って平行に運んだ
		枠で測る）。中ほどでは線維の広がりの向きを倍角で少し取り入れる（重み 0.9 未満 × 広がりの偏り。足しても 0 にならず向きが飛ばない）。
		半径: 中ほど＝入力した長径・短径の半分、両端＝付着部の楕円の半径、間は sin（_sheath と同じ）。線維がはみ出す所は線維を包む。"""
		K = len(C)
		sl = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))]
		s_ = sl / max(float(sl[-1]), 1e-9)
		n_ = np.zeros((K, 3))
		ua = np.asarray(rx["u_a"], float)
		v0 = ua - float(ua @ t[0]) * t[0]
		for ref in ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]):
			if np.linalg.norm(v0) > 1e-6:
				break
			v0 = np.cross(t[0], ref)
		n_[0] = v0 / np.linalg.norm(v0)
		for k in range(1, K):
			v_ = n_[k - 1] - float(n_[k - 1] @ t[k]) * t[k]
			if np.linalg.norm(v_) < 1e-9:
				v_ = np.cross(t[k], [1.0, 0.0, 0.0])
			n_[k] = v_ / np.linalg.norm(v_)
		b_ = np.cross(t, n_)
		ub = np.asarray(rx["u_b"], float)
		xb, yb = float(ub @ n_[-1]), float(ub @ b_[-1])
		dl = (np.arctan2(yb, xb) + 0.5 * np.pi) % np.pi - 0.5 * np.pi     # いちばん小さいねじれ（π の周期）
		if np.hypot(xb, yb) < 0.2:
			dl = 0.0
		ss = s_ * s_ * (3.0 - 2.0 * s_)
		thb = dl * ss
		Fn = Q.shape[0]
		if Fn >= 2:
			wv, V = np.linalg.eigh(np.einsum("fki,fkj->kij", Q, Q))
			ef = V[:, :, -1]
			alpha = (wv[:, -1] - wv[:, -2]) / np.maximum(wv[:, -1] + wv[:, -2], 1e-12)
			thf = np.arctan2(np.einsum("ki,ki->k", ef, b_), np.einsum("ki,ki->k", ef, n_))
			d2 = (2.0 * (thf - thb) + np.pi) % (2.0 * np.pi) - np.pi
			gam = 0.9 * np.sin(np.pi * s_) * np.clip(alpha, 0.0, 1.0)
			wn = max(3, K // 8) | 1
			ker = np.ones(wn) / wn
			gam = np.convolve(np.pad(gam, wn // 2, mode="edge"), ker, mode="valid")
			corr = 0.5 * np.arctan2(gam * np.sin(d2), 1.0 + gam * np.cos(d2))
		else:
			corr = np.zeros(K)
		th = thb + corr
		e1 = np.cos(th)[:, None] * n_ + np.sin(th)[:, None] * b_
		e2 = np.cross(t, e1)
		fa, fb = np.asarray(rx["foot_r"][0], float), np.asarray(rx["foot_r"][1], float)
		fr = (1.0 - s_)[:, None] * fa[None] + s_[:, None] * fb[None]
		w_ = np.sin(np.pi * s_)
		R = (1.0 - w_)[:, None] * fr + w_[:, None] * (0.5 * np.asarray(rx["AB"], float))[None]
		Qc = Q if Qc is None else Qc                     # 線維を包むかは皮の中心から（向きは線維の重心から）
		r1 = np.maximum(R[:, 0], np.abs(np.einsum("fki,ki->fk", Qc, e1)).max(axis=0) + rf) if (Fn and contain) else R[:, 0]
		r2 = np.maximum(R[:, 1], np.abs(np.einsum("fki,ki->fk", Qc, e2)).max(axis=0) + rf) if (Fn and contain) else R[:, 1]
		return e1, r1, r2

	def _skin_points(self, W, rf, M, foot=None, rx=None):
		"""線維の点 W (F,K,3) → 皮の点 (K*M + 両端のふた, 3)。各所で楕円（向きはなめらかにつなぐ）。
		foot=({"ring": 1点目の付着部の輪, "cap": ふたの輪}, {2点目…})（世界座標）があれば、皮の両端をそれに合わせる。
		rx（_skin_rx）があれば、輪の大きさと向きは入力した断面と付着部の楕円から（_skin_frame。骨・となりの皮とのことは _skin_shape）。"""
		if rx is not None:
			try:
				fr = self._skin_frame(W, rf, rx)
				return self._skin_build(self._skin_rings(fr, fr["C"], M)[0], M, foot)
			except Exception as ex:
				print(f"[靭帯の束] 入力した断面で皮を作れません（線維の広がりで描きます）: {ex}")
		Fn, K, _ = W.shape
		C = W.mean(axis=0)
		th = np.linspace(0, 2 * np.pi, M, endpoint=False)
		# 全部の輪をまとめて（以前は輪ごとの SVD のループ。皮を細かくしたら遅くなった）
		kk = np.arange(K)
		t = C[np.minimum(kk + 1, K - 1)] - C[np.maximum(kk - 1, 0)]
		t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
		Q = W - C[None]
		Q = Q - np.einsum("fki,ki->fk", Q, t)[:, :, None] * t[None]
		if Fn >= 2:
			_w, V = np.linalg.eigh(np.einsum("fki,fkj->kij", Q, Q))      # 線維の広がりの主成分（SVD と同じ向き）
			e1 = V[:, :, -1]
		else:
			e1 = np.cross(t, [1.0, 0.0, 0.0])
		e1 = e1 - np.einsum("ki,ki->k", e1, t)[:, None] * t
		bad = np.linalg.norm(e1, axis=1) < 1e-6
		if bad.any():
			e1[bad] = np.cross(t[bad], [0.0, 1.0, 0.0])
		e1 /= np.maximum(np.linalg.norm(e1, axis=1, keepdims=True), 1e-12)
		dd = np.einsum("ki,ki->k", e1[1:], e1[:-1])
		e1 *= np.r_[1.0, np.cumprod(np.where(dd < 0, -1.0, 1.0))][:, None]   # 向きをなめらかにつなぐ（裏返さない）
		e2 = np.cross(t, e1)
		r1 = np.abs(np.einsum("fki,ki->fk", Q, e1)).max(axis=0) + rf
		r2 = np.abs(np.einsum("fki,ki->fk", Q, e2)).max(axis=0) + rf
		out = (C[:, None] + (np.cos(th)[None, :, None] * r1[:, None, None]) * e1[:, None]
		       + (np.sin(th)[None, :, None] * r2[:, None, None]) * e2[:, None])
		return self._skin_build(out, M, foot)

	def _skin_build(self, out, M, foot=None):
		"""輪 out (K,M,3) の両端を付着部の輪へなめらかに寄せ、ふたを付ける → 皮の点 (K*M + 両端のふた, 3)。"""
		out = np.array(out, float)
		K = len(out)
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

	@staticmethod
	def _smooth_center(C, frac):
		"""中心の線 C (K,3) の細かいうねりを取る（両端は動かさない）。端どうしを結ぶ直線からのずれを、端で 0 になるように
		奇の折り返しをして、輪の数の frac 倍の幅のガウスでならす。"""
		K = len(C)
		if K < 5 or frac <= 0:
			return np.array(C, float)
		sl = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))]
		L = max(float(sl[-1]), 1e-9)
		line = C[0] + (sl / L)[:, None] * (C[-1] - C[0])
		D = C - line
		sig = max(1.0, frac * (K - 1))
		pad = min(K - 1, int(3 * sig) + 1)
		Dp = np.concatenate([-D[pad:0:-1], D, -D[-2:-pad - 2:-1]], axis=0)       # 奇の折り返し（端で 0）
		xs = np.arange(-pad, pad + 1)
		ker = np.exp(-0.5 * (xs / sig) ** 2)
		ker /= ker.sum()
		Ds = np.stack([np.convolve(Dp[:, i], ker, mode="valid") for i in range(3)], axis=1)[:K]
		return line + Ds

	def _ring_normals(self, X):
		"""輪の面の向き: 中心の線 X (K,3) の接線を、長さ方向に輪の数の SKIN_N_SIG 倍の幅でならしたもの。
		接線そのものだと、中心の線の小さなうねりで向きが変わり、太い皮は曲がりの内側で隣の輪と交差して折れ目になった（2026-10-21）。"""
		from scipy.ndimage import gaussian_filter1d
		K = len(X)
		kk = np.arange(K)
		t = X[np.minimum(kk + 1, K - 1)] - X[np.maximum(kk - 1, 0)]
		t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
		n = gaussian_filter1d(t, max(0.5, self.SKIN_N_SIG * (K - 1)), axis=0, mode="nearest")
		return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)

	def _skin_frame(self, W, rf, rx):
		"""入力した断面の皮の骨組み（線維の点 W (F,K,3) から）: 中心の線 C（線維の重心の細かいうねりを取る）・輪の長径の向き e1・半径 r1, r2。"""
		C0 = W.mean(axis=0)
		C = self._smooth_center(C0, self.SKIN_C_SMOOTH)
		n = self._ring_normals(C)
		Qp = W - C0[None]                                # 線維の重心から（線維の広がりの向き・大きさ）
		Qp = Qp - np.einsum("fki,ki->fk", Qp, n)[:, :, None] * n[None]
		# 輪を線維の広がりまで広げる量も、線維の重心から測る（皮の中心からだと、中心の線をならしたずれの分まで輪が大きくなり、
		# 入力した断面より太くなった。10/22: 8×5 が 8.8×5.6）
		e1, r1, r2 = self._rx_rings(C, n, Qp, rf, rx, contain=bool(self.SKIN_CONTAIN))
		return {"C": C, "C0": C0, "e1": e1, "r1": r1, "r2": r2}

	def _skin_rings(self, fr, X, M):
		"""中心の線 X (K,3) のまわりの輪 (K,M,3)（大きさ・向きは fr）。返り値: 輪・輪の面の向き n・長径の向き e1・短径の向き e2。"""
		n = self._ring_normals(X)
		e1 = fr["e1"] - np.einsum("ki,ki->k", fr["e1"], n)[:, None] * n
		e1 /= np.maximum(np.linalg.norm(e1, axis=1, keepdims=True), 1e-9)
		e2 = np.cross(n, e1)
		th = np.linspace(0, 2 * np.pi, M, endpoint=False)
		R = (X[:, None] + (np.cos(th)[None, :, None] * fr["r1"][:, None, None]) * e1[:, None]
		     + (np.sin(th)[None, :, None] * fr["r2"][:, None, None]) * e2[:, None])
		return R, n, e1, e2

	def _blend_w(self, K):
		"""輪ごとの、端の輪（付着部）へ寄せる重み（_skin_build と同じ。両端で 1、端から長さの SKIN_BLEND より先は 0）。"""
		tk = np.linspace(0.0, 1.0, K)
		z = float(self.SKIN_BLEND)
		w = np.zeros(K)
		for tt in (tk, 1.0 - tk):
			x = np.clip((z - tt) / z, 0.0, 1.0)
			w = np.maximum(w, x * x * (3.0 - 2.0 * x))
		return w

	def _skin_clear(self, a, b, X, si):
		"""輪ごとの、骨 si から離す距離（_skin_spec と同じ: 付く骨の端では骨の中へ SKIN_SINK・端から SKIN_SINK_LEN で SKIN_CLR）。"""
		clr = float(self.SKIN_CLR)
		ra, rb = self._end_ramp(X, -float(self.SKIN_SINK), clr, self.SKIN_SINK_LEN)
		c = np.full(len(X), clr)
		if si == a["r"]["sia"]:
			c = np.minimum(c, ra)
		if si == self._si_of_name(b["b"]["bone_name"]):
			c = np.minimum(c, rb)
		return c

	@staticmethod
	def _smooth_move(V, sig):
		"""輪ごとの動き V (K,3) を長さ方向にならす（大きさは前後の最大をとってから＝ならしても足りなくならない）。"""
		from scipy.ndimage import gaussian_filter1d, maximum_filter1d
		m = np.linalg.norm(V, axis=1)
		if not (m > 1e-6).any():
			return np.zeros_like(V)
		w = 2 * int(round(1.5 * sig)) + 1
		ms = gaussian_filter1d(maximum_filter1d(m, size=w, mode="nearest"), sig, mode="nearest")
		ds = gaussian_filter1d(V, sig, axis=0, mode="nearest")
		dn = np.linalg.norm(ds, axis=1)
		ds = np.where((dn > 1e-9)[:, None], ds / np.maximum(dn, 1e-9)[:, None], 0.0)
		return ms[:, None] * ds

	@staticmethod
	def _skin_move(it, V):
		"""中心の線と輪をまとめて V (K,3) だけ動かす（輪の面の向きはそのまま）。"""
		it["U"] = it["U"] + V
		it["X"] = it["X0"] + it["U"]
		it["R"] = it["R0"] + it["U"][:, None, :]

	def _skin_shape(self, items, fi, push, sep):
		"""入力した断面の皮を、骨・となりの皮から離す（表示だけ・2026-10-21）。items は束ごとに {"b", "a", "fr", "foot", "X"}、結果は it["S"]。
		① 皮どうしを離す（中心の線ごと・輪の面の中で）② 輪の中心を骨から少し離す（中心の線ごと）③ 輪を作り、両端を付着部へ
		④ 骨に当たる側だけ、輪の点を輪の中心へ縮める ⑤ 皮どうしの重なりも、それぞれ縮める（押し合って平らに）。
		輪の点を外へは動かさないので、ひれ（はみ出し）ができない（以前は点ごとに骨の外へ押し出していて、PCL の脛骨側に横のひれが出た）。
		輪の面の向きは中心の線の接線をならしたもの（輪どうしが交差しない＝折れない）。"""
		M = int(items[0]["a"]["M"])
		for it in items:
			K = len(it["X"])
			it["X0"] = it["X"].copy()
			it["R0"], it["n"], it["e1"], it["e2"] = self._skin_rings(it["fr"], it["X"], M)
			it["R"] = it["R0"].copy()
			it["U"] = np.zeros((K, 3))
			it["wb"] = self._blend_w(K)
			wa_, wb_ = self._end_ramp(it["X0"], 0.0, 1.0, self.SKIN_LIFT_END)
			it["ramp"] = np.minimum(wa_, wb_)
			it["sig"] = max(1.0, self.SKIN_MOVE_SIG * (K - 1))
		if sep and len(items) >= 2:
			self._skin_separate(items)
		if push:
			self._skin_lift(items, fi)
			if self._skin_center_guard(items, fi):
				self._skin_lift(items, fi)                           # 戻した所を、もう一度骨から離す
			self._skin_orient(items, fi)                             # 骨に挟まれる所は、厚みをすき間の向きへ
		for it in items:
			it["R"], it["n"], it["e1"], it["e2"] = self._skin_rings(it["fr"], it["X"], M)
			it["S"] = self._skin_build(it["R"], M, it["foot"])
		org = [it["X"] for it in items]
		pre = [it["S"] for it in items]
		skins = [(it["b"], it["a"], it["S"]) for it in items]
		if push:
			skins = self._skin_clip_bone(skins, fi, org)
		if sep and len(skins) >= 2:
			skins = self._skin_clip_mutual(skins, org)
		for it, (_b, _a, S) in zip(items, skins):
			it["S"] = S
		self._skin_last = {it["b"].get("id"): {"X": it["X"], "pre": p_} for it, p_ in zip(items, pre)}   # 確かめ用

	def _skin_center_guard(self, items, fi):
		"""中心の線が骨の中（許す距離より内）に入った所は、なめらかに線維の重心の線へ戻す（線維は骨の外。2026-10-22）。
		強くならした中心の線が骨の出っ張りを近道し、骨に挟まれて中心を離しきれない所があった（中心が骨の中の輪は縮められない）。"""
		from scipy.ndimage import gaussian_filter1d, maximum_filter1d
		moved = False
		items = [it for it in items if "C0" in it["fr"]]
		if not items:
			return moved
		Ps = [it["X"] for it in items] + [np.asarray(it["fr"]["C0"], float) for it in items]
		off = np.r_[0, np.cumsum([len(p) for p in Ps])]
		Pall = np.concatenate(Ps)
		n_ = len(items)
		mg = [np.full(len(p), np.inf) for p in Ps]                    # 骨からの距離 − 許す距離（いちばん近い骨）
		for si in range(len(self.poses)):
			o = self._disp_ob(si)
			if o is None:
				continue
			T = self.poses[si][fi]
			sd = self._ob_closest(o, (Pall - T[:3, 3]) @ T[:3, :3])[2]
			for j in range(2 * n_):
				it = items[j % n_]
				mg[j] = np.minimum(mg[j], sd[off[j]:off[j + 1]] - self._skin_clear(it["a"], it["b"], it["X"], si))
		for j, it in enumerate(items):
			# 骨に入った所のうち、線維の重心の線のほうが骨から遠い所だけ（線維が骨の出っ張りを巻くと、重心が骨の中に来ることがある）
			bd = (mg[j] < 0.05) & (mg[n_ + j] > mg[j] + 0.1) & (it["wb"] < 0.5)     # 付着部（骨の中から出る所）は除く
			if not bd.any():
				continue
			w = gaussian_filter1d(maximum_filter1d(bd.astype(float), size=5, mode="nearest"), 1.5, mode="nearest")
			w = np.clip(1.5 * w, 0.0, 1.0)[:, None]
			it["X"] = (1.0 - w) * it["X"] + w * np.asarray(it["fr"]["C0"], float)
			it["X0"], it["U"] = it["X"].copy(), np.zeros_like(it["X"])     # ここから離し直す
			it["R0"], it["n"], it["e1"], it["e2"] = self._skin_rings(it["fr"], it["X"], int(it["a"]["M"]))
			it["R"] = it["R0"].copy()
			moved = True
		return moved

	def _skin_orient(self, items, fi):
		"""反対向きの 2 つの骨に挟まれて、長径の向きでは入らない所だけ、楕円の短径（厚み）をすき間の向きへそろえる（2026-10-22）。
		it["fr"]["e1"] を差し替える（大きさは入力のまま）。大腿骨と脛骨の間を通る所で長径が骨の方を向くと、両側から縮められて
		細いくびれになった（PCL）。骨が 1 つ近いだけの所は変えない（向きの段ができた）。"""
		from scipy.ndimage import gaussian_filter1d
		M = int(items[0]["a"]["M"])
		Ps = [it["X"] for it in items]
		off = np.r_[0, np.cumsum([len(p) for p in Ps])]
		Pall = np.concatenate(Ps)
		hits = []                                                    # 骨ごとの (符号つき距離, 法線)
		for si in range(len(self.poses)):
			o = self._disp_ob(si)
			if o is None:
				continue
			T = self.poses[si][fi]
			_cp, ns, sd = self._ob_closest(o, (Pall - T[:3, 3]) @ T[:3, :3])
			hits.append((sd, ns @ T[:3, :3].T))
		if len(hits) < 2:
			return
		for j, it in enumerate(items):
			K = len(it["X"])
			_R, n, e1, e2 = self._skin_rings(it["fr"], it["X"], M)
			r1, r2 = it["fr"]["r1"], it["fr"]["r2"]
			acc = np.zeros((K, 2))
			qs = np.zeros(K)
			for a_ in range(len(hits)):
				for b_ in range(a_ + 1, len(hits)):
					sa, na = hits[a_][0][off[j]:off[j + 1]], hits[a_][1][off[j]:off[j + 1]]
					sb, nb = hits[b_][0][off[j]:off[j + 1]], hits[b_][1][off[j]:off[j + 1]]
					opp = np.einsum("ki,ki->k", na, nb) < -0.3              # 反対向き＝挟まれている
					gap = sa + sb                                            # 骨と骨のすき間（中心を通る向き）
					q = np.clip((2.0 * r1 + 1.0 - gap) / np.maximum(2.0 * (r1 - r2) + 1.0, 1e-6), 0.0, 1.0) * opp
					d = 0.5 * (na - nb)                                      # すき間の向き（厚みをこの向きへ）
					d = d - np.einsum("ki,ki->k", d, n)[:, None] * n
					dn = np.linalg.norm(d, axis=1)
					q = q * np.clip(dn / 0.5, 0.0, 1.0)
					t1 = np.cross(n, d / np.maximum(dn, 1e-9)[:, None])     # 長径の向き（すき間に沿う）
					ph = np.arctan2(np.einsum("ki,ki->k", t1, e2), np.einsum("ki,ki->k", t1, e1))
					acc += q[:, None] * np.c_[np.cos(2 * ph), np.sin(2 * ph)]
					qs += q
			if not (qs > 1e-3).any():
				continue
			w = np.minimum(qs, 1.0) * (1.0 - it["wb"])
			v = (1.0 - w)[:, None] * np.c_[np.ones(K), np.zeros(K)] + w[:, None] * (acc / np.maximum(np.linalg.norm(acc, axis=1), 1e-9)[:, None])
			v = gaussian_filter1d(v, max(1.0, self.SKIN_ORIENT_SIG * (K - 1)), axis=0, mode="nearest")
			dlt = 0.5 * np.arctan2(v[:, 1], v[:, 0])
			it["fr"] = dict(it["fr"], e1=np.cos(dlt)[:, None] * e1 + np.sin(dlt)[:, None] * e2)

	def _skin_separate(self, items):
		"""皮どうしを離す（中心の線ごと）: 輪を断面とみなして、中心線どうしの距離で（_bundle_contact。両端の 15% は動かさない）。
		動きは輪の面の中だけ・長さ方向にならし・付着部の近くでは弱める。"""
		for _ in range(int(self.SKIN_SEP_IT)):
			Ts = [np.ascontiguousarray(it["R"].transpose(1, 0, 2)) for it in items]
			T0 = [T.copy() for T in Ts]
			if _bundle_contact(Ts, [0.0] * len(Ts), end_frac=0.15, gain=self.SKIN_SEP_GAIN, margin=1.0) == 0:
				break
			for it, T, T_ in zip(items, Ts, T0):
				V = (T - T_).mean(axis=0)
				V = V - np.einsum("ki,ki->k", V, it["n"])[:, None] * it["n"]
				self._skin_move(it, self._smooth_move(V, it["sig"]) * (it["ramp"] * (1.0 - it["wb"]))[:, None])

	def _skin_lift(self, items, fi):
		"""輪の中心を骨から少し離す（中心の線ごと）: その向きの輪の半径の SKIN_LIFT[0] 倍（SKIN_LIFT[1] mm まで）＋ 許す距離。
		付着部の近くでは弱める。骨に挟まれていれば、両方の骨からの押しが打ち消し合う（残りは _skin_clip_bone で縮める）。
		輪ごと骨の外まで持ち上げると、脛骨と大腿骨の間を通る所（AM束・PCL）で反対側の骨に入った（実データ）。"""
		f0, fmax = self.SKIN_LIFT
		for _ in range(int(self.SKIN_LIFT_IT)):
			Ps = [it["X"] for it in items]
			off = np.r_[0, np.cumsum([len(p) for p in Ps])]
			Pall = np.concatenate(Ps)
			V = [np.zeros((len(it["X"]), 3)) for it in items]
			for si in range(len(self.poses)):
				o = self._disp_ob(si)
				if o is None:
					continue
				T = self.poses[si][fi]
				Rm, t = T[:3, :3], T[:3, 3]
				_cp, ns, sd = self._ob_closest(o, (Pall - t) @ Rm)
				nw = ns @ Rm.T
				for j, it in enumerate(items):
					sdj, nj = sd[off[j]:off[j + 1]], nw[off[j]:off[j + 1]]
					d = nj - np.einsum("ki,ki->k", nj, it["n"])[:, None] * it["n"]       # 骨の法線を輪の面へ
					dn = np.linalg.norm(d, axis=1)
					d = np.where((dn > 1e-6)[:, None], d / np.maximum(dn, 1e-9)[:, None], 0.0)
					rt = np.sqrt((it["fr"]["r1"] * np.einsum("ki,ki->k", d, it["e1"])) ** 2
					             + (it["fr"]["r2"] * np.einsum("ki,ki->k", d, it["e2"])) ** 2)     # その向きの輪の半径
					tgt = (np.minimum(f0 * rt, fmax) * (1.0 - it["wb"]) + self._skin_clear(it["a"], it["b"], it["X"], si)
					       + self.SKIN_CLIP_EXTRA)
					need = np.clip(tgt - sdj, 0.0, None) * (1.0 - it["wb"])
					need[sdj > 15.0] = 0.0
					V[j] += need[:, None] * d / np.maximum(dn, 0.5)[:, None]
			mx = 0.0
			for j, it in enumerate(items):
				self._skin_move(it, self._smooth_move(V[j], it["sig"]) * it["ramp"][:, None])
				mx = max(mx, float(np.linalg.norm(V[j], axis=1).max()))
			if mx < 0.05:
				break

	def _skin_clip_bone(self, skins, fi, origins):
		"""骨に当たる側だけ、皮の輪の点を輪の中心（origins: 中心の線の点）へ縮める（外へは動かさない＝ひれができない・輪の面の中＝交差しない）。
		中心から輪の点への光線が骨に当たる所まで（許す距離を残して）。中心が骨の中の輪（付着部）は縮めない。"""
		import open3d as o3d
		out = []
		for jj, (b, a, S) in enumerate(skins):
			K, M = int(a.get("Ks", a["K"])), int(a["M"])
			O = np.asarray(origins[jj], float)
			U = S[:K * M].reshape(K, M, 3) - O[:, None]
			rho = np.linalg.norm(U, axis=2)
			u = U / np.maximum(rho, 1e-9)[:, :, None]
			lim_all = np.full((K, M), np.inf)
			for si in range(len(self.poses)):
				o = self._disp_ob(si)
				if o is None:
					continue
				T = self.poses[si][fi]
				Rm, t = T[:3, :3], T[:3, 3]
				Ol = (O - t) @ Rm
				_cp, _ns, sdc = self._ob_closest(o, Ol)
				dirl = (u.reshape(-1, 3) @ Rm).astype(np.float32)
				ans = o["sc"].cast_rays(o3d.core.Tensor(np.c_[np.repeat(Ol, M, axis=0).astype(np.float32), dirl]))
				th = ans["t_hit"].numpy().astype(float).reshape(K, M)
				fn = ans["primitive_normals"].numpy().astype(float).reshape(K, M, 3)
				cosi = np.abs(np.einsum("kmi,kmi->km", fn, dirl.reshape(K, M, 3)))
				c = self._skin_clear(a, b, O, si) + self.SKIN_CLIP_EXTRA
				lim = np.where(np.isfinite(th), th - c[:, None] / np.maximum(cosi, 0.5), np.inf)
				lim[sdc < 0.0] = np.inf
				lim_all = np.minimum(lim_all, lim)
			dl = np.clip(rho - np.maximum(lim_all, self.SKIN_CLIP_MIN), 0.0, None)
			out.append((b, a, self._skin_shrink(S, K, M, O, u, rho, dl)))
		return out

	def _skin_shrink(self, S, K, M, O, u, rho, dl):
		"""輪の点を中心 O へ dl (K,M) だけ縮める。dl はならす（前後・まわりの最大をとってから＝ならしても足りなくならない）。
		付着部の近くは弱める（(1 − 端の輪へ寄せる重み)^SKIN_CLIP_END_POW。付着部の広がりを残す）。"""
		from scipy.ndimage import gaussian_filter1d, maximum_filter
		if (dl > 1e-4).any():
			sg = float(self.SKIN_CLIP_SIG[0])
			sk = max(sg, float(self.SKIN_CLIP_SIG[1]) * (K - 1))
			wj, wk = 2 * int(round(sg)) + 1, 2 * int(round(sk)) + 1
			dlx = maximum_filter(dl, size=(wk, wj), mode=("nearest", "wrap"))
			dlx = gaussian_filter1d(dlx, sg, axis=1, mode="wrap")
			dlx = gaussian_filter1d(dlx, sk, axis=0, mode="nearest")
			dl = np.maximum(dl, dlx)
		dl = dl * ((1.0 - self._blend_w(K)) ** float(self.SKIN_CLIP_END_POW))[:, None]
		rfin = np.maximum(rho - dl, np.minimum(rho, self.SKIN_CLIP_MIN))
		S = np.array(S, float)
		S[:K * M] = (O[:, None] + u * rfin[:, :, None]).reshape(-1, 3)
		return S

	@staticmethod
	def _skin_tris(a):
		"""皮のメッシュの三角形 (n,3)（つながりは変わらないので一度だけ取り出す）。"""
		t_ = a.get("skin_tris")
		if t_ is None:
			from vtk.util.numpy_support import vtk_to_numpy
			cells = vtk_to_numpy(a["skin_pd"].GetPolys().GetData()).astype(np.int64)
			tris, i_ = [], 0
			while i_ < len(cells):
				n_ = int(cells[i_])
				ids = cells[i_ + 1:i_ + 1 + n_]
				tris.extend((ids[0], ids[k], ids[k + 1]) for k in range(1, n_ - 1))
				i_ += n_ + 1
			t_ = a["skin_tris"] = np.asarray(tris, np.uint32)
		return t_

	def _skin_clip_mutual(self, skins, origins):
		"""皮どうしの重なり: もう一方の皮の中に入った輪の点を、輪の中心へ、重なりの SKIN_MUT_FRAC だけ縮める（両方とも＝押し合って平らに）。
		中心の線ごと離すだけでは、骨から離すのと押し合って重なりが残った（実データ PL→AM の筒の部分 20〜27%）。"""
		import open3d as o3d
		scs = []
		for (b, a, S) in skins:
			sc = o3d.t.geometry.RaycastingScene()
			sc.add_triangles(o3d.core.Tensor(np.asarray(S, np.float32)), o3d.core.Tensor(self._skin_tris(a)))
			scs.append(sc)
		out = []
		for i, (b, a, S) in enumerate(skins):
			K, M = int(a.get("Ks", a["K"])), int(a["M"])
			O = np.asarray(origins[i], float)
			P = S[:K * M].reshape(K, M, 3)
			U = P - O[:, None]
			rho = np.linalg.norm(U, axis=2)
			u = U / np.maximum(rho, 1e-9)[:, :, None]
			dl = np.zeros((K, M))
			for j, sc in enumerate(scs):
				if j == i:
					continue
				occ = sc.compute_occupancy(o3d.core.Tensor(P.reshape(-1, 3).astype(np.float32)), nsamples=3).numpy().reshape(K, M) > 0.5
				if not occ.any():
					continue
				oin = sc.compute_occupancy(o3d.core.Tensor(O.astype(np.float32)), nsamples=3).numpy() > 0.5
				rays = np.c_[np.repeat(O, M, axis=0), u.reshape(-1, 3)].astype(np.float32)
				th = sc.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy().astype(float).reshape(K, M)
				ov = np.where(occ & np.isfinite(th) & ~oin[:, None], np.clip(rho - th, 0.0, None), 0.0)
				dl = np.maximum(dl, self.SKIN_MUT_FRAC * ov + np.where(ov > 0, 0.05, 0.0))
			out.append((b, a, self._skin_shrink(S, K, M, O, u, rho, dl)))
		return out

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

	def _actors(self):
		"""画面の部品の一覧（名前 → アクター）。更新の間は1回だけ取り出したものを使う（呼ぶたびに作り直すので遅い）。"""
		a_ = self.__dict__.get("_acts_now")
		if a_ is not None:
			return a_
		return self.plotter.renderer.actors if self.plotter is not None else {}

	def update(self, fi, render=False):
		try:
			self._acts_now = self.plotter.renderer.actors if self.plotter is not None else None
		except Exception:
			self._acts_now = None
		try:
			return self._update_impl(fi, render)
		finally:
			self._acts_now = None

	def _update_impl(self, fi, render=False):
		try:
			self.fi = int(fi)
			fi = max(0, min(int(fi), self.n_view - 1)) if self.n_view else 0
			jobs = []
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
						elif self._flat_fits.get((si_e, "mesh")):
							Te = self.poses[si_e][fi]
							Pm_ = (W[:, col_] - Te[:3, 3]) @ Te[:3, :3]
							W[:, col_] = self._flat_map(si_e, Pm_) @ Te[:3, :3].T + Te[:3, 3]
				if a["fib_act"].GetVisibility() or a["skin_act"].GetVisibility():
					jobs.append((b, a, W))                       # 線維・皮は、全部の束をまとめて描き直す（下）
			if jobs:
				self._update_lig(jobs, fi)
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
		# 中身はスクロールできる枠に（窓を大きくしなくても下のボタンに届くように。2026-10-21）
		canvas = tk.Canvas(win, highlightthickness=0)
		vsb = ttk.Scrollbar(win, orient="vertical", command=canvas.yview)
		canvas.configure(yscrollcommand=vsb.set)
		vsb.pack(side="right", fill="y")
		canvas.pack(side="left", fill="both", expand=True)
		body = ttk.Frame(canvas)
		body_id = canvas.create_window((0, 0), window=body, anchor="nw")
		body.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
		canvas.bind("<Configure>", lambda e: canvas.itemconfigure(body_id, width=e.width))

		def _wheel(e):
			try:
				cls = e.widget.winfo_class()
			except Exception:
				cls = ""
			if cls in ("TSpinbox", "Spinbox", "TCombobox", "Listbox", "TScale", "Scale", "Text"):
				return
			try:
				top_, bot_ = canvas.yview()
				if top_ <= 0.0 and bot_ >= 1.0:
					return
				canvas.yview_scroll(int(-e.delta / 120) if e.delta else 0, "units")
			except Exception:
				pass
		win.bind("<MouseWheel>", _wheel)
		self.w["canvas"] = canvas
		self.w["body"] = body
		top = ttk.LabelFrame(body, text="段階1の付着部の点（目安）")
		top.pack(fill="x", padx=8, pady=(8, 4))
		ttk.Button(top, text="付着部の点のファイルを読み込む…", command=self._choose_guide).pack(side="left", padx=6, pady=4)
		self.w["guide_var"] = tk.BooleanVar(win, value=bool(self.data.get("guide_show", True)))
		ttk.Checkbutton(top, text="点を表示", variable=self.w["guide_var"], command=self._toggle_guide).pack(side="left", padx=6)
		self.w["guide_lbl"] = ttk.Label(top, text=os.path.basename(str(self.data.get("guide_file") or "（まだ読み込んでいません）")),
		                                foreground="gray")
		self.w["guide_lbl"].pack(side="left", padx=6)
		# バックアップ（2026-10-20）
		ttk.Button(top, text="バックアップから読み込む…", command=self._backup_import_clicked).pack(side="right", padx=4, pady=4)
		ttk.Button(top, text="💾 バックアップを書き出す…", command=self._backup_export_clicked).pack(side="right", padx=4, pady=4)

		mid = ttk.LabelFrame(body, text="束（例: ACL を AM・PL の2束、PCL を AL・PM の2束）")
		mid.pack(fill="both", expand=True, padx=8, pady=4)
		self.w["rows"] = ttk.Frame(mid)
		self.w["rows"].pack(fill="both", expand=True, padx=4, pady=4)
		ttk.Button(mid, text="＋ 束を追加", command=self._add_bundle).pack(anchor="w", padx=6, pady=(0, 6))

		ol = ttk.LabelFrame(body, text="付着部の面を描く（骨の上で外周を右クリック。5 点以上。打った点は左クリックで選べます。右ドラッグのズームは従来どおり）")
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
		mv = ttk.LabelFrame(body, text="付着部の楕円を動かす（平行移動。いまの視点で見た上下左右に、楕円の面の中で）")
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
		self.w["stump_var"] = tk.BooleanVar(win, value=False)
		ttk.Checkbutton(mvr, text="この付着部の切り残しを消す（表示だけ・開き直すと反映）", variable=self.w["stump_var"],
		                command=self._stump_toggle).pack(side="left", padx=(12, 0))
		self.w["mv_combo"].bind("<<ComboboxSelected>>", lambda e: self._stump_sync())
		self.w["mv_lbl"] = ttk.Label(mv, text="（束の面を決めると選べます。動かすと計算の結果は外れるので、もう一度「▶ 計算する」）",
		                             foreground="gray")
		self.w["mv_lbl"].pack(anchor="w", padx=6, pady=(0, 4))
		self._mv_refresh_targets()
		self._stump_sync()

		bot = ttk.Frame(body)
		bot.pack(fill="x", padx=8, pady=(4, 8))
		pp = self.params()
		ttk.Label(bot, text="表示:").pack(side="left")
		self.w["view_var"] = tk.StringVar(win, value=str(pp.get("view", "both")))
		for v_, t_ in (("fibers", "線維"), ("skin", "楕円の皮"), ("both", "両方"), ("foot", "付着部の楕円だけ")):
			ttk.Radiobutton(bot, text=t_, value=v_, variable=self.w["view_var"], command=self._set_view).pack(side="left")
		ttk.Label(bot, text="　自然長 ×").pack(side="left")
		self.w["scale_var"] = tk.StringVar(win, value=f"{100.0 * float(pp.get('rest_scale', 1.0)):g}")
		ttk.Spinbox(bot, from_=80, to=120, increment=1, width=5, textvariable=self.w["scale_var"]).pack(side="left")
		ttk.Label(bot, text="%（自然長の基準のコマで張った長さに対して）").pack(side="left")
		self.w["grav_var"] = tk.BooleanVar(win, value=float(pp.get("gravity", 0.02)) > 0)
		ttk.Checkbutton(bot, text="たるみを下へ（弱い重さ）", variable=self.w["grav_var"],
		                command=self._grav_toggle).pack(side="left", padx=8)
		self.w["outline_var"] = tk.BooleanVar(win, value=bool(pp.get("show_outline", True)))
		ttk.Checkbutton(bot, text="付着部の楕円", variable=self.w["outline_var"],
		                command=lambda: self._set_param_view("show_outline", bool(self.w["outline_var"].get()))).pack(side="left", padx=4)
		self.w["xsec_var"] = tk.BooleanVar(win, value=bool(pp.get("show_xsec", True)))
		ttk.Checkbutton(bot, text="断面のめやす", variable=self.w["xsec_var"],
		                command=lambda: self._set_param_view("show_xsec", bool(self.w["xsec_var"].get()))).pack(side="left", padx=4)
		ttk.Button(bot, text="▶ 計算する", command=self._compute_clicked).pack(side="right", padx=4)
		# 自然長の基準のコマ・束どうしの当たり（2026-10-20。どちらも計算の結果が変わる）
		bot3 = ttk.Frame(body)
		bot3.pack(fill="x", padx=8, pady=(0, 4))
		ttk.Label(bot3, text="自然長の基準:").pack(side="left")
		self.w["ref_lbl"] = ttk.Label(bot3, text="", width=38)
		self.w["ref_lbl"].pack(side="left", padx=(2, 4))
		ttk.Button(bot3, text="いまのコマを基準にする", command=self._rest_here).pack(side="left", padx=2)
		ttk.Button(bot3, text="自動（初期状態）に戻す", command=self._rest_reset).pack(side="left", padx=2)
		self.w["contact_var"] = tk.BooleanVar(win, value=bool(pp.get("bundle_contact", True)))
		ttk.Checkbutton(bot3, text="束どうしの当たり（外形どうしがめり込まない）", variable=self.w["contact_var"],
		                command=self._contact_toggle).pack(side="left", padx=(14, 4))
		ttk.Label(bot3, text="（どちらも変えると計算し直しが要ります）", foreground="gray").pack(side="left", padx=4)
		# 皮を入力した断面で描く・付着部の切り残しを消す（どちらも表示だけ。2026-10-21）
		bot4 = ttk.Frame(body)
		bot4.pack(fill="x", padx=8, pady=(0, 4))
		self.w["rx_var"] = tk.BooleanVar(win, value=bool(pp.get("skin_input_dims", True)))
		ttk.Checkbutton(bot4, text="皮の太さは入力した断面（長径・短径）で描く（断面が空の束は線維の広がりで）", variable=self.w["rx_var"],
		                command=self._rx_toggle).pack(side="left")
		self.w["sep_var"] = tk.BooleanVar(win, value=bool(pp.get("skin_separate", True)))

		def _sep_toggle():
			self.data.setdefault("params", {})["skin_separate"] = bool(self.w["sep_var"].get())
			self._save()
			self.update(self.fi, render=True)
		ttk.Checkbutton(bot4, text="皮どうしを重ねない（押し合って平らに）", variable=self.w["sep_var"],
		                command=_sep_toggle).pack(side="left", padx=(10, 0))
		ttk.Label(bot4, text="　切り残しを消す付着部:").pack(side="left")
		self.w["stump_lbl"] = ttk.Label(bot4, text="", foreground="#0b5394")
		self.w["stump_lbl"].pack(side="left", padx=2)
		self._stump_label()
		self._rest_label()
		# 靭帯（線維・皮）だけの不透明度（骨・付着部の楕円は変えない。2026-10-20）
		bot2 = ttk.Frame(body)
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
		self.w["push_var"] = tk.BooleanVar(win, value=self._skin_push_on())

		def _push_toggle():
			self.data.setdefault("params", {})["skin_push"] = bool(self.w["push_var"].get())
			self._save()
			self.update(self.fi, render=True)
		ttk.Checkbutton(bot2, text="骨が靭帯を突き抜けないように（皮・線維を見えている骨の外へ。表示だけ）",
		                variable=self.w["push_var"], command=_push_toggle).pack(side="left", padx=(12, 4))
		ttk.Label(bot2, text="（「付着部の楕円だけ」で靭帯を隠し、付着部の楕円だけを出せます）", foreground="gray").pack(side="left", padx=6)
		self.w["status"] = ttk.Label(body, text="", foreground="gray", wraplength=1000, justify="left")
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

	# ---------------------------------------------------------------- 自然長の基準のコマ・束どうしの当たり
	def _rec_id(self):
		"""この記録（タブ）の指紋。束のデータは骨リストを共有するタブで共有されるが、自然長の基準のコマは記録ごとに違うので、
		記録ごとに覚える（骨の姿勢の始め・終わりとコマの数から）。"""
		rid = self.__dict__.get("_rid")
		if rid is None:
			h = hashlib.sha1()
			h.update(f"{self.n_view}|{self.s0}|".encode())
			for P_ in self.poses:
				h.update(np.round(np.asarray(P_[[0, -1]], float), 2).tobytes())
			rid = self._rid = h.hexdigest()[:12]
		return rid

	def _rest_manual(self):
		"""この記録で手で選んだ自然長の基準のコマ（絶対のコマ番号）。無ければ None。"""
		g = (self.params().get("rest_frames") or {}).get(self._rec_id())
		return int(g) if g is not None else None

	def _auto_rest(self):
		"""初期状態の終わりの 1 秒前のコマ（絶対）。試験の段階（発表用の表示の phase_at）から:
		最初に前方（+1）か後方（−1）の引張が始まる前の、初期状態（0）の続き。見つからなければ None。"""
		memo = self.__dict__.get("_auto_rest_memo", "none")
		if not (isinstance(memo, str) and memo == "none"):
			return memo
		val = None
		try:
			fn = (self.scene.get("pres_overlay") or {}).get("phase_at")
			if callable(fn) and self.n_view > 10:
				codes = []
				for fi in range(self.n_view):
					r_ = fn(fi)
					codes.append(None if r_ is None else int(r_[0]))
				pulls = [i for i, c in enumerate(codes) if c in (1, -1)]
				if pulls:
					fa = pulls[0]
					known = [c for c in codes[:fa] if c is not None]
					if fa > 1 and known and all(c == 0 for c in known):
						times = np.asarray(self.scene.get("frame_times") or [], float)[:self.n_view]
						dt = float(np.median(np.diff(times))) if len(times) > 2 else 1.0 / 30.0
						back = int(round(1.0 / max(dt, 1e-3)))
						fi = max(0, fa - 1 - back)
						val = int(self.s0 + fi)
		except Exception as e:
			print(f"[靭帯の束] 初期状態を見つけられません（表示の始まりのコマを自然長の基準にします）: {e}")
			val = None
		self._auto_rest_memo = val
		return val

	def _rest_frame(self):
		"""この記録の、自然長の基準のコマ（絶対のコマ番号）: 手で選んだコマ → 初期状態（自動）→ None（表示の始まりのコマ）。"""
		g = self._rest_manual()
		return g if g is not None else self._auto_rest()

	# ---------------------------------------------------------------- たるみ・皮の描き方・切り残し（画面）
	def _grav_toggle(self):
		"""「たるみを下へ」: 計算済みの結果があればすぐ切り替える。無ければ計算し直しを促す。"""
		p = self.data.setdefault("params", {})
		p["gravity"] = 0.02 if bool(self.w["grav_var"].get()) else 0.0
		self._save()
		self._load_cached_results()
		ready = [b for b in self.data["bundles"] if self._ready(b)]
		self._rebuild_actors()
		self._notify()
		self._rows()
		if ready and all(self.results.get(b["id"]) is not None for b in ready):
			self._say(f"たるみを下へ {'あり' if p['gravity'] > 0 else 'なし'} の計算済みの結果に切り替えました。", "#1b5e20")
		else:
			self._say(f"たるみを下へ {'あり' if p['gravity'] > 0 else 'なし'} はまだ計算していません。「▶ 計算する」を押してください。", "#a06000")

	def _rx_toggle(self):
		self.data.setdefault("params", {})["skin_input_dims"] = bool(self.w["rx_var"].get())
		self._save()
		self.update(self.fi, render=True)

	def _stump_label(self):
		lb = self.w.get("stump_lbl")
		if lb is None:
			return
		items = []
		for b in self.data.get("bundles", []):
			for end in ("a", "b"):
				if self._stump_on(b, end):
					inf = self._flat_info.get((b.get("name"), end))
					tail = f"（{inf[0]:.1f} mm 下げた）" if inf else "（開き直すと反映）"
					items.append(f"{b.get('name', '')}・{(b.get(end) or {}).get('bone_name', '')}側{tail}")
		lb.config(text="、".join(items) if items else "なし")

	def _stump_sync(self):
		b, end = self._mv_selected()
		try:
			self.w["stump_var"].set(bool(b is not None and self._stump_on(b, end)))
		except Exception:
			pass

	def _stump_toggle(self):
		b, end = self._mv_selected()
		if b is None:
			self._say("付着部を選んでください（「動かす楕円」の一覧）。", "#a06000")
			self._stump_sync()
			return
		on = bool(self.w["stump_var"].get())
		st = b.setdefault("stump", {})
		st[end] = on
		self._save()
		self._stump_label()
		self._say(f"{self.w['mv_target'].get()} の切り残しを{'消す' if on else '消さない'}ようにしました。"
		          "可視化を開き直すと反映されます（計算し直しは要りません）。", "blue")

	def _rest_label(self):
		lb = self.w.get("ref_lbl")
		if lb is None:
			return
		g = self._rest_frame()
		if g is None:
			lb.config(text="表示の始まりのコマ（初期状態が見つからない）", foreground="black")
			return
		times = list(self.scene.get("frame_times") or [])
		fi = int(g) - self.s0
		ts = f"・{float(times[fi]):.1f} 秒" if 0 <= fi < len(times) else ""
		if self._rest_manual() is not None:
			lb.config(text=f"コマ {fi}{ts}（手で選んだ）", foreground="#0b5394")
		else:
			lb.config(text=f"初期状態（自動）コマ {fi}{ts}", foreground="#1b5e20")

	def _rest_here(self):
		g = int(self.s0 + max(0, min(int(self.fi), self.n_view - 1)))
		self.data.setdefault("params", {}).setdefault("rest_frames", {})[self._rec_id()] = g
		self._save()
		self._invalidate(rows=True)
		self._rest_label()
		self._say("いまのコマ（その時の、骨を避けて張った長さ × 自然長の %）を自然長にしました。「▶ 計算する」で計算し直してください。", "blue")

	def _rest_reset(self):
		(self.data.setdefault("params", {}).get("rest_frames") or {}).pop(self._rec_id(), None)
		self._save()
		self._invalidate(rows=True)
		self._rest_label()
		self._say("自然長の基準を自動（初期状態の終わりの 1 秒前）に戻しました。「▶ 計算する」で計算し直してください。", "blue")

	def _contact_toggle(self):
		self.data.setdefault("params", {})["bundle_contact"] = bool(self.w["contact_var"].get())
		self._save()
		self._invalidate(rows=True)
		self._say("束どうしの当たりを変えました。「▶ 計算する」で計算し直してください。", "blue")

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

	# ---------------------------------------------------------------- バックアップ（書き出す・読み込む）
	BACKUP_FORMAT = "ankle_lig_bundles_backup"

	def _base_dir(self):
		return self.store.parents[2]                      # FRS-SIMULATOR.py のフォルダ

	def _result_file(self):
		"""いまの束の計算の結果のファイル（全部の束が計算済みのときだけ）。無ければ None。"""
		ready = [b for b in self.data["bundles"] if self._ready(b)]
		if not ready or any(self.results.get(b["id"]) is None for b in ready):
			return None, None
		key = self._key(ready)
		fp = self.res_dir / f"lb_{key}.npz"
		return (fp, key) if fp.exists() else (None, None)

	def export_backup(self, fp):
		"""いまの束を zip 1つに書き出す。返り値: 説明の辞書（info）。"""
		import zipfile
		fp = Path(fp)
		rfp, rkey = self._result_file()
		ready = [b for b in self.data["bundles"] if self._ready(b)]
		info = {"format": self.BACKUP_FORMAT, "format_version": 1, "version": VERSION,
		        "created": time.strftime("%Y-%m-%d %H:%M:%S"), "store_key": self.store.stem,
		        "bones": self.names, "s0": self.s0, "n_frames": int(self.n_view),
		        "bundles": [{"name": b.get("name", ""), "n_fibers": int(b.get("n_fibers", 10)),
		                     "a": (b.get("a") or {}).get("bone_name", ""), "b": (b.get("b") or {}).get("bone_name", ""),
		                     "faces": sum(1 for e in ("a", "b") if (b.get(e) or {}).get("ellipse"))}
		                    for b in self.data["bundles"]],
		        "n_ready": len(ready), "result_key": rkey, "has_results": rfp is not None, "guide": ""}
		gf = str(self.data.get("guide_file") or "")
		fp.parent.mkdir(parents=True, exist_ok=True)
		tmp = fp.with_name(fp.stem + ".tmp.zip")
		with zipfile.ZipFile(str(tmp), "w", compression=zipfile.ZIP_DEFLATED) as z:
			z.writestr("bundles.json", json.dumps(self.data, ensure_ascii=False, indent=1))
			if rfp is not None:
				z.write(str(rfp), "results.npz")
			if gf and os.path.isfile(gf):
				info["guide"] = os.path.basename(gf)
				z.write(gf, "guide/" + os.path.basename(gf))
			z.writestr("info.json", json.dumps(info, ensure_ascii=False, indent=1))
		os.replace(str(tmp), str(fp))
		return info

	def import_backup(self, fp, auto_backup=True):
		"""zip から束を読み込んで、いまの束と置き換える。返り値: (info, 結果を戻せたか, 自動バックアップのパス, 骨の無い名前)。"""
		import zipfile
		with zipfile.ZipFile(str(fp), "r") as z:
			names = set(z.namelist())
			info = json.loads(z.read("info.json").decode("utf-8")) if "info.json" in names else {}
			if "bundles.json" not in names or info.get("format", self.BACKUP_FORMAT) != self.BACKUP_FORMAT:
				raise ValueError("靭帯の束のバックアップのファイルではありません")
			data = json.loads(z.read("bundles.json").decode("utf-8"))
			res_bytes = z.read("results.npz") if "results.npz" in names else None
			guides = {n_: z.read(n_) for n_ in names if n_.startswith("guide/") and not n_.endswith("/")}
		if not isinstance(data, dict) or not isinstance(data.get("bundles"), list):
			raise ValueError("束の定義が読めません")
		data.setdefault("params", {})
		auto = None
		if auto_backup and self.data.get("bundles"):
			auto = self.store.parent / "backup" / f"自動_読み込む前_{self.store.stem}_{time.strftime('%Y%m%d_%H%M%S')}.zip"
			self.export_backup(auto)
		if self.st.get("mode"):
			self._ol_cancel(quiet=True)
		self.__dict__.pop("_mv_undo_store", None)
		# 段階1の目安の点のファイル: 元の場所に無ければ、zip の中のものを cache/ankle_lig_bundles/guides/ へ出して使う
		gf = str(data.get("guide_file") or "")
		if guides and not (gf and os.path.isfile(gf)):
			gdir = self.store.parent / "guides"
			gdir.mkdir(parents=True, exist_ok=True)
			n_, by = next(iter(guides.items()))
			out = gdir / os.path.basename(n_)
			out.write_bytes(by)
			data["guide_file"] = str(out)
		self.data = data
		self._refit_old()
		self._save()
		# 計算の結果: 骨の形・動き・設定・束が書き出したときと同じなら戻す
		got = False
		ready = [b for b in self.data["bundles"] if self._ready(b)]
		if res_bytes is not None and ready and info.get("result_key") and self._key(ready) == info.get("result_key"):
			try:
				self.res_dir.mkdir(parents=True, exist_ok=True)
				dst = self.res_dir / f"lb_{info['result_key']}.npz"
				if not dst.exists():
					tmp = self.res_dir / "lb_tmp_import.npz"
					tmp.write_bytes(res_bytes)
					os.replace(str(tmp), str(dst))
			except Exception as e:
				print(f"[靭帯の束] バックアップの計算の結果を戻せません: {e}")
		self._load_cached_results()
		got = bool(ready) and all(self.results.get(b["id"]) is not None for b in ready)
		missing = sorted({(b.get(e) or {}).get("bone_name", "") for b in self.data["bundles"] for e in ("a", "b")
		                  if (b.get(e) or {}).get("bone_name") and self._si_of_name((b.get(e) or {}).get("bone_name")) is None})
		try:
			self._load_guides()
		except Exception:
			pass
		self._rebuild_actors()
		self._notify()
		try:
			self._rows()
			self._mv_refresh_targets()
		except Exception:
			pass
		return info, got, auto, missing

	def _backup_export_clicked(self):
		from tkinter import filedialog, messagebox
		if not self.data.get("bundles"):
			self._say("書き出す束がありません。", "#a06000")
			return
		pp = self.data.setdefault("params", {})
		idir = str(pp.get("backup_dir") or "") or str(self._base_dir() / "backup" / "lig_bundles_store")
		try:
			os.makedirs(idir, exist_ok=True)
		except Exception:
			pass
		nm = "_".join(str(b.get("name", "")) for b in self.data["bundles"])[:40]
		nm = "".join(ch if (ch.isalnum() or ch in "()_-") else "_" for ch in nm)
		fp = filedialog.asksaveasfilename(parent=self.win, title="靭帯の束のバックアップを書き出す", initialdir=idir,
		                                  initialfile=f"靭帯の束_{nm}_{time.strftime('%Y%m%d_%H%M%S')}.zip",
		                                  defaultextension=".zip", filetypes=[("靭帯の束のバックアップ (zip)", "*.zip")])
		if not fp:
			return
		try:
			info = self.export_backup(fp)
		except Exception as e:
			traceback.print_exc()
			messagebox.showerror("バックアップ", f"書き出せませんでした:\n{e}", parent=self.win)
			return
		pp["backup_dir"] = os.path.dirname(fp)
		self._save()
		res = ("計算の結果も入れました" if info["has_results"]
		       else "計算の結果は入っていません（まだ計算していない・計算し直しが要る束がある）")
		self._say(f"束 {len(info['bundles'])} 個を書き出しました（{res}）: {fp}", "#1b5e20")

	def _backup_import_clicked(self):
		import zipfile
		from tkinter import filedialog, messagebox
		pp = self.data.get("params") or {}
		idir = str(pp.get("backup_dir") or "") or str(self._base_dir() / "backup" / "lig_bundles_store")
		fp = filedialog.askopenfilename(parent=self.win, title="靭帯の束のバックアップを読み込む", initialdir=idir,
		                                filetypes=[("靭帯の束のバックアップ (zip)", "*.zip"), ("すべてのファイル", "*.*")])
		if not fp:
			return
		try:
			with zipfile.ZipFile(fp, "r") as z:
				info = json.loads(z.read("info.json").decode("utf-8")) if "info.json" in z.namelist() else {}
		except Exception as e:
			messagebox.showerror("バックアップ", f"読めませんでした:\n{e}", parent=self.win)
			return
		lst = "\n".join(f"・{b.get('name', '')}（{b.get('a', '')}→{b.get('b', '')}、面 {b.get('faces', 0)}/2）"
		                for b in (info.get("bundles") or [])[:12])
		msg = (f"{os.path.basename(fp)}（{info.get('created', '日時不明')}）の束 {len(info.get('bundles') or [])} 個で、"
		       f"いまの束 {len(self.data.get('bundles') or [])} 個を置き換えます。\n\n{lst}\n\n"
		       "いまの束は、置き換える前に自動でバックアップを書き出します"
		       "（cache/ankle_lig_bundles/backup/）。よろしいですか？")
		if not messagebox.askyesno("バックアップから読み込む", msg, parent=self.win):
			return
		try:
			info, got, auto, missing = self.import_backup(fp)
		except Exception as e:
			traceback.print_exc()
			messagebox.showerror("バックアップ", f"読み込めませんでした（いまの束はそのまま）:\n{e}", parent=self.win)
			return
		self.data.setdefault("params", {})["backup_dir"] = os.path.dirname(fp)
		self._save()
		# 表示の設定も読み込んだものにするため、画面を開き直す
		self._close_window()
		self.open_window()
		parts = [f"束 {len(self.data['bundles'])} 個を読み込みました。"]
		if got:
			parts.append("計算の結果も戻しました（計算し直しは要りません）。")
		elif any(self._ready(b) for b in self.data["bundles"]):
			parts.append("計算の結果は使えません（このタブの骨の形・動きが書き出したときと違う、または結果が入っていない）。「▶ 計算する」を押してください。")
		if missing:
			parts.append(f"このタブに無い骨: {', '.join(missing)}（その束は出ません）。")
		if auto:
			parts.append(f"前の束は {auto.name} に書き出してあります。")
		self._say(" ".join(parts), "#1b5e20" if not missing else "#a06000")

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
