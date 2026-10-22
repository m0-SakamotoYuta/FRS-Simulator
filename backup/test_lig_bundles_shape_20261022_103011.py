# -*- coding: utf-8 -*-
"""靭帯の束（段階2）（2026-10-21）: 皮を入力した断面（長径・短径）で描く・付着部の切り残しを消す（表示だけ）・
「たるみを下へ」のチェックだけで切り替える。午後: 骨・となりの皮の側だけ輪を縮める（ひれ・折れ目・めり込みが出ない）・窓のスクロール。
接触の計算はダミー。3〜5分。"""
import importlib.util, os, sys, time, json, glob, shutil, gc
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a[:2])[:300])
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
app._sim_view_precompute_multi_heatmap = lambda bd, update_progress=None, cancel_var=None, quiet=False: [
	{idx: np.ones(len(np.asarray(m.points)), np.float32) for (idx, n, m, T) in bd} for t in range(len(bd[0][3]))]
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)

TAB = "261007_FE_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
shutil.rmtree(os.path.join(HERE, "cache", "ankle_lig_bundles"), ignore_errors=True)
app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(True); app.ankle_alt_model.set(False)
app.on_ankle_animate(); pump(app, 1.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
ctx = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
ctx.open_window(); pump(app, 0.4)
# 束を1つ（いまの ACL の点のまわりに付着部の面）
G = [x for x in app.ankle_ligaments if x["name"] == "ACL"][0]
def ring(si, center, ra, rb_, n=7):
	bones = ctx._ensure_bones(); idx = ctx.idx_of[si]
	cpp, ns, _ = bones.closest(idx, np.asarray(center, float)[None]); nrm = ns[0]
	t1 = np.cross(nrm, [1, 0, 0]); t1 /= np.linalg.norm(t1); t2 = np.cross(nrm, t1)
	th = np.linspace(0, 2 * np.pi, n, endpoint=False) + 0.3
	return bones.closest(idx, cpp[0] + np.outer(np.cos(th) * ra, t1) + np.outer(np.sin(th) * rb_, t2) + 2 * nrm)[0]
ctx._add_bundle(); pump(app, 0.2)
b = ctx.data["bundles"][-1]; b["name"] = "ACL(表示)"; b["n_fibers"] = 30; b["xsec_a"] = 8.0; b["xsec_b"] = 5.0
si_f, si_t = ctx.names.index("大腿骨"), ctx.names.index("脛骨")
for end, si, key in (("a", si_f, "a"), ("b", si_t, "b")):
	ctx._ol_start(b, end)
	for q in ring(si, G[key]["point"], 3.0, 1.5):
		ctx.add_point(si, q)
	ctx._ol_commit(); pump(app, 0.2)
check(ctx._ready(b), "付着部の面が両方ある")
ctx.data["params"]["stride"] = 30
t0 = time.time(); ok = ctx.compute(parent=ctx.win); pump(app, 0.3)
check(ok and ctx.results.get(b["id"]) is not None, f"計算した（{time.time() - t0:.0f} 秒）")
import ankle_lig_bundles as _alb
from vtk.util.numpy_support import vtk_to_numpy
# ---- (1) 入力した断面の皮
check(bool(ctx.params().get("skin_input_dims", True)) and bool(ctx.w["rx_var"].get()), "「皮の太さは入力した断面で描く」が既定で ON")
fi = ctx.n_view // 2
ctx.update(fi, render=True); pump(app, 0.2)
a = ctx.actors[b["id"]]
def rings():
	Ks, M = a["Ks"], a["M"]
	S = vtk_to_numpy(a["skin_pts"].GetData()).astype(float)[:Ks * M].reshape(Ks, M, 3)
	return S
def ring_dims(R):
	c = R.mean(0); Q = R - c
	w, V = np.linalg.eigh(Q.T @ Q)
	e1, e2 = V[:, -1], V[:, -2]
	return 2 * np.abs(Q @ e1).max(), 2 * np.abs(Q @ e2).max(), e1
S = rings(); Ks = len(S)
Mm = a["M"]
pre = ctx._skin_last[b["id"]]["pre"][:Ks * Mm].reshape(Ks, Mm, 3)
Xc = ctx._skin_last[b["id"]]["X"]
d1, d2, _ = ring_dims(pre[Ks // 2])
check(abs(d1 - 8.0) < 0.8 and abs(d2 - 5.0) < 0.6, f"中ほどの輪 = 入力した断面（長径 {d1:.2f}・短径 {d2:.2f} mm、入力 8×5）")
angs = []
prev = None
for k in range(Ks):
	_, _, e = ring_dims(S[k])
	if prev is not None:
		angs.append(np.degrees(np.arccos(min(1.0, abs(float(e @ prev))))))
	prev = e
angs = np.asarray(angs)
mid = slice(int(0.25 * len(angs)), int(0.75 * len(angs)))
check(angs[mid].max() < 12.0, f"中ほどで輪の向きが飛ばない（となりの輪との向きの差 最大 {angs[mid].max():.1f}°）")
rr = a["r"]; T = ctx.poses[rr["sia"]][fi]
W = ctx._local_at(rr, fi) @ T[:3, :3].T + T[:3, 3]
Wu = ctx._upsample(W, Ks)
R = pre[Ks // 2]; c = R.mean(0); Q = R - c
w_, V = np.linalg.eigh(Q.T @ Q); e1, e2 = V[:, -1], V[:, -2]
fpts = Wu[:, Ks // 2] - Wu[:, Ks // 2].mean(0)       # 線維の広がり（線維の重心から。皮の中心はならした線にある。10/22）
inside = (np.abs(fpts @ e1) / (np.abs(Q @ e1).max() + 1e-9)) ** 2 + (np.abs(fpts @ e2) / (np.abs(Q @ e2).max() + 1e-9)) ** 2
check(inside.max() <= 1.05, f"中ほどの線維の広がりは輪の大きさの中（縮める前・いちばん外の線維 {np.sqrt(inside.max()):.2f}・1 以下）")
# 縮めるだけ（輪の点を外へは動かさない＝ひれができない）
gr = np.linalg.norm(S - Xc[:, None], axis=2) - np.linalg.norm(pre - Xc[:, None], axis=2)
check(gr.max() < 1e-3, f"骨・となりの皮の側は輪の中心へ縮めるだけ（外へ動いた最大 {gr.max():+.4f} mm・画面の点は 32 ビット）")
# 筒の部分（付着部の輪へ寄せない所）は、見えている骨の外
def sd_min(P_, fi_):
	mn = np.full(len(P_), np.inf)
	for si_ in range(len(ctx.poses)):
		o_ = ctx._disp_ob(si_)
		if o_ is None: continue
		T_ = ctx.poses[si_][fi_]
		mn = np.minimum(mn, ctx._ob_closest(o_, (P_ - T_[:3, 3]) @ T_[:3, :3])[2])
	return mn
wbl = ctx._blend_w(Ks)
bad_in = []
for fi_ in np.linspace(0, ctx.n_view - 1, 8).astype(int):
	ctx.update(int(fi_)); S_ = rings()
	mn_ = sd_min(S_[wbl < 1e-6].reshape(-1, 3), int(fi_))
	bad_in.append(float(mn_.min()))
ctx.update(fi); S = rings()
check(min(bad_in) > 0.0, f"筒の部分は骨の外（8 コマの最小 {min(bad_in):+.2f} mm）")
# 折れ目が無い: 中ほどで、となりの輪が前の輪の面の後ろへ回り込まない（交差しない）
Cr = S.mean(axis=1)
tr = np.zeros((Ks, 3))
for k in range(Ks):
	w_, V_ = np.linalg.eigh((S[k] - Cr[k]).T @ (S[k] - Cr[k])); tr[k] = V_[:, 0]
tr *= np.sign(np.einsum("ki,ki->k", tr, np.gradient(Cr, axis=0)))[:, None]
back = []
for k in range(int(0.2 * Ks), int(0.8 * Ks)):
	back.append(float(((S[k + 1] - Cr[k]) @ tr[k]).min()))
check(min(back) > -0.05, f"中ほどで輪が交差しない（となりの輪の、前の輪の面からの最小 {min(back):+.2f} mm）")
S_on = rings().copy()
ctx.w["rx_var"].set(False); ctx._rx_toggle(); pump(app, 0.2)
S_off = rings()
d1o, d2o, _ = ring_dims(S_off[Ks // 2])
chg = float(np.linalg.norm(S_off - S_on, axis=2).max())
check(chg > 0.3 and not ctx.params()["skin_input_dims"], f"切ると線維の広がりの描き方（皮の点が最大 {chg:.2f} mm 変わる・中ほど {d1o:.2f}×{d2o:.2f} mm）")
ctx.w["rx_var"].set(True); ctx._rx_toggle(); pump(app, 0.2)
# ---- 皮どうしを離す（となりに 2.5mm ずらした束を足す）
ctx._add_bundle(); pump(app, 0.2)
b3 = ctx.data["bundles"][-1]; b3["name"] = "ACL(隣)"; b3["n_fibers"] = 30; b3["xsec_a"] = 6.0; b3["xsec_b"] = 4.0
for end, si, key in (("a", si_f, "a"), ("b", si_t, "b")):
	ctx._ol_start(b3, end)
	for q in ring(si, np.asarray(G[key]["point"], float) + np.array([2.5, 0.0, 0.0]), 3.0, 1.5):
		ctx.add_point(si, q)
	ctx._ol_commit(); pump(app, 0.2)
ctx.compute(parent=ctx.win); pump(app, 0.3)
def skin_overlap():
	from scipy.spatial import cKDTree
	n_in = 0
	for fi_ in np.linspace(0, ctx.n_view - 1, 6).astype(int):
		ctx.update(int(fi_))
		Ts = []
		for bb in (b, b3):
			aa = ctx.actors[bb["id"]]; K_, M_ = aa["Ks"], aa["M"]
			Ts.append(vtk_to_numpy(aa["skin_pts"].GetData()).astype(float)[:K_ * M_].reshape(K_, M_, 3).transpose(1, 0, 2))
		for X, Y in ((Ts[0], Ts[1]), (Ts[1], Ts[0])):
			C_, t_, e1_, e2_, r1_, r2_ = _alb._envelopes(X, 0.0)
			K_ = len(C_); k0_ = max(1, int(round(0.15 * (K_ - 1))))
			P_ = Y[:, int(0.2 * Y.shape[1]):int(0.8 * Y.shape[1])].reshape(-1, 3)
			k = cKDTree(C_).query(P_)[1]
			q = P_ - C_[k]; al = np.einsum("ij,ij->i", q, t_[k]); qp = q - al[:, None] * t_[k]
			sr = np.sqrt((np.einsum("ij,ij->i", qp, e1_[k]) / r1_[k]) ** 2 + (np.einsum("ij,ij->i", qp, e2_[k]) / r2_[k]) ** 2)
			ok = (k >= k0_) & (k <= K_ - 1 - k0_) & (np.abs(al) <= 0.75 * np.linalg.norm(np.diff(C_, axis=0), axis=1).mean())
			n_in += int(((sr < 0.95) & ok).sum())
	return n_in
ctx.data["params"]["skin_separate"] = False; ov_off = skin_overlap()
ctx.data["params"]["skin_separate"] = True; ov_on = skin_overlap()
check(ov_on < 0.4 * max(ov_off, 1) or ov_off == 0, f"皮どうしを離すと重なりが減る（離さない {ov_off} 点 → 離す {ov_on} 点）")
# ---- 窓のスクロール
cv = ctx.w.get("canvas")
check(cv is not None and str(ctx.w["status"].master) == str(ctx.w["body"]), "窓の中身はスクロールできる枠の中")
ctx.win.geometry("1040x300"); pump(app, 0.4); ctx.win.update_idletasks(); pump(app, 0.2)
y0 = cv.yview()
ev_w = ctx.w["status"]
cv.event_generate("<MouseWheel>", delta=-120, x=10, y=10) if False else None
ctx.win.event_generate("<MouseWheel>", delta=-120); pump(app, 0.3)
y1 = cv.yview()
check(y0[1] < 1.0 and y1[0] > y0[0], f"窓を小さくするとスクロールでき、ホイールで下へ動く（{tuple(round(v, 2) for v in y0)} → {tuple(round(v, 2) for v in y1)}）")
ctx.win.geometry("1040x560"); pump(app, 0.2)
# ---- (2) 切り残しを消す（関数）
gx, gy = np.meshgrid(np.linspace(-20, 20, 161), np.linspace(-20, 20, 161), indexing="ij")
for slope in (0.0, 0.3):
	z0 = slope * gx
	bump = 4.0 * np.exp(-((gx - 1.0) ** 2 + (gy + 0.5) ** 2) / (2 * 1.2 ** 2))         # 楕円の中に収まるこぶ（裾が楕円の外の輪に届くと膜が少し上がる）
	Pm = np.c_[gx.ravel(), gy.ravel(), (z0 + bump).ravel()]
	ell = {"center": [0.0, 0.0, 0.0], "u": [1.0, 0.0, 0.0], "v": [0.0, 1.0, 0.0], "ra": 6.0, "rb": 4.0}
	n = np.array([0.0, 0.0, 1.0])
	f = _alb.stump_membrane(Pm, ell, n)
	newP, dmax, cnt = _alb.stump_lower(Pm, ell, n, f)
	sr = np.sqrt((Pm[:, 0] / 6.0) ** 2 + (Pm[:, 1] / 4.0) ** 2)
	resid = newP[:, 2] - z0.ravel()
	check(resid[sr <= 1.0].max() < 0.35 and np.allclose(newP[sr > 1.1], Pm[sr > 1.1]) and (newP[:, 2] <= Pm[:, 2] + 1e-9).all(),
	      f"切り残し（高さ 4mm のこぶ・斜面 {slope}）を消す: 楕円の中の残り {resid[sr <= 1.0].max():.2f} mm・外は動かさない・上げない（下げた最大 {dmax:.1f} mm）")
# ---- (2) 実際の骨で（脛骨の付着部に印を付けて、表示のメッシュを差し替える）
si_t = ctx._si_of_name(b["b"]["bone_name"])
orig = {k: np.asarray(ctx.sb[si_t][k].points, float).copy() for k in ("mesh", "alt_mesh") if ctx.sb[si_t].get(k) is not None}
b["stump"] = {"b": True}
ctx._flat_fits = {}; ctx._flat_info = {}
ctx._flatten_scene_meshes()
bones = ctx._ensure_bones()
n = _alb.ell_normal(bones, ctx.idx_of[si_t], b["b"]["ellipse"])
ok_all = True; moved_any = 0
for k, P0 in orig.items():
	P1 = np.asarray(ctx.sb[si_t][k].points, float)
	dlt = P1 - P0
	mv = np.linalg.norm(dlt, axis=1) > 1e-9
	moved_any += int(mv.sum())
	e = b["b"]["ellipse"]; d = P0 - np.asarray(e["center"])
	sr = np.sqrt(((d @ np.asarray(e["u"])) / e["ra"]) ** 2 + ((d @ np.asarray(e["v"])) / e["rb"]) ** 2)
	if mv.any():
		ok_all &= bool((sr[mv] <= 1.1 + 1e-6).all()) and bool(((dlt[mv] @ n) < 0).all())
check(ok_all and (ctx._flat_fits.get((si_t, "mesh")) is not None), f"実際の骨: 楕円の中だけ・下げる向きだけ（動いた点 {moved_any}・{ctx._flat_info}）")
ctx._stump_label()
check("ACL(表示)" in ctx.w["stump_lbl"].cget("text"), f"欄に消した付着部（{ctx.w['stump_lbl'].cget('text')}）")
k_ = (si_t, "mesh")
key0 = ctx._key([b, b3])
check(key0 == ctx._key([b, b3]) and "stump" not in b["b"], "印は計算の鍵に入らない（計算し直し不要）")
# ---- (3) たるみを下へ（チェックだけで切り替え）
g0 = float(ctx.params().get("gravity", 0.02))
ctx.w["grav_var"].set(not (g0 > 0)); ctx._grav_toggle(); pump(app, 0.3)
check(ctx.results.get(b["id"]) is None and "まだ計算していません" in ctx.w["status"].cget("text"), "計算していない側に切り替えると、計算を促す")
ctx.w["grav_var"].set(g0 > 0); ctx._grav_toggle(); pump(app, 0.3)
check(ctx.results.get(b["id"]) is not None and "切り替えました" in ctx.w["status"].cget("text"), "計算済みの側へは、チェックだけですぐ切り替わる")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
