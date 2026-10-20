# -*- coding: utf-8 -*-
"""靭帯の束（段階2）（2026-10-20）: 骨が靭帯を突き抜けないように、見えている骨から皮・線維を外へ（表示だけ）。
皮は細かく（長さ方向に補間）・付く骨の端から 1.5 mm より先は SKIN_CLR 以上外・端だけ骨の少し中まで許す・ふたは動かさない・線維は線維の太さ以上外・両端は動かさない・
チェックで切れる・速さ。接触の計算はダミー。2〜3分。"""
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
b = ctx.data["bundles"][-1]; b["name"] = "ACL(表示)"; b["n_fibers"] = 6
si_f, si_t = ctx.names.index("大腿骨"), ctx.names.index("脛骨")
for end, si, key in (("a", si_f, "a"), ("b", si_t, "b")):
	ctx._ol_start(b, end)
	for q in ring(si, G[key]["point"], 3.0, 1.5):
		ctx.add_point(si, q)
	ctx._ol_commit(); pump(app, 0.2)
check(ctx._ready(b), "付着部の面が両方ある")
t0 = time.time(); ok = ctx.compute(parent=ctx.win); pump(app, 0.3)
check(ok and ctx.results.get(b["id"]) is not None, f"計算した（{time.time() - t0:.0f} 秒）")
from vtk.util.numpy_support import vtk_to_numpy
import ankle_lig_bundles as _alb
a = ctx.actors[b["id"]]
K, M = a["K"], a["M"]; rf = a["rf"]; Ks = a["Ks"]
sia, sib = a["r"]["sia"], ctx._si_of_name(b["b"]["bone_name"])
def sd_world(P, si, fi):
	o = ctx._disp_ob(si); T = ctx.poses[si][fi]
	return ctx._ob_closest(o, (P - T[:3, 3]) @ T[:3, :3])[2]
def stats(fi):
	S = vtk_to_numpy(a["skin_pts"].GetData()).astype(float).copy()
	Wf = vtk_to_numpy(a["fib_pts"].GetData()).astype(float).reshape(a["F"], K, 3).copy()
	tube = S[:Ks * M].reshape(Ks, M, 3)
	kmid = slice(int(0.25 * (K - 1)) + 1, int(0.75 * (K - 1)))
	kmid_s = slice(int(0.25 * (Ks - 1)) + 1, int(0.75 * (Ks - 1)))
	out = {}
	for si in range(len(ctx.poses)):
		if ctx._disp_ob(si) is None: continue
		out[si] = dict(mid=float(sd_world(tube[kmid_s].reshape(-1, 3), si, fi).min()),
		               all=float(sd_world(tube.reshape(-1, 3), si, fi).min()),
		               nin=int((sd_world(tube.reshape(-1, 3), si, fi) < 0).sum()),
		               fmid=float(sd_world(Wf[:, kmid].reshape(-1, 3), si, fi).min()),
		               fall=float(sd_world(Wf[:, 1:-1].reshape(-1, 3), si, fi).min()))
	return S, out
check(ctx._skin_push_on(), "既定で ON")
pw = [w for w in walk(ctx.win) if w.winfo_class() == "TCheckbutton" and "突き抜けない" in str(w.cget("text"))]
check(len(pw) == 1, "「骨が靭帯を突き抜けないように」のチェック")
check(Ks == (K - 1) * _alb.Stage2.SKIN_UP + 1 and M == _alb.Stage2.SKIN_M, f"皮は細かく（輪 {Ks}＝線維の点 {K} の間を補間・まわり {M} 点）")
# 端から 1.5 mm より先は、付く骨にも 0.25 mm 以上外
def far_from_end(fi):
	S = vtk_to_numpy(a["skin_pts"].GetData()).astype(float)[:Ks * M].reshape(Ks, M, 3)
	Cl = S.mean(axis=1); sl = np.r_[0, np.cumsum(np.linalg.norm(np.diff(Cl, axis=0), axis=1))]
	ok = (sl > _alb.Stage2.SKIN_SINK_LEN + 0.3) & (sl < sl[-1] - _alb.Stage2.SKIN_SINK_LEN - 0.3)
	return min(float(sd_world(S[ok].reshape(-1, 3), si, fi).min()) for si in range(len(ctx.poses)) if ctx._disp_ob(si) is not None)
res_on, res_off = [], []
for fi in (0, ctx.n_view // 3, 2 * ctx.n_view // 3, ctx.n_view - 1):
	ctx.update(fi, render=True); pump(app, 0.05)
	S1, st1 = stats(fi); res_on.append((fi, S1, st1))
pw[0].invoke(); pump(app, 0.2)
check(not ctx._skin_push_on() and json.load(open(ctx.store, encoding="utf-8"))["params"]["skin_push"] is False, "チェックで切れる（保存）")
for fi, S1, st1 in res_on:
	ctx.update(fi, render=True); pump(app, 0.05)
	S0, st0 = stats(fi); res_off.append((fi, S0, st0))
for (fi, S1, st1), (_f, S0, st0) in zip(res_on, res_off):
	print(f"   コマ {fi}: " + " / ".join(f"{ctx.names[si]} 皮 中ほど {st0[si]['mid']:+.2f}→{st1[si]['mid']:+.2f}・全体 {st0[si]['all']:+.2f}→{st1[si]['all']:+.2f}"
	      f"（中に {st0[si]['nin']}→{st1[si]['nin']} 点）・線維 中ほど {st0[si]['fmid']:+.2f}→{st1[si]['fmid']:+.2f}" for si in st1), flush=True)
tot_in_off = sum(st0[si]["nin"] for (_f, _S, st0) in res_off for si in st0)
mid_on = min(st1[si]["mid"] for (_f, _S, st1) in res_on for si in st1)
all_on = min(st1[si]["all"] for (_f, _S, st1) in res_on for si in st1)
fmid_on = min(st1[si]["fmid"] for (_f, _S, st1) in res_on for si in st1)
check(tot_in_off > 0, f"（切ると皮の点が骨の中にある: {tot_in_off} 点）")
ctx.data["params"]["skin_push"] = True; ctx.update(0, render=True); ff = far_from_end(0); ctx.data["params"]["skin_push"] = False
check(ff >= _alb.Stage2.SKIN_CLR - 0.02, f"ON: 端から {_alb.Stage2.SKIN_SINK_LEN} mm より先は骨の面から {_alb.Stage2.SKIN_CLR} mm 以上外（最小 {ff:+.3f}）")
check(mid_on >= _alb.Stage2.SKIN_CLR - 0.02, f"ON: 皮の中ほどは骨の面から {_alb.Stage2.SKIN_CLR} mm 以上外（最小 {mid_on:+.3f}）")
check(all_on >= -_alb.Stage2.SKIN_SINK - 0.05, f"ON: 端でも骨の中は少しだけ（最小 {all_on:+.3f} mm）")
# 皮の三角形の中（点の間）も骨の外か（付着部の縁から 1.5mm は除く）
import pyvista as _pv
bary = np.array([[1/3, 1/3, 1/3], [0.6, 0.2, 0.2], [0.2, 0.6, 0.2], [0.2, 0.2, 0.6], [0.5, 0.5, 0], [0, 0.5, 0.5]])
def tri_pen(fi):
	sk = _pv.wrap(a["skin_pd"]).triangulate()
	Pt = np.asarray(sk.points, float); Ft = np.asarray(sk.faces).reshape(-1, 4)[:, 1:]
	quad = np.arange(len(Ft)) // 2
	tube = Pt[:Ks * M].reshape(Ks, M, 3); Cl = tube.mean(axis=1)
	sl = np.r_[0, np.cumsum(np.linalg.norm(np.diff(Cl, axis=0), axis=1))]
	okb = (sl[:-1] > 1.5) & (sl[1:] < sl[-1] - 1.5)
	band = np.where(quad < (Ks - 1) * M, quad // M, -1)
	sel = (band >= 0) & okb[np.clip(band, 0, Ks - 2)]
	X = np.einsum("tj,fjd->ftd", bary, Pt[Ft[sel]]).reshape(-1, 3)
	return np.min([sd_world(X, si, fi) for si in range(len(ctx.poses)) if ctx._disp_ob(si) is not None], axis=0)
pen_on, pen_off = [], []
for fi in (0, ctx.n_view // 2, ctx.n_view - 1):
	ctx.data["params"]["skin_push"] = True; ctx.update(fi); pen_on.append(tri_pen(fi))
	ctx.data["params"]["skin_push"] = False; ctx.update(fi); pen_off.append(tri_pen(fi))
ctx.data["params"]["skin_push"] = True
pon, poff = np.concatenate(pen_on), np.concatenate(pen_off)
check((pon < -0.05).mean() < 0.002 and pon.min() > -0.3,
      f"ON: 皮の三角形の中も骨の外（骨の中 {(pon < -0.05).sum()}/{len(pon)} 点・最も深い {pon.min():+.2f} mm。切ると {(poff < -0.05).sum()} 点・{poff.min():+.2f} mm）")
check(fmid_on >= rf - 0.02, f"ON: 線維の中ほどは線維の太さ（{rf:.3f}）以上外（最小 {fmid_on:+.3f}）")
nC = Ks * M
check(all(np.allclose(S1[nC:], S0[nC:]) for (_f, S1, _a), (_g, S0, _b) in zip(res_on, res_off)), "両端のふたは動かさない")
check(all(np.allclose(S1[:M], S0[:M], atol=0.6) for (_f, S1, _a), (_g, S0, _b) in zip(res_on, res_off)), "端の輪はほとんど動かない（0.6 mm 以内）")
mv = max(float(np.linalg.norm(S1[:nC] - S0[:nC], axis=1).max()) for (_f, S1, _a), (_g, S0, _b) in zip(res_on, res_off))
print(f"   皮の点が動いた最大 {mv:.2f} mm", flush=True)
# 速さ
pw[0].invoke(); pump(app, 0.2)
t0 = time.perf_counter()
for fi in range(0, ctx.n_view, max(1, ctx.n_view // 30)):
	ctx.update(fi)
n_ = len(range(0, ctx.n_view, max(1, ctx.n_view // 30)))
dt_on = (time.perf_counter() - t0) / n_ * 1000
pw[0].invoke(); pump(app, 0.2)
t0 = time.perf_counter()
for fi in range(0, ctx.n_view, max(1, ctx.n_view // 30)):
	ctx.update(fi)
dt_off = (time.perf_counter() - t0) / n_ * 1000
check(dt_on < 30, f"1コマの更新 {dt_on:.1f} ms（切ると {dt_off:.1f} ms）")
pw[0].invoke(); pump(app, 0.2)
check(ctx.results.get(b["id"]) is not None, "計算の結果はそのまま（計算し直さない）")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
