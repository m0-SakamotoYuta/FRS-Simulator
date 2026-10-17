# -*- coding: utf-8 -*-
"""靭帯の束（段階2）: 本数の欄に続けて打てる（1文字で入力が切れない）・付着部の楕円の表示/非表示・
断面（長径・短径）から線維の太さを自動・断面のめやす・計算後は束の中ほどが断面の楕円に収まる（2026-10-16）。
接触の計算はダミー。1〜2分。"""
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
b = ctx.data["bundles"][-1]; b["name"] = "ACL(断面)"
si_f, si_t = ctx.names.index("大腿骨"), ctx.names.index("脛骨")
for end, si, key in (("a", si_f, "a"), ("b", si_t, "b")):
	ctx._ol_start(b, end)
	for q in ring(si, G[key]["point"], 3.0, 1.5):
		ctx.add_point(si, q)
	ctx._ol_commit(); pump(app, 0.2)
check(ctx._ready(b), "付着部の面が両方ある")
rw = ctx.w["row_w"][b["id"]]
# 1) 本数の欄に「100」を1文字ずつ
sp = rw["n_fibers"]
sp.focus_force(); pump(app, 0.1)
sp.delete(0, "end"); pump(app, 0.05)
for ch in "100":
	sp.insert("end", ch); pump(app, 0.1)
	if not sp.winfo_exists():
		break
check(sp.winfo_exists() and sp.get() == "100" and int(b.get("n_fibers", 10)) == 10,
      f"本数の欄に 1文字ずつ打っても欄はそのまま（いま「{sp.get() if sp.winfo_exists() else '消えた'}」・まだ反映しない）")
sp.event_generate("<Return>"); pump(app, 0.3)
check(int(b["n_fibers"]) == 100 and sp.winfo_exists(), f"Enter で反映（本数 {b['n_fibers']}）・欄は消えない")
# 2) 断面 6 × 3 → 線維の太さは自動
for key, val in (("xsec_a", "6"), ("xsec_b", "3")):
	w_ = rw[key]; w_.delete(0, "end"); w_.insert("end", val); w_.event_generate("<FocusOut>"); pump(app, 0.2)
r_exp = np.sqrt(0.8 * 6 * 3 / (4 * 100))
check(abs(ctx._rf(b) - r_exp) < 1e-9 and f"{r_exp:.3f}" in str(rw["rlb"].cget("text")) and rw["rlb"].winfo_ismapped(),
      f"断面 6×3 mm・100 本 → 線維の半径は自動 {ctx._rf(b):.3f} mm（表示「{rw['rlb'].cget('text')}」）")
check(not rw["rsp"].winfo_ismapped(), "断面があるときは、線維の太さの手入力の欄は隠す")
# 3) 断面のめやす（計算する前）
ctx._rebuild_actors(); pump(app, 0.2)
a = ctx.actors[b["id"]]
check(a.get("xs_act") is not None and a["xs_act"].GetVisibility(), "断面のめやす（中ほどの楕円）が出る（計算する前から）")
from vtk.util.numpy_support import vtk_to_numpy
P = vtk_to_numpy(a["xs_pd"].GetPoints().GetData()).astype(float); c = P.mean(0)
_u, s_, vt = np.linalg.svd(P - c, full_matrices=False)
ext = [np.ptp((P - c) @ vt[0]), np.ptp((P - c) @ vt[1])]
check(abs(ext[0] - 6.0) < 0.1 and abs(ext[1] - 3.0) < 0.1, f"めやすの楕円は 6×3 mm（{ext[0]:.2f}×{ext[1]:.2f}）")
# 4) 付着部の楕円の表示・非表示（計算し直しにならない）
cb = [w for w in walk(ctx.win) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")) == "付着部の楕円"][0]
cb.invoke(); pump(app, 0.2)
a = ctx.actors[b["id"]]
check(not a["ea_act"].GetVisibility() and not a["eb_act"].GetVisibility() and ctx.params()["show_outline"] is False,
      "「付着部の楕円」を外すと楕円が消える（覚える）")
cb.invoke(); pump(app, 0.2)
check(ctx.actors[b["id"]]["ea_act"].GetVisibility(), "もう一度入れると楕円が出る")
xb = [w for w in walk(ctx.win) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")) == "断面のめやす"][0]
xb.invoke(); pump(app, 0.2)
check(not ctx.actors[b["id"]]["xs_act"].GetVisibility(), "「断面のめやす」も切り替えられる")
xb.invoke(); pump(app, 0.2)
# 5) 計算: 30 本にして、束の中ほどが 6×3 の楕円に収まる
sp.delete(0, "end"); sp.insert("end", "30"); sp.event_generate("<Return>"); pump(app, 0.2)
t0 = time.time(); ok = ctx.compute(parent=ctx.win); pump(app, 0.3)
check(ok and ctx.results.get(b["id"]) is not None, f"計算した（30 本・{time.time() - t0:.0f} 秒）")
r = ctx.results[b["id"]]; rf = ctx._rf(b)
worst = []
for fi in (0, 400, 1200, 1800):
	T = ctx.poses[r["sia"]][fi]; W = ctx._local_at(r, fi) @ T[:3, :3].T + T[:3, 3]
	K = W.shape[1]; k = K // 2; C = W.mean(0)
	tg = C[k + 1] - C[k - 1]; tg /= np.linalg.norm(tg)
	Q = W[:, k] - C[k]; Q = Q - np.outer(Q @ tg, tg)
	_u, _s, vt = np.linalg.svd(Q, full_matrices=False)
	e1 = vt[0]; e2 = np.cross(tg, e1)
	worst.append((np.abs(Q @ e1).max() + rf, np.abs(Q @ e2).max() + rf))
w = np.array(worst)
check(np.all(w[:, 0] <= 3.0 + 0.15) and np.all(w[:, 1] <= 1.5 + 0.15),
      f"束の中ほどの広がり（線維の太さ込みの半分の幅）は 3×1.5 mm 以内（{np.round(w.max(0), 2).tolist()}）")
try:
	ctx.data["params"]["view"] = "both"; ctx._rebuild_actors(); ctx.update(900, render=True)
	a = ctx.actors[b["id"]]; T = ctx.poses[r["sia"]][900]; W = ctx._local_at(r, 900) @ T[:3, :3].T + T[:3, 3]
	c = W.reshape(-1, 3).mean(0)
	for nm, act in p.renderer.actors.items():
		if str(nm).startswith("bone_"):
			act.GetProperty().SetOpacity(0.25)
	p.camera.focal_point = c; p.camera.position = c + np.array([30.0, -10.0, -30.0]); p.renderer.ResetCameraClippingRange(); p.render(); pump(app, 0.3)
	p.screenshot(os.path.join(HERE, "xsec_view.png"))
except Exception as e:
	print("   スクリーンショット失敗", e)
ctx._close_window()
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
