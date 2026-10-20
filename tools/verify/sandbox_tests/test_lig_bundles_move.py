# -*- coding: utf-8 -*-
"""靭帯の束（段階2）（2026-10-20）: 付着部の楕円の平行移動（画面の上下左右・楕円の面の中・外周も骨の面へ・計算の結果は外れる・
動かす前に戻す・保存）と、靭帯の不透明度 0%。接触の計算はダミー。2〜3分。"""
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
# 不透明度 0%
ctx.w["op_scale"].set(0); pump(app, 0.2)
a = ctx.actors[b["id"]]
check(a["fib_act"].GetProperty().GetOpacity() == 0.0 and a["skin_act"].GetProperty().GetOpacity() == 0.0 and ctx.w["op_lbl"].cget("text") == "0 %",
      "靭帯の不透明度を 0% まで下げられる（線維・皮とも 0）")
ctx.w["op_scale"].set(100); pump(app, 0.2)
# 平行移動
mvf = [w for w in walk(ctx.win) if w.winfo_class() == "TLabelframe" and "付着部の楕円を動かす" in str(w.cget("text"))]
check(len(mvf) == 1, "「付着部の楕円を動かす」の欄")
cbx = ctx.w["mv_combo"]
labs = list(cbx.cget("values"))
check(len(labs) == 2 and all("ACL(表示)" in l_ for l_ in labs), f"動かす楕円の候補（{labs}）")
ctx.w["mv_target"].set(labs[1])                     # 2点目（脛骨）
bones = ctx._ensure_bones()
e = b["b"]; si = ctx._si_of_name(e["bone_name"]); idx = ctx.idx_of[si]
c0 = np.asarray(e["ellipse"]["center"], float).copy(); O0 = np.asarray(e["outline"], float).copy()
ra0, rb0 = e["ellipse"]["ra"], e["ellipse"]["rb"]
ring0 = vtk_to_numpy(ctx.actors[b["id"]]["eb_pd"].GetPoints().GetData()).astype(float).copy()
# 視点: 楕円を真上から（画面の右・上が楕円の面に乗る）
import ankle_lig_bundles as _alb
n = _alb.ell_normal(bones, idx, e["ellipse"])
T = ctx.poses[si][ctx.fi]
cw = T[:3, :3] @ c0 + T[:3, 3]
p.camera.focal_point = cw; p.camera.position = cw + 80 * (T[:3, :3] @ n); p.camera.up = tuple(T[:3, :3] @ np.asarray(e["ellipse"]["u"], float))
p.render(); pump(app, 0.2)
ctx.w["mv_step"].set("1.0")
btns = {str(w.cget("text")): w for w in walk(mvf[0]) if w.winfo_class() == "TButton"}
btns["→"].invoke(); pump(app, 0.3); btns["→"].invoke(); pump(app, 0.3)
c1 = np.asarray(b["b"]["ellipse"]["center"], float)
d = c1 - c0
right, up = ctx._screen_dirs()
dr = T[:3, :3] @ d
check(abs(np.linalg.norm(d) - 2.0) < 1e-6 and abs(float(d @ n)) < 1e-6, f"→ を 2 回で 2.0 mm（{np.linalg.norm(d):.3f} mm・面に垂直な分 {float(d @ n):.1e}）")
check(float(dr @ right) / 2.0 > 0.99, f"画面の右向きに動く（右向きの成分 {float(dr @ right):.2f} mm）")
check(b["b"]["ellipse"]["ra"] == ra0 and b["b"]["ellipse"]["rb"] == rb0, "大きさ・向きは変わらない（平行移動）")
O1 = np.asarray(b["b"]["outline"], float)
dist = np.abs(bones.closest(idx, O1)[2])
check(np.median(np.linalg.norm(O1 - O0, axis=1)) > 1.5 and dist.max() < 0.3, f"外周の点も動いて骨の面の上（面からの最大 {dist.max():.3f} mm）")
ring1 = vtk_to_numpy(ctx.actors[b["id"]]["eb_pd"].GetPoints().GetData()).astype(float)
check(np.linalg.norm(ring1.mean(0) - ring0.mean(0)) > 1.5, "画面の輪（赤）も動く")
check(ctx.results.get(b["id"]) is None, "動かすと計算の結果は外れる（もう一度計算が要る）")
import json as _j
saved = _j.load(open(ctx.store, encoding="utf-8"))
sb = [x for x in saved["bundles"] if x["id"] == b["id"]][0]
check(np.allclose(sb["b"]["ellipse"]["center"], c1), "動かした楕円を保存")
btns["↑"].invoke(); pump(app, 0.3)
d2 = T[:3, :3] @ (np.asarray(b["b"]["ellipse"]["center"], float) - c1)
check(float(d2 @ up) > 0.99, f"↑ で画面の上へ（{float(d2 @ up):.2f} mm）")
btns["動かす前に戻す"].invoke(); pump(app, 0.3)
check(np.allclose(b["b"]["ellipse"]["center"], c0) and np.allclose(b["b"]["outline"], O0), "「動かす前に戻す」で元の位置に戻る")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
