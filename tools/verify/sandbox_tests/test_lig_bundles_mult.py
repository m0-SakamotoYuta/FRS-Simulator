# -*- coding: utf-8 -*-
"""靭帯の束（段階2）（2026-10-22）: 線維の見た目の本数（×1〜×20 を数字で。計算は同じ本数のまま）。窓の数字の欄・既定は ×1（計算した線維そのもの）・
切り替えると本数が変わる・覚える・増やした線維は近い計算線維の間（補間）・骨に入らない・1 本は細く。束のデータと計算結果はユーザーのもの
（cache/ankle_lig_bundles の 261007_AP_FE60_軸力100）を使う（消さない）。1〜2分。"""
import importlib.util, os, sys, time, gc, json
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk
from vtk.util.numpy_support import vtk_to_numpy
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: None
store = os.path.join(HERE, "cache", "ankle_lig_bundles", "g8a86693d.json")
keep = open(store, "rb").read() if os.path.exists(store) else None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.ankle_lig_mode.set("stage2")
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
ctx = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
try:
	ok_res = all(ctx.results.get(b["id"]) is not None for b in ctx.data["bundles"])
	check(ok_res, "計算結果がある（ユーザーのデータ）")
	for b in ctx.data["bundles"]: b["visible"] = True
	ctx.data["params"]["view"] = "fibers"; ctx.data["params"].pop("fib_mult", None)
	ctx._rebuild_actors(); pump(app, 0.3)
	ctx.open_window(); pump(app, 0.4)
	sb_m = ctx.w.get("mult_spin")
	check(sb_m is not None and ctx.w["mult_var"].get() == "1" and ctx._fib_mult() == 1, f"「線維の見た目の本数」の数字の欄（既定 ×1）")
	def set_mult(val):
		sb_m.set(str(val)); sb_m.event_generate("<FocusOut>"); pump(app, 0.5)
	b0 = ctx.data["bundles"][0]; a0 = ctx.actors[b0["id"]]
	fi = ctx.fi
	ctx.update(fi)
	r = a0["r"]; T = ctx.poses[r["sia"]][fi]
	W = ctx._local_at(r, fi) @ T[:3, :3].T + T[:3, 3]
	P1 = vtk_to_numpy(a0["fib_pts"].GetData()).astype(float).reshape(a0["Fd"], a0["K"], 3)
	check(a0["Fd"] == a0["F"] == W.shape[0] and float(np.abs(P1[:, 1:-1] - W[:, 1:-1]).max()) < 1.0, f"×1 は計算した線維そのもの（{a0['Fd']} 本・骨の外へ出した分の差 {float(np.abs(P1 - W).max()):.2f} mm）")
	r1 = a0["fib_act"].GetMapper().GetInputConnection(0, 0).GetProducer().GetRadius()
	set_mult(3)
	a3 = ctx.actors[b0["id"]]
	ctx.update(fi)
	P3 = vtk_to_numpy(a3["fib_pts"].GetData()).astype(float).reshape(a3["Fd"], a3["K"], 3)
	r3 = a3["fib_act"].GetMapper().GetInputConnection(0, 0).GetProducer().GetRadius()
	check(a3["Fd"] == 3 * a3["F"] and ctx.params().get("fib_mult") == 3, f"×3 にすると {a3['Fd']} 本（覚える: {ctx.params().get('fib_mult')}）")
	check(abs(r3 - r1 / np.sqrt(3)) < 1e-6, f"1 本は 1/√3 の太さ（{r1:.3f} → {r3:.3f} mm）")
	# 増やした線維は、近い計算線維の間（中ほどの点が計算線維の点の広がりの中）
	mid = W.shape[1] // 2
	from scipy.spatial import cKDTree
	dd = cKDTree(W[:, mid]).query(P3[:, mid])[0]
	sp_ = np.median(cKDTree(W[:, mid]).query(W[:, mid], k=2)[0][:, 1])
	check(float(np.percentile(dd, 95)) < 3 * sp_ + 0.5, f"増やした線維は計算線維の近く（中ほどで 95% が {np.percentile(dd, 95):.2f} mm 以内・線維の間隔 {sp_:.2f} mm）")
	mn = np.full(P3[:, 3:-3].reshape(-1, 3).shape[0], np.inf)
	Q = P3[:, 3:-3].reshape(-1, 3)
	for si in range(len(ctx.poses)):
		o = ctx._disp_ob(si)
		if o is None: continue
		Tb = ctx.poses[si][fi]
		mn = np.minimum(mn, ctx._ob_closest(o, (Q - Tb[:3, 3]) @ Tb[:3, :3])[2])
	check(int((mn < -0.05).sum()) == 0, f"増やした線維も骨に入らない（最小 {mn.min():+.2f} mm）")
	t0 = time.perf_counter(); ctx.update(fi); dt = time.perf_counter() - t0
	print(f"   ×3 の 1 コマの更新 {dt * 1000:.0f} ms", flush=True)
	set_mult(10)
	check(ctx.actors[b0["id"]]["Fd"] == 10 * ctx.actors[b0["id"]]["F"] and ctx.params().get("fib_mult") == 10, f"×10 も入れられる（{ctx.actors[b0['id']]['Fd']} 本）")
	set_mult(99)
	check(ctx.params().get("fib_mult") == ctx.FIB_MULT_MAX and ctx.w["mult_var"].get() == str(ctx.FIB_MULT_MAX), f"上限を超える数は上限に（{ctx.w['mult_var'].get()}）")
	set_mult("あ")
	check(ctx.params().get("fib_mult") == ctx.FIB_MULT_MAX and ctx.w["mult_var"].get() == str(ctx.FIB_MULT_MAX), "数でないものは元に戻す")
	set_mult(1)
	check(ctx.actors[b0["id"]]["Fd"] == ctx.actors[b0["id"]]["F"] and ctx.params().get("fib_mult") == 1, "×1 に戻せる")
finally:
	if keep is not None:
		open(store, "wb").write(keep)               # ユーザーの束のデータは元のまま
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
