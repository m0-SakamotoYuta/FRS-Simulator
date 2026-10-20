# -*- coding: utf-8 -*-
"""靭帯の束（段階2）の表示（2026-10-20）: 靭帯（線維・皮）だけの不透明度・「付着部の楕円だけ」（線維・皮・断面のめやすを隠し、
付着部の楕円はチェックによらず出す）・覚える・計算し直さない。接触の計算はダミー。2〜3分。"""
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
key0 = ctx._key([b])
a = ctx.actors[b["id"]]
check(a["fib_act"].GetProperty().GetOpacity() == 1.0 and abs(a["skin_act"].GetProperty().GetOpacity() - 0.55) < 1e-9, "はじめは不透明度 100%（両方のときの皮は 55%）")
sc = ctx.w["op_scale"]
bone_ops0 = {k_: x.GetProperty().GetOpacity() for k_, x in p.renderer.actors.items() if str(k_).startswith("bone_")}
sc.set(40); pump(app, 0.2)
a = ctx.actors[b["id"]]
check(abs(a["fib_act"].GetProperty().GetOpacity() - 0.40) < 1e-6 and abs(a["skin_act"].GetProperty().GetOpacity() - 0.22) < 1e-6
      and ctx.w["op_lbl"].cget("text") == "40 %", f"不透明度 40% → 線維 0.40・皮 0.22（{a['fib_act'].GetProperty().GetOpacity():.2f}・{a['skin_act'].GetProperty().GetOpacity():.2f}）")
check(a["ea_act"].GetProperty().GetOpacity() == 1.0, "付着部の楕円の不透明度は変えない")
bone_ops = {k_: x.GetProperty().GetOpacity() for k_, x in p.renderer.actors.items() if str(k_).startswith("bone_")}
check(bone_ops == bone_ops0, "骨の不透明度は変えない")
sc.event_generate("<ButtonRelease-1>"); pump(app, 0.2)
import json as _j
saved = _j.load(open(ctx.store, encoding="utf-8"))
check(abs(float(saved["params"]["lig_opacity"]) - 0.40) < 1e-6, "離したときに不透明度を覚える（ファイルに保存）")
# 付着部の楕円だけ
cbo = [w for w in walk(ctx.win) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")) == "付着部の楕円"][0]
cbo.invoke(); pump(app, 0.2)                     # 「付着部の楕円」のチェックを外しておく
rb = [w for w in walk(ctx.win) if w.winfo_class() == "TRadiobutton" and str(w.cget("text")) == "付着部の楕円だけ"]
check(len(rb) == 1, "表示に「付着部の楕円だけ」")
rb[0].invoke(); pump(app, 0.3)
a = ctx.actors[b["id"]]
check(not a["fib_act"].GetVisibility() and not a["skin_act"].GetVisibility() and (a.get("xs_act") is None or not a["xs_act"].GetVisibility()),
      "「付着部の楕円だけ」で線維・皮・断面のめやすを隠す")
check(a["ea_act"].GetVisibility() and a["eb_act"].GetVisibility(), "付着部の楕円は（チェックを外していても）出る")
check(ctx._key([b]) == key0 and ctx.results.get(b["id"]) is not None, "計算し直しにならない（結果はそのまま）")
for v_, fib, skin in (("both", True, True), ("fibers", True, False), ("skin", False, True)):
	[w for w in walk(ctx.win) if w.winfo_class() == "TRadiobutton" and str(w.cget("value")) == v_][0].invoke(); pump(app, 0.2)
	a = ctx.actors[b["id"]]
	check(bool(a["fib_act"].GetVisibility()) == fib and bool(a["skin_act"].GetVisibility()) == skin and not a["ea_act"].GetVisibility(),
	      f"表示「{v_}」に戻すと線維 {fib}・皮 {skin}（付着部の楕円はチェックどおり出ない）")
a = ctx.actors[b["id"]]
check(abs(a["skin_act"].GetProperty().GetOpacity() - 0.40) < 1e-6, "作り直しても不透明度 40% のまま（皮だけのときは 0.40）")
cbo.invoke(); pump(app, 0.2)
ctx.data["params"]["lig_opacity"] = 1.0; ctx.data["params"]["view"] = "both"; ctx._save()
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
