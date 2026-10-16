# -*- coding: utf-8 -*-
"""可視化の右上の数字（骨A→B の相対姿勢）の表示・非表示を再生コントロールで切り替える（2026-10-15）。"""
import importlib.util, os, sys, time
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
frs.messagebox.showwarning = lambda *a, **k: None
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)
def close(v):
	w = v["ctrl"]["window"]; w.tk.call(w.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
def overlay_actors(p):
	return [a for a in p.renderer.actors.values() if a.GetClassName().endswith("CornerAnnotation")]
TAB = "261007_FE_軸力100_ACL切"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.ankle_overlay_show.set(True)
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
ov = v["ctrl"].get("ov_var")
cb = [w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TCheckbutton" and "右上の数字" in str(w.cget("text"))]
check(ov is not None and ov.get() and cb, "再生コントロールに「右上の数字…を表示」（最初はオン）")
oa = overlay_actors(p)
check(oa and oa[0].GetVisibility() and "Rx:" in oa[0].GetText(3), "右上に数字が出ている")
cb[0].invoke(); pump(app, 0.3)
check(not ov.get() and not oa[0].GetVisibility(), "外すと右上の数字が消える")
ts = np.asarray(app._ankle_get_current_cache()["timestamps"]); v["seek"](float(ts[500] - ts[0])); pump(app, 0.3)
check(not oa[0].GetVisibility(), "コマを動かしても消えたまま")
check(app.ankle_overlay_show.get() is False, "選んだ状態はタブ（と共有）に覚える")
cb[0].invoke(); pump(app, 0.3)
check(ov.get() and oa[0].GetVisibility() and "Rx:" in oa[0].GetText(3), "入れると、いまのコマの数字がまた出る")
cb[0].invoke(); pump(app, 0.2)
close(v)
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
check(not v["ctrl"]["ov_var"].get() and not any(a.GetVisibility() for a in overlay_actors(p)), "次に開くと、前回の選択（消した状態）で始まる")
close(v)
app.ankle_overlay_show.set(True)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
