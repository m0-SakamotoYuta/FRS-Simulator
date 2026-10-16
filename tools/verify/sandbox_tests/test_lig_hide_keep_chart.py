# -*- coding: utf-8 -*-
"""靭帯を 3D では隠し、長さのグラフには出す（「表示」と「グラフ」のチェックを別に。2026-10-15）。"""
import importlib.util, os, sys, time, copy, gc
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
def walk(w):
	for c in w.winfo_children():
		yield c
		yield from walk(c)
TAB = "261007_FE_軸力100_ACL切"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
L0 = copy.deepcopy(app.ankle_ligaments[0]); L0["name"] = "ACL"; L0.pop("chart", None)
L1 = copy.deepcopy(L0); L1["name"] = "PCL"; L1["color"] = "#1F77B4"
app.ankle_ligaments = [L0, L1]
def lig_charts(p):
	return [c for c in p.renderer._charts if "Ligament" in str(c.y_label)]
def labels(ch):
	return sorted(pl.label for pl in ch.plots() if pl.label)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
lig = next(o for o in gc.get_objects() if isinstance(o, dict) and "open" in o and "update" in o and "close" in o and callable(o.get("open")))
lig["open"](); pump(app, 0.5)
lw = [w for w in walk(app) if isinstance(w, tk.Toplevel) and "靭帯" in w.title()][-1]
hdr = [str(w.cget("text")) for w in walk(lw) if w.winfo_class() == "TLabel" and w.grid_info() and int(w.grid_info()["row"]) == 0]
check("表示" in hdr and "グラフ" in hdr, f"靭帯一覧に「表示」と「グラフ」の2つのチェック（{hdr[:3]}）")
check(labels(lig_charts(p)[0]) == ["ACL", "PCL"], "はじめは両方グラフに出る（前に作った靭帯もグラフに出る）")
cbs = [w for w in walk(lw) if w.winfo_class() == "TCheckbutton" and w.grid_info() and int(w.grid_info()["row"]) == 1]
vis_cb = next(w for w in cbs if int(w.grid_info()["column"]) == 0)
chart_cb = next(w for w in cbs if int(w.grid_info()["column"]) == 1)
tube_actors = [a for a in p.renderer.GetActors()]
vis_cb.invoke(); pump(app, 1.0)       # ACL の「表示」を外す
it = app.ankle_ligaments[0]
check(it["visible"] is False and it.get("chart", True) is True, "「表示」を外す: 3D の靭帯は隠れる（保存も）")
check(labels(lig_charts(p)[0]) == ["ACL", "PCL"], "「表示」を外しても、ACL の長さはグラフに出たまま")
chart_cb.invoke(); pump(app, 1.0)     # ACL の「グラフ」を外す
check(app.ankle_ligaments[0]["chart"] is False and labels(lig_charts(p)[0]) == ["PCL"], "「グラフ」を外すと、グラフからも消える")
chart_cb.invoke(); pump(app, 1.0)
lig["close"]()
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
# 開き直しても、隠したまま・グラフには出る
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
check(labels(lig_charts(p)[0]) == ["ACL", "PCL"] and app.ankle_ligaments[0]["visible"] is False, "開き直しても、ACL は 3D では隠れて、グラフには出る")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
