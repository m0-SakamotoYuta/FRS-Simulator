# -*- coding: utf-8 -*-
"""右のグラフの軸: 横軸はどれも 0 から（軸力から始めたときも）・Fap など ± の力は 0 を真ん中に ±（切りのよい最大）。2026-10-15"""
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
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.ankle_side_charts.set("lig"); app.ankle_robot_log_items.set("AP,Fap,Fpd"); app.ankle_start_at_load.set(True)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
ch = sorted(p.renderer._charts, key=lambda c: -c.loc[1])
for c in ch:
	xs = [np.asarray(pl.x) for pl in c.plots() if len(np.asarray(pl.x)) > 2]
	print(f"   {str(c.y_label):16s} 横軸 {c.x_range}  縦軸 {np.round(c.y_range, 1)}  データの横 {min(float(x.min()) for x in xs):.1f}〜{max(float(x.max()) for x in xs):.1f}", flush=True)
check(all(abs(float(c.x_range[0])) < 1e-6 for c in ch), "どのグラフも横軸は 0 から")
check(all(min(float(np.asarray(pl.x).min()) for pl in c.plots() if len(np.asarray(pl.x)) > 2) >= -1e-6 for c in ch), "どのグラフも 0 秒より前を描かない")
fap = next(c for c in ch if "Fap" in str(c.y_label))
check(list(np.round(fap.y_range, 3)) == [-200.0, 200.0], f"Fap の縦軸は ±200 N（{list(fap.y_range)}）")
p.screenshot(os.path.join(HERE, "chart_axes.png"))
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
