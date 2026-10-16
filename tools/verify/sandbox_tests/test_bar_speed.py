# -*- coding: utf-8 -*-
"""色の帯をグラフと重ならない左下へ・再生速度 30 倍まで（2026-10-15）。接触の計算はダミー。"""
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
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
app._sim_view_precompute_multi_heatmap = lambda bd, update_progress=None, cancel_var=None, quiet=False: [{i: np.full(m.n_points, -0.5, np.float32) for (i, n, m, T) in bd} for _ in range(len(bd[0][3]))]
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.ankle_hm_show.set(True); app.ankle_robot_log_show.set(True)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
bars = [a for a in p.renderer.actors2D if a.GetClassName().endswith("ScalarBarActor")] if hasattr(p.renderer, "actors2D") else []
if not bars:
	bars = list(p.scalar_bars.values()) if hasattr(p, "scalar_bars") else []
x0, y0 = bars[0].GetPositionCoordinate().GetValue()[:2]; w, h = bars[0].GetWidth(), bars[0].GetHeight()
charts = [c for c in getattr(p.renderer, "_charts", [])] if hasattr(p.renderer, "_charts") else []
print(f"   色の帯: x {x0:.2f}〜{x0 + w:.2f}, y {y0:.2f}〜{y0 + h:.2f}", flush=True)
check(x0 + w <= 0.65, "色の帯は右側のグラフ（x 0.66〜）と重ならない")
sc = v["ctrl"]["speed_scale"]
check(float(sc.cget("to")) == 30.0, f"再生速度は 30 倍まで（{sc.cget('to')}）")
sc.set(30.0); pump(app, 0.2)
t0 = v["time"](); v["set_paused"](False); pump(app, 2.0); t1 = v["time"](); v["set_paused"](True)
print(f"   2 秒で進んだ映像の時間: {t1 - t0:.1f} 秒", flush=True)
check(t1 - t0 > 30.0, "30 倍で再生できる（2 秒で 30 秒以上進む）")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
