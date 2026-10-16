# -*- coding: utf-8 -*-
"""Fpd（軸力）が目標に達した時点を 0 として、可視化と右のグラフを始める（既定 ON・再生コントロールで切り替え）。2026-10-15"""
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
captured = []
_orig = app._sim_engine_run
app._sim_engine_run = lambda sc: (captured.append(sc), _orig(sc))[1]
def walk(w):
	for c in w.winfo_children():
		yield c
		yield from walk(c)
def close(v):
	w = v["ctrl"]["window"]; w.tk.call(w.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
check(app.ankle_start_at_load.get() is True, "既定はオン")
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; sc = captured[-1]
ft = sc["frame_times"]
print(f"   はじまり: 映像 {sc['_t_shift']:.2f} 秒 / コマ数 {len(ft)}", flush=True)
check(ft[0] == 0.0 and sc["_t_shift"] > 40 and len(ft) < 11059, "Fpd が 100 N に達した時点（映像 約 50 秒）を 0 として始まる")
log = app._ankle_robot_log_load(app.ankle_robot_log_path.get())
fpd0 = np.interp(0.0 + sc["_t_shift"] - sc["_log_offset"], log["t"], log["cols"]["Fpd"])
check(fpd0 >= 95.0, f"0 秒の時点の Fpd = {fpd0:.1f} N")
check("Fpd 100N から" in str(sc["window_title"]), f"窓の題名に「Fpd 100N から」（{sc['window_title']}）")
sv = v["ctrl"].get("sal_var")
cb = [w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TCheckbutton" and "時点を 0 として始める" in str(w.cget("text"))]
check(sv is not None and sv.get() and cb and "disabled" not in cb[0].state(), f"再生コントロールにチェック（オン）: {cb[0].cget('text') if cb else ''}")
# datalog のグラフの赤い線（いまの時刻 0）の下で Fpd ≒ 100
charts = list(p.renderer._charts)
fch = [c for c in charts if "Fpd" in str(c.y_label)]
if fch:
	pl = [x for x in fch[0].plots()]
	line = pl[0]
	xs, ys = np.asarray(line.x), np.asarray(line.y)
	check(abs(float(np.interp(0.0, xs, ys)) - 100) < 5, f"右のグラフも 0 秒で Fpd ≒ 100 N（{float(np.interp(0.0, xs, ys)):.1f}）")
# 切り替え → 開き直して、全部から
cb[0].invoke(); pump(app, 3.0)
v2 = app._sim_viewers_alive()[-1]; sc2 = captured[-1]
check(sc2 is not sc and sc2["frame_times"][0] == 0.0 and len(sc2["frame_times"]) == 11059 and app.ankle_start_at_load.get() is False,
      "外すと開き直して、録画のはじめから（全コマ）")
cb2 = [w for w in walk(v2["ctrl"]["window"]) if w.winfo_class() == "TCheckbutton" and "時点を 0 として始める" in str(w.cget("text"))]
cb2[0].invoke(); pump(app, 3.0)
check(app.ankle_start_at_load.get() is True and captured[-1]["_t_shift"] > 40, "入れ直すと、また軸力の時点から")
close(app._sim_viewers_alive()[-1])
# 軸力なしの試験: 選べない
T2 = "261007_AP_FE60_軸力なし"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == T2)); pump(app, 0.5)
app.on_ankle_animate(); pump(app, 2.0)
v3 = app._sim_viewers_alive()[-1]; sc3 = captured[-1]
cb3 = [w for w in walk(v3["ctrl"]["window"]) if w.winfo_class() == "TCheckbutton" and "時点を 0 として始める" in str(w.cget("text"))]
check(cb3 and "disabled" in cb3[0].state() and sc3["_t_shift"] == 0.0, f"軸力なしの試験では選べず、はじめから（{cb3[0].cget('text') if cb3 else ''}）")
close(v3)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
