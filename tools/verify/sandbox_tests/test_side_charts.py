# -*- coding: utf-8 -*-
"""右のグラフ: 再生コントロールのチェックリストで選ぶ（靭帯の長さ・膝蓋骨の移動・膝蓋骨の回転・datalog）。2026-10-15"""
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
def walk(w):
	for c in w.winfo_children():
		yield c
		yield from walk(c)
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.ankle_side_charts.set("lig"); app.ankle_robot_log_items.set("AP,Fap,Fpd"); app.ankle_robot_log_show.set(True)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
cv = v["ctrl"]["chart_vars"]
print("   チェックリスト:", list(cv.keys()), flush=True)
check({"lig", "pat_t", "pat_r", "log:AP", "log:Fpd", "log:FE"} <= set(cv.keys()), "再生コントロールに、靭帯・膝蓋骨の移動・膝蓋骨の回転・datalog の項目のチェック")
def ylabels(): 
	ch = sorted(p.renderer._charts, key=lambda c: -c.loc[1])
	return [str(c.y_label) for c in ch]
print("   はじめ:", ylabels(), flush=True)
check(ylabels() == ["Ligament (mm)", "AP [mm]", "Fap [N]", "Fpd [N]"], "はじめは、選んであるもの（靭帯の長さ＋AP・Fap・Fpd）を上から")
cbs = {str(w.cget("text")): w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TCheckbutton"}
cbs["膝蓋骨の移動"].invoke(); pump(app, 0.4)
cbs["膝蓋骨の回転"].invoke(); pump(app, 0.4)
cbs["Fap"].invoke(); pump(app, 0.4)
print("   切り替え後:", ylabels(), flush=True)
check(ylabels() == ["Ligament (mm)", "Patella move (mm)", "Patella rot (deg)", "AP [mm]", "Fpd [N]"], "チェックで足す・外すと、並べ直される")
check(app.ankle_side_charts.get() == "lig,pat_t,pat_r" and app.ankle_robot_log_items.get() == "AP,Fpd", f"選んだものは覚える（{app.ankle_side_charts.get()} / {app.ankle_robot_log_items.get()}）")
ch_all = list(p.renderer._charts)
check(all(c.loc[1] >= -1e-6 and c.loc[1] + c.size[1] <= 0.75 for c in ch_all), "5枚とも画面の中（右上の数字の下）に収まる")
pm = next(c for c in ch_all if "Patella move" in str(c.y_label))
pls = [pl for pl in pm.plots() if pl.label]
y0 = [float(np.asarray(pl.y)[0]) for pl in pls]
rng = [float(np.ptp(np.asarray(pl.y))) for pl in pls]
print("   膝蓋骨の移動: 線", [pl.label for pl in pls], " はじめ", np.round(y0, 3), " 変化の幅", np.round(rng, 2), "mm", flush=True)
check(len(pls) == 3 and all(abs(a) < 1e-6 for a in y0), "膝蓋骨の移動は 3 成分・最初のコマが 0")
pr = next(c for c in ch_all if "Patella rot" in str(c.y_label))
rr = [float(np.ptp(np.asarray(pl.y))) for pl in pr.plots() if pl.label]
print("   膝蓋骨の回転: 変化の幅", np.round(rr, 2), "deg", flush=True)
check(len(rr) == 3 and max(rr) > 0.5, "膝蓋骨の回転は 3 成分・動きがある")
ts = np.asarray(v["ctrl"]["window"] and [0])
v["seek"](100.0); pump(app, 0.4)
reds = [np.asarray(pl.x)[0] for c in ch_all for pl in c.plots() if not pl.label and len(np.asarray(pl.x)) == 2 and np.asarray(pl.x)[0] == np.asarray(pl.x)[1]]
check(reds and all(abs(x - reds[0]) < 1e-6 for x in reds) and abs(reds[0] - 100.0) < 1.0, f"赤い線（いまの時刻）がすべてのグラフで同じ（{reds[0] if reds else None:.1f} 秒）")
xr = [(float(np.min(np.asarray(pl.x))), float(np.max(np.asarray(pl.x)))) for c in ch_all for pl in c.plots() if len(np.asarray(pl.x)) > 2]
check(all(lo >= -1e-6 for lo, hi in xr), "どのグラフも、表示している時刻（0 秒〜）だけを描く（横軸がそろう）")
p.screenshot(os.path.join(HERE, "side_charts.png"))
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
check(ylabels() == ["Ligament (mm)", "Patella move (mm)", "Patella rot (deg)", "AP [mm]", "Fpd [N]"], "開き直しても同じグラフ")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_side_charts.set("lig"); app.ankle_robot_log_items.set("FE,AP,Fpd")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
