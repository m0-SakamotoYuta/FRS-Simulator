# -*- coding: utf-8 -*-
"""右のグラフの並び順を、再生コントロールの一覧と ▲▼ で入れ替える（覚える）。2026-10-15"""
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
def walk(w):
	for c in w.winfo_children():
		yield c
		yield from walk(c)
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.ankle_side_charts.set("lig"); app.ankle_robot_log_items.set("AP,Fap,Fpd"); app.ankle_chart_order.set("")
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
def ylabels():
	return [str(c.y_label) for c in sorted(p.renderer._charts, key=lambda c: -c.loc[1])]
lb = v["ctrl"]["chart_vars"].get("_order_lb")
check(lb is not None and list(lb.get(0, tk.END)) == ["靭帯の長さ", "AP", "Fap", "Fpd"], f"並びの一覧（{list(lb.get(0, tk.END)) if lb else None}）")
up = next(w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TButton" and "上へ" in str(w.cget("text")))
dn = next(w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TButton" and "下へ" in str(w.cget("text")))
lb.selection_clear(0, tk.END); lb.selection_set(3); up.invoke(); pump(app, 0.3); up.invoke(); pump(app, 0.3); up.invoke(); pump(app, 0.3)
print("   Fpd を一番上へ:", ylabels(), flush=True)
check(ylabels()[0].startswith("Fpd") and list(lb.get(0, tk.END))[0] == "Fpd", "▲ で Fpd をいちばん上へ（グラフも一覧も）")
check(lb.curselection() == (0,), "動かした項目は選ばれたまま（続けて押せる）")
lb.selection_clear(0, tk.END); lb.selection_set(1); dn.invoke(); pump(app, 0.3)
print("   靭帯を1つ下へ:", ylabels(), flush=True)
check(ylabels() == ["Fpd [N]", "AP [mm]", "Ligament (mm)", "Fap [N]"], "▼ で1つ下へ")
check(app.ankle_chart_order.get().startswith("log:Fpd,log:AP,lig,log:Fap"), f"並び順を覚える（{app.ankle_chart_order.get()}）")
cbs = {str(w.cget("text")): w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TCheckbutton"}
cbs["膝蓋骨の移動"].invoke(); pump(app, 0.3)
check(ylabels()[-1].startswith("Patella") and list(lb.get(0, tk.END))[-1] == "膝蓋骨の移動", "新しくチェックしたグラフは、いちばん下に足される")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
check(ylabels() == ["Fpd [N]", "AP [mm]", "Ligament (mm)", "Fap [N]", "Patella move (mm)"], f"開き直しても同じ並び（{ylabels()}）")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_side_charts.set("lig"); app.ankle_robot_log_items.set("FE,AP,Fpd"); app.ankle_chart_order.set("")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
