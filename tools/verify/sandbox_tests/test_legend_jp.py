# -*- coding: utf-8 -*-
"""右のグラフの凡例・軸の名前に日本語のフォント（靭帯の名前が日本語でも出る）。2026-10-15"""
import importlib.util, os, sys, time, copy
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
L = copy.deepcopy(app.ankle_ligaments[:2])
L[0]["name"] = "ACL（切除前の位置）"; L[1]["name"] = "後十字靭帯"
for _l in L: _l["chart"] = True; _l["visible"] = True
app.ankle_ligaments = L
app.ankle_side_charts.set("lig,pat_t"); app.ankle_robot_log_items.set("AP")
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
lig = next(c for c in p.renderer._charts if "Ligament" in str(c.y_label))
fonts = [lig.GetLegend().GetLabelProperties().GetFontFile()] + [lig.GetAxis(a).GetTitleProperties().GetFontFile() for a in (0, 1)]
print("   フォント:", fonts, flush=True)
check(all(f and os.path.basename(f).lower() in ("meiryo.ttc", "yugothm.ttc", "msgothic.ttc", "biz-udgothicr.ttc") for f in fonts), "凡例と軸の名前に日本語のフォント")
check(sorted(pl.label for pl in lig.plots() if pl.label) == sorted(["ACL（切除前の位置）", "後十字靭帯"]), "凡例は靭帯の名前（日本語）")
p.screenshot(os.path.join(HERE, "legend_jp_viewer.png"))
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_side_charts.set("lig"); app.ankle_robot_log_items.set("FE,AP,Fpd")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
