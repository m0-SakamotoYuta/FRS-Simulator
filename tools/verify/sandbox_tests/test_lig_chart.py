# -*- coding: utf-8 -*-
"""右側のグラフのいちばん上に、靭帯の長さのグラフ（全靭帯を1枚に）。2026-10-15。接触の計算はダミー。"""
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
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: None
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.ankle_robot_log_show.set(True)
# 靭帯を4本にする（ACL / PCL / MCL / LCL。1本目の端点を少しずつずらしたテスト用）
base = copy.deepcopy(app.ankle_ligaments[0])
ligs = []
for k, (nm, col) in enumerate((("ACL", "#D62728"), ("PCL", "#1F77B4"), ("MCL", "#2CA02C"), ("LCL", "#9467BD"))):
	b = copy.deepcopy(base); b["name"] = nm; b["color"] = col
	b["b"]["point"] = [float(v) + 3.0 * k for v in b["b"]["point"]]
	ligs.append(b)
app.ankle_ligaments = ligs
captured = {}
_orig_run = app._sim_engine_run
def _run(scene):
	captured["scene"] = scene
	return _orig_run(scene)
app._sim_engine_run = _run
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
sc = captured["scene"]
charts = list(p.renderer._charts) if hasattr(p.renderer, "_charts") else []
def info(ch):
	pos = ch.loc if hasattr(ch, "loc") else None
	return pos, ch.size if hasattr(ch, "size") else None
lig = [c for c in charts if "Ligament" in str(c.y_label)]
logs = [c for c in charts if "Ligament" not in str(c.y_label)]
print("   グラフ:", [(str(c.y_label), tuple(round(x, 2) for x in c.loc)) for c in charts], flush=True)
check(len(lig) == 1, "靭帯の長さのグラフが 1 枚（全靭帯を1枚に）")
check(lig and all(c.loc[1] + c.size[1] <= lig[0].loc[1] + 1e-6 for c in logs) and abs(lig[0].loc[1] + lig[0].size[1] - 0.73) < 0.02,
      "靭帯のグラフがいちばん上・datalog のグラフはその下")
check(lig and all(c.loc[1] >= -1e-6 for c in logs), "datalog のグラフも画面の中に収まる")
plots = list(lig[0].plots()) if lig else []
labels = [pl.label for pl in plots if pl.label]
check(sorted(labels) == ["ACL", "LCL", "MCL", "PCL"], f"線は靭帯ごと（凡例 {labels}）")
# 線の値 = 靭帯の端点どうしの距離
sb = sc["bones"]; it = sc["ligaments"]["items"][0]
Pa = np.asarray(sb[it["a"]["bone"]]["poses"]); Pb = np.asarray(sb[it["b"]["bone"]]["poses"])
L = np.linalg.norm((Pa[:, :3, :3] @ np.asarray(it["a"]["point"]) + Pa[:, :3, 3]) - (Pb[:, :3, :3] @ np.asarray(it["b"]["point"]) + Pb[:, :3, 3]), axis=1)
pl0 = next(pl for pl in plots if pl.label == it["name"])
y = np.asarray(pl0.y)
check(np.allclose(y, L[:len(y)], atol=1e-4), f"線の値は靭帯の長さ（{it['name']}: {L.min():.1f}〜{L.max():.1f} mm）")
# 赤い線が いまの時刻に動く
ts = np.asarray(sc["frame_times"]); v["seek"](float(ts[3000])); pump(app, 0.3)
redx = [np.asarray(pl.x) for pl in plots if not pl.label]
check(redx and abs(float(redx[-1][0]) - float(ts[3000])) < 0.5, "赤い線（いまの時刻）が動く")
# 可視化の中で靭帯を消す → グラフが消え、datalog が上に詰まる
sc["ligaments"]["on_change"]([]); pump(app, 0.3)
charts2 = list(p.renderer._charts)
lig2 = [c for c in charts2 if "Ligament" in str(c.y_label)]
top2 = max(c.loc[1] + c.size[1] for c in charts2) if charts2 else 0
check(not lig2 and abs(top2 - 0.73) < 0.02, "靭帯を全部消すと、靭帯のグラフが消えて datalog のグラフが上に詰まる")
sc["ligaments"]["on_change"](sc["ligaments"]["items"]); pump(app, 0.3)
check(len([c for c in p.renderer._charts if "Ligament" in str(c.y_label)]) == 1, "靭帯を足すと、またいちばん上に出る")
w = v["ctrl"]["window"]; w.tk.call(w.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
