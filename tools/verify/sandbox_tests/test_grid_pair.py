# -*- coding: utf-8 -*-
"""切除前後を並べたときの方眼紙の向き・連動、グラフの位置。"""
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
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
tabs = []
for name in ("261007_FE_軸力100", "261007_FE_軸力100_ACL切"):
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == name)); pump(app, 0.3)
	app._ankle_active_tab_dict().pop("grid_plane", None)
	app._ankle_robot_log_autofind_ui(); app.ankle_robot_log_items.set("FE,AP,Fpd")
	tabs.append(app._ankle_active_tab_dict())
	app.on_ankle_animate(); pump(app, 1.5)
vs = app._sim_viewers_alive()
check(len(vs) == 2, "2つ開いた")
G = [g for g in app._ankle_grid_states if g.get("plotter") is not None]
check(len(G) == 2, f"方眼紙の登録 2 つ（{len(G)}）")
ang = np.degrees(np.arccos(np.clip(abs(G[0]["n"] @ G[1]["n"]), -1, 1)))
check(G[0]["n"] @ G[1]["n"] > 0.9, f"自動の向きは切除前後で同じ側（法線のなす角 {np.degrees(np.arccos(np.clip(G[0]['n'] @ G[1]['n'], -1, 1))):.1f}°）")
pump(app, 0.5)
for k, g in enumerate(G):
	cam = g["plotter"].renderer.GetActiveCamera()
	side = (np.asarray(cam.GetPosition()) - g["origin"]) @ g["n"]
	check(side > 0, f"ウィンドウ{k+1}: カメラは方眼紙の表側（骨の側）から見ている（{side:.0f}）")
# 1つ目で視点を変えて合わせる → 2つ目も
p1 = G[0]["plotter"]; cam = p1.renderer.GetActiveCamera(); cam.Azimuth(-25); cam.Elevation(10); p1.render(); pump(app, 0.4)
bt = []
def walk(wd):
	for c in wd.winfo_children():
		if c.winfo_class() == "TButton": bt.append(c)
		walk(c)
walk(vs[0]["ctrl"]["window"])
[b for b in bt if b.cget("text") == "方眼紙をこの視点に合わせる"][0].invoke(); pump(app, 0.4)
check(np.allclose(G[0]["n"], G[1]["n"], atol=1e-6) and np.allclose(G[0]["v"], G[1]["v"], atol=1e-6), "1つ目で合わせると、連動している2つ目も同じ向きに")
check(np.allclose(tabs[1]["grid_plane"]["normal"], G[1]["n"].tolist(), atol=1e-6), "2つ目のタブにも向きを保存")
for k, v in enumerate(vs):
	v["seek"](100.0)
pump(app, 0.5)
for k, v in enumerate(vs):
	v["plotter"].screenshot(os.path.join(HERE, f"pair_{k+1}.png"))
print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
