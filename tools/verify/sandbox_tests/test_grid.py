# -*- coding: utf-8 -*-
"""方眼紙と移動量（実データ、接触の計算はしない）。"""
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
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", a[:2])
app = frs.MainMenuGUI(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
A = "261007_FE_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == A)); pump(app, 0.3)
check(app.ankle_grid_show.get() and app.ankle_disp_show.get(), "既定で 方眼紙・移動量 を表示")
tab = app._ankle_active_tab_dict(); tab.pop("grid_plane", None)
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
names = list(p.renderer.actors.keys())
check("grid_minor" in names and "grid_major" in names, "方眼紙（細い線・太い線）が描かれる")
rens = p.render_window.GetRenderers(); rens.InitTraversal(); rl = [rens.GetNextItem() for _ in range(rens.GetNumberOfItems())]
ov = [r for r in rl if r.GetLayer() == 1 and r.GetActors().GetNumberOfItems() > 0]
print("   層:", [(r.GetLayer(), r.GetActors().GetNumberOfItems()) for r in rl], "同じカメラ:", (ov[0].GetActiveCamera() == p.renderer.GetActiveCamera()) if ov else None, flush=True)
check(len(ov) == 1 and ov[0].GetActors().GetNumberOfItems() >= 4 and ov[0].GetActiveCamera() == p.renderer.GetActiveCamera(), "移動の軌跡（影）と印は、骨に隠れない層に描く（同じカメラ）")
txt = [a for a in p.renderer.actors.values() if a.__class__.__name__ in ("Text", "CornerAnnotation")]
w = v["ctrl"]["window"]
bt = []
def walk(wd):
	for c in wd.winfo_children():
		if c.winfo_class() == "TButton": bt.append(c)
		walk(c)
walk(w)
labels = [b.cget("text") for b in bt]
check(all(x in labels for x in ("方眼紙をこの視点に合わせる", "方眼紙に正対する", "移動の基準をいまのコマにする")), "再生コントロールのボタン: " + " / ".join(l for l in labels if "方眼" in l or "基準" in l))
# コマを進めて移動量
v["seek"](0.0); pump(app, 0.2)
v["seek"](120.0); pump(app, 0.3)
corner = None
for a in p.renderer.actors.values():
	if hasattr(a, "GetText"):
		try:
			t0 = a.GetText(0)
			if t0 and "移動" in t0: corner = t0
		except Exception: pass
print("   表示:", (corner or "").replace("\n", " | "), flush=True)
check(corner is not None and "横" in corner and "縦" in corner and "奥行き" in corner, "移動量の文字が出る")
# 基準をいまのコマに
[b for b in bt if b.cget("text") == "移動の基準をいまのコマにする"][0].invoke(); pump(app, 0.3)
corner2 = [a.GetText(0) for a in p.renderer.actors.values() if hasattr(a, "GetText") and a.GetText(0) and "移動" in a.GetText(0)][0]
check("横 +0.00 mm" in corner2 and "縦 +0.00 mm" in corner2 or "横 -0.00" in corner2, "基準をいまのコマにすると 0 に: " + corner2.splitlines()[1])
# この視点に合わせる → 保存
cam = p.renderer.GetActiveCamera(); cam.Azimuth(30); p.render()
[b for b in bt if b.cget("text") == "方眼紙をこの視点に合わせる"][0].invoke(); pump(app, 0.3)
gp = tab.get("grid_plane")
n_cam = np.asarray(cam.GetPosition()) - np.asarray(cam.GetFocalPoint()); n_cam /= np.linalg.norm(n_cam)
check(gp is not None and np.allclose(gp["normal"], n_cam, atol=1e-6), "向きをタブに保存（法線 = いまの視線）")
v["seek"](0.0); pump(app, 0.2); v["seek"](60.0); pump(app, 0.4)
p.screenshot(os.path.join(HERE, "grid_view.png"))
from capture_util import capture
capture(w, os.path.join(HERE, "grid_panel.png"))
print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
