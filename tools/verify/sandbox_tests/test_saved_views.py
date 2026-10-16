# -*- coding: utf-8 -*-
"""再生コントロールで、視点を名前を付けて保存・呼び出し・名前の変更・削除（2026-10-15）。"""
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
frs.messagebox.askyesno = lambda *a, **k: True
answers = []
frs.simpledialog.askstring = lambda *a, **k: answers.pop(0) if answers else None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
vp = app._ankle_views_path()
if vp.exists(): vp.unlink()
def walk(w):
	for c in w.winfo_children():
		yield c
		yield from walk(c)
def btn(v, text):
	return next(w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TButton" and str(w.cget("text")) == text)
def combo(v):
	return next(w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TCombobox" and w.master.winfo_class() == "TLabelframe" and "視点" in str(w.master.cget("text")))
def cam(p):
	return np.array(list(app._sim_cam_get(p)[0]) + list(app._sim_cam_get(p)[1]) + list(app._sim_cam_get(p)[2]))
TAB = "261007_FE_軸力100_ACL切"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
check(v["ctrl"].get("views_var") is not None, "再生コントロールに「視点」（保存・この視点にする・名前を変える・削除）")
p.camera_position = [(300, 0, 0), (0, 0, 0), (0, 0, 1)]; p.render(); A = cam(p)
answers.append("横から"); btn(v, "この視点を保存…").invoke(); pump(app, 0.2)
p.camera_position = [(0, 300, 50), (0, 0, 0), (0, 0, 1)]; p.render(); B = cam(p)
answers.append(""); btn(v, "この視点を保存…").invoke(); pump(app, 0.2)       # 空 → 保存しない
answers.append("前から"); btn(v, "この視点を保存…").invoke(); pump(app, 0.2)
cb = combo(v)
check(list(cb.cget("values")) == ["横から", "前から"], f"いくつも保存できる（{list(cb.cget('values'))}）")
v["ctrl"]["views_var"].set("横から"); cb.event_generate("<<ComboboxSelected>>"); pump(app, 0.2)
check(np.allclose(cam(p), A, atol=1e-6), "選ぶと、その視点になる（横から）")
v["ctrl"]["views_var"].set("前から"); btn(v, "この視点にする").invoke(); pump(app, 0.2)
check(np.allclose(cam(p), B, atol=1e-6), "「この視点にする」でも変わる（前から）")
answers.append("正面"); btn(v, "名前を変える…").invoke(); pump(app, 0.2)
check(list(cb.cget("values")) == ["横から", "正面"], f"名前を変えられる（{list(cb.cget('values'))}）")
answers.append("横から"); v["ctrl"]["views_var"].set("正面"); btn(v, "名前を変える…").invoke(); pump(app, 0.2)
check(list(cb.cget("values")) == ["横から", "正面"], "同じ名前があると変えない")
p.camera_position = [(10, 10, 300), (0, 0, 0), (0, 1, 0)]; p.render(); C = cam(p)
answers.append("横から"); btn(v, "この視点を保存…").invoke(); pump(app, 0.2)
v["ctrl"]["views_var"].set("正面"); cb.event_generate("<<ComboboxSelected>>"); pump(app, 0.2)
v["ctrl"]["views_var"].set("横から"); cb.event_generate("<<ComboboxSelected>>"); pump(app, 0.2)
check(np.allclose(cam(p), C, atol=1e-6) and len(cb.cget("values")) == 2, "同じ名前で保存すると上書き")
v["ctrl"]["views_var"].set("正面"); btn(v, "削除").invoke(); pump(app, 0.2)
check(list(cb.cget("values")) == ["横から"], "削除できる")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
# 骨リストを共有しているほかの試験でも使える（開き直しても残る）
T2 = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == T2)); pump(app, 0.5)
app.on_ankle_animate(); pump(app, 2.0)
v2 = app._sim_viewers_alive()[-1]; p2 = v2["plotter"]; cb2 = combo(v2)
check(list(cb2.cget("values")) == ["横から"], "骨リストを共有しているほかの試験でも、保存した視点を選べる")
v2["ctrl"]["views_var"].set("横から"); cb2.event_generate("<<ComboboxSelected>>"); pump(app, 0.2)
check(np.allclose(cam(p2), C, atol=1e-6), "ほかの試験でも、同じ視点になる")
w_ = v2["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
