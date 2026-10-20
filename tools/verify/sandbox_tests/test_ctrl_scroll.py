# -*- coding: utf-8 -*-
"""再生コントロールの窓がスクロールできる（2026-10-19）: 窓は画面に収まる・いちばん下の「骨の膨張」に届く・ホイールで動く。1分。"""
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
frs.messagebox.showwarning = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
app._sim_view_precompute_multi_heatmap = lambda bd, update_progress=None, cancel_var=None, quiet=False: [
	{idx: np.ones(len(np.asarray(m.points)), np.float32) for (idx, n, m, T) in bd} for t in range(len(bd[0][3]))]
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; win = v["ctrl"]["window"]
win.update_idletasks(); pump(app, 0.3)
scr_h = win.winfo_screenheight()
check(win.winfo_height() <= scr_h - 100, f"窓は画面に収まる（高さ {win.winfo_height()} / 画面 {scr_h}）")
cv = [w for w in win.winfo_children() if w.winfo_class() == "Canvas"]
sb = [w for w in win.winfo_children() if w.winfo_class() == "TScrollbar"]
check(len(cv) == 1 and len(sb) == 1, "窓にスクロールできる枠とスクロールバー")
cv = cv[0]
dl = [w for w in walk(win) if w.winfo_class() == "TLabelframe" and "膨張" in str(w.cget("text"))]
check(len(dl) == 1, "「骨の膨張（臨時）」の欄がある")
body_h = cv.bbox("all")[3] if cv.bbox("all") else 0
check(body_h > cv.winfo_height(), f"中身は窓より高い（中身 {body_h} / 見えている {cv.winfo_height()}）→ スクロールが要る")
def visible(w):
	y = w.winfo_rooty() - cv.winfo_rooty()
	return 0 <= y and y + w.winfo_height() <= cv.winfo_height() + 2
check(not visible(dl[0]), "最初は「骨の膨張」は見えていない（下にある）")
cv.yview_moveto(1.0); pump(app, 0.3)
check(visible(dl[0]), "いちばん下までスクロールすると「骨の膨張」が見える")
cv.yview_moveto(0.0); pump(app, 0.2)
y0 = cv.yview()[0]
lbl = [w for w in walk(win) if w.winfo_class() == "TLabel"][0]
lbl.event_generate("<MouseWheel>", delta=-120 * 5); pump(app, 0.2)
check(cv.yview()[0] > y0, f"ホイールで下へ動く（{y0:.2f} → {cv.yview()[0]:.2f}）")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
