# -*- coding: utf-8 -*-
"""右上の文字・右のグラフの出し消しがすぐ画面に出る（2026-10-25 の不具合の修正）:
① 大きい窓で「右上の文字を3Dの動画に入れる」を外してから ① 正方形にすると、すぐ右上の文字が消える（以前はコマが変わるまで出たまま）・
   元の大きさに戻すとすぐ出る・大きい窓ではそのチェックで消えない（正方形のときだけ。欄に説明）。
② 止めている間に「右のグラフ」の「靭帯の長さ」を外すと、すぐ画面からグラフが消える（以前は描き直さず出たまま）・付けると出る。
画面を撮って、右上の文字の画素・右のグラフの線の画素で確かめる。接触の計算はダミー。1〜2分。"""
import importlib.util, os, sys, time, gc
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, vtk
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
app._sim_view_precompute_multi_heatmap = lambda bd, update_progress=None, cancel_var=None, quiet=False: [
	{idx: np.zeros(len(np.asarray(m.points)), np.float32) for (idx, n, m, T) in bd} for t in range(len(bd[0][3]))]
app._ankle_heatmap_cache_load = lambda *a, **k: None
app._ankle_heatmap_cache_save = lambda *a, **k: None
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_AP_FE60_軸力100")); pump(app, 0.5)
app.ankle_lig_mode.set("stage1"); app.ankle_start_at_load.set(False); app.ankle_side_charts.set("lig"); app.ankle_clip_text_on.set(True)
app.ankle_po_force_on.set(True); app.ankle_po_contact_on.set(True)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]; v["set_paused"](True)
ft = None
def shot():
	img = np.asarray(p.screenshot(None, return_img=True))[:, :, :3].astype(int)
	H, W = img.shape[:2]
	top = img[: int(0.12 * H), int(0.55 * W):]
	red = int(((top[:, :, 0] > 150) & (top[:, :, 1] < 80) & (top[:, :, 2] < 80)).sum())     # 右上の段階の文字（赤）
	reg = img[int(0.30 * H):int(0.95 * H), int(0.75 * W):int(0.98 * W)]   # 骨（右端は幅の 72 % 付近）を避ける
	line = int(((reg.max(axis=2) - reg.min(axis=2)) > 120).sum())                           # 右のグラフの線
	return red, line
cf = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
btn = {str(w.cget("text")): w for w in walk(cf) if w.winfo_class() == "TButton"}
cbt = [w for w in walk(cf) if w.winfo_class() == "TCheckbutton" and "右上の文字を3Dの動画に入れる" in str(w.cget("text"))][0]
note = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and "正方形にしたとき（①②③）だけに効きます" in str(w.cget("text"))]
check(len(note) == 1, "欄に「正方形にしたとき（①②③）だけに効きます」の説明")
r0, l0 = shot()
check(r0 > 50 and l0 > 200, f"大きい窓: 右上の文字（{r0}）と右のグラフ（{l0}）が出ている")
# ---- ① 右上の文字
cbt.invoke(); pump(app, 0.3)
r1, _ = shot()
check(r1 > 50 and app.ankle_clip_text_on.get() is False, f"大きい窓では「右上の文字を3Dの動画に入れる」を外しても消えない（正方形のときだけ: {r1}）")
btn["① 正方形にする"].invoke(); pump(app, 0.5)
r2, _ = shot()
check(r2 == 0, f"外したまま ① 正方形にすると、コマを動かさなくてもすぐ右上の文字が消える（{r2}）")
cbt.invoke(); pump(app, 0.3)
r3, _ = shot()
check(r3 > 20, f"正方形で付けるとすぐ出る（{r3}）")
cbt.invoke(); pump(app, 0.3)
btn["元の大きさに戻す"].invoke(); pump(app, 0.5)
r4, _ = shot()
check(r4 > 50, f"外したまま元の大きさに戻すと、すぐ出る（{r4}）")
cbt.invoke(); pump(app, 0.2)
# ---- ② 右のグラフ
cs = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "右のグラフ" in str(w.cget("text"))][0]
cbl = [w for w in walk(cs) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")) == "靭帯の長さ"][0]
_, la = shot()
cbl.invoke(); pump(app, 0.4)
_, lb = shot()
check(la > 200 and lb < 0.1 * la and app.ankle_side_charts.get() == "", f"止めている間に「靭帯の長さ」を外すと、すぐ画面から消える（線 {la} → {lb} 画素）")
cbl.invoke(); pump(app, 0.4)
_, lc = shot()
check(lc > 200 and app.ankle_side_charts.get() == "lig", f"付けるとすぐ出る（{lc} 画素）")
cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
