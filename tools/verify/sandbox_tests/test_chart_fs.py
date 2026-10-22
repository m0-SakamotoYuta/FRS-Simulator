# -*- coding: utf-8 -*-
"""靭帯の長さのグラフの文字の大きさ・書き出すグラフのプレビュー（2026-10-22）:
画面右の靭帯の長さのグラフ（凡例・軸の数字・軸の名前。既定 12 px＝これまでと同じ）を再生コントロールで変える・覚える、
発表用の動画の「靭帯の長さのグラフ」の文字（0＝これまでどおり自動）・プレビューの窓（書き出すのと同じ絵・縦の線を動かせる）・
書き出した mp4 がプレビューと同じ。段階2の束（入っている結果）を使う。接触の計算はダミー。2〜3分。"""
import importlib.util, os, sys, time, gc, shutil, tempfile
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, cv2, pyvista as pv
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
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a[:2])[:200])
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
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(False)
app.ankle_lig_chart_fs.set("12,12,12"); app.ankle_clip_chart_fs.set("0,0,0")
app.ankle_clip_size.set(900); app.ankle_clip_sec.set(10.0); app.ankle_clip_kind.set("anterior")
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
cw = ctrl["window"]

def lig_chart():
	chs = []
	try:
		chs = list(p.renderer._charts)
	except Exception:
		pass
	if not chs:
		chs = [o for o in gc.get_objects() if isinstance(o, pv.Chart2D)]
	chs = [c for c in chs if "Bundle center" in str(c.GetAxis(0).GetTitle()) or "Ligament" in str(c.GetAxis(0).GetTitle())]
	return chs[-1] if chs else None

def sizes(ch):
	return (ch.GetLegend().GetLabelProperties().GetFontSize(), ch.GetAxis(1).GetLabelProperties().GetFontSize(),
	        ch.GetAxis(1).GetTitleProperties().GetFontSize(), ch.GetAxis(0).GetLabelProperties().GetFontSize())

ch = lig_chart()
check(ch is not None, f"画面右に靭帯の長さのグラフ（{ch.GetAxis(0).GetTitle() if ch is not None else '-'}）")
check(ch is not None and sizes(ch) == (12, 12, 12, 12), f"既定は 12 px（これまでと同じ）: {sizes(ch) if ch is not None else '-'}")
# ---- 再生コントロールの「右のグラフ」の欄
ff = [w for w in walk(cw) if w.winfo_class() == "TFrame" and any("靭帯の長さのグラフの文字" in str(c.cget("text")) for c in w.winfo_children() if c.winfo_class() == "TLabel")]
check(len(ff) == 1, "「右のグラフ」の欄に「靭帯の長さのグラフの文字（px）」")
sbs = [w for w in ff[0].winfo_children() if w.winfo_class() == "TSpinbox"]
check(len(sbs) == 3 and [s_.get() for s_ in sbs] == ["12", "12", "12"], f"凡例・軸の数字・軸の名前の 3 つ（{[s_.get() for s_ in sbs]}）")
for s_, val in zip(sbs, ("16", "14", "18")):
	s_.set(val)
sbs[2].event_generate("<FocusOut>"); pump(app, 0.5)
ch2 = lig_chart()
check(ch2 is not None and sizes(ch2) == (16, 14, 18, 14), f"変えるとすぐ描き直す（凡例 16・数字 14・名前 18）: {sizes(ch2) if ch2 is not None else '-'}")
check(app.ankle_lig_chart_fs.get() == "16,14,18", f"覚える（{app.ankle_lig_chart_fs.get()}）")
# ---- 発表用の動画の欄
cf = [w for w in walk(cw) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
c4b = [w for w in walk(cf) if w.winfo_class() == "TFrame" and any("グラフの文字（pt" in str(c.cget("text")) for c in w.winfo_children() if c.winfo_class() == "TLabel")]
check(len(c4b) == 1, "発表用の動画の欄に「グラフの文字（pt。0＝自動）」")
fsb = [w for w in c4b[0].winfo_children() if w.winfo_class() == "TSpinbox"]
check(len(fsb) == 3 and [s_.get() for s_ in fsb] == ["0", "0", "0"], f"既定は 0＝自動（{[s_.get() for s_ in fsb]}）")
autol = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and "自動では" in str(w.cget("text"))]
check(autol and "凡例 9.0・軸の数字 9.0・軸の名前 10.0" in autol[0], f"自動の大きさを示す（一辺 900 → これまでと同じ 9/9/10 pt）: {autol[0] if autol else '-'}")
pvb = [w for w in c4b[0].winfo_children() if w.winfo_class() == "TButton" and "プレビュー" in str(w.cget("text"))]
pvb[0].invoke(); pump(app, 0.8)
pw = ctrl["po_vars"]["clip_chart_prev_win"]["w"]
check(pw is not None and pw.winfo_exists() and getattr(pw, "photo", None) is not None, "「グラフのプレビュー…」で窓に絵が出る")
g0 = pw.g
check(g0["bg"].shape[1] == 900 and g0["bg"].shape[0] == 540, f"書き出す大きさ（一辺 900 → 900×540）: {g0['bg'].shape[1]}×{g0['bg'].shape[0]}")
cl = ctrl["po_vars"]  # noqa
check(len(g0["xs"]) == 300 and g0["xs"][0] < g0["xs"][-1], f"縦の線は書き出す各コマ（{len(g0['xs'])} コマ・左→右）")
pw.pos.set(1.0); pw._draw(); pump(app, 0.2)
check("文字 凡例 9.0・軸の数字 9.0・軸の名前 10.0 pt" in str(pw.info.cget("text")), f"窓に大きさ・文字の大きさ（{str(pw.info.cget('text'))[:80]}）")
# 文字を大きく → 開いている窓も描き直す
for s_, val in zip(fsb, ("20", "16", "22")):
	s_.set(val)
fsb[2].event_generate("<FocusOut>"); pump(app, 0.8)
g1 = pw.g
check("凡例 20.0・軸の数字 16.0・軸の名前 22.0" in g1["info"] and float(np.abs(g1["bg"].astype(int) - g0["bg"].astype(int)).mean()) > 0.5,
      f"文字の大きさを変えると、開いている窓も描き直す（{g1['info'][-40:]}）")
check(app.ankle_clip_chart_fs.get() == "20,16,22", f"覚える（{app.ankle_clip_chart_fs.get()}）")
# 書き出す（グラフの mp4 はプレビューと同じ絵）
outd = tempfile.mkdtemp(prefix="グラフ_")
fp = os.path.join(outd, "テスト_前方引張.mp4")
import tkinter.filedialog as _fd
_fd.asksaveasfilename = lambda **k: fp
btn = {str(w.cget("text")): w for w in walk(cf) if w.winfo_class() == "TButton"}
btn["① 正方形にする"].invoke(); pump(app, 0.5)
ctrl["po_vars"]["clip_sec"].set("2"); pump(app, 0.1)
btn["③ mp4 に書き出す…"].invoke()
t0 = time.time()
while time.time() - t0 < 120:
	pump(app, 0.2)
	if os.path.exists(os.path.splitext(fp)[0] + "_靭帯の長さ.mp4"):
		pump(app, 0.5); break
cp_ = os.path.splitext(fp)[0] + "_靭帯の長さ.mp4"
check(os.path.exists(cp_), "グラフの mp4 を書き出した")
if os.path.exists(cp_):
	cap = cv2.VideoCapture(cp_)
	n_ = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); ok_, fr0 = cap.read(); cap.release()
	g2, _m = ctrl["po_vars"]["clip_chart_prev_win"]["w"].g, None
	pw.pos.set(0.0)
	gp, msg = None, ""
	# いまの設定（2 秒）のプレビューと比べる
	ctrl["po_vars"]["clip_chart_preview"]()
	gp = pw.g
	ref = gp["bg"].copy(); cv2.line(ref, (gp["xs"][0], gp["y_top"]), (gp["xs"][0], gp["y_bot"]), (220, 0, 0), 2, cv2.LINE_AA)
	d_ = float(np.abs(cv2.cvtColor(fr0, cv2.COLOR_BGR2RGB).astype(int) - ref.astype(int)).mean()) if ok_ and fr0.shape == ref.shape else 999
	check(n_ == 60 and d_ < 4.0, f"書き出したグラフ（{n_} コマ＝2 秒×30）の最初のコマはプレビューと同じ（平均の差 {d_:.2f}）")
btn["元の大きさに戻す"].invoke(); pump(app, 0.3)
shutil.rmtree(outd, ignore_errors=True)
try:
	pw.destroy()
except Exception:
	pass
cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_lig_chart_fs.set("12,12,12"); app.ankle_clip_chart_fs.set("0,0,0"); app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
