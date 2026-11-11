# -*- coding: utf-8 -*-
"""発表用の動画の靭帯の長さのグラフの縦軸（2026-10-22）: 既定は「区間の始めからの長さの変化 ΔL [mm]」
（描いた値 = 各束の長さ − 区間の始めのその束の長さ。独立に求めた値と比べる）・ΔL の範囲 ± mm（0＝自動）・「長さ [mm]」に戻せる・
覚える（骨リストの共有にも）・プレビューと書き出した mp4 が同じ。段階2の束（入っている結果）を使う。接触の計算はダミー。2〜3分。"""
import importlib.util, os, sys, time, gc, shutil, tempfile
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.axes
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
# 描いた値を横取りして記録する
REC = {"plot": [], "ylabel": None, "ylim": None, "axh": []}
_plot0, _yl0, _ylim0, _axh0 = matplotlib.axes.Axes.plot, matplotlib.axes.Axes.set_ylabel, matplotlib.axes.Axes.set_ylim, matplotlib.axes.Axes.axhline
def _plot(self, *a, **k):
	if len(a) >= 2:
		REC["plot"].append((np.asarray(a[0], float), np.asarray(a[1], float), k.get("label")))
	return _plot0(self, *a, **k)
def _ylabel(self, t, *a, **k):
	REC["ylabel"] = str(t); return _yl0(self, t, *a, **k)
_xl0 = matplotlib.axes.Axes.set_xlabel
def _xlabel(self, t, *a, **k):
	REC["xlabel"] = str(t); return _xl0(self, t, *a, **k)
matplotlib.axes.Axes.set_xlabel = _xlabel
_gyl0 = matplotlib.axes.Axes.get_ylim
def _ylim(self, *a, **k):
	return _ylim0(self, *a, **k)
def _gylim(self):
	r_ = _gyl0(self)
	REC["ylim"] = (float(r_[0]), float(r_[1]))                 # 描いたあとの実際の範囲（エンジンが縦の線の位置に使う）
	return r_
def _axh(self, *a, **k):
	REC["axh"].append(a[0] if a else k.get("y")); return _axh0(self, *a, **k)
matplotlib.axes.Axes.plot, matplotlib.axes.Axes.set_ylabel, matplotlib.axes.Axes.set_ylim, matplotlib.axes.Axes.axhline = _plot, _ylabel, _ylim, _axh
matplotlib.axes.Axes.get_ylim = _gylim
_leg0, _tkp0 = matplotlib.axes.Axes.legend, matplotlib.axes.Axes.tick_params
def _legend(self, *a, **k):
	REC["legend"] = True; return _leg0(self, *a, **k)
def _tick_params(self, *a, **k):
	if k.get("labelbottom") is False or k.get("labelleft") is False:
		REC["ticks_off"] = True
	return _tkp0(self, *a, **k)
matplotlib.axes.Axes.legend, matplotlib.axes.Axes.tick_params = _legend, _tick_params
def rec_reset():
	REC.update(plot=[], ylabel=None, xlabel=None, ylim=None, axh=[], legend=False, ticks_off=False)
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
app.ankle_clip_chart_fs.set("0,0,0"); app.ankle_clip_size.set(900); app.ankle_clip_sec.set(3.0); app.ankle_clip_kind.set("anterior")
app.ankle_clip_hold.set(0.0); app.ankle_clip_chart_y.set("delta"); app.ankle_clip_chart_ylim.set(0.0); app.ankle_clip_chart_hide.set("")
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
cw = ctrl["window"]; pv_ = ctrl["po_vars"]
cf = [w for w in walk(cw) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
scn = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "_chart_series" in o][-1]
ser, _yl = scn["_chart_series"]("lig")
ft = np.asarray(scn["frame_times"], float)
# 区間の始め・終わり（独立に: 区間の始めへ・終わりへ のボタンで行ったコマの時刻）
btn = {str(w.cget("text")): w for w in walk(cf) if w.winfo_class() == "TButton"}
btn["区間の始めへ"].invoke(); pump(app, 0.3); ta = float(v["time"]())
btn["区間の終わりへ"].invoke(); pump(app, 0.3); tb = float(v["time"]())
fa = int(np.searchsorted(ft, ta + 1e-9, side="right") - 1); fb = int(np.searchsorted(ft, tb + 1e-9, side="right") - 1)
t0, t1 = ft[fa], ft[fb]
exp = {}
for x, y, col, lab in ser:
	x = np.asarray(x, float); y = np.asarray(y, float)
	m_ = (x >= t0 - 1e-9) & (x <= t1 + 1e-9) & np.isfinite(y)
	exp[lab] = (y[m_] - y[m_][0], y[m_])
check(len(exp) >= 2, f"グラフに出す靭帯（{list(exp)}・区間 {t0:.1f}〜{t1:.1f} 秒）")
# ---- 欄
rbs = {str(w.cget("text")): w for w in walk(cf) if w.winfo_class() == "TRadiobutton" and ("ΔL" in str(w.cget("text")) or str(w.cget("text")) == "長さ [mm]")}
sp_l = [w for w in walk(cf) if w.winfo_class() == "TSpinbox" and str(w.cget("textvariable")) == str(pv_.get("clip_chart_ylim"))]
check(len(rbs) == 2 and len(sp_l) == 1 and pv_["clip_chart_y"].get() == "delta", f"「グラフの縦軸」: ΔL（既定）／長さ、ΔL の範囲 ±（{list(rbs)}）")
rec_reset()
pv_["clip_chart_preview"](); pump(app, 0.6)
pw = pv_["clip_chart_prev_win"]["w"]
got = {lab: y for x, y, lab in REC["plot"]}
okd = all(lab in got and len(got[lab]) == len(exp[lab][0]) and np.allclose(got[lab], exp[lab][0], atol=1e-6) for lab in exp)
check(okd and all(abs(got[lab][0]) < 1e-9 for lab in exp), "既定（ΔL）: 描いた値 = 各束の長さ − 区間の始めのその束の長さ（どの束も 0 から始まる）")
check(REC["ylabel"] == "区間の始めからの長さの変化 ΔL [mm]" and 0.0 in REC["axh"], f"縦軸の名前と 0 の横線（{REC['ylabel']}）")
span = max(float(np.abs(exp[lab][0]).max()) for lab in exp)
check(REC["ylim"] is not None and REC["ylim"][0] <= min(0.0, min(exp[l][0].min() for l in exp)) and REC["ylim"][1] >= max(exp[l][0].max() for l in exp)
      and (REC["ylim"][1] - REC["ylim"][0]) < 3.0 * span + 0.5,
      f"自動の範囲は変化に合わせて狭い（{tuple(round(v_, 2) for v_ in REC['ylim'])}・変化の最大 {span:.2f} mm）")
img_d = pw.g["bg"].copy()
if os.environ.get("SAVE_PNG"):
	cv2.imencode(".png", cv2.cvtColor(img_d, cv2.COLOR_RGB2BGR))[1].tofile(os.environ["SAVE_PNG"])
# 範囲 ± 2 mm
rec_reset()
sp_l[0].focus_force(); pump(app, 0.2)
sp_l[0].set("2"); sp_l[0].event_generate("<Return>"); pump(app, 0.6)
check(REC["ylim"] is not None and np.allclose(REC["ylim"], (-2.0, 2.0)) and abs(app.ankle_clip_chart_ylim.get() - 2.0) < 1e-9, f"ΔL の範囲 ± 2 mm（{REC['ylim']}・覚える）")
g2 = pw.g
check(g2["y_top"] < g2["y_bot"] and g2["y_bot"] - g2["y_top"] > 200, f"縦の線は軸の範囲いっぱい（{g2['y_top']}〜{g2['y_bot']} px）")
# 長さ [mm] に戻す
rec_reset()
rbs["長さ [mm]"].invoke(); pump(app, 0.6)
got = {lab: y for x, y, lab in REC["plot"]}
check(all(lab in got and np.allclose(got[lab], exp[lab][1], atol=1e-6) for lab in exp) and REC["ylabel"] == "靭帯の長さ [mm]"
      and app.ankle_clip_chart_y.get() == "abs" and str(sp_l[0].state()).find("disabled") >= 0,
      "「長さ [mm]」にすると、これまでどおり長さそのもの（範囲の欄は使わない）・覚える")
img_a = pw.g["bg"]
check(img_a.shape == img_d.shape and float(np.abs(img_a.astype(int) - img_d.astype(int)).mean()) > 1.0, "ΔL と長さでグラフの絵が変わる")
rbs["区間の始めからの長さの変化 ΔL [mm]"].invoke(); pump(app, 0.5)
gid = app._ankle_tab_group(app._ankle_tabs[next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)])
view = app._ankle_bone_groups_dict()[gid]["view"] if gid else {}
check(view.get("ankle_clip_chart_y") == "delta" and abs(float(view.get("ankle_clip_chart_ylim", -1)) - 2.0) < 1e-9,
      f"骨リストを共有しているタブにも（{view.get('ankle_clip_chart_y')}・{view.get('ankle_clip_chart_ylim')}）")
# ---- 凡例・軸の数字・縦軸の名前・横軸の名前の出す・出さない（2026-10-22）
cbs = [w for w in walk(cf) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")) in ("出す", "縦軸", "横軸")]
check([str(w.cget("text")) for w in cbs] == ["出す", "出す", "縦軸", "横軸"] and len(pv_["clip_chart_show"]) == 4 and all(bool(v_.get()) for v_ in pv_["clip_chart_show"]),
      f"文字の大きさの横にチェック: 凡例・軸の数字は「出す」、軸の名前は「縦軸」「横軸」（既定は全部出す: {[str(w.cget('text')) for w in cbs]}）")
rec_reset(); pv_["clip_chart_preview"](); pump(app, 0.4)
def state_():
	return {"凡例": REC["legend"], "軸の数字": not REC["ticks_off"], "縦軸の名前": REC["ylabel"] is not None, "横軸の名前": REC["xlabel"] is not None}
base = state_()
img_all = pw.g["bg"].copy()
res = []
for j_, nm_ in enumerate(("凡例", "軸の数字", "縦軸の名前", "横軸の名前")):
	rec_reset(); cbs[j_].invoke(); pump(app, 0.5)
	st_ = state_()
	diff = float(np.abs(pw.g["bg"].astype(int) - img_all.astype(int)).mean()) if pw.g["bg"].shape == img_all.shape else 999
	res.append((nm_, (not st_[nm_]) and all(st_[k_] for k_ in st_ if k_ != nm_) and diff > 0.05))
	cbs[j_].invoke(); pump(app, 0.4)
check(all(base.values()) and all(ok_ for _n, ok_ in res), f"外すとそれだけ出さない・ほかは出たまま・プレビューも描き直す（{res}）")
cbs[3].invoke(); pump(app, 0.4)
check(app.ankle_clip_chart_show.get() == "1,1,1,0", f"覚える（{app.ankle_clip_chart_show.get()}）")
view = app._ankle_bone_groups_dict()[gid]["view"] if gid else {}
check(view.get("ankle_clip_chart_show") == "1,1,1,0", f"骨リストを共有しているタブにも（{view.get('ankle_clip_chart_show')}）")
cbs[3].invoke(); pump(app, 0.4)
check(frs.MainMenuGUI._ankle_chart_show4([True, True, False]) == [True, True, False, False]
      and frs.MainMenuGUI._ankle_chart_show4([False, True, True]) == [False, True, True, True]
      and frs.MainMenuGUI._ankle_chart_show4(None) == [True] * 4,
      "前の 3 つの形（凡例, 軸の数字, 軸の名前）は、軸の名前を縦・横の両方に使って読む")
# ---- グラフに出す靭帯（1 本ずつ。2026-10-22）
chs = {str(w.cget("text")): w for w in walk(cf) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")) in exp}
check(set(chs) == set(exp) and all(bool(v_.get()) for v_ in pv_["clip_chart_hide"].values()), f"「グラフに出す靭帯」に 1 本ずつのチェック（{list(chs)}・既定は全部出す）")
off = list(exp)[-1]
rec_reset(); chs[off].invoke(); pump(app, 0.5)
labs = [lab for _x, _y, lab in REC["plot"]]
lo_k = min(float(exp[l][0].min()) for l in exp if l != off); hi_k = max(float(exp[l][0].max()) for l in exp if l != off)
check(off not in labs and set(labs) == set(exp) - {off}, f"「{off}」を外すと、その線を描かない（描いた線 {labs}）")
check(app.ankle_clip_chart_hide.get() == off, f"覚える（{app.ankle_clip_chart_hide.get()}）")
view = app._ankle_bone_groups_dict()[gid]["view"] if gid else {}
check(view.get("ankle_clip_chart_hide") == off, f"骨リストを共有しているタブにも（{view.get('ankle_clip_chart_hide')}）")
# 全部外すと知らせ（グラフは出さない）
for nm_ in exp:
	if nm_ != off:
		chs[nm_].invoke(); pump(app, 0.2)
g_none, msg_none = ctrl["po_vars"]["clip_chart_prev_win"]["w"].g, None
cmsg = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and "グラフに出す靭帯がありません" in str(w.cget("text"))]
check(len(cmsg) >= 1, f"全部外すと「グラフに出す靭帯がありません」と知らせる（{cmsg[0][:40] if cmsg else '-'}）")
for nm_ in exp:
	chs[nm_].invoke() if not pv_["clip_chart_hide"][nm_].get() else None
	pump(app, 0.2)
check(app.ankle_clip_chart_hide.get() == "" and all(bool(v_.get()) for v_ in pv_["clip_chart_hide"].values()), "戻すと全部出す")
# ---- グラフだけ書き出す（mp4／png。正方形にしないまま。2026-10-22）
import tkinter.filedialog as _fd0
od0 = tempfile.mkdtemp(prefix="グラフだけ_")
bco = [w for w in walk(cf) if w.winfo_class() == "TButton" and "グラフだけ書き出す" in str(w.cget("text"))]
check(len(bco) == 1, "「グラフだけ書き出す（mp4／png）…」のボタン")
pv_["clip_chart_preview"](); pump(app, 0.4)
gq = pw.g
p_png = os.path.join(od0, "グラフ.png")
_fd0.asksaveasfilename = lambda *a, **k: p_png
bco[0].invoke(); pump(app, 0.5)
im_png = cv2.imdecode(np.fromfile(p_png, np.uint8), cv2.IMREAD_COLOR) if os.path.exists(p_png) else None
d_png = float(np.abs(cv2.cvtColor(im_png, cv2.COLOR_BGR2RGB).astype(int) - gq["bg"].astype(int)).mean()) if im_png is not None and im_png.shape[:2] == gq["bg"].shape[:2] else 999
check(d_png < 0.5, f"png: 縦の線の無い 1 枚＝プレビューの絵と同じ（{None if im_png is None else im_png.shape}・平均の差 {d_png:.2f}）")
p_mp4 = os.path.join(od0, "グラフ.mp4")
_fd0.asksaveasfilename = lambda *a, **k: p_mp4
bco[0].invoke(); pump(app, 0.5)
n_m, d_m = -1, 999
if os.path.exists(p_mp4):
	cap = cv2.VideoCapture(p_mp4); n_m = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); okm, frm = cap.read(); cap.release()
	ref = gq["bg"].copy(); cv2.line(ref, (gq["xs"][0], gq["y_top"]), (gq["xs"][0], gq["y_bot"]), (220, 0, 0), 2, cv2.LINE_AA)
	d_m = float(np.abs(cv2.cvtColor(frm, cv2.COLOR_BGR2RGB).astype(int) - ref.astype(int)).mean()) if okm and frm.shape == ref.shape else 999
msg_ = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and "グラフの動画を書き出しました" in str(w.cget("text"))]
check(n_m == len(gq["xs"]) and d_m < 4.0 and msg_, f"mp4: ③と同じコマ数（{n_m} / {len(gq['xs'])}）・最初のコマはプレビュー＋縦の線（平均の差 {d_m:.2f}）・知らせ")
check(not [f for f in os.listdir(od0) if f not in ("グラフ.png", "グラフ.mp4")], f"3D の動画は書かない（{os.listdir(od0)}）")
shutil.rmtree(od0, ignore_errors=True)
# 書き出した mp4 はプレビューと同じ（ΔL・± 2 mm）
outd = tempfile.mkdtemp(prefix="グラフ_")
fp = os.path.join(outd, "テスト_前方引張.mp4")
import tkinter.filedialog as _fd
_fd.asksaveasfilename = lambda **k: fp
btn["① 正方形にする"].invoke(); pump(app, 0.5)
btn["③ mp4 に書き出す…"].invoke()
t_ = time.time()
cp_ = os.path.splitext(fp)[0] + "_靭帯の長さ.mp4"
while time.time() - t_ < 120:
	pump(app, 0.2)
	if os.path.exists(cp_):
		pump(app, 0.5); break
ok_ = False
if os.path.exists(cp_):
	cap = cv2.VideoCapture(cp_); ok_, fr0 = cap.read(); cap.release()
	pv_["clip_chart_preview"](); pump(app, 0.4)
	gp = pw.g
	ref = gp["bg"].copy(); cv2.line(ref, (gp["xs"][0], gp["y_top"]), (gp["xs"][0], gp["y_bot"]), (220, 0, 0), 2, cv2.LINE_AA)
	d_ = float(np.abs(cv2.cvtColor(fr0, cv2.COLOR_BGR2RGB).astype(int) - ref.astype(int)).mean()) if ok_ and fr0.shape == ref.shape else 999
	check(d_ < 4.0, f"書き出したグラフの mp4 の最初のコマはプレビュー（ΔL・± 2 mm）と同じ（平均の差 {d_:.2f}）")
else:
	check(False, "グラフの mp4 を書き出した")
btn["元の大きさに戻す"].invoke(); pump(app, 0.3)
shutil.rmtree(outd, ignore_errors=True)
try: pw.destroy()
except Exception: pass
cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_clip_chart_ylim.set(0.0); app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
