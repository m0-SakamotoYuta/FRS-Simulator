# -*- coding: utf-8 -*-
"""靭帯の長さのグラフの横軸（時間／前方変位）と、グラフの画像（png）の書き出し（2026-10-25）:
横軸の選択肢（既定は時間）・「グラフの画像を書き出す（png）…」・時間で書ける・前方変位で書ける（絵が変わる・設定を覚える）・
変位（ap_frames）＝ datalog の AP を独立にコマの時刻へ合わせたもの・前方変位はほぼ増える一方・後方引張でも書ける（後方変位）・
変位のまま mp4 も書ける（縦の線が左から右へ）・屈曲伸展の試験の伸展では時間のまま（説明つき）。段階1の靭帯・接触の計算はダミー。2分。"""
import importlib.util, os, sys, time, gc, tempfile, shutil
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, cv2
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
def imread(p):
	return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)
D = os.path.join(tempfile.gettempdir(), f"グラフの画像テスト_{os.getpid()}"); os.makedirs(D, exist_ok=True)
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
from tkinter import filedialog
def open_tab(name):
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == name)); pump(app, 0.5)
	app._ankle_tabs[app._ankle_active_tab].pop("clip_manual", None)
	app.ankle_lig_mode.set("stage1"); app.ankle_start_at_load.set(False); app.ankle_side_charts.set("lig")
	app.ankle_clip_chart_x.set("time"); app.ankle_clip_chart_hide.set(""); app.ankle_clip_sec.set(3.0); app.ankle_clip_hold.set(1.0)
	app.on_ankle_animate(); pump(app, 2.0)
	v = app._sim_viewers_alive()[-1]; v["set_paused"](True)
	scn = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "pres_overlay" in o and "_t_shift" in o][-1]
	cf = [w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
	return v, scn, cf
def msg(cf):
	x = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and ("グラフの画像を保存しました" in str(w.cget("text")) or "グラフの動画を書き出しました" in str(w.cget("text")))]
	return x[0] if x else ""
def save_png(v, cf, fp):
	filedialog.asksaveasfilename = lambda **k: fp
	v["ctrl"]["po_vars"]["clip_chart_png"](); pump(app, 0.3)
	return msg(cf)
def kind(cf, head):
	[w for w in walk(cf) if w.winfo_class() == "TRadiobutton" and str(w.cget("text")).startswith(head)][0].invoke(); pump(app, 0.4)
try:
	v, scn, cf = open_tab("261007_AP_FE60_軸力100")
	kind(cf, "前方引張")
	rbx = {str(w.cget("text")): w for w in walk(cf) if w.winfo_class() == "TRadiobutton" and (str(w.cget("text")) == "時間 [s]" or str(w.cget("text")).startswith("前方変位 [mm]"))}
	bpng = [w for w in walk(cf) if w.winfo_class() == "TButton" and str(w.cget("text")) == "グラフの画像を書き出す（png）…"]
	check(len(rbx) == 2 and len(bpng) == 1 and v["ctrl"]["po_vars"]["clip_chart_x"].get() == "time", f"横軸の選択肢（{list(rbx)}・既定は時間）と png のボタン")
	f_t = os.path.join(D, "時間.png")
	m_ = save_png(v, cf, f_t)
	img_t = imread(f_t) if os.path.exists(f_t) else None
	check(img_t is not None and "横軸は 時間" in m_, f"時間で書ける（{m_[-30:]}）")
	red = lambda im: int(((im[:, :, 2] > 180) & (im[:, :, 1] < 60) & (im[:, :, 0] < 60)).sum())
	vcol = lambda im: int((((im[:, :, 2] > 180) & (im[:, :, 1] < 60) & (im[:, :, 0] < 60)).sum(axis=0)).max())   # 赤の縦の並び（いちばん長い列）
	check(img_t is not None and vcol(img_t) < 150, f"png には今の時刻の赤い縦の線が無い（赤のいちばん長い列 {vcol(img_t) if img_t is not None else -1} px）")
	rbx[next(k for k in rbx if k.startswith("前方変位"))].invoke(); pump(app, 0.3)
	f_d = os.path.join(D, "前方変位.png")
	m_ = save_png(v, cf, f_d)
	img_d = imread(f_d) if os.path.exists(f_d) else None
	diff = float(np.abs(img_d.astype(int) - img_t.astype(int)).mean()) if (img_d is not None and img_t is not None and img_d.shape == img_t.shape) else -1
	check(img_d is not None and "横軸は 変位" in m_ and diff > 0.5, f"前方変位で書ける・絵が変わる（差 {diff:.2f}・{m_[-30:]}）")
	check(app.ankle_clip_chart_x.get() == "disp", f"横軸の設定を覚える（{app.ankle_clip_chart_x.get()}）")
	# 変位 = datalog の AP を独立にコマの時刻へ
	po = scn["pres_overlay"]
	ft = np.asarray(scn["frame_times"], float)
	log = app._ankle_robot_log_load(str(app.ankle_robot_log_path.get()))
	off = scn["_log_offset"] if scn.get("_log_offset") is not None else float(app.ankle_robot_log_offset_s.get())
	tx = np.asarray(log["t"], float) + off - float(scn["_t_shift"])
	ap_ind = np.interp(ft, tx, np.asarray(log["cols"]["AP"], float))
	sm_on, sm_hz = bool(app.ankle_smooth_enable.get()), float(app.ankle_smooth_cutoff_hz.get())
	if sm_on:     # 骨の姿勢と同じ平滑化（Butterworth 4 次・filtfilt・同じカットオフ・撮影時刻からのフレームレート。2026-10-26）
		from scipy.signal import butter, filtfilt
		_wn = float(np.clip(sm_hz / (app._ankle_fps_of(ft) / 2.0), 1e-3, 0.99))
		_b, _a = butter(N=4, Wn=_wn, btype="low")
		ap_ind = filtfilt(_b, _a, ap_ind, padtype="odd", padlen=min(len(ap_ind) - 1, 3 * max(len(_a), len(_b))))
	ap = po["ap_frames"]()
	check(ap is not None and float(np.abs(np.asarray(ap) - ap_ind).max()) < 1e-6 and tuple(po["ap_smooth"]()) == (sm_on, sm_hz),
	      f"変位は datalog の AP をコマの時刻へ合わせ、骨の姿勢と同じ平滑化（{'カットオフ %g Hz' % sm_hz if sm_on else 'なし'}）をかけたもの（差 {float(np.abs(np.asarray(ap) - ap_ind).max()) if ap is not None else -1:.2e} mm）")
	lab_ = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and "変位はロボットの AP" in str(w.cget("text"))]
	check(bool(lab_) and ((f"カットオフ {sm_hz:g} Hz" in lab_[0]) == sm_on), f"欄の説明に平滑化（{lab_[0][:60] if lab_ else '-'}）")
	a, b = po["clips"]()["anterior"]
	w_ = np.asarray(ap[a:b + 1]) - ap[a]
	back = float((np.maximum.accumulate(w_) - w_).max())
	check(w_[-1] > 1.0 and back < 0.2, f"前方引張で前方変位は増える一方（0 → {w_[-1]:.2f} mm・戻りはいちばん大きくて {back:.3f} mm）")
	# 後方引張
	kind(cf, "後方引張")
	f_p = os.path.join(D, "後方変位.png")
	m_ = save_png(v, cf, f_p)
	a2, b2 = po["clips"]()["posterior"]
	check(os.path.exists(f_p) and "横軸は 変位" in m_ and float(ap[b2] - ap[a2]) < -1.0,
	      f"後方引張でも書ける（後方変位＝AP の減り {float(ap[a2] - ap[b2]):.2f} mm を＋）")
	# mp4（変位のまま）
	kind(cf, "前方引張")
	f_m = os.path.join(D, "前方変位.mp4")
	filedialog.asksaveasfilename = lambda **k: f_m
	v["ctrl"]["po_vars"]["clip_chart_only"](); pump(app, 0.5)
	cap = cv2.VideoCapture(shutil.copy(f_m, os.path.join(tempfile.gettempdir(), f"frs_t_{os.getpid()}.mp4")) if os.path.exists(f_m) else "")
	xs, n = [], 0
	while True:
		ok, fr = cap.read()
		if not ok: break
		n += 1
		m = (fr[:, :, 2] > 180) & (fr[:, :, 1] < 60) & (fr[:, :, 0] < 60)
		cols = np.where(m.sum(axis=0) > 20)[0]
		if len(cols): xs.append(float(cols.mean()))
	cap.release()
	check(n == 120 and len(xs) > 100 and xs[-1] > xs[0] + 100, f"変位のまま mp4 も書ける（{n} コマ・縦の線 x {xs[0] if xs else -1:.0f} → {xs[-1] if xs else -1:.0f}）")
	cw = v["ctrl"]["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
	# 屈曲伸展の試験
	v, scn, cf = open_tab("261007_FE_軸力100")
	v["ctrl"]["po_vars"]["clip_chart_x"].set("disp")
	[w for w in walk(cf) if w.winfo_class() == "TRadiobutton" and str(w.cget("text")).startswith("前方変位")][0].invoke(); pump(app, 0.3)
	kind(cf, "伸展")
	f_e = os.path.join(D, "伸展.png")
	m_ = save_png(v, cf, f_e)
	check(os.path.exists(f_e) and "横軸は 時間" in m_ and "前方・後方引張の区間だけ" in m_, f"伸展では時間のまま（{m_[-45:]}）")
	cw = v["ctrl"]["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	app.ankle_clip_chart_x.set("time"); app.ankle_clip_kind.set("anterior")
	shutil.rmtree(D, ignore_errors=True)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
