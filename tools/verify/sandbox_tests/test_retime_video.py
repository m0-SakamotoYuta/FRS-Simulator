# -*- coding: utf-8 -*-
"""実際の動画を発表用の動画と同じ長さにする（2026-10-25）: 日本語の名前のフォルダに作った試しの動画（7 秒・25 fps・641×481 px。
各コマの明るさ＝コマの番号）を、秒数 3・止める 1 で → 120 コマ（3D の動画の計画と同じ数）・30 fps・640×480・最初のコマ＝元の最初・
本体の最後＝元の最後・途中は等速（元のコマ ≒ 線形）・止める 30 コマは最後のコマのまま・窓のボタンと説明（秒数を変えると書き直す）・
ボタンから（ファイルの窓は差し替え）書き出せる・元と同じ名前には保存しない。接触の計算はダミー。1分。"""
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
def read_all(path):
	tmp = os.path.join(tempfile.gettempdir(), f"frs_t_{os.getpid()}.mp4"); shutil.copyfile(path, tmp)
	cap = cv2.VideoCapture(tmp); out = []
	fps_ = cap.get(cv2.CAP_PROP_FPS)
	while True:
		ok, f = cap.read()
		if not ok: break
		out.append(f)
	cap.release(); os.remove(tmp)
	return out, fps_
D = os.path.join(tempfile.gettempdir(), f"実際の動画テスト_{os.getpid()}"); os.makedirs(D, exist_ok=True)
SRC = os.path.join(D, "切り抜き済み.mp4")
N_SRC = 175
tmp_src = os.path.join(tempfile.gettempdir(), f"frs_src_{os.getpid()}.avi")
w = cv2.VideoWriter(tmp_src, cv2.VideoWriter_fourcc(*"MJPG"), 25.0, (641, 481))
for i in range(N_SRC):
	w.write(np.full((481, 641, 3), i, np.uint8))
w.release(); shutil.move(tmp_src, SRC)
app = frs.MainMenuGUI(); app.withdraw(); app.update()
# ---- 関数だけ
dst = os.path.join(D, "合わせた.mp4")
r = None
for r in app._ankle_retime_video_gen(SRC, dst, 3.0, 1.0, 30.0):
	pass
check(r is not None and r[0] == "done" and r[1], f"書き出せた（{str(r[2]).splitlines()[-1] if r else '-'}）")
fr, fps_o = read_all(dst)
n_m, n_h = app._ankle_clip_frame_counts(3.0, 1.0)
check(len(fr) == 120 == n_m + n_h and abs(fps_o - 30.0) < 0.01, f"120 コマ・30 fps（{len(fr)} コマ・{fps_o:g} fps。3D の動画の数え方 {n_m}+{n_h}）")
check(fr and fr[0].shape[:2] == (480, 640), f"大きさ 640×480（奇数を偶数に: {fr[0].shape[:2] if fr else '-'}）")
val = np.array([float(f.mean()) for f in fr])
exp = np.rint(np.linspace(0, N_SRC - 1, n_m))
check(abs(val[0] - 0) < 3 and abs(val[n_m - 1] - (N_SRC - 1)) < 3, f"最初＝元の最初（{val[0]:.1f}）・本体の最後＝元の最後（{val[n_m - 1]:.1f}）")
check(float(np.abs(val[:n_m] - exp).max()) < 3, f"途中は等速（元のコマとの差 最大 {float(np.abs(val[:n_m] - exp).max()):.1f}）")
check(float(np.abs(val[n_m:] - val[n_m - 1]).max()) < 1.5, f"止める {n_h} コマは最後のコマのまま（差 {float(np.abs(val[n_m:] - val[n_m - 1]).max()):.2f}）")
check("2.33" in r[2] and "×" in r[2] and "4 秒" not in r[2] or "4.00 秒" in r[2], f"説明（{r[2].splitlines()[-1]}）")
# ---- 窓から
app._show_precompute_dialog = lambda N, has_cartilage=False: (type("W", (), {"winfo_exists": lambda s: False, "destroy": lambda s: None})(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
app._sim_view_precompute_multi_heatmap = lambda bd, update_progress=None, cancel_var=None, quiet=False: [
	{idx: np.zeros(len(np.asarray(m.points)), np.float32) for (idx, n, m, T) in bd} for t in range(len(bd[0][3]))]
app._ankle_heatmap_cache_load = lambda *a, **k: None
app._ankle_heatmap_cache_save = lambda *a, **k: None
app._ankle_skip_debug_images = True
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_AP_FE60_軸力100")); pump(app, 0.5)
app.ankle_lig_mode.set("stage1"); app.ankle_start_at_load.set(False); app.ankle_clip_kind.set("anterior")
app.ankle_clip_sec.set(3.0); app.ankle_clip_hold.set(1.0)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; ctrl = v["ctrl"]; v["set_paused"](True)
cf = [w_ for w_ in walk(ctrl["window"]) if w_.winfo_class() == "TLabelframe" and "発表用の動画" in str(w_.cget("text"))][0]
bt = [w_ for w_ in walk(cf) if w_.winfo_class() == "TButton" and str(w_.cget("text")) == "実際の動画を同じ長さにする…"]
lab = lambda: [str(w_.cget("text")) for w_ in walk(cf) if w_.winfo_class() == "TLabel" and "自分で正方形に切り抜き" in str(w_.cget("text"))]
check(len(bt) == 1 and lab() and "120 コマ・4 秒" in lab()[0], f"ボタンと説明（{lab()[0][-60:] if lab() else '-'}）")
pv = ctrl["po_vars"]
cm = [str(w_.cget("text")) for w_ in walk(cf) if w_.winfo_class() == "TLabel" and "区間: 映像の" in str(w_.cget("text"))]
check(bool(cm) and "＝ 4.0 秒・120 コマ" in cm[0], f"3D の動画も 120 コマ（{cm[0][-30:] if cm else '-'}）")
sp = [w_ for w_ in walk(cf) if w_.winfo_class() == "TSpinbox" and str(w_.cget("textvariable")) and w_.get() == "3"]
if sp:
	sp[0].delete(0, "end"); sp[0].insert(0, "5"); sp[0].focus_force(); pump(app, 0.2); sp[0].event_generate("<Return>"); pump(app, 0.4)
check(lab() and "180 コマ・6 秒" in lab()[0], f"秒数を 5 にすると説明も（{lab()[0][-40:] if lab() else '-'}）")
from tkinter import filedialog
dst2 = os.path.join(D, "窓から.mp4")
filedialog.askopenfilename = lambda **k: SRC
filedialog.asksaveasfilename = lambda **k: dst2
bt[0].invoke()
t0 = time.time()
msg = lambda: [str(w_.cget("text")) for w_ in walk(cf) if w_.winfo_class() == "TLabel" and "同じ長さにしました" in str(w_.cget("text"))]
while time.time() - t0 < 60 and not msg():
	pump(app, 0.1)
fr2, _ = read_all(dst2) if os.path.exists(dst2) else ([], 0)
check(bool(msg()) and len(fr2) == 180, f"ボタンから書き出せた（{len(fr2)} コマ＝5 秒＋止める 1 秒）")
filedialog.asksaveasfilename = lambda **k: SRC
bt[0].invoke(); pump(app, 0.5)
bad = [str(w_.cget("text")) for w_ in walk(cf) if w_.winfo_class() == "TLabel" and "元の動画と同じ名前" in str(w_.cget("text"))]
check(bool(bad) and os.path.getsize(SRC) > 0, "元の動画と同じ名前には保存しない")
cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
shutil.rmtree(D, ignore_errors=True)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
