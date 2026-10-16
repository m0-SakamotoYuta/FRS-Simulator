# -*- coding: utf-8 -*-
"""⓪「座標系だけの映像を書き出す（白背景）」: ④の姿勢から描き直した映像が、録画と同じ視点か（実データ）。
元の _color.mp4 の同じコマで ArUco を検出し直し、描いたマーカーの角とのずれを画素で測る。"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, cv2
from pathlib import Path
import re
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
msgs = []
frs.messagebox.showinfo = lambda *a, **k: (msgs.append(a), print("   [info]", a[:2]))
frs.messagebox.showerror = lambda *a, **k: (msgs.append(a), print("   [error]", a[:2]))
OUTDIR = os.path.join(HERE, "保存先 テスト（日本語）")
os.makedirs(OUTDIR, exist_ok=True)
OUT = os.path.join(OUTDIR, "ankle_axes_テスト.mp4")
frs.filedialog.asksaveasfilename = lambda **k: (print("   保存先の候補:", k.get("initialdir"), k.get("initialfile")), OUT)[1]
app = frs.MainMenuGUI(); app.update()
TAB = sys.argv[1] if len(sys.argv) > 1 else "261007_FE_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.3)
# ボタン
btns = []
def walk(w):
	for c in w.winfo_children():
		try:
			if c.winfo_class() == "TButton": btns.append(str(c.cget("text")))
		except Exception: pass
		walk(c)
walk(app)
check("座標系だけの映像を書き出す（白背景）" in btns, "⓪にボタンがある")
i_rec = btns.index("プレビュー+録画 (.db3保存)") if "プレビュー+録画 (.db3保存)" in btns else -1
check(i_rec >= 0 and btns[i_rec + 1] == "座標系だけの映像を書き出す（白背景）", "「プレビュー+録画」の右どなり")
cache = app._ankle_get_current_cache()
N = int(cache["frame_count"])
if os.path.exists(OUT): os.remove(OUT)
t0 = time.time()
app.on_ankle_export_axes_video()
pump(app, 1.0)
check(getattr(app, "_ankle_axes_export_busy", False) and not os.path.exists(OUT), "書き出し中は、保存先の名前の半端なファイルを作らない（一時ファイルに書く）")
while getattr(app, "_ankle_axes_export_busy", False) and time.time() - t0 < 900:
	pump(app, 0.2)
dt = time.time() - t0
print(f"   書き出し {dt:.0f} 秒", flush=True)
check(os.path.exists(OUT) and any("書き出しました" in str(m) for m in msgs), "書き出しが終わり、知らせが出る")
cap = cv2.VideoCapture(OUT)
n_out = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); fps = cap.get(cv2.CAP_PROP_FPS)
w_ = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); h_ = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
check(n_out == N and w_ == 1280 and h_ == 720 and abs(fps - 30) < 0.5, f"コマ数 {n_out}/{N}・{w_}x{h_}・{fps:.2f} fps（録画と同じ）")
data = open(OUT, "rb").read()
check(b"avc1" in data and any("H.264" in str(m) for m in msgs), "H.264 で書く（Windows 標準の書き出し）")
import shutil, tempfile
_asc = os.path.join(tempfile.gettempdir(), "frs_axes_check.mp4"); shutil.copy(OUT, _asc)
_mf = cv2.VideoCapture(_asc, cv2.CAP_MSMF); _ok = _mf.isOpened() and _mf.read()[0]; _mf.release(); os.remove(_asc)
check(bool(_ok), "Windows の再生の仕組み（Media Foundation）で開ける")
check(not [f for f in os.listdir(tempfile.gettempdir()) if f.startswith(f"frs_axes_{os.getpid()}_")], "一時ファイルは残らない")
# 元の映像
src = Path(str(cache["source"]))
col = src.with_name(src.stem + "_color.mp4"); aru = src.with_name(src.stem + "_aruco.mp4")
cc = cv2.VideoCapture(str(col)); n_col = int(cc.get(cv2.CAP_PROP_FRAME_COUNT))
print(f"   元の _color.mp4: {n_col} コマ", flush=True)
intr = cache["intrinsics"]
K = np.array([[intr["fx"], 0, intr["cx"]], [0, intr["fy"], intr["cy"]], [0, 0, 1]], float); dist = np.asarray(intr["dist"], float)
size = float(cache["marker_size_mm"]); obj = np.asarray(app._ankle_marker_obj_points(size), float)
dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, str(cache["aruco_dict"])))
det = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters())
fps_col = cc.get(cv2.CAP_PROP_FPS)
print(f"   元の _color.mp4: {fps_col:.2f} fps", flush=True)
# 元の mp4 は録画中の表示間引き（1/n）ごとに1コマ。コマ数の比から n を求め、録画のコマとの対応（ずれ 0〜n-1）を探す
step = int(round(N / max(n_col, 1)))

def corner_err(k, t, want_img=False):
	cc.set(cv2.CAP_PROP_POS_FRAMES, k); ok1, fr = cc.read()
	if not ok1 or t >= N: return [], None
	corners, ids, _ = det.detectMarkers(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY))
	out = []
	if ids is not None:
		for c, i in zip(corners, ids.ravel()):
			b = cache["bones"].get(int(i))
			if b is None or not b["detected"][t]: continue
			T = np.asarray(b["poses"][t], float)
			rv, _ = cv2.Rodrigues(T[:3, :3]); pts, _ = cv2.projectPoints(obj, rv, T[:3, 3], K, dist)
			out.append(np.linalg.norm(pts.reshape(-1, 2) - c.reshape(-1, 2), axis=1).mean())
	return out, fr
cands = list(range(-2 * step, 3 * step))
sc = {o: np.median(sum((corner_err(k, k * step + o)[0] for k in (20, 150, 300)), []) or [1e9]) for o in cands}
best = min(cands, key=lambda o: sc[o])
print("   対応のずれごとの角のずれ（中央値 px）:", {o: round(float(v), 2) for o, v in sorted(sc.items()) if v < 1e8 and (o % 5 == 0 or o == best)}, flush=True)
print(f"   元の mp4 の k コマ目 = 録画の {step}k+{best} コマ目", flush=True)
errs = []; whites = []; shots = []
for k in [int(x) for x in np.linspace(5, n_col - 5, 10)]:
	t = step * k + best
	e, fr = corner_err(k, t)
	errs += e
	cap.set(cv2.CAP_PROP_POS_FRAMES, t); ok2, ax = cap.read()
	if not ok2: continue
	whites.append(np.mean(np.all(ax > 240, axis=2)))
	if len(shots) < 2 and fr is not None:
		shots.append(np.hstack([cv2.resize(fr, (640, 360)), cv2.resize(ax, (640, 360))]))
errs = np.asarray(errs)
print(f"   角のずれ（描いた角 ↔ 元の映像で検出し直した角）: {len(errs)} 個, 中央値 {np.median(errs):.2f} px, 最大 {errs.max():.2f} px", flush=True)
check(len(errs) >= 10 and np.median(errs) < 2.0 and errs.max() < 4.0, "録画と同じ視点（描いた角が元の映像の角と 2px 以内で重なる。mp4 の圧縮と検出器の違いで 1px ほどは出る）")
check(min(whites) > 0.95, f"背景は白（白い画素 {min(whites)*100:.1f}% 以上）")
cv2.imwrite(os.path.join(HERE, "axes_compare.png"), np.vstack(shots))
# 同じ名前のファイルを開いたまま（プレーヤーで再生中）だと置けない → 知らせて、書き出したファイルは残す
hold = open(OUT, "rb"); msgs.clear(); t0 = time.time()
app.on_ankle_export_axes_video()
while getattr(app, "_ankle_axes_export_busy", False) and time.time() - t0 < 900:
	pump(app, 0.2)
hold.close()
m_ = " ".join(str(m) for m in msgs)
kept = re.findall(r"残してあります: (\S+?\.mp4)", m_)
check("閉じて" in m_ and kept and os.path.exists(kept[0]), f"開いたままなら知らせて、書き出したファイルは残す（{kept[:1]}）")
for k_ in kept:
	try: os.remove(k_)
	except Exception: pass
# 2回押しても、書き出し中は二重に始めない
app._ankle_axes_export_busy = True; msgs.clear(); app.on_ankle_export_axes_video(); app._ankle_axes_export_busy = False
check(any("書き出し中" in str(m) for m in msgs), "書き出し中に押すと知らせるだけ")
# ④の結果がないタブ
cur = app._ankle_active_tab
empty = next((i for i, t in enumerate(app._ankle_tabs) if not app._ankle_pose_cache.get(t["name"])), None)
if empty is not None:
	app.on_ankle_tab_select(empty); pump(app, 0.2); msgs.clear(); app.on_ankle_export_axes_video()
	check(any("④の結果" in str(m) for m in msgs), "④の結果がないタブでは知らせる")
print(f"\n合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(1 if fails else 0)
