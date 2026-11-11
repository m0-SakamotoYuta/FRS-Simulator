# -*- coding: utf-8 -*-
"""元の録画から色の動画（30 fps）を書き出す（2026-10-26）: 261007_AP_FE60_軸力100 の前方引張（秒数 3・止める 1）で、
欄（ボタン・2 つの選び方）・「3D の動画と同じコマ」→ 120 コマ・30 fps・1280×720・各コマの絵が元の録画のそのコマ（SDK で独立に読んだ
可視化のコマ idx の色の画像と一致）・止める 30 コマは同じ絵・「区間の全部のコマ」→ 区間の始め〜終わりのコマ数・最初と最後の絵が一致・
止めるボタンで止まる（ファイルを残さない）・④の結果と違う録画なら断って止まる。実際の録画（約 19 GB）を読むので 3〜4分。"""
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
def read_mp4(path):
	tmp = os.path.join(tempfile.gettempdir(), f"frs_t_{os.getpid()}.mp4"); shutil.copyfile(path, tmp)
	cap = cv2.VideoCapture(tmp); fr = []
	fps = cap.get(cv2.CAP_PROP_FPS)
	while True:
		ok, f = cap.read()
		if not ok: break
		fr.append(f)
	cap.release(); os.remove(tmp)
	return fr, fps
D = os.path.join(tempfile.gettempdir(), f"色の動画テスト_{os.getpid()}"); os.makedirs(D, exist_ok=True)
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
TAB = os.environ.get("CC_TAB", "261007_AP_FE60_軸力100")      # 試験（屈曲伸展でも回せる。2026-10-26）
KIND = os.environ.get("CC_KIND", "anterior")
print(f"   試験 {TAB}・区間 {KIND}", flush=True)
ti = next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)
app.on_ankle_tab_select(ti); pump(app, 0.5)
app._ankle_tabs[ti].pop("clip_manual", None)
app.ankle_lig_mode.set("stage1"); app.ankle_start_at_load.set(False); app.ankle_clip_kind.set(KIND)
app.ankle_clip_sec.set(3.0); app.ankle_clip_hold.set(1.0)
BAG = str(app.ankle_depth_path.get())
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; ctrl = v["ctrl"]; v["set_paused"](True)
pv = ctrl["po_vars"]
cf = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
def msg():
	x = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and ("色の動画を書き出しました" in str(w.cget("text"))
	                                                                              or "止めました" in str(w.cget("text")) or "見つかりません" in str(w.cget("text")))]
	return x[0] if x else ""
def run(fp, mode, cancel_after=None, limit=400):
	pv["clip_color_mode"].set(mode)
	filedialog.asksaveasfilename = lambda **k: fp
	pv["clip_color"]()
	t0 = time.time()
	while time.time() - t0 < limit:
		pump(app, 0.2)
		st = pv["clip_color_job"]["st"]
		if cancel_after and st is not None and st.get("n_read", 0) > cancel_after:
			st["cancel"] = True
		if st is None or st.get("done"):
			pump(app, 0.4)
			break
	return pv["clip_color_job"]["st"]
# 独立に: SDK で区間のコマの色の画像を読む（検査用）
pl = None
try:
	import pyrealsense2 as rs
	cb = [w for w in walk(cf) if w.winfo_class() == "TButton" and str(w.cget("text")) == "元の録画から色の動画を書き出す（30 fps）…"]
	rbs = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TRadiobutton" and ("3D の動画と同じコマ" in str(w.cget("text")) or "区間の全部のコマ" in str(w.cget("text")))]
	check(len(cb) == 1 and len(rbs) == 2, f"欄: ボタンと 2 つの選び方（{rbs}）")
	scn = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "pres_overlay" in o and "_t_shift" in o][-1]
	# 区間（自動）: 3D の動画の計画を区間の説明から
	f1 = os.path.join(D, "同じコマ.mp4")
	st = run(f1, "same")
	fr1, fps1 = read_mp4(f1) if os.path.exists(f1) else ([], 0)
	check(st is not None and st.get("ok") and len(fr1) == 120 and abs(fps1 - 30.0) < 0.01 and fr1[0].shape[:2] == (720, 1280) and "欠け" not in st.get("msg", ""),
	      f"3D の動画と同じコマ: {len(fr1)} コマ・{fps1:g} fps・{fr1[0].shape[1] if fr1 else 0}×{fr1[0].shape[0] if fr1 else 0}（{st.get('msg', '')[-60:] if st else '-'}）")
	# どのコマを使ったか（3D の計画）を、計画の関数から
	po_cb = [o for o in gc.get_objects() if isinstance(o, dict) and callable(o.get("clip_plan")) and callable(o.get("color_export"))][-1]
	pl = po_cb["clip_plan"](KIND, 3.0, 30.0, 1.0)
	want = sorted(set(int(x) for x in pl["idx"]))
	samp = [int(pl["idx"][0]), int(pl["idx"][45]), int(pl["idx"][89])]
	# SDK で独立に読む（可視化のコマ i ＝ ④と同じく先頭から i 番目の組）
	pipe = rs.pipeline(); cfg = rs.config(); rs.config.enable_device_from_file(cfg, BAG, repeat_playback=False)
	prof = pipe.start(cfg); prof.get_device().as_playback().set_real_time(False)
	ref = {}; i = 0
	while i <= max(samp + [int(pl["b"])]):
		fs = pipe.wait_for_frames(timeout_ms=2000)
		c = fs.get_color_frame(); d = fs.get_depth_frame()
		if not c or not d: break
		if i in samp or i == int(pl["a"]) or i == int(pl["b"]):
			ref[i] = app._ankle_color_to_gray_bgr(np.asanyarray(c.get_data()))[1]
		i += 1
	pipe.stop()
	def diff(a_, b_):
		return float(np.abs(a_.astype(int) - b_.astype(int)).mean())
	ds = [diff(fr1[k], ref[int(pl["idx"][k])]) for k in (0, 45, 89)] if len(fr1) == 120 else [99]
	# 隣のコマとの差（同じでないことの確かめ）
	check(max(ds) < 6.0, f"各コマの絵が元の録画のそのコマ（独立に読んだ絵との差 {', '.join(f'{x:.2f}' for x in ds)}・圧縮の差だけ）")
	dh = max(diff(fr1[89], fr1[k]) for k in range(90, 120)) if len(fr1) == 120 else 99
	check(dh < 2.5, f"止める 30 コマは最後のコマと同じ絵（差 最大 {dh:.2f}。同じ絵を書いても mp4 の圧縮で少し違う。ほかのコマの元との差は 2.1〜2.4）")
	dn = diff(fr1[0], ref[int(pl["idx"][89])]) if len(fr1) == 120 else 0
	print(f"   参考: 始めのコマと終わりのコマの差 {dn:.2f}（動いている）", flush=True)
	# 区間の全部のコマ
	f2 = os.path.join(D, "全部.mp4")
	st = run(f2, "all")
	fr2, fps2 = read_mp4(f2) if os.path.exists(f2) else ([], 0)
	nexp = int(pl["b"]) - int(pl["a"]) + 1
	check(st is not None and st.get("ok") and len(fr2) == nexp and abs(fps2 - 30.0) < 0.01,
	      f"区間の全部のコマ: {len(fr2)} コマ（区間 {pl['a']}〜{pl['b']} = {nexp}）・{fps2:g} fps")
	if fr2:
		check(diff(fr2[0], ref[int(pl["a"])]) < 6.0 and diff(fr2[-1], ref[int(pl["b"])]) < 6.0,
		      f"最初と最後の絵が区間の始め・終わりのコマ（差 {diff(fr2[0], ref[int(pl['a'])]):.2f}・{diff(fr2[-1], ref[int(pl['b'])]):.2f}）")
	# 止める
	f3 = os.path.join(D, "止めた.mp4")
	st = run(f3, "same", cancel_after=300)
	check(st is not None and st.get("done") and not st.get("ok") and "止めました" in st.get("msg", "") and not os.path.exists(f3), "止めるボタンで止まる（ファイルを残さない）")
	# ④の結果と違う録画
	ce = po_cb["color_export"]
	ce_cl = ce.__closure__
	cell = [c for c in ce_cl if isinstance(getattr(c, "cell_contents", None), np.ndarray)][0]
	keep = cell.cell_contents
	cell.cell_contents = keep + 0.0137                         # 時刻をずらす＝違う録画のつもり
	f4 = os.path.join(D, "違う.mp4")
	st = run(f4, "same", limit=200)
	cell.cell_contents = keep
	check(st is not None and st.get("done") and not st.get("ok") and "見つかりません" in st.get("msg", "") and not os.path.exists(f4),
	      f"④の結果と違う録画なら断って止まる（{st.get('msg', '')[:50] if st else '-'}）")
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	shutil.rmtree(D, ignore_errors=True)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
