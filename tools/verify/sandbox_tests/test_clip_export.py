# -*- coding: utf-8 -*-
"""発表用の動画（mp4）の書き出し（2026-10-20）: 区間（前方引張 0→+200 N・後方引張 0→−200 N）を datalog の Fap から求める・
可視化ウィンドウを正方形にする（見たまま）・秒数から倍率・mp4（正方形・30 fps・日本語のパス）・右上の段階の表示が区間に合う・
設定を覚える・元の大きさに戻す。
10/20 改め: 正方形は骨と右上の文字だけ（ヒートマップ・色の帯・グラフ・靭帯は隠す）・始めは初期状態・最後は ±200 N・
ヒートマップの画像（始め・終わり）・靭帯の長さのグラフも同じコマ数の mp4。接触の計算はダミー、datalog は本物（261007_AP_FE60_軸力100）。2〜3分。"""
import importlib.util, os, sys, time, gc, shutil
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, vtk, cv2
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
	{idx: ((np.asarray(m.points)[:, 0] % 3.0) - 1.0).astype(np.float32) for (idx, n, m, T) in bd} for t in range(len(bd[0][3]))]
app._ankle_heatmap_cache_load = lambda *a, **k: None
app._ankle_heatmap_cache_save = lambda *a, **k: None
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.ankle_dilate_mm.set(0.15); app.ankle_cp_ha.set(2.8); app.ankle_cp_h.set(1.6); app.ankle_lig_mode.set("stage1")
app.ankle_po_force_on.set(True); app.ankle_po_contact_on.set(True); app.ankle_start_at_load.set(False)
app.ankle_clip_size.set(900); app.ankle_clip_sec.set(10.0); app.ankle_clip_kind.set("anterior")
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
scn = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "heatmap" in o and "_t_shift" in o][-1]
ft = np.asarray(scn["frame_times"], float)
po = scn["pres_overlay"]
log = app._ankle_robot_log_load(str(app.ankle_robot_log_path.get()))
off = scn["_log_offset"] if scn.get("_log_offset") is not None else float(app.ankle_robot_log_offset_s.get())
fap = np.interp(ft, np.asarray(log["t"]) + off - float(scn["_t_shift"]), np.asarray(log["cols"]["Fap"], float), left=np.nan, right=np.nan)
cl = po["clips"]()
check(set(cl) == {"anterior", "posterior"}, f"区間が 2 つ見つかる（{cl}）")
a0, a1 = cl["anterior"]; b0, b1 = cl["posterior"]
check(abs(fap[a0]) < 15 and fap[a1] >= np.nanmax(fap) - 6, f"前方引張の区間: 0 N 付近（{fap[a0]:+.1f}）→ 最大（{fap[a1]:+.1f} / {np.nanmax(fap):+.1f} N）")
check(-15 < fap[b0] < 15 and fap[b1] <= np.nanmin(fap) + 6, f"後方引張の区間: 0 N 付近（{fap[b0]:+.1f}）→ 最小（{fap[b1]:+.1f} / {np.nanmin(fap):+.1f} N）")
check(a1 < b0, "前方の区間のあとに後方の区間")
check(np.all(np.diff(fap[b0:b1 + 1][::20]) < 5), "後方の区間の中は Fap が減っていく（+200→0 は入らない）")
# 再生コントロール
cf = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))]
check(len(cf) == 1, "再生コントロールに「発表用の動画（mp4）」の欄")
btn = {str(w.cget("text")): w for w in walk(cf[0]) if w.winfo_class() == "TButton"}
lbls = [str(w.cget("text")) for w in walk(cf[0]) if w.winfo_class() == "TLabel"]
info_l = [t for t in lbls if "倍速" in t]
a0s = int(np.searchsorted(ft, ft[a0] - 2.0))          # 初期状態を 2 秒見せてから
T_a = ft[a1] - ft[a0s]
check(info_l and f"×{T_a / 10:.2f} 倍速" in info_l[0], f"秒数（10 秒）から倍率を出す（{info_l[0] if info_l else '-'}）")
pv_ = ctrl["po_vars"]
pv_["clip_sec"].set("4"); [w for w in walk(cf[0]) if w.winfo_class() == "TSpinbox"][1].event_generate("<FocusOut>"); pump(app, 0.2)
info_l = [str(w.cget("text")) for w in walk(cf[0]) if w.winfo_class() == "TLabel" and "倍速" in str(w.cget("text"))]
check(info_l and "120 コマ" in info_l[0] and f"×{T_a / 4:.2f}" in info_l[0], f"秒数を変えると倍率とコマ数が変わる（{info_l[0] if info_l else '-'}）")
# 正方形
orig = tuple(p.window_size)
pv_["clip_size"].set("401")
btn["① 正方形にする（プレビュー）"].invoke(); pump(app, 0.5)
check(tuple(p.window_size) == (400, 400) and pv_["clip_size"].get() == "400", f"可視化ウィンドウを正方形に（奇数は偶数へ: {tuple(p.window_size)}）")
def vis_props():
	out = []
	props = p.renderer.GetViewProps(); props.InitTraversal()
	for _ in range(props.GetNumberOfItems()):
		a_ = props.GetNextProp()
		if a_.GetVisibility():
			out.append(a_)
	return out
bone_acts = [a_ for k_, a_ in p.renderer.actors.items() if str(k_).startswith("bone_")]
vp = vis_props()
others = [a_ for a_ in vp if a_ not in bone_acts and not (isinstance(a_, vtk.vtkTextActor) and ("引張中" in str(a_.GetInput()) or "初期状態" in str(a_.GetInput()) or "接触圧" in str(a_.GetInput())))]
check(not others, f"正方形の間は骨と右上の文字だけ（ほかに見えている物 {len(others)}: {[type(o).__name__ for o in others][:5]}）")
check(all(not a_.GetMapper().GetScalarVisibility() for a_ in bone_acts if a_.GetVisibility()), "骨にヒートマップの色を出さない（骨の色）")
btn["区間の始めへ"].invoke(); pump(app, 0.3)
def stage_text():
	props = p.renderer.GetViewProps(); props.InitTraversal()
	for _ in range(props.GetNumberOfItems()):
		a_ = props.GetNextProp()
		if isinstance(a_, vtk.vtkTextActor) and a_.GetVisibility() and ("引張中" in str(a_.GetInput()) or "初期状態" in str(a_.GetInput())):
			return str(a_.GetInput())
	return ""
fi_now = int(np.searchsorted(ft, float(v["time"]()) + 1e-9, side="right") - 1)
check(abs(fi_now - a0s) <= 1 and stage_text() == "初期状態", f"「区間の始めへ」で前方引張の 2 秒前（初期状態）へ（{stage_text()}・{ft[a0] - ft[fi_now]:.2f} 秒前）")
btn["区間の終わりへ"].invoke(); pump(app, 0.3)
check(stage_text() == "前方引張中：+200 N", f"「区間の終わりへ」は「前方引張中：+200 N」（実際 {fap[a1]:+.1f} N・{stage_text()}）")
# ヒートマップの画像
hb = [w for w in walk(cf[0]) if w.winfo_class() == "TButton" and "ヒートマップの画像" in str(w.cget("text"))]
check(len(hb) == 1, "「ヒートマップの画像を保存」のボタン")
outdir0 = os.path.join(HERE, "動画の試し"); shutil.rmtree(outdir0, ignore_errors=True); os.makedirs(outdir0)
import tkinter.filedialog as fd
hp = os.path.join(outdir0, "試し_ヒートマップ.png")
fd.asksaveasfilename = lambda *a, **k: hp
hb[0].invoke(); pump(app, 0.5)
pngs = [os.path.join(outdir0, f"試し_ヒートマップ_{t}.png") for t in ("始め", "終わり")]
imgs = [cv2.imdecode(np.fromfile(q, np.uint8), cv2.IMREAD_COLOR) if os.path.exists(q) else None for q in pngs]
colorful = [im is not None and im.shape[:2] == (400, 400) and float(np.mean(np.abs(im[:, :, 2].astype(int) - im[:, :, 0].astype(int)) > 60)) > 0.002 for im in imgs]
check(all(colorful), f"ヒートマップの画像を 2 枚（始め・終わり、400×400、色つき）: {[os.path.basename(q) for q in pngs]} {colorful}")
vp = vis_props()
others = [a_ for a_ in vp if a_ not in bone_acts and not (isinstance(a_, vtk.vtkTextActor) and ("引張中" in str(a_.GetInput()) or "初期状態" in str(a_.GetInput()) or "接触圧" in str(a_.GetInput())))]
check(not others and all(not a_.GetMapper().GetScalarVisibility() for a_ in bone_acts if a_.GetVisibility()), "画像を保存したあとは、また骨と右上の文字だけに戻る")
# 書き出し（前方・後方）
outdir = outdir0
for kind, lab in (("anterior", "前方引張"), ("posterior", "後方引張")):
	path = os.path.join(outdir, f"試し_{lab}.mp4")
	fd.asksaveasfilename = lambda *a, _p=path, **k: _p
	pv_["clip_kind"].set(kind)
	t0 = time.time()
	btn["② mp4 に書き出す…"].invoke()
	while time.time() - t0 < 300:
		pump(app, 0.2)
		if os.path.exists(path):
			break
	pump(app, 0.5)
	ok = os.path.exists(path)
	cap = cv2.VideoCapture(path) if ok else None
	n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if cap else 0
	w_ = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) if cap else 0
	h_ = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) if cap else 0
	fps = cap.get(cv2.CAP_PROP_FPS) if cap else 0
	okf, fr = cap.read() if cap else (False, None)
	if cap: cap.release()
	if okf:
		top = fr[: int(400 * 0.09), int(400 * 0.45):]
		g = ((top[:, :, 1].astype(int) > 100) & (top[:, :, 2].astype(int) < 80) & (top[:, :, 0].astype(int) < 100)).sum()
		r = ((top[:, :, 2].astype(int) > 150) & (top[:, :, 1].astype(int) < 80)).sum()
		b = ((top[:, :, 0].astype(int) > 150) & (top[:, :, 2].astype(int) < 80)).sum()
		check(g > 20 and g > r and g > b, f"{lab}の動画の最初のコマは緑の文字（初期状態）: 緑 {g}・赤 {r}・青 {b} 画素")
	check(ok and 115 <= n <= 125 and (w_, h_) == (400, 400) and abs(fps - 30) < 0.5 and okf and fr.std() > 5,
	      f"{lab}の mp4（日本語のパス）: {n} コマ・{w_}×{h_}・{fps:.1f} fps・{time.time() - t0:.1f} 秒で書けた")
	st = stage_text()
	exp_end = f"{lab}中：{'+200' if kind == 'anterior' else '−200'} N"
	check(st == exp_end, f"書き出しの終わりのコマの右上は「{exp_end}」（{st}）")
	cpath = os.path.splitext(path)[0] + "_靭帯の長さ.mp4"
	cap = cv2.VideoCapture(cpath) if os.path.exists(cpath) else None
	nc = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if cap else 0
	fc = cap.get(cv2.CAP_PROP_FPS) if cap else 0
	okc, frc = cap.read() if cap else (False, None)
	frs_ = []
	if cap:
		cap.set(cv2.CAP_PROP_POS_FRAMES, nc - 1); frs_ = [frc, cap.read()[1]]; cap.release()
	moved = len(frs_) == 2 and frs_[1] is not None and float(np.abs(frs_[0].astype(int) - frs_[1].astype(int)).mean()) > 0.05
	check(cap is not None and nc == n and abs(fc - 30) < 0.5 and okc and moved, f"靭帯の長さのグラフの mp4 も同じ {nc} コマ・{fc:.1f} fps（縦の線が動く: {moved}）")
msgs = [str(w.cget("text")) for w in walk(cf[0]) if w.winfo_class() == "TLabel" and "書き出しました" in str(w.cget("text"))]
check(len(msgs) == 1 and "×" in msgs[0], f"終わったら知らせる（{msgs[0][:60] if msgs else '-'}…）")
check(app.ankle_clip_size.get() == 400 and abs(app.ankle_clip_sec.get() - 4.0) < 1e-9 and app.ankle_clip_kind.get() == "posterior"
      and os.path.normcase(app.ankle_clip_dir.get()) == os.path.normcase(outdir), "大きさ・秒数・区間・保存先を覚える")
btn["元の大きさに戻す"].invoke(); pump(app, 0.4)
check(tuple(p.window_size) == orig, f"元の大きさに戻せる（{tuple(p.window_size)}）")
check(any(a_.GetMapper().GetScalarVisibility() for a_ in bone_acts if a_.GetVisibility()) and len(p.scalar_bars) >= 1
      and any(a_.GetVisibility() for a_ in vis_props() if type(a_).__name__ == "vtkScalarBarActor"), "元に戻すとヒートマップと色の帯も戻る")
w_ = ctrl["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_clip_size.set(900); app.ankle_clip_sec.set(10.0); app.ankle_clip_kind.set("anterior"); app.ankle_clip_dir.set("")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
