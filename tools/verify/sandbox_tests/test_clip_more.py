# -*- coding: utf-8 -*-
"""発表用の動画（2026-10-22）: 終わったあと止める秒数（3D・グラフ・数字の動画で同じ長さ）・右上の文字を 3D の動画に入れるか・
右上の数字だけの動画（白地・文字に合わせた横長・文字の大きさ・プレビュー・一緒に／だけ書き出す）。
接触の計算はダミー、datalog は本物（261007_AP_FE60_軸力100）。2〜4分。"""
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
app.ankle_clip_size.set(400); app.ankle_clip_sec.set(2.0); app.ankle_clip_kind.set("anterior")
app.ankle_clip_hold.set(0.0); app.ankle_clip_text_on.set(True); app.ankle_clip_nums.set(False); app.ankle_clip_nums_fs.set("60,48")
app.ankle_clip_chart.set(True); app.ankle_clip_lig_on.set(True)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
cf = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
btn = {str(w.cget("text")): w for w in walk(cf) if w.winfo_class() == "TButton"}
pv_ = ctrl["po_vars"]
def info_text():
	t = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and "倍速" in str(w.cget("text"))]
	return t[0] if t else ""
# ---- 止める秒数
check("clip_hold" in pv_, "「終わったあと … 秒 止める」の欄")
pv_["clip_hold"].set("1"); [w for w in walk(cf) if w.winfo_class() == "TSpinbox" and str(w.cget("textvariable")) == str(pv_["clip_hold"])][0].event_generate("<FocusOut>"); pump(app, 0.3)
check("止める 1 秒" in info_text() and "3.0 秒・90 コマ" in info_text(), f"説明に止める秒数と全体の長さ（{info_text()}）")
check(abs(app.ankle_clip_hold.get() - 1.0) < 1e-9, "止める秒数を覚える")
# ---- 右上の文字を 3D の動画に入れるか
def text_vis():
	props = p.renderer.GetViewProps(); props.InitTraversal(); n_ = 0
	for _ in range(props.GetNumberOfItems()):
		a_ = props.GetNextProp()
		if isinstance(a_, vtk.vtkTextActor) and a_.GetVisibility() and ("引張中" in str(a_.GetInput()) or "初期状態" in str(a_.GetInput()) or "接触圧" in str(a_.GetInput())):
			n_ += 1
	return n_
lig_ctx_acts = [o for o in gc.get_objects() if isinstance(o, dict) and isinstance(o.get("_ligs"), list)]
ligs = lig_ctx_acts[-1]["_ligs"] if lig_ctx_acts else []
for lg in ligs:
	lg["actor"].SetVisibility(1)                     # 試しに靭帯を見せておく
p.render()
n_lig0 = sum(1 for lg in ligs if lg["actor"].GetVisibility())
btn["① 正方形にする"].invoke(); pump(app, 0.5)
n_lig1 = sum(1 for lg in ligs if lg["actor"].GetVisibility())
check(n_lig0 > 0 and n_lig1 == n_lig0, f"「靭帯も映す」なら正方形でも靭帯が残る（{n_lig0} → {n_lig1} 本）")
lab_vis = sum(1 for lg in ligs if lg.get("label") is not None and lg["label"].GetVisibility())
check(lab_vis == 0, f"靭帯の名前の文字は隠す（見えている {lab_vis}）")
cbl = [w for w in walk(cf) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")) == "靭帯も映す"][0]
cbl.invoke(); pump(app, 0.3)
check(sum(1 for lg in ligs if lg["actor"].GetVisibility()) == 0 and app.ankle_clip_lig_on.get() is False, "外すと正方形の靭帯が消える・覚える")
cbl.invoke(); pump(app, 0.3)
check(sum(1 for lg in ligs if lg["actor"].GetVisibility()) == n_lig0, "また付けると靭帯が戻る")
# ② その場で再生（保存しない）
t0p = time.time()
btn["② その場で再生して確かめる"].invoke()
while time.time() - t0p < 60:
	pump(app, 0.05)
	m_ = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and "再生しました" in str(w.cget("text"))]
	if m_: break
elp = time.time() - t0p
check(bool(m_) and "90 / 90 コマ" in m_[0] and elp >= 2.5, f"② その場で再生: 90 コマを {elp:.1f} 秒で流した（書き出す動画は 3.0 秒）・{m_[0][:50] if m_ else '-'}")
check(not any(f.endswith(".mp4") for f in os.listdir(HERE)), "② では何も保存しない")
btn["区間の終わりへ"].invoke(); pump(app, 0.3)
n_on = text_vis()
cb_text = [w for w in walk(cf) if w.winfo_class() == "TCheckbutton" and "右上の文字を3Dの動画に入れる" in str(w.cget("text"))]
check(len(cb_text) == 1 and n_on >= 1, f"「右上の文字を3Dの動画に入れる」（既定は入れる・見えている文字 {n_on}）")
cb_text[0].invoke(); pump(app, 0.3)
n_off = text_vis()
check(n_off == 0 and app.ankle_clip_text_on.get() is False, f"外すと正方形の 3D から右上の文字が消える（{n_off}）・覚える")
# ---- 数字の動画のプレビュー
nb = [w for w in walk(cf) if w.winfo_class() == "TButton" and "数字のプレビュー" in str(w.cget("text"))]
nb[0].invoke(); pump(app, 0.6)
nw = pv_["clip_nums_prev_win"]["w"]
check(nw is not None and nw.winfo_exists() and getattr(nw, "photo", None) is not None, "「数字のプレビュー…」で窓に出る")
g0 = nw.g
W0, H0 = g0["size"]
check(g0["n"] == 90 and W0 % 2 == 0 and H0 % 2 == 0 and W0 > H0, f"数字の動画: 3D と同じ 90 コマ・横長（{W0}×{H0}）・{g0['info']}")
def colors(img):
	r = ((img[:, :, 0] > 150) & (img[:, :, 1] < 80)).sum(); gr = ((img[:, :, 1] > 100) & (img[:, :, 0] < 80) & (img[:, :, 2] < 100)).sum()
	k = ((img.max(axis=2) < 60)).sum()
	return int(r), int(gr), int(k)
r0, g0c, k0 = colors(g0["frame"](0)); r1, g1c, k1 = colors(g0["frame"](g0["n"] - 1))
check(g0c > 50 and g0c > r0, f"最初のコマは緑の文字（初期状態）: 緑 {g0c}・赤 {r0}")
check(r1 > 50 and r1 > g1c and k1 > 30, f"最後のコマは赤の文字（前方引張中：+200 N）と黒の接触圧: 赤 {r1}・黒 {k1}")
img_white = g0["frame"](0)
check(float((img_white.min(axis=2) > 245).mean()) > 0.6, "白地")
for sb_, val in zip([w for w in walk(cf) if w.winfo_class() == "TSpinbox" and str(w.cget("textvariable")) in [str(x) for x in pv_["clip_nums_fs"]]], ("90", "60")):
	sb_.set(val)
	sb_.event_generate("<FocusOut>"); pump(app, 0.4)
g1 = nw.g
check(g1["size"][1] > H0 * 1.3 and app.ankle_clip_nums_fs.get() == "90,60", f"文字を大きくすると開いている窓も描き直す（{g0['size']} → {g1['size']}）・覚える（{app.ankle_clip_nums_fs.get()}）")
# ---- 書き出す（3D・グラフ・数字の 3 つ）
outdir = os.path.join(HERE, "動画の試し2"); shutil.rmtree(outdir, ignore_errors=True); os.makedirs(outdir)
import tkinter.filedialog as fd
path = os.path.join(outdir, "試し_前方引張.mp4")
fd.asksaveasfilename = lambda *a, **k: path
cbn = [w for w in walk(cf) if w.winfo_class() == "TCheckbutton" and "一緒に書き出す" in str(w.cget("text"))]
cbn[0].invoke(); pump(app, 0.2)
check(app.ankle_clip_nums.get() is True, "「② で 3D の動画と一緒に書き出す」をオン（覚える）")
t0 = time.time()
btn["③ mp4 に書き出す…"].invoke()
npath = os.path.splitext(path)[0] + "_数字.mp4"
while time.time() - t0 < 300:
	pump(app, 0.2)
	if os.path.exists(npath):
		pump(app, 0.5); break
def info(fp):
	if not os.path.exists(fp): return 0, (0, 0), None, None
	cap = cv2.VideoCapture(fp); n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
	wh = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
	frs_ = []
	ok, f = cap.read()
	while ok:
		frs_.append(f); ok, f = cap.read()
	cap.release()
	return n, wh, frs_[0] if frs_ else None, frs_
n3, wh3, f3, all3 = info(path)
nc, whc, fc, allc = info(os.path.splitext(path)[0] + "_靭帯の長さ.mp4")
nn, whn, fn, alln = info(npath)
check(n3 == 90 and nc == 90 and nn == 90, f"3 つとも 90 コマ（2 秒 ＋ 止める 1 秒）: 3D {n3}・グラフ {nc}・数字 {nn}")
dh = [float(np.abs(all3[-1].astype(int) - all3[-k].astype(int)).mean()) for k in (2, 10, 30, 31, 40)] if all3 else []
d0 = float(np.abs(all3[-31].astype(int) - all3[0].astype(int)).mean()) if all3 else 0
print("   最後のコマとの差（-2,-10,-30,-31,-40）", [round(x, 2) for x in dh], "始めと止める直前", round(d0, 2), flush=True)
hold_same = all3 and max(dh[:4]) < 0.5 and d0 > 0.2      # 2 秒の短い動画なので、始めと止める直前の差は小さい
check(bool(hold_same), f"3D の動画の最後の 1 秒（30 コマ）は止まっている（差 {[round(x, 2) for x in dh[:3]]}）")
hold_c = allc and float(np.abs(allc[-1].astype(int) - allc[-30].astype(int)).mean()) < 1.0
check(bool(hold_c), "グラフの動画も最後の 1 秒は止まっている（縦の線が終わりのまま）")
check(whn == tuple(g1["size"]), f"数字の動画の大きさはプレビューと同じ（{whn}）")
top3 = f3[: int(400 * 0.12), int(400 * 0.4):] if f3 is not None else None
txt_in_3d = top3 is not None and (((top3[:, :, 1].astype(int) > 100) & (top3[:, :, 2].astype(int) < 80) & (top3[:, :, 0].astype(int) < 100)).sum() > 20)
check(not txt_in_3d, "右上の文字を外したので、3D の動画の右上に文字がない")
if fn is not None:
	ref = cv2.cvtColor(nw.g["frame"](0), cv2.COLOR_RGB2BGR)
	d_ = float(np.abs(fn.astype(int) - ref.astype(int)).mean())
	check(d_ < 4.0, f"数字の動画の最初のコマはプレビューと同じ（平均の差 {d_:.2f}）")
msgs = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and "書き出しました" in str(w.cget("text"))]
check(msgs and "数字:" in msgs[0] and "グラフ:" in msgs[0], f"知らせに 3 つ（{msgs[0][:90] if msgs else '-'}…）")
# 数字の動画だけ
p2 = os.path.join(outdir, "数字だけ.mp4")
fd.asksaveasfilename = lambda *a, **k: p2
nob = [w for w in walk(cf) if w.winfo_class() == "TButton" and "数字の動画だけ書き出す" in str(w.cget("text"))]
nob[0].invoke(); pump(app, 0.5)
n2, wh2, _f, _a = info(p2)
check(n2 == 90 and wh2 == tuple(g1["size"]), f"「数字の動画だけ書き出す…」: {n2} コマ・{wh2}")
# 文字を戻す
cb_text[0].invoke(); pump(app, 0.3)
check(text_vis() >= 1 and app.ankle_clip_text_on.get() is True, "チェックを戻すと右上の文字がまた出る")
btn["元の大きさに戻す"].invoke(); pump(app, 0.4)
try: nw.destroy()
except Exception: pass
shutil.rmtree(outdir, ignore_errors=True)
w_ = ctrl["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_clip_size.set(900); app.ankle_clip_sec.set(10.0); app.ankle_clip_hold.set(0.0); app.ankle_clip_nums.set(False); app.ankle_clip_nums_fs.set("60,48")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
