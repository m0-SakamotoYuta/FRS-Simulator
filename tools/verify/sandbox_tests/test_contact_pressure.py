# -*- coding: utf-8 -*-
"""接触圧（推定。2026-10-19 改め）: 骨の色・めり込み量はそのまま、色の帯の下に接触圧（MPa）の目盛りを書き足すだけ。
帯の上 = 「distance [mm]」と距離の目盛り、帯の下 = 同じ位置の接触圧の数字と「contact pressure [MPa]」（文字はエンジンが置く）。
p = H_A × めり込み ÷ 厚さ・0 は断る・覚える・範囲を変えても／ヒートマップを外して戻しても／膨張しても目盛りが付く・
外すと消える。接触の計算はダミー（距離が分かっている）。1〜2分。"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk
from vtk.util.numpy_support import vtk_to_numpy
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a[:2])[:200])
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
def fake_multi(bd, update_progress=None, cancel_var=None, quiet=False):
	N = len(bd[0][3])
	base = {idx: ((np.asarray(m.points)[:, 0] % 3.0) - 1.5).astype(np.float32) for (idx, n, m, T) in bd}   # −1.5〜+1.5 mm
	return [{idx: base[idx] for idx in base} for t in range(N)]
app._sim_view_precompute_multi_heatmap = fake_multi
# 保存済みの接触の結果は使わない・ダミーは保存しない（いつもダミーの距離で確かめる）
app._ankle_heatmap_cache_load = lambda *a, **k: None
app._ankle_heatmap_cache_save = lambda *a, **k: None
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)
def close(v):
	w = v["ctrl"]["window"]; w.tk.call(w.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
def texts(p):
	"""見えている 2D の文字 [(文字, x, y)]"""
	import vtk as _vtk
	out = []
	props = p.renderer.GetViewProps(); props.InitTraversal()
	for _ in range(props.GetNumberOfItems()):
		a_ = props.GetNextProp()
		if isinstance(a_, _vtk.vtkTextActor) and a_.GetVisibility():
			x_, y_ = a_.GetPosition()
			out.append((str(a_.GetInput()), float(x_), float(y_)))
	return out
def bar(p):
	"""(帯の上の題名, {距離の目盛り: x}, {接触圧の数字の x: 値}, 帯の下の名前があるか)。接触圧を出していなければ (pyvista の題名, {}, {}, False)"""
	p.render()
	T = texts(p)
	lab = [t for t in T if t[0] == "contact pressure [MPa]"]
	if not lab:
		t_ = list(p.scalar_bars.keys())[0] if p.scalar_bars else None
		return t_, {}, {}, False
	yl = lab[0][2]
	def _isnum(x):
		try:
			float(x); return True
		except ValueError:
			return False
	title = next((t[0] for t in T if t[0].startswith("distance")), None)      # 靭帯の名前などの文字は除く
	nums = [t for t in T if _isnum(t[0])]
	# 帯の下（接触圧）は名前より上・帯より下、帯の上（距離・題名）はそれより上
	low = [t for t in nums if t[2] < yl + 60 and t[2] > yl]
	ylow = min(t[2] for t in low) if low else None
	pn = {round(t[1]): float(t[0]) for t in nums if ylow is not None and abs(t[2] - ylow) < 1}
	up = [t for t in nums if ylow is None or t[2] > ylow + 1]
	ytk = min(t[2] for t in up)
	tk_ = {float(t[0]): round(t[1]) for t in up if abs(t[2] - ytk) < 1}
	return title, tk_, pn, True
def ann_ok(r, HA, h, lo):
	title, tk_, pn, has = r
	if not (has and tk_ and pn):
		return False
	# 接触圧の数字は、距離の目盛り（0 以下）の真下にあり、値 = H_A × めり込み ÷ 厚さ
	for v, x in tk_.items():
		if v <= 0:
			if x not in pn or abs(pn[x] - HA * (-v) / h) > 0.006:
				return False
	return abs(min(tk_) - lo) < 0.006 and len(pn) >= 2
def scal(acts, k0):
	return vtk_to_numpy(acts[k0].GetMapper().GetInput().GetPointData().GetScalars()).astype(float)

TAB = "261007_FE_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.ankle_cp_on.set(False); app.ankle_cp_ha.set(0.0); app.ankle_cp_h.set(0.0)
app.ankle_dilate_mm.set(0.0); app.ankle_hm_show.set(True); app.ankle_alt_model.set(False); app.ankle_joint_only.set(False)
app.ankle_lig_mode.set("stage1")
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]; acts = p.renderer.actors
v["seek"](0.0); pump(app, 0.3)
k0 = [k for k in acts.keys() if str(k).startswith("bone_") and str(k).count("_") == 1][0]
S0 = scal(acts, k0)
rng0 = acts[k0].GetMapper().GetScalarRange()
r_ = bar(p)
check(not r_[3] and p.scalar_bars and list(p.scalar_bars.values())[0].GetDrawTickLabels(), f"外しているときは目盛りなし・帯は元のまま（題名「{r_[0]}」）")
cpv = ctrl.get("cp_vars")
check(cpv is not None and set(cpv) == {"on", "HA", "h"}, "再生コントロールに「接触圧（推定）」の欄（H_A・軟骨の厚さ。色の上限は無し）")
fr_ = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "接触圧" in str(w.cget("text"))][0]
btn = [w for w in walk(fr_) if w.winfo_class() == "TButton" and str(w.cget("text")) == "適用"][0]
cb = [w for w in walk(fr_) if w.winfo_class() == "TCheckbutton"][0]
check("書き足す" in str(cb.cget("text")), f"チェックの文字「{cb.cget('text')}」")
# 0 のままでは断る
cpv["on"].set(True); btn.invoke(); pump(app, 0.2)
msg = [w for w in walk(fr_) if w.winfo_class() == "TLabel" and "入れてください" in str(w.cget("text"))]
check(not cpv["on"].get() and msg and not bar(p)[3], "H_A・厚さが 0 のままだと断る（チェックは外れる・理由を出す・目盛りなし）")
# H_A 0.6 MPa・厚さ 2 mm
cpv["HA"].set("0.6"); cpv["h"].set("2"); cpv["on"].set(True); btn.invoke(); pump(app, 0.4)
v["seek"](0.0); pump(app, 0.3)
S1 = scal(acts, k0)
check(np.allclose(S1, S0) and acts[k0].GetMapper().GetScalarRange() == rng0,
      "骨の色の値（めり込み量）と色の範囲は変わらない")
r_ = bar(p)
lo = float(rng0[0])
check(r_[0] == "distance [mm]", f"帯の上の題名は「distance [mm]」だけ（{r_[0]}）")
check(ann_ok(r_, 0.6, 2.0, lo), f"帯の下に、距離の目盛りの真下で接触圧 = 0.6 × めり込み ÷ 2（距離 {r_[1]}・接触圧 {r_[2]}）")
txt = str(ctrl["max_pent_label"].cget("text"))
check(txt.startswith("Max Pent:"), f"めり込みの最大の表示はそのまま（{txt}）")
check(app.ankle_cp_on.get() and abs(app.ankle_cp_ha.get() - 0.6) < 1e-9 and abs(app.ankle_cp_h.get() - 2.0) < 1e-9,
      "設定はタブに覚える")
# 範囲を変えても目盛りが付く（新しい範囲で）
hr = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "ヒートマップの範囲" in str(w.cget("text"))]
if hr:
	ents = [w for w in walk(hr[0]) if w.winfo_class() in ("TEntry", "TSpinbox")]
	if ents:
		ents[0].delete(0, "end"); ents[0].insert(0, "1")
	[w for w in walk(hr[0]) if w.winfo_class() == "TButton" and str(w.cget("text")) == "適用"][0].invoke(); pump(app, 0.3)
	lo2 = float(acts[k0].GetMapper().GetScalarRange()[0])
	r_ = bar(p)
	check(ann_ok(r_, 0.6, 2.0, lo2), f"ヒートマップの範囲を適用しても目盛りが付く（範囲 {lo2:g}〜・{r_[1]}・{r_[2]}）")
	lo = lo2
# ヒートマップを外して戻す
if v.get("set_heatmap_visible"):
	v["set_heatmap_visible"](False); pump(app, 0.2)
	check(not bar(p)[3], "ヒートマップを外すと目盛りの文字も消える")
	v["set_heatmap_visible"](True); pump(app, 0.2)
	check(ann_ok(bar(p), 0.6, 2.0, lo), "ヒートマップを外して戻しても目盛りが付く")
close(v)
# 次に開くと目盛りつきで始まる・膨張しても目盛りが付く・色は距離 − 2δ
app.ankle_dilate_mm.set(0.1); app.ankle_dil_title_on.set(True)   # 色の帯の膨張の値は既定で出さない（10/20）。ここでは出す
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]; acts = p.renderer.actors
v["seek"](0.0); pump(app, 0.3)
r_ = bar(p)
lo = float(acts[k0].GetMapper().GetScalarRange()[0])
check(bool(ctrl["cp_vars"]["on"].get()) and ann_ok(r_, 0.6, 2.0, lo) and "dilated" in str(r_[0]),
      f"次に開くと目盛りつきで始まる・膨張の題名も（{r_[0]}）")
check(np.allclose(scal(acts, k0), S0 - 0.2, atol=1e-5), "骨の色の値 = 距離 − 2δ（接触圧で塗らない）")
# 外すと消える
ctrl["cp_vars"]["on"].set(False)
fr2 = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "接触圧" in str(w.cget("text"))][0]
[w for w in walk(fr2) if w.winfo_class() == "TButton" and str(w.cget("text")) == "適用"][0].invoke(); pump(app, 0.3)
r_ = bar(p)
check(not r_[3] and list(p.scalar_bars.values())[0].GetDrawTickLabels(), f"外すと目盛りが消え、帯は元のまま（{r_[0]}）")
close(v)
app.ankle_dilate_mm.set(0.0); app.ankle_cp_on.set(False); app.ankle_dil_title_on.set(False)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
