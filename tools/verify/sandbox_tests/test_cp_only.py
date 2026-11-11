# -*- coding: utf-8 -*-
"""接触圧だけを出す（2026-10-26）: 再生コントロールの「接触圧だけを出す」・入れると骨の色の表はめり込みの範囲（下限〜0）の jet の逆
（0 = 青・下限 = 赤＝接触圧 0 〜 p_max）・離れている所は骨の色・下限より深い所は濃い赤・色の帯は題名「contact pressure [MPa]」で
目盛りが 0〜p_max（= H_A × |下限| ÷ 厚さ）・距離の目盛りの書き足し文字は消える・保存してある距離（色を塗る値）はそのまま・
右上の接触圧の数字も同じ・範囲を変えても接触圧のまま（p_max が変わる）・外すと元の距離の色と帯・「書き足す」を外すと「だけ」も外れる・
設定を覚えて、開き直すと接触圧だけで開く。接触の距離は試しの値（骨の高さで −1.5〜+1.5 mm）。1〜2分。"""
import importlib.util, os, sys, time, gc
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, vtk
from matplotlib import cm as _cm
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
def _fake(bd, update_progress=None, cancel_var=None, quiet=False):
	out = []
	for t in range(len(bd[0][3])):
		fr = {}
		for (idx, n, m, T) in bd:
			z = np.asarray(m.points)[:, 2]
			fr[idx] = (3.0 * (z - z.min()) / max(1e-9, z.max() - z.min()) - 1.5).astype(np.float32)   # −1.5〜+1.5 mm
		out.append(fr)
	return out
app._sim_view_precompute_multi_heatmap = _fake
app._ankle_heatmap_cache_load = lambda *a, **k: None
app._ankle_heatmap_cache_save = lambda *a, **k: None
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_AP_FE60_軸力100")); pump(app, 0.5)
app.ankle_lig_mode.set("stage1"); app.ankle_start_at_load.set(False); app.ankle_hm_show.set(True)
app.ankle_cp_on.set(True); app.ankle_cp_ha.set(0.894); app.ankle_cp_h.set(2.0); app.ankle_cp_only.set(False)
app.ankle_dilate_mm.set(0.0)
HA, H = 0.894, 2.0
def open_v():
	app.on_ankle_animate(); pump(app, 2.5)
	v = app._sim_viewers_alive()[-1]; v["set_paused"](True)
	return v
def bones_luts(p):
	out = []
	props = p.renderer.GetViewProps(); props.InitTraversal()
	for _ in range(props.GetNumberOfItems()):
		a_ = props.GetNextProp()
		if isinstance(a_, vtk.vtkActor) and a_.GetVisibility() and a_.GetMapper() is not None and a_.GetMapper().GetScalarVisibility():
			arr = a_.GetMapper().GetInput().GetPointData().GetScalars() if a_.GetMapper().GetInput() is not None else None
			if arr is not None and arr.GetNumberOfTuples() > 1000:
				out.append(a_)
	return out
def col(lut, v):
	c = [0.0, 0.0, 0.0]
	lut.GetColor(float(v), c)
	return tuple(c)
def bar(p):
	bs = list(p.scalar_bars.values())
	return bs[0] if bs else None
def txt_visible(p):
	out = []
	props = p.renderer.GetViewProps(); props.InitTraversal()
	for _ in range(props.GetNumberOfItems()):
		a_ = props.GetNextProp()
		if isinstance(a_, vtk.vtkTextActor) and a_.GetVisibility():
			out.append(str(a_.GetInput()))
	return out
try:
	v = open_v(); p = v["plotter"]; ctrl = v["ctrl"]
	acts = bones_luts(p)
	lo, hi = [float(x) for x in acts[0].GetMapper().GetScalarRange()] if acts else (0, 0)
	vals0 = np.array([acts[0].GetMapper().GetInput().GetPointData().GetScalars().GetValue(j) for j in range(0, 2000, 50)]) if acts else None
	check(len(acts) >= 1 and lo < 0 < hi and any("contact pressure [MPa]" == t for t in txt_visible(p)), f"はじめは距離の色（範囲 {lo:.2f}〜{hi:.2f}）・帯の下に接触圧の書き足し")
	cb_only = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")).startswith("接触圧だけを出す")]
	check(len(cb_only) == 1, "欄に「接触圧だけを出す」")
	cb_only[0].invoke(); pump(app, 0.5)
	acts = bones_luts(p)
	lut = acts[0].GetMapper().GetLookupTable()
	r_ = [float(x) for x in acts[0].GetMapper().GetScalarRange()]
	c_mid = col(lut, lo / 2.0); c_lo = col(lut, lo * 0.999); c_0 = col(lut, -1e-6)
	j_mid = _cm.jet(0.5)[:3]; j_hi = _cm.jet(1.0)[:3]; j_lo = _cm.jet(0.0)[:3]
	check(abs(r_[0] - lo) < 1e-6 and abs(r_[1]) < 1e-9 and np.allclose(c_mid, j_mid, atol=0.03) and np.allclose(c_lo, j_hi, atol=0.03) and np.allclose(c_0, j_lo, atol=0.03),
	      f"骨の色の表: めり込みの範囲（{r_[0]:.2f}〜{r_[1]:.2f}）を jet の逆（0 = 青 {np.round(c_0, 2)}・中 {np.round(c_mid, 2)}・下限 = 赤 {np.round(c_lo, 2)}）")
	bc = acts[0].GetProperty().GetColor()
	ab = lut.GetAboveRangeColor()[:3]; bl = lut.GetBelowRangeColor()[:3]
	check(lut.GetUseAboveRangeColor() and np.allclose(ab, bc, atol=0.01) and lut.GetUseBelowRangeColor() and np.allclose(bl, (0.5, 0, 0), atol=0.01),
	      f"離れている所は骨の色（{np.round(ab, 2)}）・下限より深い所は濃い赤（{np.round(bl, 2)}）")
	sb = bar(p)
	pmax = HA * abs(lo) / H
	br = sb.GetLookupTable().GetRange() if sb is not None else (0, 0)
	check(sb is not None and str(sb.GetTitle()) == "contact pressure [MPa]" and abs(br[0]) < 1e-9 and abs(br[1] - pmax) < 1e-6 and sb.GetDrawTickLabels(),
	      f"色の帯は「contact pressure [MPa]」・目盛り 0〜{br[1]:.3f}（H_A×|下限|÷厚さ = {pmax:.3f}）")
	sbc = col(sb.GetLookupTable(), pmax / 2.0) if sb is not None else (0, 0, 0)
	check(np.allclose(sbc, c_mid, atol=0.03), f"帯と骨の色が同じ対応（p_max の半分 {np.round(sbc, 2)} ＝ 下限の半分のめり込み {np.round(c_mid, 2)}）")
	tv = txt_visible(p)
	check(not any(t.startswith("distance") for t in tv) and "contact pressure [MPa]" not in tv, f"距離の目盛りの書き足し文字は出ない（{[t for t in tv if 'distance' in t or 'pressure' in t]}）")
	vals1 = np.array([acts[0].GetMapper().GetInput().GetPointData().GetScalars().GetValue(j) for j in range(0, 2000, 50)])
	check(np.allclose(vals0, vals1), "塗る値（距離）はそのまま")
	# 範囲を変える
	rcb = [o for o in gc.get_objects() if isinstance(o, dict) and callable(o.get("apply")) and "presets" in o and callable(o.get("get"))]
	rcb = [o for o in rcb if isinstance(o.get("get")(), tuple)][-1]
	rcb["apply"](1.0, 2.0); pump(app, 0.4)
	acts = bones_luts(p)
	r2 = [float(x) for x in acts[0].GetMapper().GetScalarRange()]
	br2 = bar(p).GetLookupTable().GetRange()
	check(abs(r2[0] + 1.0) < 1e-6 and abs(r2[1]) < 1e-9 and abs(br2[1] - HA * 1.0 / H) < 1e-6 and str(bar(p).GetTitle()) == "contact pressure [MPa]",
	      f"範囲を変えても接触圧のまま（{r2[0]:.2f}〜{r2[1]:.2f} mm → 0〜{br2[1]:.3f} MPa）")
	print("   いまの設定 ankle_cp_only:", app.ankle_cp_only.get(), flush=True)
	check(app.ankle_cp_only.get() is True, "設定を覚える")
	# 開き直す
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.5)
	v = open_v(); p = v["plotter"]; ctrl = v["ctrl"]
	acts = bones_luts(p)
	r3 = [float(x) for x in acts[0].GetMapper().GetScalarRange()] if acts else [0, 1]
	check(bar(p) is not None and str(bar(p).GetTitle()) == "contact pressure [MPa]" and abs(r3[1]) < 1e-9, f"開き直すと接触圧だけで開く（{r3}）")
	# 外す
	cb_only = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")).startswith("接触圧だけを出す")][0]
	cb_only.invoke(); pump(app, 0.4)
	acts = bones_luts(p)
	r4 = [float(x) for x in acts[0].GetMapper().GetScalarRange()]
	check(r4[1] > 0 and str(bar(p).GetTitle()) != "contact pressure [MPa]" and any("contact pressure [MPa]" == t for t in txt_visible(p)),
	      f"外すと元の距離の色（{r4}）と帯（{bar(p).GetTitle()}・下に接触圧の書き足し）")
	cb_only.invoke(); pump(app, 0.3)
	cb_on = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")) == "色の帯に接触圧（MPa）を書き足す"][0]
	cb_on.invoke(); pump(app, 0.4)
	vars_ = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")).startswith("接触圧だけを出す")][0]
	check(not app.ankle_cp_only.get() and "distance" in str(bar(p).GetTitle()), f"「書き足す」を外すと「だけ」も外れる（{bar(p).GetTitle()}）")
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	app.ankle_cp_only.set(False)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
