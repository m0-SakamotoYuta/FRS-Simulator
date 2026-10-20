# -*- coding: utf-8 -*-
"""骨の膨張（臨時。2026-10-16）。
⑤と再生コントロールの欄・色は「距離 − 2δ」・見た目の骨（関節面・表示変更用モデルも）は外へ δ・色の帯の題名・
タブ（と共有）に覚える・次に開くと前回の δ・CSV も 2δ を引く・0 で元どおり・接触の計算はし直さない。
接触の計算はダミー。可視化ウィンドウが数秒出る。"""
import importlib.util, os, sys, time, csv
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
calc = {"n": 0}
def fake_multi(bd, update_progress=None, cancel_var=None, quiet=False):
	calc["n"] += 1
	N = len(bd[0][3])
	base = {idx: (np.asarray(m.points)[:, 0] % 3.0).astype(np.float32) - 1.0 for (idx, n, m, T) in bd}
	return [{idx: base[idx] + 0.001 * (t % 7) for idx in base} for t in range(N)]
app._sim_view_precompute_multi_heatmap = fake_multi
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)
def close(v):
	w = v["ctrl"]["window"]; w.tk.call(w.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
def pts(a): return vtk_to_numpy(a.GetMapper().GetInput().GetPoints().GetData()).astype(float).copy()
def scl(a): return vtk_to_numpy(a.GetMapper().GetInput().GetPointData().GetScalars()).astype(float).copy()

TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
app.ankle_dilate_mm.set(0.0); app.ankle_dil_title_on.set(True); app.ankle_joint_only.set(False); app.ankle_alt_model.set(False); app.ankle_hm_show.set(True)   # 色の帯の膨張の値は 10/20 から既定で出さない（ここでは出して確かめる）
sp5 = [w for w in walk(app) if w.winfo_class() == "TSpinbox" and str(w.cget("textvariable")) == str(app.ankle_dilate_mm)]
check(len(sp5) == 1, "⑤に「骨の膨張（臨時）δ」の欄")

app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
check(ctrl.get("dl_var") is not None and float(ctrl["dl_var"].get()) == 0.0, "再生コントロールに「骨の膨張（臨時）」（最初は 0）")
acts = p.renderer.actors
keys = [k for k in acts.keys() if k.startswith("bone_") and k.count("_") == 1]
alt_keys = [k for k in acts.keys() if k.endswith("_alt")]
print(f"   骨 {keys} / 表示変更用モデル {alt_keys}", flush=True)
ts = np.asarray(app._ankle_get_current_cache()["timestamps"]); v["seek"](float(ts[200] - ts[0])); pump(app, 0.3)
P0 = {k: pts(acts[k]) for k in keys + alt_keys}; S0 = {k: scl(acts[k]) for k in keys + alt_keys}
n_calc = calc["n"]
# 再生コントロールで 0.1
ctrl["dl_var"].set("0.1")
btn = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TButton" and str(w.cget("text")) == "適用"
       and w.master.winfo_class() == "TLabelframe" and "膨張" in str(w.master.cget("text"))]
check(len(btn) == 1, "膨張の「適用」ボタン")
btn[0].invoke(); pump(app, 0.4)
P1 = {k: pts(acts[k]) for k in keys + alt_keys}; S1 = {k: scl(acts[k]) for k in keys + alt_keys}
for k in keys:
	fin = np.isfinite(S0[k]) & (S0[k] < 1e5)
	check(np.allclose(S1[k][fin], S0[k][fin] - 0.2, atol=1e-4), f"{k}: 色は「距離 − 2δ」（{int(fin.sum()):,} 点）")
	mv = np.linalg.norm(P1[k] - P0[k], axis=1)
	check(np.allclose(mv, 0.1, atol=1e-3), f"{k}: 見た目の骨は 0.1 mm 動く")
# 外向きか: 関節の近く（ほかの骨から 1mm 以内）で、骨どうしの距離が 2δ＝0.2mm 縮むか（実際の形で測る）
import open3d as o3d, pyvista as pv
def world(k, P):
	m = acts[k].GetUserMatrix(); M = np.array([[m.GetElement(r, c) for c in range(4)] for r in range(4)])
	return P @ M[:3, :3].T + M[:3, 3]
def scene_of(k, P):
	F_ = np.asarray(pv.wrap(acts[k].GetMapper().GetInput()).triangulate().faces).reshape(-1, 4)[:, 1:].astype(np.int32)
	sc_ = o3d.t.geometry.RaycastingScene()
	sc_.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(world(k, P).astype(np.float32)), o3d.core.Tensor(F_)))
	return sc_
for k in keys:
	others = [j for j in keys if j != k]
	sc0 = [scene_of(j, P0[j]) for j in others]; sc1 = [scene_of(j, P1[j]) for j in others]
	d0 = np.min([s_.compute_distance(o3d.core.Tensor(world(k, P0[k]).astype(np.float32))).numpy() for s_ in sc0], axis=0)
	d1 = np.min([s_.compute_distance(o3d.core.Tensor(world(k, P1[k]).astype(np.float32))).numpy() for s_ in sc1], axis=0)
	near = d0 < 1.0
	if near.sum() < 10:
		print(f"   {k}: ほかの骨から 1mm 以内の点が {int(near.sum())} 点だけ（確かめない）", flush=True)
		continue
	# 向き: 関節の近くの点が、いちばん近いほかの骨の中心の方へ動くか（めり込んでいる所でも使える）
	dist_each = np.array([s_.compute_distance(o3d.core.Tensor(world(k, P0[k]).astype(np.float32))).numpy() for s_ in sc0])
	who = np.argmin(dist_each, axis=0)
	cen = np.array([world(j, P0[j]).mean(0) for j in others])
	Pw0, Pw1 = world(k, P0[k]), world(k, P1[k])
	toward = np.einsum("ij,ij->i", (Pw1 - Pw0)[near], cen[who[near]] - Pw0[near]) > 0
	gap = (d0 > 0.3) & near
	msg = ""
	if gap.sum() >= 5:
		msg = f"・離れている所（0.3〜1mm）の距離の縮み 中央値 {np.median((d0 - d1)[gap]):.3f} mm"
	check(toward.mean() > 0.9, f"{k}: 関節の近く {int(near.sum())} 点が、相手の骨の方へ膨らむ（{100*toward.mean():.0f}%{msg}）")
# 表示変更用モデルの色は、見せている間だけ塗る → 見せて比べる（δ=0 の色はその前に取る）
rb_alt = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TRadiobutton" and "表示変更用モデル" in str(w.cget("text"))]
ctrl["dl_var"].set("0"); btn[0].invoke(); pump(app, 0.3)
rb_alt[0].invoke(); pump(app, 0.4)
S0.update({k: scl(acts[k]) for k in alt_keys})
ctrl["dl_var"].set("0.1"); btn[0].invoke(); pump(app, 0.4)
S1.update({k: scl(acts[k]) for k in alt_keys})
for k in alt_keys:
	mv = np.linalg.norm(P1[k] - P0[k], axis=1)
	check(np.allclose(mv, 0.1, atol=1e-3), f"{k}: 表示変更用モデルも 0.1 mm 膨らむ")
	near = S0[k] < 1e5
	check(np.allclose(S1[k][near], S0[k][near] - 0.2, atol=1e-4) and np.all(S1[k][~near] > 1e5), f"{k}: 表示変更用モデルの色も − 2δ（離れた所は骨の色のまま）")
[w for w in walk(ctrl["window"]) if w.winfo_class() == "TRadiobutton" and "計算に使った" in str(w.cget("text"))][0].invoke(); pump(app, 0.3)
bars = list(p.scalar_bars.keys())
check(any("dilated 0.10 mm/bone" in b for b in bars), f"色の帯の題名に膨張の値（{bars}）")
check(abs(app.ankle_dilate_mm.get() - 0.1) < 1e-9, "δ はタブに覚える")
gid = app._ankle_tab_group(app._ankle_active_tab_dict())
check(not gid or abs(float(app._ankle_bone_groups_dict()[gid].get("view", {}).get("ankle_dilate_mm", -1)) - 0.1) < 1e-9, "骨リストを共有しているタブにも同じ δ")
# コマを動かしても −2δ のまま
v["seek"](float(ts[400] - ts[0])); pump(app, 0.3)
raw = app._ankle_heatmap_cache_load  # 参照用（使わない）
k0 = keys[0]; Sx = scl(acts[k0])
v["seek"](float(ts[200] - ts[0])); pump(app, 0.3)
check(np.allclose(scl(acts[k0])[np.isfinite(S0[k0])], S0[k0][np.isfinite(S0[k0])] - 0.2, atol=1e-4), "コマを動かして戻っても「距離 − 2δ」")
# CSV
out_csv = os.path.join(HERE, "dilate_test.csv")
frs.filedialog.asksaveasfilename = lambda *a, **k: out_csv
cb = [w for w in walk(ctrl["window"]) if w.winfo_class() in ("TButton", "Button") and "CSV" in str(w.cget("text"))]
check(len(cb) >= 1, "CSV のボタン")
cb[0].invoke(); pump(app, 0.3)
rows = list(csv.reader(open(out_csv, encoding="utf-8-sig")))
check("dilated 0.1mm/bone" in rows[0][2], f"CSV の見出しに膨張（{rows[0][2]}）")
close(v)
# 次に開くと 0.1 で始まる（計算し直さない）
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]; acts = p.renderer.actors
check(calc["n"] == n_calc, "開き直しても接触の計算はし直さない")
check(abs(float(ctrl["dl_var"].get()) - 0.1) < 1e-9, "次に開くと前回の δ（0.1）")
v["seek"](float(ts[200] - ts[0])); pump(app, 0.3)
check(np.allclose(np.linalg.norm(pts(acts[k0]) - P0[k0], axis=1), 0.1, atol=1e-3), "開いた時から膨らんでいる")
fin = np.isfinite(S0[k0])
check(np.allclose(scl(acts[k0])[fin], S0[k0][fin] - 0.2, atol=1e-4), "開いた時から「距離 − 2δ」")
# 0 で元どおり
ctrl["dl_var"].set("0")
[w for w in walk(ctrl["window"]) if w.winfo_class() == "TButton" and str(w.cget("text")) == "適用"
 and w.master.winfo_class() == "TLabelframe" and "膨張" in str(w.master.cget("text"))][0].invoke(); pump(app, 0.4)
check(all(np.allclose(pts(acts[k]), P0[k], atol=1e-4) for k in keys + alt_keys), "0 にすると形が元どおり")
check(np.allclose(scl(acts[k0])[fin], S0[k0][fin], atol=1e-5), "0 にすると色も元どおり")
check(not any("dilated" in b for b in p.scalar_bars.keys()), "0 のとき色の帯の題名は元のまま")
close(v)
app.ankle_dilate_mm.set(0.0)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
