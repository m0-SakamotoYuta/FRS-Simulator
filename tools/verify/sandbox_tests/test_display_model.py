# -*- coding: utf-8 -*-
"""ヒートマップ計算後の，表示変更用モデル（臨時。2026-10-15）。
③の欄・座標の確かめ・可視化の「骨のモデル」切り替え・計算済みのヒートマップをそのまま写す・計算し直さない。
接触の計算はダミー（点の座標から作る距離）。可視化ウィンドウが数秒出る。"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, pyvista as pv
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
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: (msgs.append(("warn",) + a), print("   [warning]", str(a[:2])[:200]))
frs.messagebox.showinfo = lambda *a, **k: msgs.append(("info",) + a)
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
dlg = {"n": 0}
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
def _dlg(N, has_cartilage=False):
	dlg["n"] += 1
	return (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
app._show_precompute_dialog = _dlg
calc = {"n": 0}
def fake_multi(bd, update_progress=None, cancel_var=None, quiet=False):
	calc["n"] += 1
	N = len(bd[0][3])
	# 骨の点の x 座標から作る、コマごとに少し変わる距離（写し方を確かめるため）
	base = {idx: (np.asarray(m.points)[:, 0] % 3.0).astype(np.float32) - 1.0 for (idx, n, m, T) in bd}
	return [{idx: base[idx] + 0.001 * (t % 7) for idx in base} for t in range(N)]
app._sim_view_precompute_multi_heatmap = fake_multi
def close(v):
	w = v["ctrl"]["window"]; w.tk.call(w.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)

TAB = "261007_FE_軸力100_ACL切"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
ti = next(i for i, b in enumerate(app.ankle_bones) if b["name"] == "脛骨")
for b in app.ankle_bones:
	b.pop("display_model_path", None)
app.ankle_alt_model.set(False); app.ankle_joint_only.set(False); app.ankle_pairs_show.set(False)
# このタブの前の結果（サンドボックスに残っているもの）を消してから始める
import json as _json
for f in app._ankle_heatmap_dir().glob("hm_*.npz"):
	try:
		with np.load(str(f), allow_pickle=False) as z:
			if _json.loads(str(z["meta_json"])).get("tab") == TAB:
				f.unlink()
	except Exception:
		pass
# 1回目: 接触を計算して保存（ダミー）
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]
check(v["ctrl"].get("av_var") is not None, "再生コントロールに「骨のモデル」")
rbs = [w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TRadiobutton" and "表示変更用モデル" in str(w.cget("text"))]
check(rbs and "disabled" in rbs[0].state(), "表示変更用モデルが無いときは、グレーアウト")
close(v)

# 表示変更用モデルを作る: 脛骨のモデルの、骨幹の端（関節から遠い所）の面を消し、穴を埋めた代わりに 2mm 浮いた板を足す
D = os.path.join(HERE, "display_models"); os.makedirs(D, exist_ok=True)
src = pv.read(app.ankle_bones[ti]["model_path"]).extract_surface().triangulate().clean()
P = np.asarray(src.points); c0 = P.mean(0); u = np.linalg.svd(P - c0, full_matrices=False)[2][0]
proj = (P - c0) @ u
# 関節から遠い側 = 大腿骨のモデルの重心から遠い側
fi = next(i for i, b in enumerate(app.ankle_bones) if b["name"] == "大腿骨")
lo_side = -1.0 if True else 1.0
cc = np.asarray(src.cell_centers().points); cp = (cc - c0) @ u
far_end = np.percentile(cp, 3) if abs(np.percentile(cp, 3)) > abs(np.percentile(cp, 97)) else np.percentile(cp, 97)
cut = (cp < np.percentile(cp, 5)) if far_end < 0 else (cp > np.percentile(cp, 95))
kept = src.extract_cells(np.where(~cut)[0]).extract_surface()
endc = cc[cut].mean(0)
patch = pv.Disc(center=tuple(endc + u * (2.0 if far_end > 0 else -2.0) * 3), inner=0.0, outer=4.0, normal=tuple(u)).triangulate()
alt = (kept + patch).clean()
alt_fp = os.path.join(D, "tibia_display.ply"); alt.save(alt_fp)
print(f"   表示変更用モデル（テスト用）: 元 {src.n_cells:,} 面 → 端の {int(cut.sum()):,} 面を消し、板 {patch.n_cells} 面を足した", flush=True)
w = app._ankle_bone_editor_widgets
check("display_model_var" in w, "③に「ヒートマップ計算後の，表示変更用モデル（省略可・…・臨時）」の欄")
lbl = [x for x in walk(app) if x.winfo_class() == "TLabel" and "表示変更用モデル（省略可" in str(x.cget("text"))]
check(bool(lbl) and str(lbl[0].cget("text")).startswith("ヒートマップ計算後の，表示変更用モデル（省略可・ヒートマップに関係ない所だけ直したもの・臨時）"), "欄の名前がご指定どおり")
app._ankle_selected_bone = ti; app._ankle_load_editor_from_bone(); msgs.clear()
app._ankle_set_bone_file_from_drop("display_model_path", alt_fp, "表示変更用モデル"); pump(app, 0.1)
check(app.ankle_bones[ti].get("display_model_path") == alt_fp and w["display_model_var"].get() == alt_fp and not [m for m in msgs if m[0] == "warn"],
      "ドラッグ＆ドロップで入る（同じ座標なので警告なし）")
bad = os.path.join(D, "bad.ply"); alt.translate((3, 0, 0), inplace=False).save(bad)
msgs.clear(); app._ankle_set_bone_file_from_drop("display_model_path", bad, "表示変更用モデル"); pump(app, 0.1)
check(any(m[0] == "warn" and "同じ座標" in str(m) for m in msgs), "座標がずれたモデルは警告する")
app._ankle_set_bone_file_from_drop("display_model_path", alt_fp, "表示変更用モデル"); pump(app, 0.1)

# 2回目: 計算し直さない（表示変更用モデルは計算の鍵に入らない）
d0, c0_ = dlg["n"], calc["n"]
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
check(dlg["n"] == d0 and calc["n"] == c0_, "表示変更用モデルを入れても、接触の計算はし直さない（保存済みを使う）")
av = v["ctrl"]["av_var"]
names = list(p.renderer.actors.keys())
alt_k = f"bone_{[k for k in names if k.endswith('_alt')][0].split('_')[1]}_alt" if any(k.endswith("_alt") for k in names) else None
check(alt_k is not None and sum(1 for k in names if k.endswith("_alt")) == 1, f"表示変更用モデルのアクターは入れた骨だけ（{[k for k in names if k.endswith('_alt')]}）")
bi_ = int(alt_k.split("_")[1]); base_k = f"bone_{bi_}"
def vis(k): return bool(p.renderer.actors[k].GetVisibility())
check(av.get() == "calc" and vis(base_k) and not vis(alt_k), "最初は「計算に使ったモデル」")
rb = [w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TRadiobutton" and "表示変更用モデル" in str(w.cget("text"))][0]
check("disabled" not in rb.state(), "入れた骨があれば選べる")
rb.invoke(); pump(app, 0.4)
check(av.get() == "alt" and vis(alt_k) and not vis(base_k), "「表示変更用モデル」を選ぶと、その骨だけ差し替わる")
check(all(vis(f"bone_{j}") for j in range(len(app.ankle_bones)) if f"bone_{j}" in names and j != bi_), "ほかの骨は元のまま")
check(app.ankle_alt_model.get() is True, "選んだ状態はタブ（と共有）に覚える")
# 色: 元の表面に乗っている点は、いちばん近い三角形の上の値（3 頂点を重心座標で混ぜる。2026-10-19 から。
# いちばん近い点の値だと縁がギザギザになった）／離れた点（足した板）は骨の色（範囲より上）
from vtk.util.numpy_support import vtk_to_numpy
from scipy.spatial import cKDTree
ts = np.asarray(app._ankle_get_current_cache()["timestamps"]); v["seek"](float(ts[300] - ts[0])); pump(app, 0.4)
a_alt, a_base = p.renderer.actors[alt_k], p.renderer.actors[base_k]
P_alt = vtk_to_numpy(a_alt.GetMapper().GetInput().GetPoints().GetData()).astype(float)
S_alt = vtk_to_numpy(a_alt.GetMapper().GetInput().GetPointData().GetScalars()).astype(float)
P_base = vtk_to_numpy(a_base.GetMapper().GetInput().GetPoints().GetData()).astype(float)
S_base = vtk_to_numpy(a_base.GetMapper().GetInput().GetPointData().GetScalars()).astype(float)
far = S_alt > 1e5
import vtk as _vtk
_bt = pv.wrap(a_base.GetMapper().GetInput()).triangulate()
_loc = _vtk.vtkCellLocator(); _loc.SetDataSet(_bt); _loc.BuildLocator()
_rng = np.random.default_rng(0)
_sel = _rng.choice(np.nonzero(~far)[0], size=min(800, int((~far).sum())), replace=False)
_err = []
for _i in _sel:
	_cp = [0.0, 0.0, 0.0]; _cid = _vtk.reference(0); _sid = _vtk.reference(0); _d2 = _vtk.reference(0.0)
	_loc.FindClosestPoint(P_alt[_i].tolist(), _cp, _cid, _sid, _d2)
	_cell = _bt.GetCell(int(_cid)); _w = [0.0] * _cell.GetNumberOfPoints(); _pc = [0.0, 0.0, 0.0]
	_cell.EvaluatePosition(_cp, [0.0, 0.0, 0.0], _vtk.reference(0), _pc, _vtk.reference(0.0), _w)
	_ids = [_cell.GetPointId(k) for k in range(_cell.GetNumberOfPoints())]
	_err.append(abs(S_alt[_i] - sum(w * S_base[j] for w, j in zip(_w, _ids))))
check(float(np.max(_err)) < 1e-3, f"元の表面に乗っている点は、いちばん近い三角形の上の値（重心座標で混ぜる）を写す（{len(_sel)} 点を確かめた・最大の差 {float(np.max(_err)):.2e} mm）")
_, ii = cKDTree(P_base).query(P_alt)
check(not np.allclose(S_alt[~far], S_base[ii][~far], atol=1e-5), "いちばん近い点の値をそのまま写すのではない（縁が滑らか）")
pd = pv.PolyData(P_alt); patch_pts = np.abs(np.asarray(pd.compute_implicit_distance(a_base.GetMapper().GetInput())["implicit_distance"])) > 0.3
check(far.sum() > 0 and np.array_equal(far, patch_pts), f"元の表面から離れた所（足した板 {int(far.sum())} 点）は色を付けない（骨の色）")
mj, mf = a_alt.GetUserMatrix(), a_base.GetUserMatrix()
check(all(abs(mj.GetElement(r, c) - mf.GetElement(r, c)) < 1e-9 for r in range(4) for c in range(4)), "表示変更用モデルは骨と同じ姿勢で動く")
# 関節面だけ表示は、表示変更用モデルより優先（関節面モデルが無ければ、その骨は隠れる）
v["ctrl"]["jv_var"]
jcb = [w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TCheckbutton" and "関節面だけ表示" in str(w.cget("text"))]
if jcb and "disabled" not in jcb[0].state():
	jcb[0].invoke(); pump(app, 0.3)
	check(not vis(alt_k), "関節面だけ表示のときは、表示変更用モデルは出さない")
	jcb[0].invoke(); pump(app, 0.3)
	check(vis(alt_k), "関節面だけ表示を外すと、表示変更用モデルに戻る")
[w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TRadiobutton" and "計算に使った" in str(w.cget("text"))][0].invoke(); pump(app, 0.3)
check(av.get() == "calc" and vis(base_k) and not vis(alt_k) and app.ankle_alt_model.get() is False, "「計算に使ったモデル」に戻せる")
close(v)
# 次に開くと前回の選択（表示変更用モデル）で始まる
app.ankle_alt_model.set(True)
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
check(v["ctrl"]["av_var"].get() == "alt" and p.renderer.actors[alt_k].GetVisibility(), "前回の選択を覚えている（表示変更用モデルで始まる）")
close(v)
app.ankle_alt_model.set(False)
app.ankle_bones[ti].pop("display_model_path", None)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
