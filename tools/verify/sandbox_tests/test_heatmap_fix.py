# -*- coding: utf-8 -*-
"""ankle のヒートマップ: 距離の向きの修正（実データ）と、範囲・プリセット（合成データ）。"""
import importlib.util, os, sys, time, json
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, pyvista as pv
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(frs.tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
app = frs.MainMenuGUI(); app.update()
class V:
	def get(self): return False
# ---- 1. 実データ ----
if "real" in sys.argv:
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_FE_軸力100")); app.update()
	anim, N, _ = app._ankle_build_bone_transforms(app._ankle_get_current_cache())
	fr = [1500, 3000, 4500]
	bd = [(idx, app.ankle_bones[idx]["name"], app._sim_simplify_for_heatmap(app.ankle_bones[idx]["name"], m, 15000), np.asarray(Ts)[fr]) for (idx, m, Ts) in anim]
	res = app._sim_view_precompute_multi_heatmap(bd, update_progress=lambda *a: True, cancel_var=V(), quiet=True)
	surf = {b[0]: b[2].extract_surface(algorithm='dataset_surface').triangulate() for b in bd}
	for k, t in enumerate(fr):
		for (i, ni, Mi, Ti) in bd:
			best = np.full(Mi.n_points, np.inf)
			for (j, nj, Mj, Tj) in bd:
				if j == i: continue
				M = np.linalg.inv(Tj[k]) @ Ti[k]
				q = pv.PolyData((M[:3, :3] @ np.asarray(Mi.points).T).T + M[:3, 3])
				best = np.minimum(best, np.asarray(q.compute_implicit_distance(surf[j])["implicit_distance"]))
			got = res[k][i]
			check(np.median(np.abs(got - best)) < 1e-3 and abs(got.min() - best.min()) < 1e-3,
			      f"コマ{t} {ni}: 最小 {got.min():+.2f}mm（正しい {best.min():+.2f}mm）、点ごとの差 中央値 {np.median(np.abs(got-best)):.4f}mm")
# ---- 2. 範囲・プリセット（合成データ） ----
times = [i / 30 for i in range(60)]
box = pv.Cube(x_length=30, y_length=20, z_length=10).triangulate().subdivide(2)
P = np.array([np.eye(4) for _ in times])
sc = [np.linspace(-3, 3, box.n_points).astype(np.float32) for _ in times]
cfg = app._ankle_hm_scene_cfg()
check(cfg["clim"] == (-2.0, 2.0), f"既定の範囲は デフォルト（−2〜＋2）: {cfg['clim']}")
pres = app._ankle_hm_presets()
check(list(pres)[:3] == ["デフォルト", "接触だけ（−2〜0）", "従来（−10〜0）"], "プリセット: " + " / ".join(pres))
scene = {"window_title": "test: heatmap", "bones": [{"name": "box", "mesh": box, "poses": P, "color": "#DEB887", "opacity": 1.0, "scalars": sc}],
         "frame_times": times, "heatmap": {"enabled": True, "title": "distance [mm]", **cfg},
         "features": {"csv": False, "export_model": True, "screenshot": True}}
app._sim_engine_run(scene); pump(app, 0.5)
v = app._sim_viewers_alive()[-1]
act = v["plotter"].renderer.actors["bone_0"]
check(tuple(np.round(act.mapper.scalar_range, 3)) == (-2.0, 2.0), f"開いたときの範囲 {act.mapper.scalar_range}")
from capture_util import capture
w = v["ctrl"]["window"]
texts = []
def walk(wd):
	for c in wd.winfo_children():
		try: texts.append(str(c.cget("text")))
		except Exception: pass
		walk(c)
walk(w)
check(any("ヒートマップの範囲" in t for t in texts), "再生コントロールに「ヒートマップの範囲」の行")
# 範囲を変える
app.ankle_hm_neg_mm.set(2.0)
for c in w.winfo_children():
	pass
hr = [c for c in w.winfo_children() if c.winfo_class() == "TLabelframe" and "範囲" in str(c.cget("text"))][0]
ents = [c for c in hr.winfo_children() if c.winfo_class() == "TEntry"]
ents[0].delete(0, "end"); ents[0].insert(0, "1"); ents[1].delete(0, "end"); ents[1].insert(0, "0.5")
[c for c in hr.winfo_children() if c.winfo_class() == "TButton" and c.cget("text") == "適用"][0].invoke(); pump(app, 0.3)
check(tuple(np.round(act.mapper.scalar_range, 3)) == (-1.0, 0.5), f"適用 → 範囲 {act.mapper.scalar_range}")
lut = act.mapper.lookup_table
check(tuple(np.round(lut.below_range_color[:3], 2)) == (0.5, 0.0, 0.0), "−側より深いめり込みは濃い赤")
check(np.allclose(lut.above_range_color[:3], app._sim_hex_to_rgb01("#DEB887"), atol=0.01), "＋側より離れたところは骨の色")
check(abs(app.ankle_hm_neg_mm.get() - 1.0) < 1e-9 and abs(app.ankle_hm_pos_mm.get() - 0.5) < 1e-9, "変えた範囲は開いたタブの⑤に戻る")
# 色: 0 が緑、−側の端が赤、＋側の端が青
cm = app._sim_view_heatmap_cmap(-1.0, 0.5)
c0 = cm((0 + 1.0) / 1.5)[:3]; cneg = cm(0.0)[:3]; cpos = cm(1.0)[:3]
check(c0[1] > 0.7 and c0[0] < 0.2, f"0mm は緑 {np.round(c0, 2)}")
check(cneg[0] > 0.7 and cneg[1] < 0.1, f"−側の端は赤 {np.round(cneg, 2)}")
check(cpos[2] > 0.9, f"＋側の端は青 {np.round(cpos, 2)}")
c_old = app._sim_view_heatmap_cmap()
check(np.allclose(c_old(0.0)[:3], (0.75, 0, 0)) and np.allclose(c_old(1.0)[:3], (0.1, 0.75, 0.1)), "引数なし（hip/knee）は従来の色のまま")
# プリセットの保存
app._ankle_hm_save_preset("テスト用", 1.5, 0.7)
check(app._ankle_hm_presets().get("テスト用") == {"neg": 1.5, "pos": 0.7}, "プリセットを保存できる")
check(os.path.exists(os.path.join(HERE, "cache", "ankle_heatmap_presets.json")), "保存先はサンドボックスの cache/ankle_heatmap_presets.json")
try:
	app._ankle_hm_save_preset("デフォルト", 9, 9); check(False, "デフォルトは上書きできない")
except ValueError:
	check(True, "デフォルトは上書きできない")
capture(w, os.path.join(HERE, "panel.png"))
v["plotter"].screenshot(os.path.join(HERE, "view.png"))
print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
