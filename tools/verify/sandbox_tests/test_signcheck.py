# -*- coding: utf-8 -*-
"""ヒートマップの偽のめり込み（光線がスキャンの穴を抜けて符号を誤る）の直し（2026-10-14）と、色の帯が1本だけになること。
実データ: 261007_AP_FE60_軸力なし_ACL切 の 11947 コマ付近（脛骨 ↔ 大腿骨。実際は 2.6mm 離れている）。"""
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
app = frs.MainMenuGUI(); app.update()
class V:
	def get(self): return False
TAB = "261007_AP_FE60_軸力なし_ACL切"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.3)
check(app._ANKLE_HM_VERSION == "hm3", "保存結果の版が hm3（前の結果は使わずに計算し直す）")
cache = app._ankle_cache_for_current_size(app._ankle_get_current_cache())
anim, N, _ = app._ankle_build_bone_transforms(cache)
fr = list(range(11940, 11955))
bd = [(idx, app.ankle_bones[idx]["name"], app._sim_simplify_for_heatmap(app.ankle_bones[idx]["name"], m, 15000), np.asarray(Ts)[fr])
      for (idx, m, Ts) in anim]
t0 = time.time()
res = app._sim_view_precompute_multi_heatmap(bd, update_progress=lambda *a: True, cancel_var=V(), quiet=True)
print(f"   計算 {time.time()-t0:.1f}s", flush=True)
ti = next(k for k, b in enumerate(bd) if b[1] == "脛骨")
tib_idx = bd[ti][0]
mins = [float(np.min(res[k][tib_idx])) for k in range(len(fr))]
print("   脛骨の最小距離:", [round(x, 2) for x in mins], flush=True)
check(min(mins) > -1.0, f"11940〜11954 コマで、脛骨の偽の深いめり込みが出ない（最小 {min(mins):+.2f} mm。前は −40 mm）")
# 本当の距離（VTK の面の向きの方式、そろえた法線）と一致
k = fr.index(11947)
Ti = bd[ti][3][k]
P = (Ti[:3, :3] @ np.asarray(bd[ti][2].points).T).T + Ti[:3, 3]
best = np.full(len(P), np.inf)
for (j, nj, Mj, Tj) in bd:
	if j == tib_idx: continue
	M = np.linalg.inv(Tj[k]); q = (M[:3, :3] @ P.T).T + M[:3, 3]
	o = Mj.extract_surface().triangulate().compute_normals(cell_normals=True, point_normals=False, consistent_normals=True, auto_orient_normals=True, split_vertices=False)
	best = np.minimum(best, np.asarray(pv.PolyData(q).compute_implicit_distance(o)["implicit_distance"]))
got = res[k][tib_idx]
near = best < 5
check(np.median(np.abs(got[near] - best[near])) < 0.01 and np.mean(np.sign(got[near]) == np.sign(best[near])) > 0.99,
      f"11947 コマ: 面の向きの方式と一致（近い点 {near.sum()} 点、符号の一致 {np.mean(np.sign(got[near]) == np.sign(best[near]))*100:.1f}%）")
# 色の帯は1本
times = [i / 30 for i in range(20)]
box = pv.Cube().triangulate().subdivide(2)
P2 = np.array([np.eye(4) for _ in times])
sc = [np.linspace(-1, 1, box.n_points).astype(np.float32) for _ in times]
scene = {"window_title": "test: bar", "frame_times": times,
         "bones": [{"name": f"b{k}", "mesh": box.translate((k * 3, 0, 0), inplace=False), "poses": P2, "color": "#3366CC", "opacity": 1.0, "scalars": sc} for k in range(3)],
         "heatmap": {"enabled": True, "title": "distance [mm]", "clim": (-2.0, 2.0), "show": True, "editable": True},
         "features": {"csv": False, "export_model": True, "screenshot": True}, "bone_style": {}}
app._sim_engine_run(scene); pump(app, 0.5)
v = app._sim_viewers_alive()[-1]
check(len(v["plotter"].scalar_bars) == 1, f"ヒートマップの色の帯は1本だけ（{len(v['plotter'].scalar_bars)} 本。前は骨の数だけ出ていた）")
print(f"\n合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(1 if fails else 0)
