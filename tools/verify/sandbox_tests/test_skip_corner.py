# -*- coding: utf-8 -*-
"""手動キャリブで角を1つ飛ばす: 3点の精度と、キャリブの流れ・ピック画面の S キー。"""
import importlib.util, os, sys, time, copy
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, pyvista as pv
from scipy.spatial.transform import Rotation as Rot
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
app = frs.MainMenuGUI(); app.update()
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_FE_軸力100")); app.update()
b = app.ankle_bones[0]
T0 = np.asarray(b["marker_to_bone_T"], dtype=float)
obj = np.asarray(app._ankle_marker_obj_points(15.0), dtype=float)
corners = (T0 @ np.c_[obj, np.ones(4)].T).T[:, :3]
# ---- 1. 精度: クリックの誤差 0.2mm で、4点 vs 3点 ----
rng = np.random.default_rng(0)
lever = 60.0     # マーカーから関節面までの距離の目安 (mm)
res = {k: [] for k in ("4点", "1飛ばし", "2飛ばし", "3飛ばし", "4飛ばし")}
for _ in range(2000):
	pk = corners + rng.normal(0, 0.2, corners.shape)
	for key, use in (("4点", [0, 1, 2, 3]), ("1飛ばし", [1, 2, 3]), ("2飛ばし", [0, 2, 3]), ("3飛ばし", [0, 1, 3]), ("4飛ばし", [0, 1, 2])):
		T = app._rigid_from_correspondences(obj[use], pk[use], allow_scale=False)
		dR = np.degrees(np.linalg.norm(Rot.from_matrix(T[:3, :3] @ T0[:3, :3].T).as_rotvec()))
		dt = np.linalg.norm(T[:3, 3] - T0[:3, 3])
		res[key].append((dR, dt, dt + np.radians(dR) * lever))
print("   クリックの誤差 0.2mm（各角・各方向）のときの、キャリブの誤差（2000回の 95%）", flush=True)
for k, v in res.items():
	a = np.asarray(v)
	print(f"     {k}: 向き {np.percentile(a[:,0],95):.2f}° / マーカー位置 {np.percentile(a[:,1],95):.2f} mm / 関節面(60mm先)の目安 {np.percentile(a[:,2],95):.2f} mm", flush=True)
r4 = np.percentile(np.asarray(res["4点"])[:, 2], 95); r3 = max(np.percentile(np.asarray(res[k])[:, 2], 95) for k in res if k != "4点")
check(r3 < r4 * 1.6, f"3点でも誤差は 4点の {r3 / r4:.2f} 倍まで（大きく悪化しない）")
# 誤差なしなら完全一致
T3 = app._rigid_from_correspondences(obj[[0, 1, 3]], corners[[0, 1, 3]], allow_scale=False)
check(np.allclose(T3, T0, atol=1e-6), "誤差なしなら 3点でも 4点と完全に同じ")
# ---- 2. キャリブの流れ（角3を飛ばした） ----
pk = [corners[0], corners[1], None, corners[3]]
app._ankle_calib_guide_dialog = lambda *a, **k: True
app._ankle_pick_marker_corners = lambda *a, **k: list(pk)
seen = {}
app._ankle_visualize_calibration = lambda mesh, picked, T, mm, title, texture=None: seen.update(picked=picked, T=T)
msgs = []
frs.messagebox.showinfo = lambda t, m, **k: msgs.append(m)
frs.messagebox.showwarning = lambda t, m, **k: msgs.append(m)
app._ankle_selected_bone = 0
bone_before = copy.deepcopy(app.ankle_bones[0])
app.on_ankle_calibrate_marker_to_bone()
b2 = app.ankle_bones[0]
check(np.allclose(np.asarray(b2["marker_to_bone_T"]), T0, atol=1e-6), "角3を飛ばしても、同じキャリブ結果が保存される")
check(b2.get("marker_to_bone_corners_used") == 3, "3点で求めたことを記録")
check(any("3（右下）は飛ばし" in m for m in msgs), "完了の知らせに、飛ばした角と注意")
check(seen.get("picked") is not None and seen["picked"][2] is None, "確認画面には飛ばした角を None で渡す（推定位置を橙で描く）")
# 2点しかない
pk = [corners[0], None, None, corners[3]]
msgs.clear(); app.on_ankle_calibrate_marker_to_bone()
check(any("いまは 2 点" in m for m in msgs), "2点しかないときは止める")
app.ankle_bones[0] = bone_before
# ---- 3. 確認画面（推定した角を描く） ----
spec2 = importlib.util.spec_from_file_location("frs2", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs2 = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(frs2)
orig_vis = frs2.MainMenuGUI._ankle_visualize_calibration
mesh, tex = app._ankle_load_mesh_and_texture(b["calib_model_path"])
_orig_show = pv.Plotter.show
def _shot_show(self, *a, **k):
	_orig_show(self, auto_close=False, interactive_update=True); self.render()
	self.screenshot(os.path.join(HERE, "confirm.png")); self.close()
pv.Plotter.show = _shot_show
orig_vis(app, mesh, [corners[0], corners[1], None, corners[3]], T0, 15.0, title="test", texture=tex)
check(os.path.exists(os.path.join(HERE, "confirm.png")), "確認画面を描ける（飛ばした角を含む）")
# ---- 4. ピック画面の S キー（タイマーでキーを送って画面を撮る） ----
orig_pick = frs2.MainMenuGUI._ankle_pick_marker_corners
state = {"step": 0}
def fake_show(self, *a, **k):
	_orig_show(self, auto_close=False, interactive_update=True)
	iren = self.iren.interactor
	def send(key):
		iren.SetKeySym(key); iren.SetKeyCode(key); iren.InvokeEvent("KeyPressEvent")
	self.render()
	send("s"); self.render(); self.screenshot(os.path.join(HERE, "pick_after_skip.png"))
	send("s"); self.render()     # 2つ目は飛ばせない
	send("u"); self.render(); self.screenshot(os.path.join(HERE, "pick_after_undo.png"))
	send("s"); self.render()
	self.close()
	return None
pv.Plotter.show = fake_show
out = orig_pick(app, mesh, b, 15.0, "test pick", texture=tex)
check(out == [None], f"S で1つ飛ばす → U で戻す → もう一度 S（2つ目の S は効かない）: {out}")
print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
