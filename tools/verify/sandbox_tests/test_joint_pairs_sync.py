# -*- coding: utf-8 -*-
"""関節面だけの表示・2点の距離の変化・表示の設定の共有（2026-10-14、niki の実データ＋合成データ）。"""
import importlib.util, os, sys, time, json, csv, re
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
frs.messagebox.showwarning = lambda *a, **k: (msgs.append(("warn",) + a), print("   [warning]", str(a[:2])[:160]))
frs.messagebox.showinfo = lambda *a, **k: (msgs.append(("info",) + a), print("   [info]", str(a[:2])[:160]))
app = frs.MainMenuGUI(); app.update()
app._ankle_skip_debug_images = True
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
def tab_i(name): return next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == name)
def sel_tab(name): app.on_ankle_tab_select(tab_i(name)); pump(app, 0.3)
def bone_i(name): return next(i for i, b in enumerate(app.ankle_bones) if b.get("name") == name)
def close(v):
	w = v["ctrl"]["window"]; w.tk.call(w.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)

X, Y, Z = "niki 2872 PMF fix 0DF-IE", "niki 2872 PMF fix -15DF-IE", "261007_FE_軸力100"
# ================= 1. 表示の設定の共有 =================
print("== 共有", flush=True)
sel_tab(X)
check(app._ankle_tab_group(app._ankle_active_tab_dict()) == app._ankle_tab_group(app._ankle_tabs[tab_i(Y)]), "X と Y は骨リストを共有している")
app.ankle_grid_show.set(False); app.ankle_grid_minor_mm.set(2.0); app.ankle_disp_show.set(False)
app.ankle_hm_show.set(False); app.ankle_hm_neg_mm.set(3.0); app.ankle_hm_pos_mm.set(1.0); app.ankle_hm_preset.set("接触だけ（−2〜0）")
app.ankle_joint_only.set(True); app.ankle_pairs_show.set(False)
z_before = {k: app._ankle_tabs[tab_i(Z)]["snapshot"].get(k) for k in app._ANKLE_VIEW_SHARED_KEYS}
sel_tab(Y)
vals = {k: app._ankle_state_vars()[k][0].get() for k in app._ANKLE_VIEW_SHARED_KEYS}
check(vals["ankle_grid_show"] is False and abs(vals["ankle_grid_minor_mm"] - 2.0) < 1e-9 and vals["ankle_disp_show"] is False,
      "方眼紙・移動量の行が、共有しているタブにも入る")
check(vals["ankle_hm_show"] is False and abs(vals["ankle_hm_neg_mm"] - 3.0) < 1e-9 and abs(vals["ankle_hm_pos_mm"] - 1.0) < 1e-9
      and vals["ankle_hm_preset"] == "接触だけ（−2〜0）", "ヒートマップの表示・範囲・プリセットも入る")
check(vals["ankle_joint_only"] is True and vals["ankle_pairs_show"] is False, "関節面だけ表示・2点の距離の表示も入る")
sel_tab(Z)
z_after = {k: app._ankle_state_vars()[k][0].get() for k in app._ANKLE_VIEW_SHARED_KEYS}
check(z_after["ankle_hm_neg_mm"] != 3.0 and z_after["ankle_grid_minor_mm"] != 2.0, f"共有していないタブ（{Z}）は変わらない")
# 方眼紙の向き（可視化の「この視点に合わせる」は _ankle_grid_plane_write を通る）
sel_tab(X)
plane = {"normal": [0.0, 1.0, 0.0], "up": [0.0, 0.0, 1.0]}
app._ankle_grid_plane_write(app._ankle_active_tab_dict(), plane)
check(all(app._ankle_tabs[i].get("grid_plane") == plane for i in app._ankle_group_members(app._ankle_tab_group(app._ankle_active_tab_dict()))),
      "方眼紙の向きも共有しているタブすべてに入る")
check(app._ankle_tabs[tab_i(Z)].get("grid_plane") != plane, "共有していないタブの方眼紙の向きは変わらない")
# 開いていないタブの可視化から書いた場合（ヒートマップの範囲の書き戻し）
app._ankle_view_write(app._ankle_tabs[tab_i(Y)], {"ankle_hm_neg_mm": 4.0})
check(abs(app.ankle_hm_neg_mm.get() - 4.0) < 1e-9, "共有しているほかのタブの画面で変えても、いまのタブに反映")
# 保存されるか
app._save_ankle_state()
d = json.load(open(app._ankle_state_file_path(), encoding="utf-8"))
gid = app._ankle_tab_group(app._ankle_active_tab_dict())
check(d["bone_groups"][gid].get("view", {}).get("ankle_hm_neg_mm") == 4.0 and d["bone_groups"][gid].get("grid_plane") == plane,
      "状態ファイルの共有に、表示の設定と方眼紙の向きが保存される")
# 元に戻す（以降のテストのため）
app.ankle_grid_show.set(True); app.ankle_grid_minor_mm.set(1.0); app.ankle_disp_show.set(True)
app.ankle_hm_show.set(True); app.ankle_hm_neg_mm.set(2.0); app.ankle_hm_pos_mm.set(2.0); app.ankle_hm_preset.set("デフォルト")
app.ankle_joint_only.set(False); app.ankle_pairs_show.set(True)
app._ankle_grid_plane_write(app._ankle_active_tab_dict(), None) if False else None
for i in app._ankle_group_members(gid): app._ankle_tabs[i].pop("grid_plane", None)
app._ankle_bone_groups_dict()[gid].pop("grid_plane", None)

# ================= 2. 関節面モデル =================
print("== 関節面", flush=True)
cache = app._ankle_cache_for_current_size(app._ankle_get_current_cache())
anim, N, _ = app._ankle_build_bone_transforms(cache)
by = {i: (m, Ts) for (i, m, Ts) in anim}
ti, tal, fm_ = bone_i("脛骨"), bone_i("距骨"), bone_i("骨片（内側）")
JD = os.path.join(HERE, "joint_models"); os.makedirs(JD, exist_ok=True)
def crop_near_talus(i, name):
	m = by[i][0].extract_surface().triangulate().clean()
	Ti = np.asarray(by[i][1][0]); Tt = np.asarray(by[tal][1][0])
	talw = by[tal][0].transform(np.linalg.inv(Ti) @ Tt, inplace=False)
	cc = m.cell_centers()
	dd = np.abs(np.asarray(cc.compute_implicit_distance(talw)["implicit_distance"]))
	c = m.extract_cells(np.where(dd < 6.0)[0]).extract_surface()
	fp = os.path.join(JD, name + ".ply"); c.save(fp); return fp, c
jt, ct = crop_near_talus(ti, "tibia_joint"); jf, cf = crop_near_talus(fm_, "frag_med_joint")
print(f"   関節面（テスト用の切り抜き）: 脛骨 {ct.n_cells} 面 / 骨片（内側） {cf.n_cells} 面", flush=True)
w = app._ankle_bone_editor_widgets
check("joint_model_var" in w, "③に「関節面モデル（省略可）」の欄")
app._ankle_selected_bone = ti; app._ankle_load_editor_from_bone(); msgs.clear()
app._ankle_set_bone_file_from_drop("joint_model_path", jt, "関節面モデル"); pump(app, 0.1)
check(app.ankle_bones[ti].get("joint_model_path") == jt and w["joint_model_var"].get() == jt and not [m for m in msgs if m[0] == "warn"],
      "ドラッグ＆ドロップで入る（骨モデルの上に乗っているので警告なし）")
# ずれたモデルは警告
bad = os.path.join(JD, "bad.ply"); ct.translate((10, 0, 0), inplace=False).save(bad)
app._ankle_selected_bone = fm_; app._ankle_load_editor_from_bone(); msgs.clear()
app._ankle_set_bone_file_from_drop("joint_model_path", bad, "関節面モデル"); pump(app, 0.1)
check(any(m[0] == "warn" and "ずれています" in str(m) for m in msgs), "骨モデルからずれた関節面モデルは警告する")
app._ankle_set_bone_file_from_drop("joint_model_path", jf, "関節面モデル"); pump(app, 0.1)
app._ankle_apply_editor_field("joint_model_path", None) if False else None
# 可視化
app.ankle_joint_only.set(False); app.ankle_pairs_show.set(False)
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
jv = v["ctrl"].get("jv_var")
check(jv is not None and not jv.get(), "再生コントロールに「関節面だけ表示」（最初はオフ）")
names = list(p.renderer.actors.keys())
bones_names = [k for k in names if k.startswith("bone_") and not k.endswith("_joint")]
joints = [k for k in names if k.endswith("_joint")]
check(len(joints) == 2, f"関節面のアクターは関節面モデルのある骨だけ（{joints}）")
def vis(k): return bool(p.renderer.actors[k].GetVisibility())
check(all(vis(k) for k in bones_names) and not any(vis(k) for k in joints), "オフ: いつもの骨")
jv.set(True)
[c for c in v["ctrl"]["window"].winfo_children()]
v["ctrl"]["window"].update()
# チェックを押したのと同じ
def press_jv():
	def walk(wd):
		for c in wd.winfo_children():
			if c.winfo_class() == "TCheckbutton" and "関節面だけ表示" in str(c.cget("text")):
				return c
			r = walk(c)
			if r: return r
	return walk(v["ctrl"]["window"])
cbw = press_jv(); jv.set(False); cbw.invoke(); pump(app, 0.3)
check(jv.get() and all(vis(k) for k in joints) and not any(vis(k) for k in bones_names),
      "オン: 関節面モデルのある骨は関節面だけ、ほかの骨（距骨・腓骨など）は隠れる")
check(app.ankle_joint_only.get() is True, "選んだ状態はタブ（と共有）に覚える")
# 関節面は骨と同じ動き: 関節面の点を骨の動きで動かしたものと一致
v["seek"](float(np.asarray(app._ankle_get_current_cache()["timestamps"])[200] - np.asarray(app._ankle_get_current_cache()["timestamps"])[0])); pump(app, 0.3)
a_full = p.renderer.actors[f"bone_{[k for k in bones_names]}"] if False else None
si_t = next(k for k in joints if True)
aj = p.renderer.actors[joints[0]]; bi_ = int(joints[0].split("_")[1]); af = p.renderer.actors[f"bone_{bi_}"]
mj, mf = aj.GetUserMatrix(), af.GetUserMatrix()
same = all(abs(mj.GetElement(r, c) - mf.GetElement(r, c)) < 1e-9 for r in range(4) for c in range(4))
check(same, "関節面は骨と同じ姿勢で動く")
cbw.invoke(); pump(app, 0.3)
check(not jv.get() and all(vis(k) for k in bones_names) and not any(vis(k) for k in joints), "外すと元の骨に戻る")
p.screenshot(os.path.join(HERE, "jv_off.png")); cbw.invoke(); pump(app, 0.3); p.screenshot(os.path.join(HERE, "jv_on.png"))
close(v)
# 次に開くと覚えている
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
joints = [k for k in p.renderer.actors.keys() if k.endswith("_joint")]
check(v["ctrl"]["jv_var"].get() and all(p.renderer.actors[k].GetVisibility() for k in joints), "次に開くと関節面だけの表示で始まる")
close(v)
app.ankle_joint_only.set(False)

# ヒートマップを関節面に写す（合成データ）
print("== 関節面のヒートマップ（合成）", flush=True)
times = [i / 30 for i in range(40)]
box = pv.Cube(x_length=30, y_length=20, z_length=10).triangulate().subdivide(2).clean()
P = np.array([np.eye(4) for _ in times])
for t in range(len(times)): P[t][0, 3] = t * 0.2
sc = [(np.asarray(box.points)[:, 0] * 0.1 + 0.05 * t).astype(np.float32) for t in range(len(times))]
top = box.extract_cells(np.where(np.asarray(box.cell_centers().points)[:, 2] > 4.9)[0]).extract_surface().clean()
from scipy.spatial import cKDTree
jidx = cKDTree(np.asarray(box.points)).query(np.asarray(top.points))[1]
scene = {"window_title": "test: joint", "frame_times": times,
         "bones": [{"name": "box", "mesh": box, "poses": P, "color": "#3366CC", "opacity": 1.0, "scalars": sc,
                    "joint_mesh": top, "joint_idx": jidx},
                   {"name": "other", "mesh": pv.Sphere(radius=3, center=(0, 0, 20)), "poses": P, "color": "#888888", "opacity": 1.0, "scalars": None}],
         "heatmap": {"enabled": True, "title": "distance [mm]", "clim": (-2.0, 2.0), "show": True, "editable": True},
         "joint_view": {"show": True, "on_change": lambda on: None},
         "features": {"csv": False, "export_model": True, "screenshot": True}, "bone_style": {}}
app._sim_engine_run(scene); pump(app, 0.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
aj = p.renderer.actors["bone_0_joint"]
check(aj.GetVisibility() and not p.renderer.actors["bone_1"].GetVisibility() and not p.renderer.actors["bone_0"].GetVisibility(),
      "合成: 関節面だけ（関節面の無い骨は隠れる）")
from vtk.util.numpy_support import vtk_to_numpy
v["seek"](times[25]); pump(app, 0.2)
arr = vtk_to_numpy(aj.GetMapper().GetInput().GetPointData().GetScalars())
check(np.allclose(arr, sc[25][jidx], atol=1e-5), "合成: 関節面のヒートマップは、骨全体の距離を同じ場所から写す（コマが進んでも）")
check(bool(aj.GetMapper().GetScalarVisibility()), "合成: 関節面もヒートマップの色")
close(v)

# ================= 3. 2点の距離 =================
print("== 2点の距離", flush=True)
sel_tab(X)
for b in app.ankle_bones: b.pop("disp_pairs", None)
cache = app._ankle_cache_for_current_size(app._ankle_get_current_cache())
anim, N, _ = app._ankle_build_bone_transforms(cache)
anim, _n = app._ankle_apply_fragment_trim(anim, cache)
by = {i: (m, Ts) for (i, m, Ts) in anim}
fl = bone_i("骨片（外側）")
# 点を置く画面: 右クリックの代わりに、拾った関数を呼ぶ
picks = {}
_orig_esp = pv.Plotter.enable_surface_point_picking
_orig_show = pv.Plotter.show
texts = []
_orig_text = pv.Plotter.add_text
def _esp(self, callback=None, **k):
	picks["cb"] = callback
def _txt(self, text, *a, **k):
	texts.append(str(text)); return _orig_text(self, text, *a, **k)
Ti0 = np.asarray(by[ti][1][0]); Tm0 = np.asarray(by[fm_][1][0])
fw0 = by[fm_][0].transform(np.linalg.inv(Ti0) @ Tm0, inplace=False)
# 脛骨と内側骨片の、近い2点（脛骨の点 ↔ いちばん近い骨片の点）
tp = np.asarray(by[ti][0].points); _dd, _ii = cKDTree(np.asarray(fw0.points)).query(tp)
k0 = int(np.argsort(_dd)[len(_dd) // 50])           # 近い所（いちばん近い所ではなく少し離れた所）
A_click = tp[k0] + np.array([0.3, 0, 0]); B_click = np.asarray(fw0.points)[_ii[k0]] + np.array([0, 0.3, 0])
def _show(self, *a, **k):
	_orig_show(self, auto_close=False, interactive_update=True)
	picks["cb"](np.array([500.0, 500.0, 500.0]))      # 骨の上ではない → 断る
	picks["cb"](A_click); picks["cb"](B_click)
	self.render(); self.screenshot(os.path.join(HERE, "pair_pick.png")); self.close()
pv.Plotter.enable_surface_point_picking = _esp; pv.Plotter.show = _show; pv.Plotter.add_text = _txt
res = app._ankle_pair_pick(ti, fm_)
pv.Plotter.enable_surface_point_picking = _orig_esp; pv.Plotter.show = _orig_show; pv.Plotter.add_text = _orig_text
check(res is not None, "点を置く画面で2点を置ける")
check(any("の上をクリックしてください" in t for t in texts), "骨の上でない所をクリックしたら断る")
check(any("できるだけ近く" in t and "横ずれ" in t for t in texts), "点を置く画面に、2点を近くに置く注意書き")
pa, pb = res
# 骨モデルの座標で覚えている: 最初のコマの位置に戻すと、置いた所（骨の表面）
qa = pa; Mb = np.linalg.inv(Ti0) @ Tm0; qb = Mb[:3, :3] @ pb + Mb[:3, 3]
check(np.linalg.norm(qa - A_click) < 1.0 and np.linalg.norm(qb - B_click) < 1.0, f"点は骨の表面に置かれ、骨の座標で覚える（{np.linalg.norm(qa - A_click):.2f} / {np.linalg.norm(qb - B_click):.2f} mm）")
d0_pick = np.linalg.norm(qb - qa)
app.ankle_bones[ti]["disp_pairs"] = [
	{"id": "t1", "name": "脛骨–骨片（内側）", "color": "#d62728", "visible": True, "point_a": list(map(float, pa)), "bone_b": "骨片（内側）", "point_b": list(map(float, pb))}]
# 2組目（脛骨–外側骨片）
Tl0 = np.asarray(by[fl][1][0]); flw = by[fl][0].transform(np.linalg.inv(Ti0) @ Tl0, inplace=False)
_dd2, _ii2 = cKDTree(np.asarray(flw.points)).query(tp); k2 = int(np.argsort(_dd2)[len(_dd2) // 50])
pb2 = np.linalg.inv(np.linalg.inv(Ti0) @ Tl0); pb2 = pb2[:3, :3] @ np.asarray(flw.points)[_ii2[k2]] + pb2[:3, 3]
app.ankle_bones[ti]["disp_pairs"].append({"id": "t2", "name": "脛骨–骨片（外側）", "color": "#1f77b4", "visible": True,
                                           "point_a": list(map(float, tp[k2])), "bone_b": "骨片（外側）", "point_b": list(map(float, pb2))})
app._ankle_refresh_pairs_ui(); pump(app, 0.1)
rows = [c for c in app._ankle_pairs_frame.winfo_children()]
check(len(rows) == 2, f"⑤に組が並ぶ（{len(rows)} 行）")
app.ankle_pairs_show.set(True); app.ankle_joint_only.set(False)
app.ankle_smooth_enable.set(False)        # 平滑化を切って、④の姿勢そのもので比べる
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
v["seek"](float(np.asarray(cache["timestamps"])[1500] - np.asarray(cache["timestamps"])[0])); pump(app, 0.4)
# 別に計算した距離（④の姿勢から）
Tsi = by[ti][1]; Tsm = by[fm_][1]
def world(T, q): return np.asarray(T)[:3, :3] @ q + np.asarray(T)[:3, 3]
dist = np.array([np.linalg.norm(world(Tsm[t], pb) - world(Tsi[t], pa)) for t in range(N)])
ta_list = []
c2 = p.renderer.GetActors2D(); c2.InitTraversal()
for _ in range(c2.GetNumberOfItems()):
	a = c2.GetNextItem()
	if "TextActor" in a.__class__.__name__: ta_list.append(str(a.GetInput()))
print("   左上:", [t for t in ta_list if t.startswith("#")], flush=True)
fi = int(re.search(r"Frame: (\d+)", str(v["ctrl"]["frame_label"].cget("text"))).group(1))
t1 = next((t for t in ta_list if t.startswith("#1")), "")
mm = re.search(r"距離 ([0-9.]+) mm（変化 ([+-][0-9.]+) mm）", t1)
check(mm is not None and abs(float(mm.group(1)) - dist[fi]) < 0.006 and abs(float(mm.group(2)) - (dist[fi] - dist[0])) < 0.006,
      f"左上の数値が、別に計算した距離と一致（コマ {fi}: {t1} / 計算 {dist[fi]:.2f} mm, 変化 {dist[fi]-dist[0]:+.2f}）")
check(any(t.startswith("#2") for t in ta_list), "2組を同時に表示")
rens = p.render_window.GetRenderers(); rens.InitTraversal(); rl = [rens.GetNextItem() for _ in range(rens.GetNumberOfItems())]
_nch = len(getattr(p.renderer, "_charts", None) or [])
check(_nch >= 1, f"右下にグラフ（{_nch} 枚）")
rens = p.render_window.GetRenderers(); rens.InitTraversal(); rl = [rens.GetNextItem() for _ in range(rens.GetNumberOfItems())]
ov = [r for r in rl if r.GetLayer() == 1 and r.GetActors().GetNumberOfItems() > 0]
check(len(ov) == 1 and ov[0].GetActors().GetNumberOfItems() >= 6 + 4,
      f"点と線は骨に隠れない層に（方眼紙の印と同じ層: {[r.GetActors().GetNumberOfItems() for r in ov]} 個）")
# CSV
OUTCSV = os.path.join(HERE, "pairs_test.csv")
frs.filedialog.asksaveasfilename = lambda **k: OUTCSV
names_btn = []
def walkb(wd):
	for c in wd.winfo_children():
		if c.winfo_class() == "TButton": names_btn.append(c)
		walkb(c)
walkb(v["ctrl"]["window"])
b_csv = [b for b in names_btn if "CSV に保存" in str(b.cget("text")) and "2点" in str(b.cget("text"))]
check(len(b_csv) == 1, "再生コントロールに「2点の距離を CSV に保存…」")
b_csv[0].invoke(); pump(app, 0.2)
rows_ = list(csv.reader(open(OUTCSV, encoding="utf-8-sig")))
_cd = np.array([float(r[2]) for r in rows_[1:]])
check(len(rows_) == N + 1 and "#1 脛骨–骨片（内側） 距離[mm]" in rows_[0] and np.max(np.abs(_cd - dist)) < 1e-3
      and abs(float(rows_[1][3])) < 1e-9, f"CSV: {len(rows_) - 1} コマ・全コマで別に計算した距離と一致（最大の差 {np.max(np.abs(_cd - dist)):.5f} mm）・最初のコマの変化は 0")
p.screenshot(os.path.join(HERE, "pairs_view.png"))
close(v)
# 表示しない設定
app.ankle_pairs_show.set(False); app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]
names_btn = []; walkb(v["ctrl"]["window"])
c2 = v["plotter"].renderer.GetActors2D(); c2.InitTraversal()
tl = [str(a.GetInput()) for a in (c2.GetNextItem() for _ in range(c2.GetNumberOfItems())) if hasattr(a, "GetInput")]
check(not any(t.startswith("#1") for t in tl) and not any("2点" in str(b.cget("text")) for b in names_btn), "表示しない設定なら出さない")
close(v)
app.ankle_smooth_enable.set(True)
app.ankle_pairs_show.set(True)
# 共有: ほかの niki タブにも組が入っている
sel_tab(Y)
check(len(app._ankle_pairs_all()) == 2, "骨リストを共有しているタブにも、2点の組が入る")
# ⑤ の注意書き
labels = []
def walkl(wd):
	for c in wd.winfo_children():
		try:
			if c.winfo_class() == "TLabel": labels.append(str(c.cget("text")))
		except Exception: pass
		walkl(c)
walkl(app)
check(any("できるだけ近く" in t and "横ずれ" in t for t in labels), "⑤にも、2点を近くに置く注意書き")
print(f"\n合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(1 if fails else 0)
