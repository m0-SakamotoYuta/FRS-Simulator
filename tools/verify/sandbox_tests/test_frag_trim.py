# -*- coding: utf-8 -*-
"""骨片つきでスキャンした骨を、骨片と重なる部分だけ削る（niki の実データ。接触の計算はしない）。
③の設定・intact では削らない・可視化で差し替わる・削った量・割れ面のふたの向き・確認画面・設定を外すと元どおり。"""
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
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", a[:2])
infos = []
frs.messagebox.showinfo = lambda *a, **k: (infos.append(a), print("   [info]", a[:2]))
app = frs.MainMenuGUI(); app.update()
app._ankle_skip_debug_images = True
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))

def sel_tab(name):
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == name)); pump(app, 0.3)

def sel_bone(name):
	i = next(i for i, b in enumerate(app.ankle_bones) if b.get("name") == name)
	app._ankle_selected_bone = i
	try:
		app.ankle_bones_listbox.selection_clear(0, "end"); app.ankle_bones_listbox.selection_set(i)
	except Exception:
		pass
	app._ankle_load_editor_from_bone(); pump(app, 0.1)
	return i

FIX = "niki 2872 PMF fix 0DF-IE"
INTACT = "niki 2872 intact 0DF-IE"
sel_tab(FIX)
# 複製した状態ファイルに すでに設定が入っていることがあるので、はじめに外す（サンドボックスの中だけ）
for b in app.ankle_bones:
	b.pop("trim_fragments", None)
ti = sel_bone("脛骨")
w = app._ankle_bone_editor_widgets
check(all(k in w for k in ("trim_on_var", "trim_margin_var", "trim_cap_var", "trim_by_lbl")), "③に「骨片つきでスキャンしてしまったとき」の欄がある")
check(not w["trim_on_var"].get() and app._ankle_trim_cfg(app.ankle_bones[ti])["enabled"] is False, "はじめは使わない")
cache = app._ankle_cache_for_current_size(app._ankle_get_current_cache())
anim, N, _ = app._ankle_build_bone_transforms(cache)
a2, notes = app._ankle_apply_fragment_trim(anim, cache)
check(all(x[1] is y[1] for x, y in zip(anim, a2)) and not notes, "使わない設定では、メッシュはそのまま")

# 使う: 名前に「骨片」が入っている骨が はじめの候補になる
w["trim_on_var"].set(True); app._ankle_trim_toggle(); pump(app, 0.2)
cfg = app._ankle_trim_cfg(app.ankle_bones[ti])
check(cfg["enabled"] and cfg["by"] == ["骨片（内側）", "骨片（外側）"], f"チェックを入れると 骨片（内側）・（外側）が選ばれる: {cfg['by']}")
check("骨片（内側）" in str(w["trim_by_lbl"].cget("text")), f"欄の表示: {w['trim_by_lbl'].cget('text')}")
check(abs(cfg["margin_mm"] - 1.5) < 1e-9 and cfg["cap"], "余裕 1.5mm・ふたをする が既定")
check(cfg["refine"] and abs(cfg["clean_mm"] - 2.0) < 1e-9, "微調整する・細い切れ端を消す幅 2mm が既定")
check(all(k in w for k in ("trim_refine_var", "trim_clean_var")) and w["trim_refine_var"].get() and abs(w["trim_clean_var"].get() - 2.0) < 1e-9, "③に「微調整」と「細い切れ端を消す幅」の欄")
t0 = time.time()
a3, notes = app._ankle_apply_fragment_trim(anim, cache)
dt = time.time() - t0
tib0 = next(m for (i, m, T) in anim if i == ti); tib1 = next(m for (i, m, T) in a3 if i == ti)
print("   ", notes, f"{dt:.1f}s", flush=True)
check(tib1 is not tib0 and any("を削りました" in n for n in notes), "PMF fix: 脛骨が削ったメッシュに差し替わる")
check(all(m is mm for (i, m, T), (j, mm, TT) in zip(a3, anim) if i != ti), "ほかの骨（骨片・距骨・腓骨）は そのまま")
check(all(np.array_equal(T, TT) for (i, m, T), (j, mm, TT) in zip(a3, anim)), "動き（姿勢）は変えない")
check(dt < 30, f"計算時間 {dt:.1f}s")
# 削った量と、残った脛骨が骨片と重ならないこと
base = tib0.extract_surface().triangulate().clean()
m_, note, frags = app._ankle_fragment_trim_for_bone(ti, anim, cache)
info = app._ankle_trim_memo[next(iter(app._ankle_trim_memo))][1]
check(1000 < info["removed_points"] < 0.3 * base.n_points, f"削った点 {info['removed_points']} / {base.n_points}（3割未満）")
check(info["cap_cells"] > 500, f"割れ面のふた {info['cap_cells']} 面")
# 残った脛骨のうち ふた以外の点は、骨片の表面から 余裕 以上離れている（= 埋まっていない）
P1 = np.asarray(tib1.points)
cap_like = np.zeros(len(P1), bool)
for (fname, fm, M) in frags:
	fw = fm.extract_surface().triangulate().clean().transform(M, inplace=False)
	d = np.abs(np.asarray(pv.PolyData(P1).compute_implicit_distance(fw)["implicit_distance"]))
	cap_like |= d < 1e-3
other = ~cap_like
for (fname, fm, M) in frags:
	fw = fm.extract_surface().triangulate().clean().transform(M, inplace=False)
	d_o = np.abs(np.asarray(pv.PolyData(P1[other]).compute_implicit_distance(fw)["implicit_distance"]))
	check(np.min(d_o) >= 1.5 - 1e-6, f"{fname}: 残った脛骨（ふた以外）は骨片の表面から 1.5mm 以上（最小 {np.min(d_o):.2f}mm）")
check(cap_like.sum() > 300, f"ふたの点は骨片の割れ面とぴったり同じ位置（{cap_like.sum()} 点）")
# ふたの向き: 脛骨の外向き（このスキャンは外向き）= 骨片の中へ向く
sgn = app._ankle_mesh_outward_sign(base)
check(sgn > 0, "脛骨のスキャンの面は外向き")
nm = tib1.compute_normals(cell_normals=True, point_normals=False, auto_orient_normals=False, consistent_normals=False, split_vertices=False)
cc = np.asarray(nm.cell_centers().points); nn = np.asarray(nm.cell_data["Normals"])
capc = np.zeros(len(cc), bool)
for (fname, fm, M) in frags:
	fw = fm.extract_surface().triangulate().clean().transform(M, inplace=False)
	capc |= np.abs(np.asarray(pv.PolyData(cc).compute_implicit_distance(fw)["implicit_distance"])) < 1e-3
# 骨片ごとに、表へ 0.2mm 出た点・裏へ 0.2mm 出た点が骨片の中か。どちらか片方だけ中の面（向きを決められる面）で数える
# （縁の薄いところや骨片のスキャンの穴のそばは、どちらも外になり向きを決められない）
for (fname, fm, M) in frags:
	fw = fm.extract_surface().triangulate().clean().transform(M, inplace=False)
	fc = fw.fill_holes(1e6).clean()
	mine = np.abs(np.asarray(pv.PolyData(cc).compute_implicit_distance(fw)["implicit_distance"])) < 1e-3
	pin = np.asarray(pv.PolyData(cc[mine] + 0.2 * nn[mine]).select_enclosed_points(fc, tolerance=1e-6, check_surface=False)["SelectedPoints"]).astype(bool)
	pout = np.asarray(pv.PolyData(cc[mine] - 0.2 * nn[mine]).select_enclosed_points(fc, tolerance=1e-6, check_surface=False)["SelectedPoints"]).astype(bool)
	dec = pin ^ pout
	ok = (pin & dec).sum() / max(dec.sum(), 1)
	check(ok > 0.9, f"{fname} のふたの面の向き: 骨片の中（脛骨の外）を向く {ok*100:.0f}%（向きを決められる {dec.mean()*100:.0f}% の面のうち）")

# 外側に取り残された殻: 外側の骨片だけ位置の微調整が使われ（3点キャリブの骨片）、その位置でも削られている
rf = info.get("refine", {})
print("    微調整:", {k: {kk: (round(vv, 2) if isinstance(vv, float) else vv) for kk, vv in v.items()} for k, v in rf.items()}, flush=True)
check(rf.get("骨片（外側）", {}).get("used") is True and rf["骨片（外側）"]["med1"] < 0.6 * rf["骨片（外側）"]["med0"],
      f"骨片（外側）: 位置を微調整（{rf.get('骨片（外側）', {}).get('deg', 0):.1f}°・{rf.get('骨片（外側）', {}).get('mm', 0):.1f}mm、表面とのずれ {rf.get('骨片（外側）', {}).get('med0', 0):.2f}→{rf.get('骨片（外側）', {}).get('med1', 0):.2f}mm）")
check(rf.get("骨片（内側）", {}).get("used") is False, "骨片（内側）: 良くならないので微調整は使わない")
check("微調整" in note and "細い切れ端" in note, f"お知らせに 微調整と切れ端: {note[:120]}")
fl = next(f for f in frags if f[0] == "骨片（外側）")
fcl = fl[1].extract_surface().triangulate().clean()
M2, used2, _ = app._ankle_trim_refine_pose(base, fcl, fl[2])
fw2 = fcl.transform(M2, inplace=False)
d2 = np.abs(np.asarray(pv.PolyData(P1[other]).compute_implicit_distance(fw2)["implicit_distance"]))
check(used2 and np.min(d2) >= 1.5 - 1e-6, f"微調整した位置の骨片（= スキャンの中の骨片の写し）の上にも脛骨が残っていない（最小 {np.min(d2):.2f}mm）")
# リボン状の切れ端: ふた以外の脛骨で、骨片の近く（10mm）に小さなかたまり（2% 未満）が残っていない
import scipy.sparse as sps
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
Fc = np.asarray(tib1.faces).reshape(-1, 4)[:, 1:]
noncap = ~capc
kidx = np.where(noncap)[0]
A = sps.csr_matrix((np.ones(3 * len(kidx)), (np.repeat(np.arange(len(kidx)), 3), Fc[kidx].ravel())), shape=(len(kidx), tib1.n_points))
_, lab = connected_components(A @ A.T, directed=False)
sizes = np.bincount(lab)
fpts_all = np.vstack([np.asarray(f[1].extract_surface().clean().transform(f[2], inplace=False).points) for f in frags])
dnear = cKDTree(fpts_all).query(cc[kidx])[0]
small_near = [int(sizes[l]) for l in np.unique(lab) if sizes[l] < 0.02 * len(kidx) and np.min(dnear[lab == l]) < 10]
check(not small_near, f"骨片の近くに小さな切れ端のかたまりが残っていない（{small_near[:5]}）")

# 2回目は覚えている結果を使う
t0 = time.time(); app._ankle_apply_fragment_trim(anim, cache); check(time.time() - t0 < 2.0, f"同じ条件の2回目は計算を省く（{time.time()-t0:.2f}s）")

# intact: 骨片が写っていないので削らない
sel_tab(INTACT)
ti2 = next(i for i, b in enumerate(app.ankle_bones) if b.get("name") == "脛骨")
check(app._ankle_trim_cfg(app.ankle_bones[ti2])["enabled"], "共有の骨リストなので intact のタブでも設定は入っている")
c_int = app._ankle_cache_for_current_size(app._ankle_get_current_cache())
an_i, _, _ = app._ankle_build_bone_transforms(c_int)
a_i, notes_i = app._ankle_apply_fragment_trim(an_i, c_int)
print("   ", notes_i, flush=True)
check(all(x[1] is y[1] for x, y in zip(an_i, a_i)) and not any("を削りました" in n for n in notes_i), "intact: 骨片が写っていないので削らない")
check(any("intact" in n and "マーカーが写っていない" in n for n in notes_i), f"intact: お知らせが分かりやすい: {notes_i[:1]}")

# 可視化: 脛骨が削ったメッシュで描かれる・題名にしるし
sel_tab(FIX)
app.ankle_hm_show.set(True)
app.on_ankle_animate(); pump(app, 1.5)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
check("骨片で削った" in v["title"], f"題名: {v['title']}")
names = [b.get("name") for b in app.ankle_bones]
# 骨のアクターの点の数（脛骨）
act_n = []
for k_, a in p.renderer.actors.items():
	if k_.startswith("bone_"):
		act_n.append(a.GetMapper().GetInput().GetNumberOfPoints())
check(tib1.n_points in act_n, f"可視化の脛骨は削ったメッシュ（{tib1.n_points} 点; アクター {act_n}）")
p.screenshot(os.path.join(HERE, "frag_trim_view.png"))
wc = v["ctrl"]["window"]; wc.tk.call(wc.protocol("WM_DELETE_WINDOW")); pump(app, 0.3)

# 確認画面（開いたらすぐ閉じる）
sel_bone("脛骨")
_orig_show = pv.Plotter.show
shots = []
def _show(self, *a, **k):
	_orig_show(self, auto_close=False, interactive_update=True)   # 待たずに開く
	self.render(); self.screenshot(os.path.join(HERE, "frag_trim_preview.png")); shots.append(1); self.close()
pv.Plotter.show = _show
app._ankle_trim_preview(); pump(app, 0.2)
pv.Plotter.show = _orig_show
check(shots == [1], "「削った結果を見る…」で 削る前・削った後・骨片を隠した の3つを並べて見せる")

# 余裕・ふたの切り替えで結果が変わる
w["trim_margin_var"].set(1.0); app._ankle_trim_set(margin_mm=1.0)
_, note1, _ = app._ankle_fragment_trim_for_bone(ti, anim, cache)
check("余裕 1 mm" in note1, f"余裕を変えると削る量が変わる: {note1[:80]}")
app._ankle_trim_set(cap=False)
m_nc, note2, _ = app._ankle_fragment_trim_for_bone(ti, anim, cache)
check("ふた" not in note2 and m_nc.n_points < tib1.n_points, "ふたをしない設定も選べる")
app._ankle_trim_set(cap=True, refine=False, clean_mm=0.0)
m_old, note3, _ = app._ankle_fragment_trim_for_bone(ti, anim, cache)
check("微調整" not in note3 and "細い切れ端" not in note3, "微調整・切れ端消しを切ると、前の削り方に戻る")
app._ankle_trim_set(refine=True, clean_mm=2.0, margin_mm=1.5)
# 重ならない骨（距骨）にチェックを入れても、メッシュは差し替えない
tai = next(i for i, b in enumerate(app.ankle_bones) if b.get("name") == "距骨")
app.ankle_bones[tai]["trim_fragments"] = {"enabled": True, "by": ["骨片（内側）", "骨片（外側）"], "margin_mm": 1.5, "cap": True}
m_t, note_t, _ = app._ankle_fragment_trim_for_bone(tai, anim, cache)
check(m_t is None and "重なるところがない" in note_t, f"重ならない骨（距骨）は差し替えない: {note_t}")
app.ankle_bones[tai].pop("trim_fragments", None)
# 切れ端消し（合成データ）: 60x60mm の板に、幅 2mm・長さ 20mm の細い帯を 2mm の首でつないだもの
g = pv.Plane(center=(0, 0, 0), i_size=60, j_size=60, i_resolution=120, j_resolution=120).triangulate()
strip = pv.Plane(center=(40, 0, 0), i_size=20, j_size=2, i_resolution=40, j_resolution=4).triangulate()
plate = g.merge(strip).clean(tolerance=1e-6)
neck = pv.Plane(center=(30.5, 0, 0), i_size=1.0, j_size=2, i_resolution=2, j_resolution=4).triangulate()
plate = plate.merge(neck).clean(tolerance=1e-3)
op = app._ankle_mesh_open_near(plate, np.array([[40.0, 0, 0]]), 2.0, zone_mm=40)
bx = np.asarray(op.cell_centers().points)[:, 0]
check(np.sum(bx > 31) == 0, f"合成: 細い帯は消える（残り {np.sum(bx > 31)} 面）")
area0 = g.area; area1 = op.extract_cells(np.where(bx <= 30.0)[0]).extract_surface().area
check(area1 > 0.98 * area0, f"合成: 板の本体はほぼそのまま（{area1:.0f}/{area0:.0f} mm²）")
# 外すと元どおり
w["trim_on_var"].set(False); app._ankle_trim_toggle(); pump(app, 0.1)
a4, notes4 = app._ankle_apply_fragment_trim(anim, cache)
check(all(x[1] is y[1] for x, y in zip(anim, a4)) and not notes4, "チェックを外すと元のメッシュに戻る")
print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
