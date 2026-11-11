# -*- coding: utf-8 -*-
"""付着部の楕円をマウスで回す（2026-10-24）: 「マウスで回す（中心はそのまま）」→ 中心の点・骨の面に沿った輪・
左ドラッグ（VTK のイベントを送る）で、画面の上で中心を囲んだ角度だけ回る（中心・長径・短径はそのまま・法線を軸に）・視点は回らない・
輪と外周の点は骨の面の上・離したら保存して計算の結果は外れる・⟲⟳ のボタン・Esc で終わる（輪を消し、マウスの登録も外す）・
「動かす前に戻す」で元の向き・回している途中に窓を閉じても大丈夫。束の保存はテストの後に元へ戻す。2分。"""
import importlib.util, os, sys, time, gc, shutil, json, math
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk
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
STORE = os.path.join(HERE, "cache", "ankle_lig_bundles", "g8a86693d.json")
STB = STORE + ".testbak"
shutil.copy2(STORE, STB)
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a[:2])[:200])
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
app._sim_view_precompute_multi_heatmap = lambda bd, update_progress=None, cancel_var=None, quiet=False: [
	{idx: np.zeros(len(np.asarray(m.points)), np.float32) for (idx, n, m, T) in bd} for t in range(len(bd[0][3]))]
app._ankle_heatmap_cache_load = lambda *a, **k: None
app._ankle_heatmap_cache_save = lambda *a, **k: None
TAB = "261007_AP_FE60_軸力100"
def signed_angle(u0, u1, n):
	return math.degrees(math.atan2(float(np.cross(u0, u1) @ n), float(u0 @ u1)))
try:
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
	app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(False)
	app.on_ankle_animate(); pump(app, 2.0)
	v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
	v["set_paused"](True)
	s2 = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
	s2._load_cached_results(); s2._rebuild_actors()
	s2.open_window(); pump(app, 0.6)
	labs = [t[0] for t in s2._mv_targets()]
	lab = next(l for l in labs if l.startswith("PL束") and "脛骨" in l)
	s2.w["mv_target"].set(lab)
	b, end = s2._mv_selected()
	e0 = json.loads(json.dumps(b[end]))
	ell0 = e0["ellipse"]
	bones = s2._ensure_bones()
	si = s2._si_of_name(b[end]["bone_name"]); idx = s2.idx_of[si]
	n0 = frs_n = None
	import ankle_lig_bundles as alb
	n0 = alb.ell_normal(bones, idx, ell0)
	n_res0 = len(s2.results)
	# ---- 始める
	btn = [w for w in walk(s2.win) if w.winfo_class() == "TButton" and "マウスで回す" in str(w.cget("text"))]
	check(len(btn) == 1, "「マウスで回す（中心はそのまま）」のボタン")
	btn[0].invoke(); pump(app, 0.3)
	rs = s2.__dict__.get("_rot")
	check(rs is not None and rs.get("ring") is not None and rs.get("cen") is not None and "回し終える" in str(btn[0].cget("text")),
	      "押すと、楕円の輪と中心の点が出る（ボタンは「回し終える（Esc）」に）")
	Pw, cw, nw = s2._rot_ring_world(b, end)
	T = s2.poses[si][max(0, min(int(s2.fi), s2.n_view - 1))]
	Pm = (Pw - T[:3, 3]) @ T[:3, :3]
	sd = np.abs(bones.closest(idx, Pm)[2])
	check(float(np.percentile(sd, 95)) < 0.05, f"輪は骨の面の上（面からの距離 95 % 値 {np.percentile(sd, 95):.3f} mm）")
	cen_world = np.asarray(rs["cen_src"].GetCenter(), float)
	check(np.allclose(cen_world, T[:3, :3] @ np.asarray(ell0["center"], float) + T[:3, 3], atol=1e-6), "中心の点は楕円の中心")
	# ---- 左ドラッグで 30° 回す（画面の上で中心を反時計回りに）
	cam = p.camera
	cam0 = (tuple(cam.GetPosition()), tuple(cam.GetFocalPoint()), tuple(cam.GetViewUp()))
	cx, cy = s2._rot_screen(cw)
	it = p.iren.interactor
	R_ = 80.0
	def ev(name, ang):
		x, y = cx + R_ * math.cos(math.radians(ang)), cy + R_ * math.sin(math.radians(ang))
		it.SetEventInformation(int(round(x)), int(round(y)))
		it.InvokeEvent(name)
	ev("LeftButtonPressEvent", 0.0)
	for a in range(3, 31, 3):
		ev("MouseMoveEvent", float(a))
	ev("LeftButtonReleaseEvent", 30.0)
	pump(app, 0.5)
	ell1 = b[end]["ellipse"]
	dop = np.asarray(cam.GetDirectionOfProjection(), float)
	sgn = 1.0 if float(nw @ dop) < 0 else -1.0
	ang = signed_angle(np.asarray(ell0["u"], float), np.asarray(ell1["u"], float), n0)
	exp = 30.0 * sgn
	px = [(int(round(cx + R_ * math.cos(math.radians(a)))), int(round(cy + R_ * math.sin(math.radians(a))))) for a in (0.0, 30.0)]
	exp_px = math.degrees(math.atan2(px[1][1] - cy, px[1][0] - cx) - math.atan2(px[0][1] - cy, px[0][0] - cx)) * sgn
	check(abs(ang - exp_px) < 1.0, f"左ドラッグで、画面の上で中心を囲んだ角度だけ回る（{ang:+.1f}°・期待 {exp_px:+.1f}°。法線が{'手前' if sgn > 0 else '奥'}向き）")
	check(np.allclose(ell1["center"], ell0["center"]) and abs(ell1["ra"] - ell0["ra"]) < 1e-12 and abs(ell1["rb"] - ell0["rb"]) < 1e-12,
	      "中心・長径・短径はそのまま")
	u1, v1 = np.asarray(ell1["u"], float), np.asarray(ell1["v"], float)
	n1 = alb.ell_normal(bones, idx, ell1)
	check(abs(float(u1 @ v1)) < 1e-9 and abs(np.linalg.norm(u1) - 1) < 1e-9 and float(n1 @ n0) > 0.999999, "向きは楕円の面の中で回る（法線は同じ）")
	cam1 = (tuple(cam.GetPosition()), tuple(cam.GetFocalPoint()), tuple(cam.GetViewUp()))
	check(all(np.allclose(a_, b_) for a_, b_ in zip(cam0, cam1)), "回している間、視点は回らない")
	saved = json.load(open(STORE, encoding="utf-8"))
	sb = next(x for x in saved["bundles"] if x.get("id") == b.get("id"))
	check(np.allclose(sb[end]["ellipse"]["u"], ell1["u"]) and len(s2.results) == 0 and n_res0 > 0,
	      "離したら保存・計算の結果は外れる（動かすのと同じ）")
	O1 = np.asarray(b[end].get("outline") or [], float)
	if len(O1):
		sdo = np.abs(bones.closest(idx, O1)[2])
		ango = signed_angle(np.asarray(e0["outline"][0], float) - np.asarray(ell0["center"], float), O1[0] - np.asarray(ell0["center"], float), n0)
		check(float(sdo.max()) < 0.05 and abs(ango - ang) < 3.0, f"打った外周の点も回って骨の面の上（{ango:+.1f}°・面からの距離 最大 {sdo.max():.3f} mm）")
	lbl = str(s2.w["rot_lbl"].cget("text"))
	check("回しています" in lbl and f"{ang:+.1f}°" in lbl, f"欄に回した角度（{lbl[:50]}…）")
	# ---- ⟳（1 回 5°）
	s2.w["rot_step"].set("5")
	bt = {str(w.cget("text")): w for w in walk(s2.win) if w.winfo_class() == "TButton"}
	bt["⟳"].invoke(); pump(app, 0.3)
	ang2 = signed_angle(np.asarray(ell0["u"], float), np.asarray(b[end]["ellipse"]["u"], float), n0)
	check(abs((ang2 - ang) + 5.0) < 1e-6, f"⟳ で −5°（{ang:+.1f} → {ang2:+.1f}°）")
	# ---- Esc で終わる
	n_obs = len(rs["oids"])
	it.SetKeySym("Escape"); it.InvokeEvent("KeyPressEvent"); pump(app, 0.4)
	acts = list(p.renderer.GetActors())
	check(s2.__dict__.get("_rot") is None and rs["ring"] not in acts and "マウスで回す" in str(btn[0].cget("text")),
	      f"Esc で終わる（輪と点を消す・マウスの登録 {n_obs} 個を外す）")
	ev("LeftButtonPressEvent", 0.0); ev("MouseMoveEvent", 20.0); ev("LeftButtonReleaseEvent", 20.0); pump(app, 0.3)
	check(abs(signed_angle(np.asarray(ell0["u"], float), np.asarray(b[end]["ellipse"]["u"], float), n0) - ang2) < 1e-9,
	      "終えたあとの左ドラッグでは回らない（ふつうの視点の操作）")
	# ---- 戻す
	bt["動かす前に戻す"].invoke(); pump(app, 0.3)
	check(np.allclose(b[end]["ellipse"]["u"], ell0["u"]) and np.allclose(b[end]["ellipse"]["v"], ell0["v"]), "「動かす前に戻す」で元の向き")
	# ---- 回している途中に窓を閉じる
	btn = [w for w in walk(s2.win) if w.winfo_class() == "TButton" and "マウスで回す" in str(w.cget("text"))]
	btn[0].invoke(); pump(app, 0.2)
	s2._close_window(); pump(app, 0.3)
	check(s2.__dict__.get("_rot") is None, "回している途中に窓を閉じても、回す操作を終える")
	cw_ = ctrl["window"]; cw_.tk.call(cw_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	shutil.move(STB, STORE)
	app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
