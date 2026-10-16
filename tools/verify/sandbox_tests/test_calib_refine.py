# -*- coding: utf-8 -*-
"""キャリブの精密化（板面合わせ＋模様合わせ）とスキャンの縮尺（2026-10-14）。
1) 合成データ（正解あり）: クリックの誤差 0.3mm で、クリックだけ / 板面 / 板面＋模様 の誤差、スキャンの縮尺の測り方
2) 実データ（261007）: 保存済みのキャリブの精密化・元に戻す・手動4点の流れ・スキャンの縮尺で骨のすき間
（2026-10-14 夕: テクスチャの「白のにじみ」を縮尺と取り違えていたのを直した。にじみ入りの合成も確かめる）"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, pyvista as pv, cv2
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
frs.messagebox.askyesno = lambda *a, **k: (msgs.append(("ask",) + a), True)[1]
frs.messagebox.showinfo = lambda *a, **k: msgs.append(("info",) + a)
frs.messagebox.showwarning = lambda *a, **k: (msgs.append(("warn",) + a), print("   [warning]", str(a)[:200]))
app = frs.MainMenuGUI(); app.update()
DICT = "DICT_4X4_100"; MID = 7; S = 15.0
rng = np.random.default_rng(1)

# ================= 1. 合成データ =================
print("== 合成データ", flush=True)
def make_plate(k_scan=1.0, half=15.0, n=120, bloom=0.0):
	"""z=0 の板（±half mm）。テクスチャ = 白地の中央に 15·k_scan mm の ArUco。点は後で剛体変換する。"""
	g = np.linspace(-half, half, n)
	X, Y = np.meshgrid(g, g)
	pts = np.column_stack([X.ravel(), Y.ravel(), np.zeros(X.size)])
	faces = []
	for r in range(n - 1):
		for c in range(n - 1):
			i = r * n + c
			faces += [3, i, i + 1, i + n, 3, i + 1, i + n + 1, i + n]
	m = pv.PolyData(pts, np.array(faces))
	m.active_texture_coordinates = np.column_stack([(pts[:, 0] + half) / (2 * half), (pts[:, 1] + half) / (2 * half)])
	px = 1200; img = np.full((px, px), 255, np.uint8)
	dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT))
	side = int(round(px * (S * k_scan) / (2 * half)))
	mk = cv2.aruco.generateImageMarker(dic, MID, side)
	o = (px - side) // 2; img[o:o + side, o:o + side] = mk
	if bloom > 0:
		# 白のにじみ: 白黒の境を bloom mm だけ黒の側へ（白が太る）、そのあとぼかす（実際のスキャンに似せる）
		ppm = px / (2 * half); w = (img > 127).astype(np.uint8)
		sd = cv2.distanceTransform(w, cv2.DIST_L2, 5) - cv2.distanceTransform(1 - w, cv2.DIST_L2, 5)
		img = np.where(sd + bloom * ppm > 0, 255, 0).astype(np.uint8)
		img = cv2.GaussianBlur(img, (0, 0), 0.25 * ppm)
	return m, img
def texture_of(img):
	return pv.Texture(np.dstack([img] * 3))
# テクスチャの向き（u,v と画像の行・列）は描画して確かめる: 正対の平行投影で ArUco を検出し、4隅の 3D を正解にする
def truth_corners(mesh, tex, G):
	W = 1600; half = 15.0
	c = G[:3, 3]; n = G[:3, 2]; up = -G[:3, 1]; right = np.cross(-n, up)
	p = pv.Plotter(off_screen=True, window_size=(W, W)); p.set_background("white")
	p.add_mesh(mesh, texture=tex, lighting=False)
	# 目印: 板の上の既知の点に赤い玉（描画と座標の対応を独立に確かめる）
	mark = G[:3, :3] @ np.array([6.0, -9.0, 0.3]) + c
	p.add_mesh(pv.Sphere(radius=0.4, center=mark), color="red", lighting=False)
	p.camera_position = [tuple(c + n * 200), tuple(c), tuple(up)]; p.enable_parallel_projection(); p.camera.parallel_scale = half
	p.render(); im = p.screenshot(return_img=True); p.close()
	k = (W / 2) / half
	red = np.argwhere((im[:, :, 0] > 200) & (im[:, :, 1] < 60) & (im[:, :, 2] < 60))
	ctr = red.mean(axis=0)[::-1]
	exp = np.array([W / 2 + (mark - c) @ right * k, W / 2 - (mark - c) @ up * k])
	gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
	det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT)), cv2.aruco.DetectorParameters())
	cs, ids, _ = det.detectMarkers(gray)
	cc = cs[list(ids.ravel()).index(MID)].reshape(-1, 2)
	P = c + np.outer((cc[:, 0] - W / 2) / k, right) + np.outer(-(cc[:, 1] - W / 2) / k, up)
	return P, float(np.linalg.norm(ctr - exp))
def rand_rigid():
	ax = rng.normal(size=3); ax /= np.linalg.norm(ax)
	R = cv2.Rodrigues(ax * np.radians(rng.uniform(10, 60)))[0]
	G = np.eye(4); G[:3, :3] = R; G[:3, 3] = rng.uniform(-50, 50, 3); return G
def err(T, Tt, far):
	Rr = T[:3, :3].T @ Tt[:3, :3]
	rot = float(np.degrees(np.arccos(np.clip((np.trace(Rr) - 1) / 2, -1, 1))))
	pf = T[:3, :3] @ far + T[:3, 3]; pt = Tt[:3, :3] @ far + Tt[:3, 3]
	return rot, float(np.linalg.norm(T[:3, 3] - Tt[:3, 3])), float(np.linalg.norm(pf - pt))
far = np.array([0.0, 80.0, -40.0])          # マーカーから約 90mm の骨の点（てこ）
obj = np.asarray(app._ankle_marker_obj_points(S), float)
res = {"click": [], "plate": [], "full": []}; kmeas = []; mapping_err = []; kbloom = []
for kscan, bl in ((1.0, 0.0), (0.985, 0.0), (1.0, 0.15), (0.985, 0.15)):
	for trial in range(6 if bl == 0 else 3):
		m0, img = make_plate(kscan, bloom=bl); tex = texture_of(img); G = rand_rigid()
		mesh = m0.transform(G, inplace=False)
		if bl > 0:
			# 正解の4隅は、にじみのないテクスチャで測る（にじみがあると検出の4隅が内側へ寄るため）
			Pt, merr = truth_corners(mesh, texture_of(make_plate(kscan)[1]), G)
		else:
			Pt, merr = truth_corners(mesh, tex, G)
		mapping_err.append(merr)
		Tt = app._rigid_from_correspondences(obj, Pt, allow_scale=False)       # 正解（中心・向き）
		clicks = Pt + rng.normal(scale=0.3, size=Pt.shape)                       # クリックの誤差 0.3mm（各方向）
		Tc = app._rigid_from_correspondences(obj, clicks, allow_scale=False)
		Tp, ip = app._ankle_calib_plate_snap(mesh, Tc, S)
		Tf, inf = app._ankle_calib_refine(mesh, tex, Tc, MID, DICT, S)
		res["click"].append(err(Tc, Tt, far)); res["plate"].append(err(Tp if Tp is not None else Tc, Tt, far)); res["full"].append(err(Tf, Tt, far))
		if bl > 0:
			kbloom.append((kscan, inf["pattern"].get("tex_scale"), inf["pattern"].get("bloom_mm"), err(Tf, Tt, far)[2]))
			continue
		if kscan != 1.0 and inf["pattern"].get("tex_scale"): kmeas.append(inf["pattern"]["tex_scale"])
check(max(mapping_err) < 2.0, f"描画の座標の対応（目印の玉）: ずれ 最大 {max(mapping_err):.2f} px（1px = 0.019mm）")
for kk, lab in (("click", "クリックだけ"), ("plate", "板面合わせ"), ("full", "板面＋模様合わせ")):
	a = np.array(res[kk])
	print(f"   {lab:<10}: 回転 平均 {a[:,0].mean():.3f}° 最大 {a[:,0].max():.3f}° / 中心 平均 {a[:,1].mean():.3f} mm / 90mm 先の骨 平均 {a[:,2].mean():.3f} 最大 {a[:,2].max():.3f} mm", flush=True)
a_c, a_f = np.array(res["click"]), np.array(res["full"])
check(a_f[:, 2].mean() < 0.35 * a_c[:, 2].mean() and a_f[:, 2].max() < 0.5, f"精密化で 90mm 先の骨の誤差が小さくなる（{a_c[:,2].mean():.2f} → {a_f[:,2].mean():.3f} mm、最大 {a_f[:,2].max():.3f} mm）")
check(a_f[:, 0].max() < 0.3, f"精密化の回転の誤差は最大 {a_f[:,0].max():.3f}°")
check(len(kmeas) >= 5 and max(abs(np.array(kmeas) - 0.985)) < 0.004, f"スキャンが 1.5% 小さいとき、縮尺を測れる（{np.round(kmeas, 4)}、正解 0.985）")
print("   にじみ 0.15mm の合成: " + " / ".join(f"正解 {a} → 縮尺 {b:.4f}・にじみ {c:.2f}・90mm 先 {d:.3f}mm" for a, b, c, d in kbloom), flush=True)
check(len(kbloom) == 6 and all(abs(b - a) < 0.004 and abs(c - 0.15) <= 0.05 and d < 0.3 for a, b, c, d in kbloom),
      "白のにじみ（0.15mm）があっても、縮尺を取り違えない（にじみも測れる・位置も正しい）")

# ================= 2. 実データ =================
print("== 実データ（261007）", flush=True)
TAB = "261007_FE_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.3)
w_btn = []
def walk(wd):
	for c in wd.winfo_children():
		try:
			if c.winfo_class() in ("TButton", "TCheckbutton", "TRadiobutton"): w_btn.append(str(c.cget("text")))
		except Exception: pass
		walk(c)
walk(app)
check(any("保存済みのキャリブを精密化" in t for t in w_btn) and "精密化の前に戻す" in w_btn and any("精密化する" in t for t in w_btn), "③に精密化のチェックとボタン")
check(any("スキャンの縮尺を直す" in t for t in w_btn) and any("補正後" in t for t in w_btn), "②に大きさの合わせ方の選択")
# 「精密化の前に戻す」は、いちばん最初（精密化する前）の値に戻す
before = {b["name"]: np.asarray(b.get("marker_to_bone_T_before_refine") or b["marker_to_bone_T"]) for b in app.ankle_bones}
# にじみを入れずに測った古い縮尺は使わない
_keep = [dict(b.get("scan_scale") or {}) for b in app.ankle_bones]
for b in app.ankle_bones: b["scan_scale"] = {"k": 0.985, "aniso": 1.0, "cc": 0.95, "edges": [14.8] * 4}
app.ankle_scale_mode.set("scan")
check(app._ankle_scan_scale_common() is None and all(app._ankle_scan_scale_for(b) is None for b in app.ankle_bones),
      "にじみを入れずに測った古い縮尺（2026-10-14 の最初の版）は使わない")
app.ankle_scale_mode.set("depth")
for b, kp in zip(app.ankle_bones, _keep):
	if kp: b["scan_scale"] = kp
	else: b.pop("scan_scale", None)
msgs.clear(); app.on_ankle_refine_saved_calibs(); pump(app, 0.3)
ask = [m for m in msgs if m[0] == "ask"]
print("   " + str(ask[0][2])[:600].replace("\n", "\n   ") if ask else "   (確認なし)", flush=True)
check(ask and all(b.get("marker_to_bone_refine") == "plate+pattern" for b in app.ankle_bones), "保存済みのキャリブを精密化（3本とも 板面＋模様）")
check(all(b.get("marker_to_bone_T_before_refine") is not None for b in app.ankle_bones), "精密化の前の値を残す")
ks = {b["name"]: b.get("scan_scale", {}).get("k") for b in app.ankle_bones}
blm = {b["name"]: b.get("scan_scale", {}).get("bloom_mm") for b in app.ankle_bones}
check(abs(ks["大腿骨"] - 1.0) < 0.006 and abs(ks["脛骨"] - 1.0) < 0.006 and all(0.1 <= (blm[n] or 0) <= 0.25 for n in ("大腿骨", "脛骨")),
      f"にじみを入れて測ると、スキャンの縮尺は ≈1.00（{ks}、にじみ {blm}）")
check(app._ankle_scan_scale_common() is not None and abs(app._ankle_scan_scale_common() - 1.0) < 0.005, f"共通の縮尺 {app._ankle_scan_scale_common():.4f}（ゆがみの大きい膝蓋骨は除く）")
# 共有しているタブにも
other = "261007_AP_FE60_軸力100"
refined = {b["name"]: np.asarray(b["marker_to_bone_T"]) for b in app.ankle_bones}
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == other)); pump(app, 0.3)
check(all(np.allclose(np.asarray(b["marker_to_bone_T"]), refined[b["name"]]) for b in app.ankle_bones), "骨リストを共有しているタブにも精密化したキャリブが入る")
# スキャンの縮尺を直すやり方: 実寸はノギス、モデルはマーカーまわりに 1/k
app.ankle_scale_mode.set("scan"); app.ankle_scan_scale_common.set(True); app._ankle_refresh_scan_scale_lbl()
cache = app._ankle_get_current_cache()
r, old, cur = app._ankle_cache_size_ratio(cache)
check(abs(cur - float(app.ankle_marker_size_true_mm.get())) < 1e-9, f"スキャンの縮尺のやり方では、実寸はノギス（{cur} mm）")
anim, N, _ = app._ankle_build_bone_transforms(app._ankle_cache_for_current_size(cache))
fb = next(b for b in app.ankle_bones if b["name"] == "大腿骨"); fi = app.ankle_bones.index(fb)
m_s = next(m for (i, m, T) in anim if i == fi); m_o = pv.read(fb["model_path"])
c = np.asarray(fb["marker_to_bone_T"])[:3, 3]; kc = app._ankle_scan_scale_common()
d_s = np.linalg.norm(np.asarray(m_s.points) - c, axis=1); d_o = np.linalg.norm(np.asarray(m_o.points) - c, axis=1)
check(np.allclose(d_s, d_o / kc, rtol=1e-6), f"骨のモデルはマーカーの中心まわりに 1/k（{1/kc:.4f} 倍）")
lbl = str(app._ankle_scan_scale_lbl.cget("text"))
check("共通" in lbl and "膝蓋骨" in lbl, f"②の表示: {lbl}")
# 骨のすき間（接触 = 軸力100）: 大腿骨–脛骨の最小距離の中央値
app.ankle_smooth_enable.set(False)
import open3d as o3d
def gap(anim):
	Ts = {i: np.asarray(T) for (i, m, T) in anim}; ms = {i: m for (i, m, T) in anim}
	ti = next(i for i in ms if app.ankle_bones[i]["name"] == "脛骨")
	dm = app._sim_simplify_for_heatmap("大腿骨", ms[fi], 15000).extract_surface().triangulate()
	sc = o3d.t.geometry.RaycastingScene(); F = np.asarray(dm.faces).reshape(-1, 4)[:, 1:].astype(np.int32)
	sc.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(np.asarray(dm.points, np.float32)), o3d.core.Tensor(F)))
	o = dm.compute_normals(cell_normals=True, point_normals=False, consistent_normals=True, auto_orient_normals=True, split_vertices=False)
	tn = np.asarray(o.cell_data["Normals"]); P = np.asarray(app._sim_simplify_for_heatmap("脛骨", ms[ti], 15000).points)
	vals = []
	for f in np.linspace(0, len(Ts[fi]) - 1, 40).astype(int):
		Pw = (Ts[ti][f][:3, :3] @ P.T).T + Ts[ti][f][:3, 3]; M = np.linalg.inv(Ts[fi][f]); q = ((M[:3, :3] @ Pw.T).T + M[:3, 3]).astype(np.float32)
		sd = sc.compute_signed_distance(o3d.core.Tensor(q)).numpy(); neg = np.where(sd < 0)[0]
		if neg.size:
			cp = sc.compute_closest_points(o3d.core.Tensor(q[neg])); out = np.einsum("ij,ij->i", q[neg] - cp["points"].numpy(), tn[cp["primitive_ids"].numpy()]) > 0; sd[neg[out]] *= -1
		vals.append(sd.min())
	return float(np.median(vals))
g_scan = gap(anim)
app.ankle_scale_mode.set("depth")
anim_d, _, _ = app._ankle_build_bone_transforms(app._ankle_cache_for_current_size(cache))
g_depth = gap(anim_d)
app.ankle_marker_size_mm.set(float(app.ankle_marker_size_true_mm.get()))
anim_t, _, _ = app._ankle_build_bone_transforms(app._ankle_cache_for_current_size(cache))
g_true = gap(anim_t)
print(f"   {other}: 大腿骨–脛骨の最小距離の中央値  ノギス 15.0 のまま {g_true:+.2f} / 深度の補正後 {g_depth:+.2f} / スキャンの縮尺 {g_scan:+.2f} mm", flush=True)
check(abs(g_depth) < 0.3 and abs(g_scan - g_true) < 0.15,
      "深度の補正後で骨が接触する。スキャンの縮尺は ≈1 なので、直してもすき間はほぼ変わらない")
# 元に戻す
app.on_ankle_undo_refine(); pump(app, 0.2)
check(all(np.allclose(np.asarray(b["marker_to_bone_T"]), before[b["name"]]) for b in app.ankle_bones), "精密化の前に戻せる")
# 手動4点の流れ（クリックは保存済みの値の4隅＋誤差。ガイドと確認画面は飛ばす）
app._ankle_calib_guide_dialog = lambda *a, **k: True
app._ankle_visualize_calibration = lambda *a, **k: None
tb = next(b for b in app.ankle_bones if b["name"] == "脛骨"); ti = app.ankle_bones.index(tb)
T_true = np.asarray(tb["marker_to_bone_T"]); corners = (T_true[:3, :3] @ obj.T).T + T_true[:3, 3]
app._ankle_pick_marker_corners = lambda *a, **k: list(corners + rng.normal(scale=0.3, size=corners.shape))
app._ankle_selected_bone = ti; app._ankle_load_editor_from_bone(); msgs.clear()
app.ankle_calib_refine.set(True); app.on_ankle_calibrate_marker_to_bone(); pump(app, 0.3)
info = [m for m in msgs if m[0] == "info"]
check(tb.get("marker_to_bone_T_click") is not None and tb.get("marker_to_bone_refine") == "plate+pattern" and info and "【精密化】" in str(info[-1]),
      "手動4点のあと精密化し、クリックだけの結果も残す・完了の知らせに精密化の結果")
print("   " + str(info[-1][2]).split("【精密化】")[-1][:400].replace("\n", "\n   ") if info else "", flush=True)
Tr = np.asarray(tb["marker_to_bone_T"]); Tc = np.asarray(tb["marker_to_bone_T_click"])
e_c = err(Tc, T_true, far); e_r = err(Tr, T_true, far)
print(f"   保存済みの値を正解とみなすと: クリックだけ 90mm 先 {e_c[2]:.2f} mm → 精密化 {e_r[2]:.2f} mm", flush=True)
app.ankle_calib_refine.set(False); app.on_ankle_calibrate_marker_to_bone(); pump(app, 0.2)
check(tb.get("marker_to_bone_refine") in (None, "none") or np.allclose(np.asarray(tb["marker_to_bone_T"]), np.asarray(tb["marker_to_bone_T_click"])), "チェックを外すと、クリックの結果をそのまま使う")
print(f"\n合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(1 if fails else 0)
