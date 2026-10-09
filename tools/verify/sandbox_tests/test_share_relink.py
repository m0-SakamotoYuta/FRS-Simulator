# -*- coding: utf-8 -*-
"""骨リストの共有・パスの付け替え・キャリブのガイド のテスト（サンドボックスで実行）。"""
import importlib.util, json, os, sys, shutil, copy, traceback, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(frs)
PART = sys.argv[1] if len(sys.argv) > 1 else "all"

fails = []
def check(cond, label):
	print(("  OK  " if cond else "  NG  ") + label, flush=True)
	if not cond:
		fails.append(label)

def pump(app, n=3):
	for _ in range(n):
		app.update_idletasks(); app.update()

dialogs = []
def _rec(kind, ret=None):
	def f(*a, **k):
		dialogs.append((kind, a[:2]))
		return ret
	return f
for kind in ("showwarning", "showerror", "showinfo"):
	setattr(frs.messagebox, kind, _rec(kind))
frs.messagebox.askyesno = _rec("askyesno", True)

def show_all(app):
	app._ankle_folder_filter = app._ANKLE_FOLDER_ALL
	app._ankle_rebuild_tabbar()

def walk(w):
	yield w
	for c in w.winfo_children():
		yield from walk(c)

def new_toplevel(app, before):
	return [w for w in app.winfo_children() if w not in before and w.winfo_class() == "Toplevel"][0]

def tidx(app, sub):
	for i, t in enumerate(app._ankle_tabs):
		if sub == t["name"]:
			return i
	raise KeyError(sub)

# =====================================================================
if PART in ("all", "share"):
	print("=== 骨リストの共有 ===")
	app = frs.MainMenuGUI(); pump(app, 5); show_all(app)
	A, B, C, D = ("260910_ブタ膝を使ったArUco試験_AP_FE90", "260910_ブタ膝を使ったArUco試験_FE",
	              "260910_ブタ膝を使ったArUco試験_IE_FE60", "260910_ブタ膝を使ったArUco試験_FE(PD)")
	app.on_ankle_tab_select(tidx(app, A)); pump(app)
	bonesA = copy.deepcopy(app.ankle_bones)
	scanA = app.ankle_initial_scan_path.get()
	print(f"   {A}: 骨 {[ (b['name'], b['aruco_id']) for b in bonesA]}")
	D_before = copy.deepcopy(app._ankle_tabs[tidx(app, D)]["snapshot"]["_bones"])
	check(app._ankle_share_label_var.get().startswith("共有なし"), "最初は「共有なし」と表示")

	# 画面から: B と C にチェック → 共有
	before = set(app.winfo_children())
	app._ankle_open_bone_share_dialog(); pump(app, 5)
	win = new_toplevel(app, before)
	cbs = [w for w in walk(win) if w.winfo_class() == "TCheckbutton"]
	rowcbs = cbs[2:]                       # [0]=①初期スキャン [1]=ArUco辞書
	check(len(rowcbs) == len(app._ankle_tabs), f"全タブの行がある ({len(rowcbs)})")
	check(str(rowcbs[tidx(app, A)].cget("state")) == "disabled", "このタブの行は外せない")
	rowcbs[tidx(app, B)].invoke(); rowcbs[tidx(app, C)].invoke(); pump(app)
	btn = [w for w in walk(win) if w.winfo_class() == "Button" and "共有する" in str(w.cget("text"))][0]
	dialogs.clear()
	btn.invoke(); pump(app, 5)
	check([d[0] for d in dialogs] == ["askyesno", "showinfo"], f"確認 → 完了 {[d[0] for d in dialogs]}")
	ia, ib, ic, id_ = tidx(app, A), tidx(app, B), tidx(app, C), tidx(app, D)
	g = app._ankle_tabs[ia].get("bone_group")
	check(bool(g) and app._ankle_tabs[ib].get("bone_group") == g and app._ankle_tabs[ic].get("bone_group") == g,
	      "A・B・C が同じ共有に入る")
	check(app._ankle_tabs[id_].get("bone_group") is None, "チェックしていない D は入らない")
	grp = app._ankle_bone_groups[g]
	check(app._ankle_tabs[ib]["snapshot"]["_bones"] is grp["bones"], "B のスナップショットは共有のリストそのものを指す")
	check([(b["name"], b["aruco_id"]) for b in grp["bones"]] == [(b["name"], b["aruco_id"]) for b in bonesA],
	      "共有の中身 = A の骨リスト")
	check(app._ankle_tabs[ib]["snapshot"].get("ankle_initial_scan") == scanA, "①初期スキャンのパスもコピーされる")
	check("共有中" in app._ankle_share_label_var.get() and "3タブ" in app._ankle_share_label_var.get(),
	      "③に「共有中（3タブ）」と出る: " + app._ankle_share_label_var.get().replace("\n", " / "))

	# B を開いて骨を書き換える → C・A にも反映、D は変わらない
	app.on_ankle_tab_select(ib); pump(app)
	check([b["name"] for b in app.ankle_bones] == [b["name"] for b in bonesA], "B を開くと A と同じ骨リスト")
	app.ankle_bones[0]["name"] = "共有テスト骨"
	app.ankle_bones[0]["model_path"] = "C:/dummy/new_model.stl"
	app.on_ankle_tab_select(ic); pump(app)
	check(app.ankle_bones[0]["name"] == "共有テスト骨", "B で変えた骨名が C に反映")
	check(app.ankle_bones[0]["model_path"] == "C:/dummy/new_model.stl", "B で変えたモデルのパスが C に反映")
	check(app._ankle_tabs[ia]["snapshot"]["_bones"][0]["name"] == "共有テスト骨", "A にも反映")
	check(app._ankle_tabs[id_]["snapshot"]["_bones"] == D_before, "共有していない D は変わらない")
	# 骨を足す・消す
	app.ankle_bones.append(app._ankle_default_bone(len(app.ankle_bones)))
	app.on_ankle_tab_select(ia); pump(app)
	check(len(app.ankle_bones) == len(bonesA) + 1, "C で足した骨が A に出る")
	app.ankle_bones.pop()
	app.on_ankle_tab_select(ib); pump(app)
	check(len(app.ankle_bones) == len(bonesA), "A で消した骨が B からも消える")

	# 可視化画面からの色の書き戻し（開いているのは B、書き戻し先は共有相手の C）
	app._ankle_store_bone_style(app._ankle_tabs[ic], 0, "共有テスト骨", "#123456", 0.5)
	check(app.ankle_bones[0].get("color") == "#123456", "共有相手への色の書き戻しは、開いているタブにも反映")
	app.on_ankle_tab_select(ia); pump(app)
	check(app.ankle_bones[0].get("color") == "#123456", "…そして A にも反映")

	# ＋ は「帯の右端のタブ」を複製する（既存の仕様）。右端が共有中のタブなら、新しいタブも同じ共有に入る
	app.on_ankle_tab_select(ic); pump(app)
	app._ankle_tab_move(ic, len(app._ankle_tabs) - 1); pump(app)
	ia, ib, ic, id_ = tidx(app, A), tidx(app, B), tidx(app, C), tidx(app, D)
	check(app._ankle_tabs[-1]["name"] == C, "（準備）C を右端へ")
	n0 = len(app._ankle_tabs)
	app.on_ankle_tab_add(); pump(app)
	newt = app._ankle_tabs[-1]
	check(newt.get("bone_group") == g and newt["snapshot"]["_bones"] is grp["bones"], "＋で作ったタブも同じ共有に入る")
	check(app.ankle_bones[0]["name"] == "共有テスト骨", "新しいタブを開くと共有の骨リスト")

	# 保存 → 別のインスタンスで復元
	app._save_ankle_state(save_pose_caches=False)
	saved = json.load(open("frs2015_gui_state_ankle_sim.json", encoding="utf-8"))
	check(g in saved.get("bone_groups", {}), "状態ファイルに共有が保存される")
	app.destroy()
	app = frs.MainMenuGUI(); pump(app, 5); show_all(app)
	ia, ib, ic = tidx(app, A), tidx(app, B), tidx(app, C)
	g2 = app._ankle_tabs[ia].get("bone_group")
	grp2 = app._ankle_bone_groups.get(g2)
	check(g2 == g and grp2 is not None, "再起動後も共有が残る")
	check(all(app._ankle_tabs[i]["snapshot"]["_bones"] is grp2["bones"] for i in (ia, ib, ic)),
	      "再起動後も3タブが同じリストを指す（つなぎ直し）")
	app.on_ankle_tab_select(ia); pump(app)
	app.ankle_bones[0]["name"] = "再起動後の変更"
	app.on_ankle_tab_select(ic); pump(app)
	check(app.ankle_bones[0]["name"] == "再起動後の変更", "再起動後も、変更が共有相手に反映")

	# このタブの共有を解除（画面から）
	before = set(app.winfo_children())
	app._ankle_open_bone_share_dialog(); pump(app, 5)
	win = new_toplevel(app, before)
	ub = [w for w in walk(win) if w.winfo_class() == "TButton" and "解除" in str(w.cget("text"))][0]
	dialogs.clear(); ub.invoke(); pump(app, 5)
	check(app._ankle_tabs[ic].get("bone_group") is None, "C は共有から外れる")
	check(app._ankle_tabs[ia].get("bone_group") == g, "A・B と＋のタブの共有は続く")
	check(app.ankle_bones[0]["name"] == "再起動後の変更", "外したタブは、そのときの写しを持つ")
	app.ankle_bones[0]["name"] = "Cだけの変更"
	app.on_ankle_tab_select(ia); pump(app)
	check(app.ankle_bones[0]["name"] == "再起動後の変更", "外したタブの変更は、もう他に伝わらない")

	# タブを消して相手が1つだけになったら共有は解除
	mem = app._ankle_group_members(g)
	print(f"   残りのメンバー {len(mem)} タブ")
	frs.messagebox.askyesno = _rec("askyesno", True)
	while len(app._ankle_group_members(g)) > 1:
		victim = [i for i in app._ankle_group_members(g) if app._ankle_tabs[i]["name"] not in (A,)][-1]
		app.on_ankle_tab_delete(victim); pump(app)
		if g not in app._ankle_bone_groups:
			break
	check(g not in app._ankle_bone_groups and app._ankle_tabs[tidx(app, A)].get("bone_group") is None,
	      "相手がいなくなった共有は自動で解除")
	app.destroy()

# =====================================================================
if PART in ("all", "relink"):
	print("=== パスの付け替え（テスト用フォルダ） ===")
	base = os.path.join(HERE, "relink_test")
	shutil.rmtree(base, ignore_errors=True)
	old_root = os.path.join(base, "実験データ", "260930_札幌")
	files = {
		"db3": os.path.join(old_root, "trial_A", "ankle_d405_20260930_111111.db3"),
		"db3b": os.path.join(old_root, "trial_B", "ankle_d405_20260930_222222.db3"),
		"scan": os.path.join(old_root, "scan", "initial_scan.ply"),
		"model": os.path.join(old_root, "bones", "tibia.stl"),      # 同名ファイルがほかにもある
		"gone": os.path.join(old_root, "bones", "lost_file.stl"),   # どこにも無い
	}
	for key, p in files.items():
		os.makedirs(os.path.dirname(p), exist_ok=True)
		if key != "gone":                 # 「どこにも無い」ファイルは作らない
			open(p, "w").write("x")
	# 別の場所にも同名の tibia.stl（紛らわしい候補）
	other = os.path.join(base, "別の実験", "bones_old", "tibia.stl")
	os.makedirs(os.path.dirname(other), exist_ok=True); open(other, "w").write("y")

	app = frs.MainMenuGUI(); pump(app, 5); show_all(app)
	# 2つのタブにパスを入れ、骨リストを共有させる
	i1, i2 = 0, 1
	for i, key in ((i1, "db3"), (i2, "db3b")):
		app.on_ankle_tab_select(i); pump(app)
		app.ankle_depth_path.set(files[key])
		app.ankle_initial_scan_path.set(files["scan"])
		app.ankle_bones = [app._ankle_default_bone(0), app._ankle_default_bone(1)]
		app.ankle_bones[0]["model_path"] = files["model"]
		app.ankle_bones[1]["model_path"] = files["gone"]
	app.on_ankle_tab_select(i1); pump(app)
	app._ankle_tabs[i1]['snapshot'] = app._ankle_snapshot_current()
	gid = "gtest"
	app._ankle_bone_groups[gid] = {"name": "テスト", "bones": copy.deepcopy(app.ankle_bones), "ligaments": []}
	app._ankle_link_tab_to_group(app._ankle_tabs[i1], gid)
	app._ankle_link_tab_to_group(app._ankle_tabs[i2], gid)
	app._ankle_restore_snapshot(app._ankle_tabs[i1]['snapshot'])

	# エクスプローラで整理したつもり: 「実験データ\260930_札幌」→「整理後\2026\260930_札幌_cadaver」
	new_root = os.path.join(base, "整理後", "2026", "260930_札幌_cadaver")
	os.makedirs(os.path.dirname(new_root), exist_ok=True)
	shutil.move(old_root, new_root)
	check(not os.path.exists(files["db3"]), "（準備）元の場所にはもう無い")

	refs = app._ankle_collect_path_refs()
	miss = sorted({r["path"] for r in refs if not os.path.exists(r["path"])})
	ours = [p for p in miss if p.startswith(base)]
	check(len(ours) == 5, f"見つからないパス（テスト分）{len(ours)} 個")
	roots = app._ankle_relink_default_roots(ours)
	print("   探す場所の初期値:", roots)
	nr = os.path.normcase(new_root)
	check(any(nr.startswith(os.path.normcase(r).rstrip("\\/") + os.sep) for r in roots)
	      and all(os.path.normcase(r).startswith(os.path.normcase(HERE)) for r in roots),
	      "探す場所の初期値は、移動先を含み、しかも広すぎない（ホームやドライブ全体ではない）")
	found = app._ankle_relink_scan([base], {os.path.basename(p).lower() for p in ours})
	res = app._ankle_relink_resolve(ours, found)
	for p in ours:
		print(f"   {os.path.relpath(p, base)}\n      → {res[p]['status']}: {os.path.relpath(res[p]['new'], base) if res[p]['new'] else ''}  {res[p]['how']}")
	exp = lambda key: os.path.join(new_root, os.path.relpath(files[key], old_root))
	check(res[files["db3"]]["new"] == exp("db3"), "録画1: 名前で見つかる")
	check(res[files["db3b"]]["new"] == exp("db3b"), "録画2: 名前で見つかる")
	check(res[files["scan"]]["new"] == exp("scan"), "初期スキャン: 見つかる")
	check(res[files["model"]]["new"] == exp("model"), "同名が2つある骨モデル: 上のフォルダ名の一致で正しい方を選ぶ")
	check(res[files["gone"]]["status"] == "none", "どこにも無いファイルは「見つからない」")

	# 画面から: 探す → 付け替える
	before = set(app.winfo_children())
	app._ankle_open_relink_dialog(); pump(app, 5)
	win = new_toplevel(app, before)
	lb = [w for w in walk(win) if w.winfo_class() == "Listbox"][0]
	lb.delete(0, "end"); lb.insert("end", base)
	scan_btn = [w for w in walk(win) if w.winfo_class() == "Button" and "探す" in str(w.cget("text"))][0]
	scan_btn.invoke(); pump(app, 5)
	tv = [w for w in walk(win) if w.winfo_class() == "Treeview"][0]
	rows = [tv.item(i, "values") for i in tv.get_children()]
	ours_rows = [r for r in rows if str(r[3]).startswith(base)]
	print("   画面の行（テスト分）:")
	for r in ours_rows:
		print(f"     {r[0]} | {r[1]} | {r[2]}")
	check(sum(1 for r in ours_rows if r[0] == "☑") == 4, "画面: 見つかった4つに ☑")
	apply_btn = [w for w in walk(win) if w.winfo_class() == "Button" and "付け替える" in str(w.cget("text"))][0]
	dialogs.clear()
	apply_btn.invoke(); pump(app, 5)
	check([d[0] for d in dialogs] == ["askyesno"], f"確認だけ出る {[d[0] for d in dialogs]}")
	t1s = app._ankle_tabs[i1]["snapshot"]; t2s = app._ankle_tabs[i2]["snapshot"]
	check(app.ankle_depth_path.get() == exp("db3"), "開いているタブの②が新しい場所に（画面にも反映）")
	check(t2s["ankle_depth"] == exp("db3b"), "もう1つのタブの②も新しい場所に")
	check(t1s["ankle_initial_scan"] == exp("scan") and t2s["ankle_initial_scan"] == exp("scan"), "①初期スキャンも両タブで")
	check(app._ankle_bone_groups[gid]["bones"][0]["model_path"] == exp("model"), "共有中の骨モデルのパスも付け替わる")
	check(app.ankle_bones[0]["model_path"] == exp("model"), "開いているタブの骨リストにも反映")
	check(app._ankle_bone_groups[gid]["bones"][1]["model_path"] == files["gone"], "見つからないものは元のまま")
	saved = json.load(open("frs2015_gui_state_ankle_sim.json", encoding="utf-8"))
	check(saved["tabs"][i2]["snapshot"]["ankle_depth"] == exp("db3b"), "状態ファイルに保存される")
	win.destroy(); pump(app)
	app.destroy()

# =====================================================================
if PART in ("all", "realdata"):
	print("=== 実際の86タブで、見つからないパスがいくつ見つかるか（書き換えはしない） ===")
	app = frs.MainMenuGUI(); pump(app, 5); show_all(app)
	app._ankle_tabs[app._ankle_active_tab]['snapshot'] = app._ankle_snapshot_current()
	refs = app._ankle_collect_path_refs()
	miss = sorted({r["path"] for r in refs if not os.path.exists(r["path"])})
	print(f"   パスの数 {len({r['path'] for r in refs})} / 見つからない {len(miss)}")
	roots = app._ankle_relink_default_roots(miss)
	print("   探す場所の初期値:", roots)
	t0 = time.time()
	found = app._ankle_relink_scan(roots, {os.path.basename(p).lower() for p in miss})
	res = app._ankle_relink_resolve(miss, found)
	dt = time.time() - t0
	cnt = {}
	for r in res.values():
		cnt[r["status"]] = cnt.get(r["status"], 0) + 1
	print(f"   探した時間 {dt:.1f} 秒 / 結果 {cnt}")
	for p in miss:
		r = res[p]
		print(f"   [{r['status']}] {p}")
		if r["new"]:
			print(f"          → {r['new']}  （{r['how']}）")
		elif r["cands"]:
			for c in r["cands"][:3]:
				print(f"          ? {c}")
	app.destroy()

# =====================================================================
if PART in ("all", "guide"):
	print("=== キャリブのガイド ===")
	import cv2
	app = frs.MainMenuGUI(); pump(app, 5); show_all(app)
	dname = app.ankle_aruco_dict_var.get()
	print("   辞書:", dname)
	# 図の「1=左上…4=左下」が、ArUco の検出が返す角の順番と一致するか（向きの確認）
	dic = app._ankle_resolve_aruco_dict(dname)
	for aid in (0, 1, 2, 7):
		mk = cv2.aruco.generateImageMarker(dic, aid, 300)
		canvas = np.full((420, 420), 255, np.uint8); canvas[60:360, 60:360] = mk
		det = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters())
		c, ids, _ = det.detectMarkers(canvas)
		ok = ids is not None and int(np.ravel(ids)[0]) == aid
		if ok:
			q = c[0].reshape(4, 2)
			exp = np.array([[60, 60], [360, 60], [360, 360], [60, 360]], float)
			ok = np.max(np.abs(q - exp)) < 2.0
		check(ok, f"ID{aid}: 図の向きの 1左上・2右上・3右下・4左下 = 検出の角0〜3")
	# 回して貼っても、模様の向きで数えれば検出と同じ角になる（90°回転した画像で確認）
	mk = cv2.aruco.generateImageMarker(dic, 2, 300)
	canvas = np.full((420, 420), 255, np.uint8); canvas[60:360, 60:360] = np.rot90(mk)  # 反時計回りに90°
	c, ids, _ = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters()).detectMarkers(canvas)
	q = c[0].reshape(4, 2)
	# 反時計回り90°: 元の左上(角0)は画像の左下に来る
	check(np.max(np.abs(q[0] - [60, 360])) < 2.0, "90°回して貼ったとき、角0（=図の1）は模様についていく（画像の左下）")

	for k in (0, 2, 4):
		arr = app._ankle_marker_guide_image(2, dname, k, bone_name="脛骨", size_mm=15.0)
		from PIL import Image
		Image.fromarray(arr).save(os.path.join(HERE, f"guide_next{k}.png"))
	check(arr.shape == (700, 560, 3), f"ガイド画像 {arr.shape}")

	# ピック画面: クリックを疑似的に再現（点を置く → U で戻す → 置き直す）
	import pyvista as pv
	cap = {}
	orig_enable = pv.Plotter.enable_surface_point_picking
	orig_key = pv.Plotter.add_key_event
	orig_show = pv.Plotter.show
	pv.Plotter.enable_surface_point_picking = lambda self, callback=None, **k: cap.__setitem__("cb", callback)
	pv.Plotter.add_key_event = lambda self, key, fn: cap.setdefault("keys", {}).__setitem__(key, fn)
	L = 15.0
	true_pts = [np.array([-L/2, -L/2, 0.0]), np.array([L/2, -L/2, 0.0]), np.array([L/2, L/2, 0.0]), np.array([-L/2, L/2, 0.0])]
	def fake_show(self, *a, **k):
		cb = cap["cb"]
		cb(true_pts[0]); cb(np.array([99.0, 99.0, 0.0]))        # 2点目を打ち間違えた
		cap["keys"]["u"]()                                       # U で戻す
		cb(true_pts[1]); cb(true_pts[2])
		img = self.screenshot(os.path.join(HERE, "pick_window.png"), return_img=True)
		cap["img_shape"] = img.shape
		cb(true_pts[3]); cb(np.array([1.0, 2.0, 3.0]))          # 5点目は無視される
		self.close()
	pv.Plotter.show = fake_show
	orig_init = pv.Plotter.__init__
	def off_init(self, *a, **k):
		k["off_screen"] = True
		orig_init(self, *a, **k)
	pv.Plotter.__init__ = off_init
	try:
		mesh = pv.Plane(i_size=40, j_size=40)
		bone = {"name": "脛骨", "aruco_id": 2}
		picked = app._ankle_pick_marker_corners(mesh, bone, 15.0, "テスト", texture=None)
	finally:
		pv.Plotter.enable_surface_point_picking = orig_enable
		pv.Plotter.add_key_event = orig_key
		pv.Plotter.show = orig_show
		pv.Plotter.__init__ = orig_init
	check(picked.shape == (4, 3) and np.allclose(picked, np.array(true_pts)), "U で戻したあと、正しい4点が返る（5点目は無視）")
	print("   ピック画面のスクショ:", cap.get("img_shape"))

	# キャリブ前のガイド画面: 開いて「クリックを始める」を押す
	from PIL import Image
	sys.path.insert(0, HERE)
	shot = {}
	def press_start():
		tops = [w for w in app.winfo_children() if w.winfo_class() == "Toplevel"]
		w = tops[-1]
		try:
			import capture_util as S   # PrintWindow のヘルパ（shot.py の capture を使う）
		except Exception:
			S = None
		shot["title"] = w.title()
		btn = [x for x in walk(w) if x.winfo_class() == "Button" and "始める" in str(x.cget("text"))][0]
		if S is not None:
			try:
				S.capture(w, os.path.join(HERE, "guide_dialog.png"))
			except Exception as e:
				print("   capture:", e)
		btn.invoke()
	app.after(800, press_start)
	r = app._ankle_calib_guide_dialog({"name": "脛骨", "aruco_id": 2}, 15.0, "キャリブ用モデル (切り出し)", "C:/x/tibia_marker.ply")
	check(r is True, f"ガイド画面で「クリックを始める」→ True（{shot.get('title')}）")
	app.after(800, lambda: [w.destroy() for w in app.winfo_children() if w.winfo_class() == "Toplevel"])
	r = app._ankle_calib_guide_dialog({"name": "脛骨", "aruco_id": 2}, 15.0, "x", "C:/x/y.ply")
	check(r is False, "閉じたら False（キャリブは始まらない）")
	app.destroy()

print()
print("RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL: {fails}")
