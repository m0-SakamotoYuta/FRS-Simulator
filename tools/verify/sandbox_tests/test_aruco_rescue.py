# -*- coding: utf-8 -*-
"""④の ArUco 拾い直し（2026-11-06）: AprilTag 方式で見つからない ID だけ、既定の方式で場所を見つけ、
黒枠の外 2px から先を余白の白で塗って AprilTag 方式でもう一度探す。
・合成画像: 余白の角を暗い所とつなげると AprilTag 方式は見落とす → 拾い直せる・角は余白が無傷のときと 0.3px 以内
・余白が無傷なら拾い直しは動かず、結果は前と同じ
・④の状態の文字に「拾い直し」・接触の指紋は拾い直し前の結果では変わらず、版ありでは変わる
・まとめて解析: 拾い直し前で検出率の低いタブは「拾い直し前」と出る・「検出率の低いタブだけチェック」でそのタブだけ
  チェックされ、④と接触の両方が入る。キャッシュは画面の中だけ差し替える（ファイルは書かない）。1分ほど。"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, cv2
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
frs.messagebox.showinfo = lambda *a, **k: print("   [info]", str(a)[:160])
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
app._ankle_autosave_pose_cache = lambda *a, **k: None      # 本物のキャッシュのファイルは書かない

# ---- 1. 実際の録画の切り出し（2935 PLF loading200N.db3 の id=4、240x240）----
# miss = 右上の角の白い余白が切れて AprilTag 方式が見落とすコマ（2130）、hit = 見つかるコマ（800）
det, dictionary, params, use_new = app._ankle_make_detector("DICT_4X4_100")
def imread(name):
	p = os.path.join(HERE, "data", name)
	return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_GRAYSCALE)
def corners_of(gray, rescue):
	c, i, _ = det.detectMarkers(gray)
	r = set()
	if rescue:
		c, i, r = app._ankle_rescue_missing(gray, c, i, {4}, det, dictionary, params, use_new)
	if i is None:
		return None, r
	for cc, ii in zip(c, i):
		if int(np.ravel(ii)[0]) == 4:
			return cc.reshape(4, 2), r
	return None, r
g_ok, g_leak = imread("aruco_rescue_hit.png"), imread("aruco_rescue_miss.png")
q_ok, r_ok = corners_of(g_ok, True)
q_ok0, _ = corners_of(g_ok, False)
check(q_ok is not None and not r_ok and np.array_equal(q_ok, q_ok0),
      "AprilTag 方式で見つかるコマでは拾い直しは動かず、角も前と同じ")
q0, _ = corners_of(g_leak, False)
check(q0 is None, "余白の切れたコマは、AprilTag 方式だけでは見落とす（元の不具合の再現）")
q1, r1 = corners_of(g_leak, True)
check(q1 is not None and r1 == {4}, f"拾い直しで見つかる（拾い直した ID: {r1}）")
# 既定の方式の四角形（角の精緻化なし）とは 1px 以内・AprilTag 方式で塗った版の角そのもの
c0, i0, _ = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters()).detectMarkers(g_leak)
q_def = next(c.reshape(4, 2) for c, i in zip(c0, i0) if int(np.ravel(i)[0]) == 4)
if q1 is not None:
	dmax = float(np.max(np.linalg.norm(q1 - q_def, axis=1)))
	check(dmax < 4.0, f"拾い直した角は既定の方式の四角形（精緻化なし）から {dmax:.2f}px（< 4px＝別の物を拾っていない）")
	side = np.mean([np.linalg.norm(q1[k] - q1[(k + 1) % 4]) for k in range(4)])
	side_ok = np.mean([np.linalg.norm(q_ok[k] - q_ok[(k + 1) % 4]) for k in range(4)])
	check(abs(side - side_ok) < 1.0, f"一辺の長さ {side:.2f}px（見つかるコマの AprilTag 方式 {side_ok:.2f}px）")
# 1コマの姿勢まで（_ankle_detect_markers_in_frame）
K = np.array([[651.0, 0, 120], [0, 650.0, 120], [0, 0, 1]]); dist = np.zeros(5)
obj = app._ankle_marker_obj_points(20.0)
depth = np.full(g_leak.shape, 1800, np.uint16)
app._ankle_rescue_counts = {}
out = app._ankle_detect_markers_in_frame(g_leak, depth, K, dist, obj, 20.0, 0.1, {4}, det, dictionary, params, use_new)
check(4 in out and out[4].get("rescued") is True and app._ankle_rescue_counts.get(4) == 1,
      f"1コマの検出で姿勢が出て「拾い直し」と記録される（{app._ankle_rescue_counts}）")

# ---- 2. 状態の文字・接触の指紋 ----
name = next(t["name"] for t in app._ankle_tabs if int((app._ankle_pose_cache.get(t["name"]) or {}).get("frame_count", 0)) > 100)
orig = app._ankle_pose_cache[name]
old = dict(orig); old.pop("detect_version", None); old.pop("rescued", None)
bones = {}
for k, (aid, b) in enumerate(sorted((orig.get("bones") or {}).items())):
	d = np.ones(len(b["detected"]), bool)
	if k == 0:
		d[::2] = False                          # 1本目の ID を検出率 50% に
	bones[aid] = dict(b, detected=d)
old["bones"] = bones
new = dict(old, detect_version=app._ANKLE_DETECT_VERSION, rescued={str(min(bones)): 12})
need_o, low_o = app._ankle_cache_needs_rescue(old)
need_n, _ = app._ankle_cache_needs_rescue(new)
check(need_o and abs(low_o - 50.0) < 1.0 and not need_n, f"拾い直し前・50% は要やり直し（{need_o}, {low_o:.1f}%）、版ありは不要（{need_n}）")
st_o, st_n = app._ankle_cache_status_text(old), app._ankle_cache_status_text(new)
check("拾い直しの機能より前" in st_o and "拾い直し 12枚" in st_n, "④の状態の文字に「拾い直し前」の注意／拾い直した枚数")
fp_a = app._ankle_hm_fingerprint({}, [], old)
fp_b = app._ankle_hm_fingerprint({}, [], dict(old))
fp_c = app._ankle_hm_fingerprint({}, [], new)
check(fp_a == fp_b and fp_a != fp_c, "接触の指紋: 拾い直し前は変わらない（計算済みのまま）・版ありになると変わる（要計算）")

# ---- 3. まとめて解析 ----
app._ankle_folder_filter = app._ANKLE_FOLDER_ALL
app._ankle_pose_cache[name] = old
def open_dialog():
	for w in app.winfo_children():
		if isinstance(w, tk.Toplevel) and "まとめて解析" in w.title():
			w.destroy()
	app._ankle_open_batch_dialog(); pump(app, 0.3)
	return next(w for w in app.winfo_children() if isinstance(w, tk.Toplevel) and "まとめて解析" in w.title())
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)
def row_texts(win, tabname):
	L = []
	for c in walk(win):
		try:
			if c.winfo_class() in ("TLabel", "TCheckbutton") and c.grid_info():
				gi = c.grid_info(); tv = str(c.cget("textvariable"))
				t = str(c.tk.globalgetvar(tv)) if (tv and c.winfo_class() == "TLabel") else str(c.cget("text"))
				L.append((int(gi["row"]), int(gi["column"]), t, c))
		except Exception:
			pass
	r = next(r for (r, cc, t, w) in L if cc == 1 and t == tabname)
	return {cc: (t, w) for (rr, cc, t, w) in L if rr == r}, L
win = open_dialog()
rw, L = row_texts(win, name)
src = str((next(t for t in app._ankle_tabs if t["name"] == name).get("snapshot") or {}).get("ankle_depth", ""))
print(f"   {name}: ④={rw[6][0]!r}（録画 {'あり' if os.path.exists(src) else 'なし'}）", flush=True)
check("拾い直し前" in rw[6][0] and "50%" in rw[6][0], "一覧の④の欄に「拾い直し前（最低 50%）」")
btn = next((c for c in walk(win) if c.winfo_class() == "TButton" and str(c.cget("text")) == "検出率の低いタブだけチェック"), None)
check(btn is not None, "「検出率の低いタブだけチェック」のボタンがある")
dv, cv_ = win._task_vars
dv.set(False); cv_.set(False)
btn.invoke(); pump(app, 0.2)
_, L = row_texts(win, name)
def checked_names():
	out = []
	for (r, c, t, w) in L:
		if c == 0 and r > 0:
			v = w.cget("variable")
			if str(w.tk.globalgetvar(str(v))) in ("1", "True", "true"):
				out.append(next(tt for (rr, cc, tt, ww) in L if rr == r and cc == 1))
	return out
ch = checked_names()
def _src_ok(t):
	sn = t.get("snapshot") or {}
	s = str(sn.get("ankle_depth", "") or "").strip() or str(sn.get("ankle_video", "") or "").strip()
	return bool(s) and os.path.exists(s)
expect = [t["name"] for t in app._ankle_tabs
          if _src_ok(t) and app._ankle_cache_needs_rescue(app._ankle_pose_cache.get(t["name"]))[0]]
print(f"   チェックされたタブ {len(ch)} 個: {ch}", flush=True)
check(sorted(ch) == sorted(expect), f"やり直すとよいタブ（録画あり）がちょうどチェックされる（期待 {len(expect)} 個）")
check((name in ch) == os.path.exists(src), "作った「50%」のタブは、録画があればチェックされる")
check(not any(app._ankle_cache_needs_rescue(app._ankle_pose_cache.get(t["name"]))[1] == 100.0
              for t in app._ankle_tabs if t["name"] in ch), "検出率 100% のタブはチェックされない")
check(bool(dv.get()) and bool(cv_.get()), "「やること」に ④ と 接触 の両方が入る")
# 版ありに戻すと、ふつうの「済み」
app._ankle_pose_cache[name] = new
win = open_dialog()
rw, _ = row_texts(win, name)
check(rw[6][0].startswith("済み") and "拾い直し前" not in rw[6][0] and "※拾い直しあり" in rw[6][0],
      f"拾い直したコマがある結果は「済み …※拾い直しあり」（{rw[6][0]!r}）")
import tkinter.font as tkf
win.update_idletasks()
check(tkf.nametofont("TkDefaultFont").measure(rw[6][0]) <= rw[6][1].winfo_width() + 2, "「※拾い直しあり」の文字が欄に収まる")
win.destroy()
app._ankle_pose_cache[name] = dict(new, rescued={})
win = open_dialog()
rw, _ = row_texts(win, name)
check(rw[6][0].startswith("済み") and "拾い直し" not in rw[6][0], f"拾い直しが 0 枚なら書き足さない（{rw[6][0]!r}）")
win.destroy()
app._ankle_pose_cache[name] = orig

print(f"\n{'すべて OK' if not fails else f'NG {len(fails)} 件'}", flush=True)
try:
	app.destroy()
except Exception:
	pass
sys.exit(1 if fails else 0)
