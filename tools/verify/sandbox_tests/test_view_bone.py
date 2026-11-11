# -*- coding: utf-8 -*-
"""骨に対する視点（2026-10-22）: ある試験（261007_AP_FE60_軸力100）で保存した視点を、骨リストを共有している別の試験
（…_ACL切）で選ぶと、基準の骨（固定している骨）が画面に同じように見える（基準の骨だけを残して撮った絵の重なりで確かめる）。
これまでのワールドのままの視点では、ずれる（比べのため）。古い視点（local なし）は知らせを出す。
接触の計算はダミー。視点の保存先（cache/ankle_saved_views.json）はテストの後に元へ戻す。2〜3分。"""
import importlib.util, os, sys, time, gc, json, shutil
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, vtk
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
VP = os.path.join(HERE, "cache", "ankle_saved_views.json")
VB = VP + ".testbak"
if os.path.exists(VP):
	shutil.copy2(VP, VB)
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a[:2])[:200])
frs.simpledialog.askstring = lambda *a, **k: "テスト視点"
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
app._sim_view_precompute_multi_heatmap = lambda bd, update_progress=None, cancel_var=None, quiet=False: [
	{idx: np.full(len(np.asarray(m.points)), 5.0, np.float32) for (idx, n, m, T) in bd} for t in range(len(bd[0][3]))]
app._ankle_heatmap_cache_load = lambda *a, **k: None
app._ankle_heatmap_cache_save = lambda *a, **k: None
TAB, TAB2 = "261007_AP_FE60_軸力100", "261007_AP_FE60_軸力100_ACL切"
def sel(name):
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == name)); pump(app, 0.5)
def open_viewer():
	app.ankle_start_at_load.set(False); app.ankle_lig_mode.set("stage1")
	app.on_ankle_animate(); pump(app, 2.0)
	v = app._sim_viewers_alive()[-1]
	v["set_paused"](True); pump(app, 0.3)
	return v
def scene_bones():
	scn = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "heatmap" in o and "_t_shift" in o][-1]
	return [str(b.get("name", "")) for b in scn["bones"]]
def only_bone(p, names, keep_name):
	"""基準の骨だけを残して撮る（色は骨の色）。"""
	bi = names.index(keep_name)
	keep = [a_ for k_, a_ in p.renderer.actors.items() if str(k_) in (f"bone_{bi}", f"bone_{bi}_joint", f"bone_{bi}_alt") and a_.GetVisibility()]
	props = p.renderer.GetViewProps(); props.InitTraversal(); hid = []
	for _ in range(props.GetNumberOfItems()):
		a_ = props.GetNextProp()
		if all(a_ is not k_ for k_ in keep) and a_.GetVisibility():
			a_.SetVisibility(0); hid.append(a_)
	for k_ in keep:
		k_.GetMapper().ScalarVisibilityOff()
	p.render(); img = np.asarray(p.screenshot(None, return_img=True)).astype(int)
	img[int(0.8 * img.shape[0]):] = 255                 # 下の色の帯の目盛りの文字（描くたびに出る）は比べない
	for a_ in hid:
		a_.SetVisibility(1)
	return img
def mask(img):
	bg = img[2, 2]
	return np.abs(img - bg).sum(axis=2) > 40
def iou(a, b):
	return float((a & b).sum()) / max(1, float((a | b).sum()))
def views_ui(v):
	cw = v["ctrl"]["window"]
	vf = [w for w in walk(cw) if w.winfo_class() == "TLabelframe" and "視点" in str(w.cget("text"))][0]
	btn = {str(w.cget("text")): w for w in walk(vf) if w.winfo_class() == "TButton"}
	cb = [w for w in walk(vf) if w.winfo_class() == "TCombobox"][0]
	msg = [w for w in walk(vf) if w.winfo_class() == "TLabel"][0]
	return cw, btn, cb, msg

FIX0 = None
def fix(name):
	global FIX0
	if FIX0 is None:
		FIX0 = [bool(b_.get("fixed")) for b_ in app.ankle_bones]
	for b_ in app.ankle_bones:
		b_["fixed"] = (str(b_.get("name", "")) == name)
try:
	# ---- 試験 A（大腿骨を固定）で視点を保存
	sel(TAB); fix("大腿骨")
	vA = open_viewer(); pA = vA["plotter"]
	names = scene_bones()
	scnA = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "heatmap" in o and "_t_shift" in o][-1]
	PfA = np.asarray(scnA["bones"][names.index("大腿骨")]["poses"], float)
	check(np.allclose(PfA, np.eye(4), atol=1e-9), "固定した骨は、どのコマでも単位行列（スキャンのモデル座標のまま描く）")
	cam = pA.renderer.GetActiveCamera()
	cam.Azimuth(35); cam.Elevation(20); cam.Zoom(1.3); pA.render(); pump(app, 0.2)
	cwA, btnA, cbA, msgA = views_ui(vA)
	btnA["この視点を保存…"].invoke(); pump(app, 0.3)
	saved = [x for x in json.load(open(VP, encoding="utf-8")).get("group:g8a86693d", []) if x["name"] == "テスト視点"]
	check(len(saved) == 1 and isinstance(saved[0].get("local"), dict) and saved[0]["local"].get("bone") == "大腿骨",
	      f"保存すると、骨に対する視点（local）も入る（基準の骨: {saved[0]['local'].get('bone') if saved and saved[0].get('local') else '-'}＝固定している骨）")
	check("骨に対する視点" in str(msgA.cget("text")), f"保存の知らせ（{str(msgA.cget('text'))[:60]}）")
	ref = "大腿骨"
	imgA = only_bone(pA, names, ref)
	camA_world = saved[0]["cam"]
	cwA.tk.call(cwA.protocol("WM_DELETE_WINDOW")); pump(app, 0.5)
	# ---- 試験 B（脛骨を固定＝保存したときと違う骨）で選ぶ
	sel(TAB2); fix("脛骨")
	vB = open_viewer(); pB = vB["plotter"]
	namesB = scene_bones()
	cwB, btnB, cbB, msgB = views_ui(vB)
	cbB.set("テスト視点"); btnB["この視点にする"].invoke(); pump(app, 0.4)
	imgB = only_bone(pB, namesB, ref)
	check(imgA.shape == imgB.shape, f"同じ大きさの窓（{imgA.shape} / {imgB.shape}）")
	mA, mB = mask(imgA), mask(imgB)
	i_new = iou(mA, mB)
	dpx = float(np.abs(imgA - imgB).mean()) if imgA.shape == imgB.shape else 999
	frs.MainMenuGUI._sim_cam_set(pB, (tuple(camA_world[0]), tuple(camA_world[1]), tuple(camA_world[2]), float(camA_world[3]), float(camA_world[4]), int(camA_world[5])))
	pump(app, 0.2)
	i_old = iou(mA, mask(only_bone(pB, namesB, ref)))
	check(mA.sum() > 1000 and i_new > 0.97 and dpx < 3.0,
	      f"別の試験・別の骨を固定していても、{ref}が同じに見える（重なり IoU {i_new:.3f}・画素の差の平均 {dpx:.2f}・{int(mA.sum())} 画素）")
	check(i_old < i_new - 0.05, f"比べ: ワールドのままのカメラ（これまでの呼び出し）では、ずれる（IoU {i_old:.3f}）")
	# ---- 古い視点（local なし・大腿骨を固定して保存）も、そのまま骨に対する視点として使える
	d = json.load(open(VP, encoding="utf-8"))
	d["group:g8a86693d"].append({"name": "古い視点", "cam": camA_world, "fixed": "大腿骨", "time": "2026-10-01T00:00:00"})
	d["group:g8a86693d"].append({"name": "固定なしの古い視点", "cam": camA_world, "fixed": "", "time": "2026-10-01T00:00:00"})
	json.dump(d, open(VP, "w", encoding="utf-8"), ensure_ascii=False)
	cbB.set("古い視点"); btnB["この視点にする"].invoke(); pump(app, 0.3)
	i_leg = iou(mA, mask(only_bone(pB, namesB, ref)))
	check(i_leg > 0.97, f"古い視点（大腿骨を固定して保存・local なし）も、脛骨を固定した別の試験で同じに見える（IoU {i_leg:.3f}）")
	cbB.set("固定なしの古い視点"); btnB["この視点にする"].invoke(); pump(app, 0.3)
	check("固定せずに保存" in str(msgB.cget("text")) and "保存し直す" in str(msgB.cget("text")), f"固定なしで保存した古い視点には知らせ（{str(msgB.cget('text'))[:50]}…）")
	cwB.tk.call(cwB.protocol("WM_DELETE_WINDOW")); pump(app, 0.5)
	sel(TAB)
	if FIX0 is not None:
		for b_, f_ in zip(app.ankle_bones, FIX0):
			b_["fixed"] = f_
finally:
	if os.path.exists(VB):
		shutil.move(VB, VP)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
