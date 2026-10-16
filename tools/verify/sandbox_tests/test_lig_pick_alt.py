# -*- coding: utf-8 -*-
"""表示変更用モデルを表示中でも、靭帯の点が骨に当たる（2026-10-15 の不具合）。接触の計算はダミー。"""
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
frs.messagebox.showwarning = lambda *a, **k: None
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
TAB = "261007_FE_軸力100_ACL切"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
D = os.path.join(HERE, "display_models"); os.makedirs(D, exist_ok=True)
for b in app.ankle_bones:       # どの骨にも表示変更用モデル（元のモデルの写し）
	fp = os.path.join(D, f"alt_{b['aruco_id']}.ply")
	pv.read(b["model_path"]).extract_surface().triangulate().clean().save(fp)
	b["display_model_path"] = fp
app.ankle_alt_model.set(True)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
names = list(p.renderer.actors.keys())
alt = [k for k in names if k.endswith("_alt")]
check(alt and all(p.renderer.actors[k].GetVisibility() for k in alt) and not p.renderer.actors["bone_0"].GetVisibility(),
      "表示変更用モデルを表示中（元の骨は隠れている）")
# 靭帯の機能の中の _do_pick を取り出す
def find_fn(fn, name, seen=None):
	seen = seen or set()
	if id(fn) in seen: return None
	seen.add(id(fn))
	for c in (getattr(fn, "__closure__", None) or []):
		try: o = c.cell_contents
		except ValueError: continue
		if callable(o) and getattr(o, "__name__", "") == name: return o
		if callable(o) and hasattr(o, "__closure__"):
			r = find_fn(o, name, seen)
			if r: return r
		if isinstance(o, dict):
			for vv in o.values():
				if callable(vv):
					r = find_fn(vv, name, seen)
					if r: return r
	return None
eng = None
# viewer_ref の関数から lig_ctx をたどる
for key in ("seek", "set_paused", "time"):
	fn = v.get(key)
	r = find_fn(fn, "_do_pick") if fn else None
	if r: eng = r; break
lig = None
import gc
for o in gc.get_objects():
	if isinstance(o, dict) and "open" in o and "update" in o and "close" in o and callable(o.get("open")):
		lig = o
do_pick = find_fn(lig["open"], "_do_pick") or find_fn(lig["update"], "_do_pick")
st = None
for c in (do_pick.__closure__ or []):
	try:
		if isinstance(c.cell_contents, dict) and "pick_on" in c.cell_contents: st = c.cell_contents
	except ValueError:
		pass
lig["open"](); pump(app, 0.5)
st["pick_var"].set(True); st["pick_on"] = True
# 脛骨の表示変更用モデルの上の点を画面の座標に
a = p.renderer.actors[alt[1]]
pts = a.GetMapper().GetInput().GetPoints()
ren = p.renderer
hit_ok = False
msgs = []
_orig_status = None
for k in range(0, pts.GetNumberOfPoints(), max(1, pts.GetNumberOfPoints() // 40)):
	w = np.array(pts.GetPoint(k)); um = a.GetUserMatrix()
	M = np.array([[um.GetElement(r, c) for c in range(4)] for r in range(4)]) if um is not None else np.eye(4)
	wp = M @ np.append(w, 1.0)
	ren.SetWorldPoint(*wp); ren.WorldToDisplay(); x, y, _z = ren.GetDisplayPoint()
	st["pending"] = None
	do_pick(int(x), int(y)); pump(app, 0.05)
	if st["pending"] is not None:
		hit_ok = True
		si, local = st["pending"]
		break
check(hit_ok, "表示変更用モデルの上を右クリックすると、骨に当たる（1点目が置ける）")
if hit_ok:
	# 当たった点（骨のモデルの座標）が、その骨のモデルの表面の上にある（手前の骨に当たってもよい）
	sm = pv.read(app.ankle_bones[si]["display_model_path"]).extract_surface().triangulate()
	dd = abs(float(pv.PolyData(np.asarray([local])).compute_implicit_distance(sm)["implicit_distance"][0]))
	check(dd < 0.5, f"当たった点は、当たった骨（{app.ankle_bones[si]['name']}）の表面の上（{dd:.3f} mm）")
lig["close"]() if callable(lig.get("close")) else None
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
for b in app.ankle_bones: b.pop("display_model_path", None)
app.ankle_alt_model.set(False)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
