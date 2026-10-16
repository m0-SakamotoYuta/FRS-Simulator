# -*- coding: utf-8 -*-
"""靭帯の 1点目 / 2点目 を打ち直す（2026-10-15）。右クリックは靭帯の機能の中の _do_pick を直接呼ぶ。"""
import importlib.util, os, sys, time, copy, gc
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
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: None
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
def walk(w):
	for c in w.winfo_children():
		yield c
		yield from walk(c)
TAB = "261007_FE_軸力100_ACL切"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
L0 = copy.deepcopy(app.ankle_ligaments[0]); L0["name"] = "ACL"; app.ankle_ligaments = [L0]
app.ankle_alt_model.set(False); app.ankle_joint_only.set(False)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
lig = next(o for o in gc.get_objects() if isinstance(o, dict) and "open" in o and "update" in o and "close" in o and callable(o.get("open")))
def find_fn(fn, name, seen=None):
	seen = seen if seen is not None else set()
	if id(fn) in seen: return None
	seen.add(id(fn))
	for c in (getattr(fn, "__closure__", None) or []):
		try: o = c.cell_contents
		except ValueError: continue
		if callable(o) and getattr(o, "__name__", "") == name: return o
		if callable(o) and hasattr(o, "__closure__"):
			r = find_fn(o, name, seen)
			if r: return r
	return None
do_pick = find_fn(lig["open"], "_do_pick") or find_fn(lig["update"], "_do_pick")
st = next(c.cell_contents for c in do_pick.__closure__ if isinstance(getattr(c, "cell_contents", None), dict) and "pick_on" in c.cell_contents)
lig["open"](); pump(app, 0.5)
lw = [w for w in walk(app) if isinstance(w, tk.Toplevel) and "靭帯" in w.title()][-1]
b1 = [w for w in walk(lw) if w.winfo_class() == "TButton" and str(w.cget("text")) == "1点目を打ち直す"]
b2 = [w for w in walk(lw) if w.winfo_class() == "TButton" and str(w.cget("text")) == "2点目を打ち直す"]
check(b1 and b2, "各行に「1点目を打ち直す」「2点目を打ち直す」")
old = copy.deepcopy(app.ankle_ligaments[0])
# 画面上の、脛骨の上の点を探す（手前に見えている点）
ren = p.renderer
names_ = [a.get("name") for a in []]
ti = next(i for i, b in enumerate(app.ankle_bones) if b["name"] == "脛骨")
a = p.renderer.actors[f"bone_{ti}"]
pts = a.GetMapper().GetInput().GetPoints()
um = a.GetUserMatrix(); M = np.array([[um.GetElement(r, c) for c in range(4)] for r in range(4)]) if um else np.eye(4)
def click_on_tibia(k0):
	for k in range(k0, pts.GetNumberOfPoints(), max(1, pts.GetNumberOfPoints() // 60)):
		wp = M @ np.append(np.array(pts.GetPoint(k)), 1.0)
		ren.SetWorldPoint(*wp); ren.WorldToDisplay(); x, y, _ = ren.GetDisplayPoint()
		before = copy.deepcopy(app.ankle_ligaments[0]) if app.ankle_ligaments else None
		do_pick(int(x), int(y)); pump(app, 1.0)
		if st.get("repick") is None:
			return True
	return False
b2[0].invoke(); pump(app, 0.2)
check(st.get("repick") is not None and st.get("pick_on"), "「2点目を打ち直す」を押すと、右クリックを待つ（右クリックで指定がオンになる）")
ok = click_on_tibia(0)
new = app.ankle_ligaments[0]
check(ok and new["a"]["point"] == old["a"]["point"] and new["b"]["point"] != old["b"]["point"], "右クリックで 2点目だけが変わる（1点目はそのまま）")
check(new["b"]["bone_name"] in ("脛骨", "大腿骨", "膝蓋骨") and len(app.ankle_ligaments) == 1, f"靭帯は増えない・2点目の骨は {new['b']['bone_name']}")
old2 = copy.deepcopy(new)
b1 = [w for w in walk(lw) if w.winfo_class() == "TButton" and str(w.cget("text")) == "1点目を打ち直す"]
b1[0].invoke(); pump(app, 0.2)
ok = click_on_tibia(7)
new2 = app.ankle_ligaments[0]
check(ok and new2["b"]["point"] == old2["b"]["point"] and new2["a"]["point"] != old2["a"]["point"], "右クリックで 1点目だけが変わる")
# 取り消し
b1 = [w for w in walk(lw) if w.winfo_class() == "TButton" and str(w.cget("text")) == "1点目を打ち直す"]
b1[0].invoke(); pump(app, 0.2)
cancel = [w for w in walk(lw) if w.winfo_class() == "TButton" and str(w.cget("text")) == "1点目を取り消す"]
cancel[0].invoke(); pump(app, 0.2)
check(st.get("repick") is None, "「1点目を取り消す」で打ち直しをやめられる")
lig["close"]()
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
