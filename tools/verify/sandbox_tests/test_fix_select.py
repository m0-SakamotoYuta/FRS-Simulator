# -*- coding: utf-8 -*-
"""再生コントロールで、固定する骨を変える（③の「固定」を書き換えて開き直す。接触は計算し直さない）。2026-10-15"""
import importlib.util, os, sys, time
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
frs.messagebox.showinfo = lambda *a, **k: None
frs.messagebox.askyesno = lambda *a, **k: True
app = frs.MainMenuGUI(); app.withdraw(); app.update()
dlg = {"n": 0}
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
def _d(N, has_cartilage=False):
	dlg["n"] += 1
	return (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
app._show_precompute_dialog = _d
captured = []
_orig = app._sim_engine_run
app._sim_engine_run = lambda sc: (captured.append(sc), _orig(sc))[1]
def walk(w):
	for c in w.winfo_children():
		yield c
		yield from walk(c)
TAB = "261007_FE_軸力100_ACL切"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
for b in app.ankle_bones: b["fixed"] = (b["name"] == "大腿骨")
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]
fv = v["ctrl"].get("fix_var")
cbs = [w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TCombobox" and "（固定しない）" in list(w.cget("values"))]
check(fv is not None and fv.get() == "大腿骨" and cbs, f"再生コントロールに「固定する骨」（いまは {fv.get() if fv else None}）")
sc = captured[-1]
fi = next(i for i, b in enumerate(sc["bones"]) if b["name"] == "大腿骨")
check(np.allclose(np.asarray(sc["bones"][fi]["poses"])[::500], np.eye(4), atol=1e-9), "大腿骨を固定: 大腿骨は動かない")
n0 = len(captured); d0 = dlg["n"]
fv.set("脛骨"); cbs[0].event_generate("<<ComboboxSelected>>"); pump(app, 4.0)
check(len(captured) == n0 + 1, "変えると開き直す")
sc2 = captured[-1]
ti = next(i for i, b in enumerate(sc2["bones"]) if b["name"] == "脛骨")
check(np.allclose(np.asarray(sc2["bones"][ti]["poses"])[::500], np.eye(4), atol=1e-9), "脛骨を固定: 脛骨は動かない")
check([b["fixed"] for b in app.ankle_bones] == [b["name"] == "脛骨" for b in app.ankle_bones], "③の骨リストの「固定」も脛骨になる（1本だけ）")
check(dlg["n"] == d0, "接触の計算はし直さない（保存済みを使う）")
v2 = app._sim_viewers_alive()[-1]
check(v2["ctrl"]["fix_var"].get() == "脛骨", "開き直した再生コントロールも「脛骨」")
cb2 = [w for w in walk(v2["ctrl"]["window"]) if w.winfo_class() == "TCombobox" and "（固定しない）" in list(w.cget("values"))][0]
v2["ctrl"]["fix_var"].set("（固定しない）"); cb2.event_generate("<<ComboboxSelected>>"); pump(app, 4.0)
check(not any(b.get("fixed") for b in app.ankle_bones), "「（固定しない）」で固定を外せる")
v3 = app._sim_viewers_alive()[-1]
v3["ctrl"]["fix_var"].set("大腿骨"); [w for w in walk(v3["ctrl"]["window"]) if w.winfo_class() == "TCombobox" and "（固定しない）" in list(w.cget("values"))][0].event_generate("<<ComboboxSelected>>"); pump(app, 4.0)
check([b["fixed"] for b in app.ankle_bones] == [b["name"] == "大腿骨" for b in app.ankle_bones], "大腿骨に戻せる")
w_ = app._sim_viewers_alive()[-1]["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
check(len(app._sim_viewers_alive()) == 0, "前の窓は残らない（開き直しのたびに閉じる）")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
