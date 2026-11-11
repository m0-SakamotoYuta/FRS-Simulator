# -*- coding: utf-8 -*-
"""「▶ 計算する」が、前の計算を代わりに見せているときに計算を始める（2026-10-25 の不具合の修正）:
癒着オフ（計算済み）→ 計算するは計算せず「保存した結果を使いました」・癒着オン（この設定の結果は無い＝前の計算を代わりに見せる）
→ 計算するで計算を始める（以前は「保存した結果を使いました」で何もしなかった）・初期長を計算していない値にしても同じ。
本当の計算はしない（compute を差し替えて呼ばれたかだけ見る）。束の設定と結果は本番から写したものを使う
（cache/ankle_lig_bundles/g8a86693d.json・results の lb_43d1…（癒着なし・前方引張）と pct_index.json）。1分。"""
import importlib.util, os, sys, time, gc, json, shutil
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
try:
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_AP_FE60_軸力100")); pump(app, 0.5)
	app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(False)
	app.on_ankle_animate(); pump(app, 3.0)
	v = app._sim_viewers_alive()[-1]; p = v["plotter"]; v["set_paused"](True)
	s2 = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
	s2.open_window(); pump(app, 0.8)
	calls = []
	s2.compute = lambda *a, **k: (calls.append(1), False)[1]
	cb = [w for w in walk(s2.win) if w.winfo_class() == "TCheckbutton" and "癒着" in str(w.cget("text"))][0]
	btn = [w for w in walk(s2.win) if w.winfo_class() == "TButton" and str(w.cget("text")) == "▶ 計算する"][0]
	st = lambda: str(s2.w["status"].cget("text"))
	if s2.w["adh_on"].get():
		cb.invoke(); pump(app, 3.0)
	check(not s2.w["adh_on"].get() and not s2.rest_stale and len(s2.results) == 3, f"癒着オフ: この設定の計算済みの結果（{st()[:30]}）")
	btn.invoke(); pump(app, 0.5)
	check(len(calls) == 0 and "保存した結果を使いました" in st(), f"計算済みなら、計算するで計算し直さない（{st()[:30]}）")
	cb.invoke(); pump(app, 6.0)
	check(s2.w["adh_on"].get() and s2.rest_stale and len(s2.results) == 3, "癒着オン: この設定の結果は無く、前の計算を代わりに見せている")
	btn.invoke(); pump(app, 0.5)
	check(len(calls) == 1 and "保存した結果を使いました" not in st(), f"そこで計算するを押すと、計算を始める（呼ばれた {len(calls)} 回・{st()[:30]}）")
	cb.invoke(); pump(app, 3.0)
	# 初期長を計算していない値に
	b0 = s2.data["bundles"][0]
	old = s2._rest_pct(b0)
	s2._len_set_put(b0, old + 3.0); s2._save(); s2._load_cached_results()
	check(s2.rest_stale and len(s2.results) == 3, f"初期長 {old:g} → {old + 3:g} %: 前の計算を代わりに見せている")
	n0 = len(calls)
	btn.invoke(); pump(app, 0.5)
	check(len(calls) == n0 + 1, "そこで計算するを押すと、計算を始める")
finally:
	shutil.move(STB, STORE)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
