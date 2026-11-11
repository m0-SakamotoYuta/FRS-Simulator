# -*- coding: utf-8 -*-
"""付着部の楕円だけ（大腿骨側のみ・脛骨側のみ）（2026-10-25）: 表示の選択肢が増える・それぞれで出る楕円の端（骨の名前）・
線維・皮・断面のめやすは隠す・覚える・「付着部の楕円」のチェックを切り替えても片側のまま・ほかの表示に戻すと今までどおり。束の保存は元へ戻す。1分。"""
import importlib.util, os, sys, time, gc, shutil
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, json
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
TAB = "261007_AP_FE60_軸力100"
try:
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
	app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(False)
	app.on_ankle_animate(); pump(app, 2.0)
	v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
	v["set_paused"](True)
	s2 = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
	for b in s2.data["bundles"]:
		b["visible"] = True
	s2.data["params"]["show_outline"] = False
	s2._save(); s2._load_cached_results(); s2._rebuild_actors()
	s2.open_window(); pump(app, 0.6)
	rbs = {str(w.cget("text")): w for w in walk(s2.win) if w.winfo_class() == "TRadiobutton" and "付着部の楕円だけ" in str(w.cget("text"))}
	check(set(rbs) == {"付着部の楕円だけ", "付着部の楕円だけ（大腿骨側のみ）", "付着部の楕円だけ（脛骨側のみ）"}, f"表示の選択肢（{list(rbs)}）")
	def shown():
		ell, other = [], []
		for b in s2.data["bundles"]:
			a = s2.actors.get(b.get("id")) or {}
			for end in ("a", "b"):
				act = a.get(f"e{end}_act")
				if act is not None and act.GetVisibility():
					ell.append(str((b.get(end) or {}).get("bone_name", "")))
			for k_ in ("fib_act", "skin_act", "xs_act"):
				if a.get(k_) is not None and a[k_].GetVisibility():
					other.append(k_)
		return ell, other
	n_ends = sum(1 for b in s2.data["bundles"] for end in ("a", "b") if (s2.actors.get(b.get("id")) or {}).get(f"e{end}_act") is not None)
	for lab, want in (("付着部の楕円だけ（大腿骨側のみ）", "大腿骨"), ("付着部の楕円だけ（脛骨側のみ）", "脛骨")):
		rbs[lab].invoke(); pump(app, 0.4)
		ell, other = shown()
		check(len(ell) > 0 and all(want in x for x in ell) and len(ell) == sum(1 for b in s2.data["bundles"] for end in ("a", "b") if want in str((b.get(end) or {}).get("bone_name", ""))) and not other,
		      f"{lab}: 楕円は{want}側だけ（{len(ell)} 個）・線維・皮・断面のめやすは出さない（{other}）")
	saved = json.load(open(STORE, encoding="utf-8"))
	check(saved["params"].get("view") == "foot_tib", f"覚える（{saved['params'].get('view')}）")
	# 「付着部の楕円」のチェックを切り替えても片側のまま
	cbo = [w for w in walk(s2.win) if w.winfo_class() == "TCheckbutton" and str(w.cget("text")) == "付着部の楕円"]
	if cbo:
		cbo[0].invoke(); pump(app, 0.3); cbo[0].invoke(); pump(app, 0.3)
	s2._old_ring_vis(); pump(app, 0.2)
	ell, other = shown()
	check(len(ell) > 0 and all("脛骨" in x for x in ell), f"見え方を合わせ直しても片側のまま（{len(ell)} 個）")
	rbs["付着部の楕円だけ"].invoke(); pump(app, 0.4)
	ell, other = shown()
	check(len(ell) == n_ends and not other, f"「付着部の楕円だけ」は今までどおり両側（{len(ell)} / {n_ends} 個）")
	rb_f = [w for w in walk(s2.win) if w.winfo_class() == "TRadiobutton" and str(w.cget("text")) == "線維"][0]
	rb_f.invoke(); pump(app, 0.4)
	ell, other = shown()
	check("fib_act" in other and len(ell) == 0, f"「線維」に戻すと今までどおり（線維あり・楕円は「付着部の楕円」のチェックがオフなので出さない: {len(ell)} 個）")
	try: s2.win.destroy()
	except Exception: pass
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	shutil.move(STB, STORE)
	app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
