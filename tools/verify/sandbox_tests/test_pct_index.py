# -*- coding: utf-8 -*-
"""初期長を変えたあとも計算結果が消えない（2026-10-23 夕の不具合の修正）:
本番と同じ状況（初期長 AM 96・PL 100・PCL 102 %、結果は AM 91・PL 100・PCL 未設定 で計算したもの）で、
初期長の一覧（results/pct_index.json）が無いと結果が見つからない（不具合の再現）・一覧があれば見つかって靭帯が出る（たるみはその計算の %）・
計算すると一覧に書く・その後に初期長を変えて開き直しても、その結果を使う。
本番の束のデータと計算結果 lb_ff59…（AM 91 %）を写してから回す。区間だけを実際に計算する。4〜5分。"""
import importlib.util, os, sys, time, gc, shutil, json
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
STORE = os.path.join(HERE, "cache", "ankle_lig_bundles", "g8a86693d.json")
RES = os.path.join(HERE, "cache", "ankle_lig_bundles", "results")
IDX = os.path.join(RES, "pct_index.json")
STB = STORE + ".testbak"
shutil.copy2(STORE, STB)
if os.path.exists(IDX):
	os.remove(IDX)
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
try:
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
	app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(False)
	app.on_ankle_animate(); pump(app, 2.0)
	v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
	v["set_paused"](True)
	s2 = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
	s2.data.setdefault("params", {})["view"] = "fibers"
	s2.data["params"]["calc_range"] = "all"
	s2.data["params"].pop("adhere", None)                    # 癒着なしの前提（癒着の鍵は test_adhere で）
	s2._cf_memo = None
	ready = [b for b in s2.data["bundles"] if s2._ready(b)]
	byname = {b["name"]: b for b in ready}
	s2._len_set_put(byname["AM束"], 96.0); s2._len_set_put(byname["PL束"], 100.0); s2._len_set_put(byname["PCL"], 102.0)
	s2._save()
	# ---- 不具合の再現（一覧なし）
	s2._load_cached_results()
	check(len(s2.results) == 0, f"初期長の一覧が無いと、AM 91 % の結果があっても見つからない（不具合の再現: {len(s2.results)} 束）")
	# ---- 一覧（本番の 13:31 の結果の 1 行）
	pct91 = {}
	for b in ready:
		pct91[b["id"]] = s2._pct_explicit(b)
	s2._len_set_put(byname["AM束"], 91.0); s2._len_set_put(byname["PL束"], 100.0); s2._len_set().setdefault("pct", {}).pop(byname["PCL"]["id"], None)
	k91, base = s2._key(ready), s2._key(ready, with_pct=False)
	pmap = {b["id"]: s2._rest_pct(b) for b in ready}
	s2._len_set_put(byname["AM束"], 96.0); s2._len_set_put(byname["PL束"], 100.0); s2._len_set_put(byname["PCL"], 102.0)
	check(os.path.exists(os.path.join(RES, f"lb_{k91}.npz")), f"AM 91 %・PL 100 %・PCL 未設定 の結果がある（lb_{k91}）")
	json.dump({base: [{"key": k91, "pct": pmap, "time": "2026-10-23T13:31:02"}]}, open(IDX, "w", encoding="utf-8"))
	s2._load_cached_results(); s2._rebuild_actors(); s2.update(s2.fi, render=True)
	done = s2.__dict__.get("rest_pct_done") or {}
	check(len(s2.results) == len(ready) and s2.rest_stale is True and abs(done.get(byname["AM束"]["id"], -1) - 91.0) < 1e-9
	      and abs(done.get(byname["PCL"]["id"], -1) - 100.0) < 1e-9,
	      f"一覧があれば見つかる（{len(s2.results)} 束・たるみは AM {done.get(byname['AM束']['id'])} %・PCL {done.get(byname['PCL']['id'])} % の計算）")
	vis = [bool(s2.actors[b["id"]]["fib_act"].GetVisibility()) for b in ready]
	check(all(vis), f"靭帯が出る（線維の見え方 {vis}）")
	# ---- 計算すると一覧に書く → 初期長を変えて開き直しても、その結果を使う
	s2.data["params"]["calc_range"] = "anterior"; s2._cf_memo = None
	ok_ = s2.compute()
	idx = json.load(open(IDX, encoding="utf-8"))
	b2 = s2._key(ready, with_pct=False)
	ent = (idx.get(b2) or [{}])[-1]
	check(ok_ and ent.get("key") == s2._key(ready) and abs(float(ent["pct"][byname["AM束"]["id"]]) - 96.0) < 1e-9,
	      f"計算すると一覧に書く（{ent.get('key')}・AM {ent.get('pct', {}).get(byname['AM束']['id'])} %）")
	s2._len_set_put(byname["AM束"], 80.0)
	s2._save(); s2._load_cached_results(); s2._rebuild_actors()
	done = s2.__dict__.get("rest_pct_done") or {}
	check(len(s2.results) == len(ready) and abs(done.get(byname["AM束"]["id"], -1) - 96.0) < 1e-9,
	      f"その後に AM を 80 % にして開き直しても、さっきの結果を使う（{len(s2.results)} 束・たるみは AM {done.get(byname['AM束']['id'])} % の計算）")
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	shutil.move(STB, STORE)
	if os.path.exists(IDX):
		os.remove(IDX)
	app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
