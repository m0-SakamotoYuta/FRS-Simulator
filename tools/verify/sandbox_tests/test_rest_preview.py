# -*- coding: utf-8 -*-
"""初期長を変えたら、いまのコマだけすぐ計算（2026-10-23）: スライダーの幅（50〜150 %・数の欄 30〜200 %）・
変えると 0.25 秒止まってからいまのコマだけ計算して 3D の線維が変わる（かかった秒数）・線維の長さ ≤ 新しい初期長（伸びない）・
初期長を大きくするとたるむ・ほかのコマは前の計算のまま・くり返しを増やした計算との差・チェックを外すと計算しない。
段階2の束（入っている結果）を使う。接触の計算はダミー。束の保存はテストの後に元へ戻す。3〜4分。"""
import importlib.util, os, sys, time, gc, shutil
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk
from vtk.util.numpy_support import vtk_to_numpy
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
def plen(X):
	return np.linalg.norm(np.diff(X, axis=1), axis=2).sum(axis=1)
try:
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
	app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(False)
	app.on_ankle_animate(); pump(app, 2.0)
	v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
	v["set_paused"](True)
	s2 = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
	for b in s2.data["bundles"]:
		b["visible"] = True                                   # 初期長はユーザーの設定のまま（その計算結果を使う）
	s2.data.setdefault("params", {})["rest_preview"] = True
	s2.data["params"]["view"] = "fibers"                      # 線維を見せる（本番の設定は「付着部の楕円だけ」のことがある）
	s2._save(); s2._load_cached_results(); s2._rebuild_actors()
	ready = [b for b in s2.data["bundles"] if s2._ready(b) and s2.results.get(b.get("id")) is not None]
	check(len(ready) >= 2, f"計算済みの束（{[b.get('name') for b in ready]}）")
	ft = np.asarray(s2.scene["frame_times"], float)
	fi_go = int(0.45 * s2.n_view)
	v["seek"](float(ft[fi_go]) + 0.003); pump(app, 0.4)
	fi = int(s2.fi)
	s2.open_window(); pump(app, 0.6)
	rw = s2.w["rest_w"]
	b0 = ready[0]
	_b, vv0, lab0, now0 = rw[b0["id"]]
	scs = [w for w in walk(s2.w["rest_box"]) if w.winfo_class() == "TScale"]
	check(len(scs) == len(rw) and float(scs[0].cget("from")) == 50.0 and float(scs[0].cget("to")) == 150.0, "スライダーは 50〜150 %（以前は 80〜120 %）")
	a0 = s2.actors[b0["id"]]
	P_before = vtk_to_numpy(a0["fib_pts"].GetData()).copy()
	r0 = s2.results[b0["id"]]
	pct0 = float((s2.__dict__.get("rest_pct_done") or {}).get(b0["id"], 100.0))
	Lt = np.asarray(r0["L0"], float) / (pct0 / 100.0)          # 基準のコマで張った長さ（前の計算の % で割り戻す）
	check(not s2.__dict__.get("rest_stale"), f"いまの初期長（{b0['name']} {pct0:g} %）で計算した結果がある")
	# ---- 85 % に（スライダー）→ 0.25 秒止まってからこのコマだけ計算
	t0 = time.time()
	vv0.set(85.0); s2._rest_slide(b0, vv0, lab0)
	while time.time() - t0 < 60 and not (s2.__dict__.get("_preview") or {}).get(b0["id"]):
		pump(app, 0.05)
	el = time.time() - t0
	pv = (s2.__dict__.get("_preview") or {}).get(b0["id"])
	check(pv is not None and pv[0] == fi and abs(pv[2] - 85.0) < 1e-9, f"85 % にすると、いまのコマ（{fi}）だけ計算した（止まってから {el:.1f} 秒で表示）")
	msg = str(s2.w["status"].cget("text")) if s2.w.get("status") is not None else ""
	check("このコマ" in msg and "秒" in msg, f"知らせ（{msg[:70]}…）")
	P_after = vtk_to_numpy(a0["fib_pts"].GetData()).copy()
	mv = float(np.abs(P_after - P_before).max()) if P_after.shape == P_before.shape else 999
	check(mv > 0.05, f"3D の線維が変わった（いちばん動いた点 {mv:.2f} mm）")
	loc85 = pv[1].astype(float)
	L85 = plen(loc85)
	Lprev = plen(s2._local_at(r0, fi))
	check(float(np.median(L85)) <= float(np.median(Lprev)) + 0.05,
	      f"初期長を短くすると、線維は縮む・張る（長さの中央値 {np.median(Lprev):.2f} → {np.median(L85):.2f} mm。短すぎて届かない線維はぴんと張ったまま）")
	# 同じ条件でくり返しを増やした計算（収束した形）との差
	import ankle_lig_bundles as _alb
	g = int(s2.s0 + fi)
	Ts = {int(i): np.asarray(T, float) for (i, nm, m, T) in s2.bones_full}
	rts = []
	for b in ready:
		rt = s2._runtime(b)
		Tf = Ts[int(rt["a"])][g]
		X0 = s2._local_at(s2.results[b["id"]], fi) @ Tf[:3, :3].T + Tf[:3, 3]
		_pd = float((s2.__dict__.get("rest_pct_done") or {}).get(b["id"], 100.0))
		rts.append(dict(rt, rest_scale=s2._rest_pct(b) / 100.0, _X0=X0, _Lt=np.asarray(s2.results[b["id"]]["L0"], float) / (_pd / 100.0)))
	ref = _alb.simulate(s2.app, s2.bones_full, rts, s2.s0, dict(s2.params(), rest_frame=s2._rest_frame(), only_frames=[g], repeat=20, skip_center=True),
	                    bones=s2._ensure_bones())
	dref = float(np.linalg.norm(ref[0]["local"][0].astype(float) - loc85, axis=2).mean())
	dold = float(np.linalg.norm(s2._local_at(r0, fi) - loc85, axis=2).mean())
	check(dref < 0.5 * max(dold, 1e-9) or dref < 0.15, f"くり返しを 5 倍にした計算との差は小さい（平均 {dref:.3f} mm。前の計算からは {dold:.3f} mm 動いた）")
	# ---- 130 % に（数の欄）→ たるむ
	t0 = time.time()
	ents = [w for w in walk(s2.w["rest_box"]) if w.winfo_class() == "TSpinbox"]
	ents[0].focus_force(); pump(app, 0.2)
	vv0.set(130.0); ents[0].event_generate("<Return>")
	while time.time() - t0 < 60 and not ((s2.__dict__.get("_preview") or {}).get(b0["id"]) or (0, 0, 0))[2] == 130.0:
		pump(app, 0.05)
	pv = (s2.__dict__.get("_preview") or {}).get(b0["id"])
	L130 = plen(pv[1].astype(float)) if pv is not None else np.zeros(1)
	r130 = L130 / (Lt * 1.30)
	check(pv is not None and abs(pv[2] - 130.0) < 1e-9 and float(np.median(L130)) > float(np.median(L85)) + 0.5 and r130.max() <= 1.03,
	      f"130 %（数の欄）にすると、たるむ（線維の長さの中央値 {np.median(L85):.1f} → {np.median(L130):.1f} mm・長さ ÷ 初期長 {r130.min():.3f}〜{r130.max():.3f} ≤ 1＝伸びない）")
	P130 = vtk_to_numpy(a0["fib_pts"].GetData()).copy()
	check(float(np.abs(P130 - P_after).max()) > 0.5 if P130.shape == P_after.shape else False, f"3D の線維もたるんだ形に（いちばん動いた点 {float(np.abs(P130 - P_after).max()) if P130.shape == P_after.shape else -1:.2f} mm）")
	# 数の欄なら 30〜200 %
	vv0.set(40.0); ents[0].event_generate("<Return>"); pump(app, 0.3)
	check(abs((s2._pct_explicit(b0) if s2._pct_explicit(b0) is not None else -1) - 40.0) < 1e-9, f"数の欄ならスライダーの外（40 %）も入れられる（{b0.get('rest_pct')}）")
	vv0.set(500.0); ents[0].event_generate("<Return>"); pump(app, 0.3)
	check(abs((s2._pct_explicit(b0) if s2._pct_explicit(b0) is not None else -1) - 200.0) < 1e-9, f"上限は 200 %（{b0.get('rest_pct')}）")
	t0 = time.time()
	while time.time() - t0 < 60 and not ((s2.__dict__.get("_preview") or {}).get(b0["id"]) or (0, 0, 0))[2] == 200.0:
		pump(app, 0.05)
	# ---- ほかのコマは前の計算のまま
	check(s2._preview_local(b0, fi + 5) is None, "ほかのコマは前の計算のまま（このコマだけ）")
	# ---- チェックを外すと計算しない
	s2.w["prev_var"].set(False)
	cks = [w for w in walk(s2.w["rest_box"]) if w.winfo_class() == "TCheckbutton" and "いまのコマだけすぐ計算" in str(w.cget("text"))]
	cks[0].invoke(); cks[0].invoke() if s2.w["prev_var"].get() else None
	s2.w["prev_var"].set(False); s2.data["params"]["rest_preview"] = False
	before = dict(s2.__dict__.get("_preview") or {})
	vv0.set(90.0); s2._rest_slide(b0, vv0, lab0); pump(app, 1.0)
	check((s2.__dict__.get("_preview") or {}).get(b0["id"]) == before.get(b0["id"]), "「いまのコマだけすぐ計算」を外すと、計算しない")
	try: s2.win.destroy()
	except Exception: pass
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	shutil.move(STB, STORE)
	app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
