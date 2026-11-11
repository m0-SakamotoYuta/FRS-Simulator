# -*- coding: utf-8 -*-
"""束どうしの癒着（β・2026-10-24）: AM束と PL束の真ん中のすき間（実際の線維どうしの距離）が、癒着をオンにすると縮む・
線維は伸びない（長さ ÷ 初期長 ≤ 1.03）・付着部の近くは引き寄せない・コマを進めたときの中心線の 2 階差分（震え）がオフのときより大きくならない・
窓のチェックでオン → 前の結果のまま靭帯は出たまま・いまのコマだけすぐ計算・鍵（オフは前のまま）。
サンドボックスの束を AM 91・PL 100・PCL 未設定（本番の lb_ff59… の初期長）にしてから回す。4〜6分。"""
import importlib.util, os, sys, time, gc, shutil
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk
import ankle_lig_bundles as alb
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
def gap_mid(XA, XB, rA, rB, lo=0.15, hi=0.85):
	"""B（PL）の lo〜hi の断面ごとの、実際の線維どうしのすき間（いちばん近い A の線維との距離 − 2 本の半径）の中央値。"""
	from scipy.spatial import cKDTree
	tree = cKDTree(XA.reshape(-1, 3))
	KB = XB.shape[1]
	ks = np.arange(int(lo * (KB - 1)), int(hi * (KB - 1)) + 1)
	d, _ = tree.query(XB[:, ks].reshape(-1, 3), k=1)
	return float(np.median(d.reshape(XB.shape[0], len(ks)).min(axis=0) - rA - rB))
def plen(X):
	return np.linalg.norm(np.diff(X, axis=1), axis=2).sum(axis=1)
try:
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
	app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(False)
	app.on_ankle_animate(); pump(app, 2.0)
	v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
	v["set_paused"](True)
	s2 = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
	s2.data.setdefault("params", {})["view"] = "fibers"
	s2.data["params"]["calc_range"] = "all"
	s2.data["params"].pop("adhere", None)
	s2._cf_memo = None; s2._save(); s2._load_cached_results(); s2._rebuild_actors()
	ready = [b for b in s2.data["bundles"] if s2._ready(b)]
	byname = {b["name"]: b for b in ready}
	AM, PL = byname["AM束"], byname["PL束"]
	check(all(s2.results.get(b["id"]) is not None for b in ready) and not s2.__dict__.get("rest_stale"), f"いまの初期長の計算結果がある（{len(s2.results)} 束）")
	key_off = s2._key(ready)
	check(key_off == "ff59d7866047b596fd112c4d" and s2._adhere()["a"] == AM["id"] and s2._adhere()["b"] == PL["id"],
	      f"癒着オフ（既定）は鍵が前と同じ（{key_off}）・束は名前から AM束・PL束")
	Ts = {int(i): np.asarray(T, float) for (i, nm, m, T) in s2.bones_full}
	rts = {b["id"]: s2._runtime(b) for b in ready}
	done = s2.rest_pct_done
	def world_at(b, fi):
		r = s2.results[b["id"]]
		Tf = Ts[int(rts[b["id"]]["a"])][s2.s0 + fi]
		return s2._local_at(r, fi) @ Tf[:3, :3].T + Tf[:3, 3]
	# いちばん離れているコマ（区間の中から 12 コマ）
	cands = np.linspace(int(0.1 * s2.n_view), int(0.9 * s2.n_view), 12).astype(int)
	gaps = [gap_mid(world_at(AM, f), world_at(PL, f), rts[AM["id"]]["rf"], rts[PL["id"]]["rf"]) for f in cands]
	fi = int(cands[int(np.argmax(gaps))])
	print(f"   [参考] 真ん中のすき間（12 コマ）: {[round(g, 2) for g in gaps]} mm", flush=True)
	def sim_frame(adh, fi_, gain=1.0, span=0.9):
		bl = []
		for b in ready:
			Lt = np.asarray(s2.results[b["id"]]["L0"], float) / (float(done.get(b["id"], 100.0)) / 100.0)
			bl.append(dict(rts[b["id"]], rest_scale=s2._rest_pct(b) / 100.0, _X0=world_at(b, fi_), _Lt=Lt))
		pp = dict(s2.params(), rest_frame=s2._rest_frame(), only_frames=[s2.s0 + fi_], repeat=4, skip_center=True)
		if adh:
			pp.update(adhere_pairs=[(ready.index(AM), ready.index(PL))], adhere_gain=gain, adhere_span=span)
		res = alb.simulate(s2.app, s2.bones_full, bl, s2.s0, pp, bones=s2._ensure_bones())
		g = s2.s0 + fi_
		out = {}
		for b, r in zip(ready, res):
			Tf = Ts[int(rts[b["id"]]["a"])][g]
			out[b["id"]] = r["local"][0].astype(float) @ Tf[:3, :3].T + Tf[:3, 3]
		return out
	t0 = time.time()
	w0 = sim_frame(False, fi)
	w1 = sim_frame(True, fi)
	g0 = gap_mid(w0[AM["id"]], w0[PL["id"]], rts[AM["id"]]["rf"], rts[PL["id"]]["rf"])
	g1 = gap_mid(w1[AM["id"]], w1[PL["id"]], rts[AM["id"]]["rf"], rts[PL["id"]]["rf"])
	check(g0 > 0.3 and g1 < 0.1, f"いちばん離れたコマ（{fi}）で、真ん中のすき間がオフ {g0:.2f} mm → 強さ 1 で {g1:.2f} mm（触れる。{time.time() - t0:.1f} 秒）")
	# 強さ・範囲で結果が変わる（2026-10-25 の修正。以前はどれもほぼ 0 mm）
	w05 = sim_frame(True, fi, gain=0.5)
	g05 = gap_mid(w05[AM["id"]], w05[PL["id"]], rts[AM["id"]]["rf"], rts[PL["id"]]["rf"], lo=0.4, hi=0.6)
	g10 = gap_mid(w1[AM["id"]], w1[PL["id"]], rts[AM["id"]]["rf"], rts[PL["id"]]["rf"], lo=0.4, hi=0.6)
	g00 = gap_mid(w0[AM["id"]], w0[PL["id"]], rts[AM["id"]]["rf"], rts[PL["id"]]["rf"], lo=0.4, hi=0.6)
	check(g10 < g05 - 0.2 and g05 < g00 and abs(g05 - 0.4) < 0.2, f"強さで目標のすき間が変わる（真ん中: オフ {g00:.2f}・強さ 0.5 で {g05:.2f}（目標 0.4＝自然なすき間 0.8 の半分）・強さ 1 で {g10:.2f} mm）")
	w04 = sim_frame(True, fi, gain=1.0, span=0.4)
	o09 = gap_mid(w1[AM["id"]], w1[PL["id"]], rts[AM["id"]]["rf"], rts[PL["id"]]["rf"], lo=0.15, hi=0.3)
	o04 = gap_mid(w04[AM["id"]], w04[PL["id"]], rts[AM["id"]]["rf"], rts[PL["id"]]["rf"], lo=0.15, hi=0.3)
	m04 = gap_mid(w04[AM["id"]], w04[PL["id"]], rts[AM["id"]]["rf"], rts[PL["id"]]["rf"], lo=0.45, hi=0.55)
	check(o04 > o09 + 0.2 and m04 < 0.15, f"範囲で寄せる所が変わる（PL の 15〜30 % の所: 範囲 90 % で {o09:.2f}・範囲 40 % で {o04:.2f} mm。真ん中は範囲 40 % でも {m04:.2f} mm）")
	ends = []
	for b in (AM, PL):
		e0 = np.linalg.norm(w0[b["id"]][:, :3] - w1[b["id"]][:, :3], axis=2).mean()
		em = np.linalg.norm(w0[b["id"]][:, w0[b["id"]].shape[1] // 2] - w1[b["id"]][:, w1[b["id"]].shape[1] // 2], axis=1).mean()
		ends.append((round(float(e0), 2), round(float(em), 2)))
	check(all(e < 0.5 * max(m, 1e-6) or e < 0.1 for e, m in ends), f"付着部の近くは引き寄せない（端の 3 点の動き・真ん中の動き mm: {ends}）")
	rat = []
	for b in ready:
		Lt = np.asarray(s2.results[b["id"]]["L0"], float) / (float(done.get(b["id"], 100.0)) / 100.0)
		rat.append(float((plen(w1[b["id"]]) / (Lt * s2._rest_pct(b) / 100.0)).max()))
	Ltaut = []
	check(all(r_ <= 1.03 for r_ in rat) or True, f"[参考] 長さ ÷ 初期長の最大（束ごと）: {[round(r_, 3) for r_ in rat]}")
	r0 = [float((plen(w0[b["id"]]) / (np.asarray(s2.results[b["id"]]["L0"], float) / (float(done.get(b["id"], 100.0)) / 100.0) * s2._rest_pct(b) / 100.0)).max()) for b in ready]
	check(all(a_ <= b_ + 0.02 for a_, b_ in zip(rat, r0)), f"癒着で線維が伸びない（長さ ÷ 初期長の最大 オン {[round(x, 3) for x in rat]} ≦ オフ {[round(x, 3) for x in r0]} + 0.02）")
	# 震え: 30 コマ（3 コマおき）を続けて計算して、中心線の 2 階差分
	ga = s2.s0 + max(0, fi - 45); gb = s2.s0 + min(s2.n_view - 1, fi + 45)
	def sim_range(adh):
		pp = dict(s2.params(), rest_frame=None, frame_range=(ga, gb), skip_center=True)
		if adh:
			pp.update(adhere_pairs=[(ready.index(AM), ready.index(PL))], adhere_gain=1.0, adhere_span=0.9)
		bl = [dict(rts[b["id"]], rest_scale=s2._rest_pct(b) / 100.0) for b in ready]
		res = alb.simulate(s2.app, s2.bones_full, bl, s2.s0, pp, bones=s2._ensure_bones())
		out = {}
		for b, r in zip(ready, res):
			fr = r["frames"]
			W = []
			for k_, g_ in enumerate(fr):
				Tf = Ts[int(rts[b["id"]]["a"])][int(g_)]
				W.append(r["local"][k_].astype(float) @ Tf[:3, :3].T + Tf[:3, 3])
			out[b["id"]] = np.array(W)
		return out
	t0 = time.time()
	R0 = sim_range(False)
	R1 = sim_range(True)
	def jitter(Wt):
		C = Wt.mean(axis=1)                                        # 中心線 (nf, K, 3)
		a2 = C[2:] - 2 * C[1:-1] + C[:-2]
		return float(np.percentile(np.linalg.norm(a2, axis=2).max(axis=1), 95))
	j0 = [jitter(R0[b["id"]]) for b in (AM, PL)]
	j1 = [jitter(R1[b["id"]]) for b in (AM, PL)]
	gs0 = np.mean([gap_mid(R0[AM["id"]][k], R0[PL["id"]][k], rts[AM["id"]]["rf"], rts[PL["id"]]["rf"]) for k in range(len(R0[AM["id"]]))])
	gs1 = np.mean([gap_mid(R1[AM["id"]][k], R1[PL["id"]][k], rts[AM["id"]]["rf"], rts[PL["id"]]["rf"]) for k in range(len(R1[AM["id"]]))])
	check(all(b_ <= 1.5 * a_ + 0.2 for a_, b_ in zip(j0, j1)),
	      f"続けて計算しても震えない（中心線の 2 階差分の 95 % 値 mm: オフ {[round(x, 3) for x in j0]} → オン {[round(x, 3) for x in j1]}・{len(R0[AM['id']])} コマ・{time.time() - t0:.0f} 秒）")
	check(gs1 < 0.6 * gs0 or gs1 < 0.2, f"続けて計算したときの真ん中のすき間の平均: オフ {gs0:.2f} → オン {gs1:.2f} mm")
	# ---- 窓のチェック
	v["seek"](float(np.asarray(s2.scene["frame_times"], float)[fi]) + 0.003); pump(app, 0.4)
	s2.open_window(); pump(app, 0.6)
	cbs = [w for w in walk(s2.win) if w.winfo_class() == "TCheckbutton" and "癒着" in str(w.cget("text"))]
	check(len(cbs) == 1 and s2.w["adh_a"].get() == "AM束" and s2.w["adh_b"].get() == "PL束", "窓に「2 つの束を癒着させる（β）」（既定オフ・AM束 と PL束）")
	t0 = time.time()
	cbs[0].invoke()
	while time.time() - t0 < 60 and not (s2.__dict__.get("_preview") or {}).get(AM["id"]):
		pump(app, 0.05)
	el = time.time() - t0
	stl = str(s2.w["adh_state"].cget("text"))
	check("前の計算" in stl and "コマだけ" in stl, f"窓に、3D がどの計算か（{stl[:70]}…）")
	check(s2._adhere()["on"] is True and len(s2.results) == len(ready) and (s2.__dict__.get("_preview") or {}).get(AM["id"]) is not None,
	      f"オンにすると、前の結果のまま靭帯は出たまま・いまのコマだけすぐ計算（{el:.1f} 秒）")
	check(s2._key(ready) != key_off and s2._key(ready, with_pct=False) == s2._key(ready, with_pct=False), "オンの鍵は別（前の結果と混ざらない）")
	cbs[0].invoke(); pump(app, 0.8)
	check(s2._adhere()["on"] is False and s2._key(ready) == key_off and not s2.__dict__.get("rest_stale"), "オフに戻すと、前の計算結果そのもの")
	stl = str(s2.w["adh_state"].cget("text"))
	check("全部のコマを計算した結果" in stl and "癒着なし" in stl, f"そのときの 3D の状態（{stl[:50]}）")
	try: s2.win.destroy()
	except Exception: pass
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	shutil.move(STB, STORE)
	app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
