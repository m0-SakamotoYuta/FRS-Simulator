# -*- coding: utf-8 -*-
"""靭帯の張り（初期長）（2026-10-23）: 段階2の窓の束ごとの「初期長 %」のスライダー・張り具合 [%]
（＝いまの長さ ÷（基準のコマの長さ × %）− 1。中心の線維・骨を避けて張った長さ。独立に求めた値と比べる）が
窓の「いまのコマの張り具合」・右のグラフの「靭帯の張り具合」・発表用の動画のグラフの縦軸「張り具合 [%]」にすぐ出る・
計算結果は消えない（3D のたるみは前の計算のまま・知らせ）・計算に渡す自然長の係数・鍵（入れていなければ前の鍵のまま）。
段階2の束（入っている結果）を使う。接触の計算はダミー。束の保存（cache/ankle_lig_bundles）はテストの後に元へ戻す。2〜3分。"""
import importlib.util, os, sys, time, gc, shutil, json
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk
import matplotlib
matplotlib.use("Agg")
import matplotlib.axes
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
REC = {"ylabel": None, "plot": []}
_yl0, _plot0 = matplotlib.axes.Axes.set_ylabel, matplotlib.axes.Axes.plot
def _ylabel(self, t, *a, **k):
	REC["ylabel"] = str(t); return _yl0(self, t, *a, **k)
def _plot(self, *a, **k):
	if len(a) >= 2:
		REC["plot"].append((np.asarray(a[0], float), np.asarray(a[1], float), k.get("label")))
	return _plot0(self, *a, **k)
matplotlib.axes.Axes.set_ylabel, matplotlib.axes.Axes.plot = _ylabel, _plot
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
	app.ankle_clip_size.set(900); app.ankle_clip_sec.set(3.0); app.ankle_clip_kind.set("anterior"); app.ankle_clip_hold.set(0.0)
	app.ankle_clip_chart_y.set("delta"); app.ankle_clip_chart_ylim.set(0.0); app.ankle_clip_chart_hide.set("")
	app.on_ankle_animate(); pump(app, 2.0)
	v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
	v["set_paused"](True)
	pv_ = ctrl["po_vars"]
	s2 = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
	ready = [b for b in s2.data["bundles"] if s2._ready(b) and s2.results.get(b.get("id")) is not None]
	check(len(ready) >= 2 and all(s2._pct_explicit(b) is None for b in ready), f"計算済みの束（{[b.get('name') for b in ready]}・初期長は未設定＝全体の値）")
	key_before = s2._key([b for b in s2.data["bundles"] if s2._ready(b)])
	# 独立に: 張り具合 = L(g) / (L(基準) × %) − 1
	gr = s2._rest_frame()
	def exp_strain(b, g, pct):
		r = s2.results[b["id"]]
		fr, cl = r["frames"].astype(float), r["center_len"].astype(float)
		return (np.interp(g, fr, cl) / (np.interp(gr if gr is not None else fr[0], fr, cl) * pct / 100.0) - 1.0) * 100.0
	ft = np.asarray(s2.scene["frame_times"], float)
	fi_go = int(0.45 * s2.n_view)
	v["seek"](float(ft[fi_go]) + 0.003); pump(app, 0.4)
	g_now = int(s2.s0 + s2.fi)
	check(gr is not None, f"基準のコマ（自然長の基準）: コマ {None if gr is None else gr - s2.s0}（{'' if gr is None else f'{ft[gr - s2.s0]:.1f} 秒'}）")
	# ---- 窓のスライダー
	s2.open_window(); pump(app, 0.6)
	rw = s2.w.get("rest_w") or {}
	check(s2.w.get("rest_box") is not None and set(rw) == {b["id"] for b in [b for b in s2.data["bundles"] if s2._ready(b)]}
	      and all(abs(float(vv.get()) - 100.0) < 1e-9 for (_b, vv, _l, _n) in rw.values()),
	      f"「張り（初期長）」の欄に束ごとのスライダー（{[b.get('name') for (b, *_x) in rw.values()]}・既定 100 %）")
	pump(app, 0.6)
	b0 = ready[0]
	_b, vv0, lab0, now0 = rw[b0["id"]]
	e100 = exp_strain(b0, g_now, 100.0)
	check(f"{e100:+.1f} %" in str(now0.cget("text")), f"いまのコマの張り具合（{str(now0.cget('text'))}・独立に {e100:+.2f} %）")
	ser = dict((lab, y) for _x, y, _c, lab in s2.series_strain())
	check(b0["name"] in ser and abs(ser[b0["name"]][s2.fi] - e100) < 1e-6, "右のグラフ用の張り具合の列 = 独立に求めた値")
	# 95 % に（スライダーを動かす）
	n_res = len(s2.results)
	vv0.set(95.0); s2._rest_slide(b0, vv0, lab0); pump(app, 0.6)
	e95 = exp_strain(b0, g_now, 95.0)
	ser = dict((lab, y) for _x, y, _c, lab in s2.series_strain())
	check(abs((s2._pct_explicit(b0) if s2._pct_explicit(b0) is not None else -1) - 95.0) < 1e-9 and "95.0 %" in str(lab0.cget("text")) and abs(ser[b0["name"]][s2.fi] - e95) < 1e-6
	      and f"{e95:+.1f} %" in str(now0.cget("text")),
	      f"95 % にすると、張り具合がすぐ変わる（{e100:+.2f} → {e95:+.2f} %・窓: {str(now0.cget('text'))}）")
	check(len(s2.results) == n_res, f"計算結果は消えない（3D の靭帯もグラフも残る: {len(s2.results)} 束）")
	fi2 = int(0.7 * s2.n_view)
	v["seek"](float(ft[fi2]) + 0.003); pump(app, 0.9)
	e2 = exp_strain(b0, int(s2.s0 + s2.fi), 95.0)
	check(f"{e2:+.1f} %" in str(now0.cget("text")), f"コマを動かすと、いまのコマの張り具合も書き直す（{str(now0.cget('text'))}・独立に {e2:+.2f} %）")
	v["seek"](float(ft[fi_go]) + 0.003); pump(app, 0.6)
	saved = json.load(open(STORE, encoding="utf-8"))
	sb0 = next((x for x in saved.get("bundles", []) if x.get("id") == b0["id"]), {})
	_sv = saved.get("params", {}).get("len_sets", {}).get(s2._len_set_id(), {}).get("pct", {}).get(b0["id"])
	check(_sv is not None and abs(float(_sv) - 95.0) < 1e-9, f"保存される（このタブの初期長のセットに: {_sv}）")
	others_ok = all(abs(dict((lab, y) for _x, y, _c, lab in s2.series_strain())[b["name"]][s2.fi] - exp_strain(b, g_now, 100.0)) < 1e-6 for b in ready[1:])
	check(others_ok, "ほかの束は 100 % のまま（束ごと）")
	stp = [w for w in walk(s2.w["rest_box"]) if w.winfo_class() == "TButton" and str(w.cget("text")) == "+0.5"]
	stp[0].invoke(); pump(app, 0.5)
	check(abs((s2._pct_explicit(b0) if s2._pct_explicit(b0) is not None else -1) - 95.5) < 1e-9, f"+0.5 のボタン（{b0.get('rest_pct')}）")
	# 計算に渡す係数・鍵
	rt = s2._runtime(b0)
	check(abs(rt.get("rest_scale", -1) - 0.955) < 1e-9, f"「▶ 計算する」では束ごとの係数で自然長（{rt.get('rest_scale')}）")
	# 計算そのもの（コマを両端の 2 つだけにして速く）: 自然長が束ごとの係数で決まる
	import ankle_lig_bundles as _alb
	b1 = ready[1]
	rts_ = [s2._runtime(b0), s2._runtime(b1)]
	pp_ = dict(s2.params(), rest_frame=s2._rest_frame(), stride=10 ** 7, iters=1, inner=1)
	r95 = _alb.simulate(s2.app, s2.bones_full, rts_, s2.s0, pp_, bones=s2._ensure_bones())
	s2._len_set().setdefault("pct", {}).pop(b0["id"], None)
	r100 = _alb.simulate(s2.app, s2.bones_full, [s2._runtime(b0), s2._runtime(b1)], s2.s0, pp_, bones=s2._ensure_bones())
	s2._len_set_put(b0, 95.5)
	q0 = np.asarray(r95[0]["L0"]) / np.asarray(r100[0]["L0"]); q1 = np.asarray(r95[1]["L0"]) / np.asarray(r100[1]["L0"])
	check(np.allclose(q0, 0.955, atol=1e-6) and np.allclose(q1, 1.0, atol=1e-6),
	      f"計算: {b0['name']} の自然長は 95.5 % 倍（{q0.min():.4f}〜{q0.max():.4f}）・{b1['name']} はそのまま（{q1.min():.4f}〜{q1.max():.4f}）")
	rdy = [b for b in s2.data["bundles"] if s2._ready(b)]
	check(s2._key(rdy) != key_before and s2._key(rdy, with_pct=False) == key_before, "鍵: 初期長を入れると変わる・入れない鍵は前と同じ（前の計算結果をそのまま使える）")
	s2._load_cached_results()
	check(len(s2.results) == n_res and s2.rest_stale is True, f"開き直しても、初期長だけ違う前の結果を使う（{len(s2.results)} 束・たるみは前の計算のまま）")
	s2._rest_rows(); pump(app, 0.3)
	warn = [str(w.cget("text")) for w in walk(s2.w["rest_box"]) if w.winfo_class() == "TLabel" and "初期長を変える前の計算" in str(w.cget("text"))]
	check(len(warn) == 1, "そのときは「3D のたるみは前の計算」と知らせる")
	# ---- 右のグラフの項目
	av = [k for k, _l in frs.MainMenuGUI._ANKLE_SIDE_CHART_KEYS]
	ss, ylab = s2.scene["_chart_series"]("lig_strain")
	check("lig_strain" in av and ylab == "Strain from rest (%)" and len(ss) == len(ready), f"右のグラフに「靭帯の張り具合」（{ylab}・{len(ss)} 本）")
	# ---- 発表用の動画のグラフ: 縦軸「張り具合 [%]」
	cf = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
	rb = [w for w in walk(cf) if w.winfo_class() == "TRadiobutton" and str(w.cget("text")) == "張り具合 [%]"]
	check(len(rb) == 1, "発表用の動画のグラフの縦軸に「張り具合 [%]」")
	rb[0].invoke(); pump(app, 0.3)
	REC.update(ylabel=None, plot=[])
	pv_["clip_chart_preview"](); pump(app, 0.5)
	got = {lab: (x, y) for x, y, lab in REC["plot"]}
	x_, y_ = got.get(b0["name"], (None, None))
	ok_ = False
	if y_ is not None:
		btn = {str(w.cget("text")): w for w in walk(cf) if w.winfo_class() == "TButton"}
		btn["区間の始めへ"].invoke(); pump(app, 0.3)
		fa = int(np.searchsorted(ft, float(v["time"]()) + 1e-9, side="right") - 1)
		ok_ = abs(float(y_[0]) - exp_strain(b0, s2.s0 + fa, 95.5)) < 1e-6
	check(REC["ylabel"] == "張り具合（初期長からの伸び）[%]" and ok_ and app.ankle_clip_chart_y.get() == "strain",
	      f"描いた値 = 張り具合（区間の始め {float(y_[0]) if y_ is not None else '-':}）・縦軸の名前・覚える")
	rbd = [w for w in walk(cf) if w.winfo_class() == "TRadiobutton" and "ΔL" in str(w.cget("text"))][0]
	rbd.invoke(); pump(app, 0.3)
	try: pv_["clip_chart_prev_win"]["w"].destroy()
	except Exception: pass
	try: s2.win.destroy()
	except Exception: pass
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	shutil.move(STB, STORE)
	app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
