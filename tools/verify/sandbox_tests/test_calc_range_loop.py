# -*- coding: utf-8 -*-
"""区間だけの靭帯の計算・区間のくり返し再生（2026-10-23）:
A. 段階2の「計算する範囲: 前方引張の区間だけ」= 発表用の動画の区間（clips）の前後 2 秒・計算するコマはその中（と自然長の基準のコマ）だけ・
   かかった時間・区間の外では線維・皮を隠しグラフも NaN・「全部のコマ」へ戻すと前の全部の結果にすぐ切り替わる（鍵は別）。
B. 発表用の動画の欄「大きい窓のまま、この区間をくり返し再生する」: 窓の大きさはそのまま・区間の終わりを越えると始めへ・区間を変えると追う・外すとやめる。
段階2の束（入っている結果）を使う。接触の計算はダミー。束の保存はテストの後に元へ戻す。本番の最新の計算結果と束のデータを写してから回す。3〜5分。"""
import importlib.util, os, sys, time, gc, shutil
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
	app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(False); app.ankle_clip_kind.set("anterior")
	app.on_ankle_animate(); pump(app, 2.0)
	v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
	v["set_paused"](True)
	pv_ = ctrl["po_vars"]
	s2 = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
	s2.data.setdefault("params", {})["view"] = "fibers"
	s2.data["params"]["calc_range"] = "all"
	for b in s2.data["bundles"]:
		b["visible"] = True
	s2._cf_memo = None; s2._save(); s2._load_cached_results(); s2._rebuild_actors()
	ready = [b for b in s2.data["bundles"] if s2._ready(b)]
	full_ok = all(s2.results.get(b["id"]) is not None for b in ready)
	check(full_ok, f"全部のコマの計算結果がある（{len(s2.results)} 束）")
	key_all = s2._key(ready)
	ft = np.asarray(s2.scene["frame_times"], float)
	# ---- A. 区間だけ計算
	s2.open_window(); pump(app, 0.6)
	rbs = {str(w.cget("text")): w for w in walk(s2.win) if w.winfo_class() == "TRadiobutton"}
	check(all(k in rbs for k in ("全部のコマ", "前方引張の区間だけ", "後方引張の区間だけ")), "「計算する範囲」: 全部のコマ／前方引張の区間だけ／後方引張の区間だけ")
	rbs["前方引張の区間だけ"].invoke(); pump(app, 0.5)
	rng = s2._calc_frames()
	cl = s2.scene["pres_overlay"]["clips"]()["anterior"]
	ea = int(np.searchsorted(ft[:s2.n_view], ft[cl[0]] - 2.0)); eb = int(min(s2.n_view - 1, np.searchsorted(ft[:s2.n_view], ft[cl[1]] + 2.0)))
	check(rng == (s2.s0 + ea, s2.s0 + eb), f"区間 = 前方引張の区間（発表用の動画と同じ）の前後 2 秒（{rng}・映像の {ft[ea]:.1f}〜{ft[eb]:.1f} 秒）")
	lb = str(s2.w["crange_lbl"].cget("text"))
	check("全体の" in lb and "%" in lb, f"欄に区間の説明（{lb[:60]}）")
	check(s2._key(ready) != key_all and not any(s2.results.get(b["id"]) is not None for b in ready), "区間の結果は鍵が別（まだ計算していない＝結果なし）")
	t0 = time.time()
	ok_c = s2.compute(parent=s2.win)
	el = time.time() - t0
	r0 = s2.results.get(ready[0]["id"])
	fr_ = r0["frames"] if r0 is not None else np.zeros(0)
	gr = s2._rest_frame()
	inside = [int(x) for x in fr_ if rng[0] <= int(x) <= rng[1]]
	outside = [int(x) for x in fr_ if not (rng[0] <= int(x) <= rng[1])]
	check(ok_c and len(inside) >= (rng[1] - rng[0]) // 3 and all(x == gr for x in outside),
	      f"区間だけ計算した（{el:.1f} 秒・{len(fr_)} コマ。区間の外は自然長の基準のコマ {outside} だけ）")
	n_all = len(range(s2.s0, s2.s0 + s2.n_view, 3))
	print(f"   [参考] 全部なら約 {n_all} コマ → 区間は {len(fr_)} コマ（{100.0 * len(fr_) / n_all:.0f} %）", flush=True)
	# 区間の外では隠す・グラフは NaN
	a0 = s2.actors[ready[0]["id"]]
	fin = int(0.5 * (ea + eb))
	v["seek"](float(ft[fin]) + 0.003); pump(app, 0.4)
	vis_in = bool(a0["fib_act"].GetVisibility())
	fout = max(0, ea - 300)
	v["seek"](float(ft[fout]) + 0.003); pump(app, 0.4)
	vis_out = bool(a0["fib_act"].GetVisibility())
	v["seek"](float(ft[fin]) + 0.003); pump(app, 0.4)
	vis_back = bool(a0["fib_act"].GetVisibility())
	check(vis_in and not vis_out and vis_back, f"区間の中は線維を出し、外では隠し、戻ると出る（{vis_in}・{vis_out}・{vis_back}）")
	ser = s2.series()
	y = ser[0][1]
	check(np.isnan(y[fout]) and np.isfinite(y[fin]), "グラフの線は区間の中だけ（外は出さない）")
	check(s2.strain_at(ready[0], s2.s0 + fout) is None and s2.strain_at(ready[0], s2.s0 + fin) is not None, "張り具合も区間の中だけ")
	# 全部へ戻す → 前の結果にすぐ
	rbs["全部のコマ"].invoke(); pump(app, 0.5)
	check(s2._calc_frames() is None and all(s2.results.get(b["id"]) is not None for b in ready) and s2._key(ready) == key_all,
	      "「全部のコマ」に戻すと、前の全部の計算結果にすぐ切り替わる（計算し直し不要）")
	rbs["前方引張の区間だけ"].invoke(); pump(app, 0.5)
	check(all(s2.results.get(b["id"]) is not None for b in ready), "また「前方引張の区間だけ」にすると、さっき計算した区間の結果にすぐ切り替わる")
	rbs["全部のコマ"].invoke(); pump(app, 0.3)
	try: s2.win.destroy()
	except Exception: pass
	# ---- B. 大きい窓のまま区間をくり返し再生
	cf = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
	cbl = [w for w in walk(cf) if w.winfo_class() == "TCheckbutton" and "この区間（上の「区間」）をくり返し再生" in str(w.cget("text"))]
	check(len(cbl) == 1, "「大きい窓のまま、この区間をくり返し再生する」のチェック")
	size0 = tuple(p.window_size)
	pl_a = ctrl["po_vars"]  # noqa
	cbl[0].invoke(); pump(app, 0.5)
	tA0 = float(ft[s2.scene["pres_overlay"]["clips"]()["anterior"][0]])
	t_now = float(v["time"]())
	check(abs(t_now - (tA0 - 2.0)) < 1.0 and tuple(p.window_size) == size0, f"付けると前方引張の区間の始めへ（{t_now:.1f} 秒・区間の始め {tA0 - 2.0:.1f} 秒）・窓の大きさはそのまま {size0}")
	tB = float(ft[s2.scene["pres_overlay"]["clips"]()["anterior"][1]])
	v["seek"](tB + 3.0); pump(app, 0.4)
	t_now = float(v["time"]())
	check(t_now < tB, f"区間の終わりを越えると、始めへ戻る（終わり {tB:.1f} 秒の 3 秒あとへ動かした → {t_now:.1f} 秒）")
	rb_post = [w for w in walk(cf) if w.winfo_class() == "TRadiobutton" and str(w.cget("text")).startswith("後方引張")]
	rb_post[0].invoke(); pump(app, 0.5)
	tP0 = float(ft[s2.scene["pres_overlay"]["clips"]()["posterior"][0]])
	t_now = float(v["time"]())
	check(abs(t_now - tP0) < 1.5, f"区間を後方引張に変えると、くり返す区間も（{t_now:.1f} 秒・後方引張の始め {tP0:.1f} 秒）")
	cbl[0].invoke(); pump(app, 0.3)
	v["seek"](tB + 3.0); pump(app, 0.4)
	t_now = float(v["time"]())
	check(t_now >= tB, f"外すと、ふつうの再生（区間の外へ動かしてもそのまま: {t_now:.1f} 秒）")
	v["set_paused"](True)
	rb_ant = [w for w in walk(cf) if w.winfo_class() == "TRadiobutton" and str(w.cget("text")).startswith("前方引張")]
	rb_ant[0].invoke(); pump(app, 0.2)
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	shutil.move(STB, STORE)
	app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
