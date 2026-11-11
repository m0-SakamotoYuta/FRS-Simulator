# -*- coding: utf-8 -*-
"""屈曲伸展の試験の区間（2026-10-25）: 261007_FE_軸力100 で、区間（伸展＝動き始め〜最大伸展位・屈曲＝最大伸展位〜屈曲 90° に戻った所）が
datalog の FE から独立に求めたものと同じ・前方／後方引張の区間は出さない・右上の文字は「伸展中：○°」（赤）／「屈曲中：○°」（青。○°＝−FE）・
発表用の動画の区間は 4 つ・伸展を選ぶと動画の計画のコマと文字・大きい窓のまま伸展をくり返し再生・靭帯の束の「計算する範囲」に伸展・屈曲。
前後方の試験（AP_FE60_軸力100）では今までどおり前方／後方引張だけ。接触の計算はダミー。3分。"""
import importlib.util, os, sys, time, gc, shutil
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, vtk
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
FE_TAB, AP_TAB = "261007_FE_軸力100", "261007_AP_FE60_軸力100"
def sel(name):
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == name)); pump(app, 0.5)
def tacts(p):
	out = []
	props = p.renderer.GetViewProps(); props.InitTraversal()
	for _ in range(props.GetNumberOfItems()):
		a_ = props.GetNextProp()
		if isinstance(a_, vtk.vtkTextActor) and a_.GetVisibility():
			out.append(a_)
	return out
try:
	sel(FE_TAB)
	app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(False); app.ankle_clip_kind.set("anterior")
	app.on_ankle_animate(); pump(app, 2.0)
	v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
	v["set_paused"](True)
	scn = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "pres_overlay" in o and "_t_shift" in o][-1]
	po = scn["pres_overlay"]
	ft = np.asarray(scn["frame_times"], float)
	# 独立に: datalog の FE をコマの時刻へ
	log = app._ankle_robot_log_load(str(app.ankle_robot_log_path.get()))
	off = scn["_log_offset"] if scn.get("_log_offset") is not None else float(app.ankle_robot_log_offset_s.get())
	tx = np.asarray(log["t"], float) + off - float(scn["_t_shift"])
	fe = np.interp(ft, tx, np.asarray(log["cols"]["FE"], float))
	k = int(np.argmax(fe)); top = fe[k]
	a0 = int(np.where(np.abs(fe[:k + 1] - fe[0]) > 1.0)[0][0]); kk = int(np.where(fe[:k + 1] >= top - 0.5)[0][0])
	b0 = kk + int(np.where(fe[kk:] <= fe[-1] + 1.0)[0][0])
	cl = po["clips"]()
	check(po["fe_mode"]() and set(cl) == {"extension", "flexion"}, f"屈曲伸展の試験とわかり、区間は伸展・屈曲だけ（{sorted(cl)}・FE {fe.min():.0f}〜{fe.max():.0f}）")
	check(cl.get("extension") == (a0, kk) and cl.get("flexion") == (kk, b0),
	      f"区間 = FE から独立に求めたもの（伸展 {ft[a0]:.1f}〜{ft[kk]:.1f} 秒・屈曲 {ft[kk]:.1f}〜{ft[b0]:.1f} 秒・最大伸展位 {-top:.0f}° 屈曲）")
	# 右上の文字
	def stage_text():
		la = [x for x in tacts(p) if "伸展中" in str(x.GetInput()) or "屈曲中" in str(x.GetInput())]
		na = [x for x in tacts(p) if str(x.GetInput()).endswith("°") and x not in la]
		return (str(la[0].GetInput()) + (str(na[0].GetInput()) if na else "")) if la else "", (la[0].GetTextProperty().GetColor() if la else None)
	for f_, nm_, colx in (((a0 + kk) // 2, "伸展中", (0.85, 0.0, 0.0)), ((kk + b0) // 2, "屈曲中", (0.0, 0.25, 0.9))):
		v["seek"](float(ft[f_]) + 0.003); pump(app, 0.3)
		fi_ = int(np.searchsorted(ft, float(v["time"]()) + 1e-9, side="right") - 1)
		st_, col_ = stage_text()
		exp_ = f"{nm_}：{-fe[fi_]:.0f}°"
		check(st_ == exp_ and col_ is not None and np.allclose(col_, colx, atol=0.01), f"右上の文字「{exp_}」（表示「{st_}」）")
	# 発表用の動画の区間
	cf = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
	rbs = {str(w.cget("text")): w for w in walk(cf) if w.winfo_class() == "TRadiobutton" and not str(w.cget("text")).startswith("前方変位") and ("引張" in str(w.cget("text")) or "伸展" in str(w.cget("text")) or "屈曲" in str(w.cget("text")))}
	check(len(rbs) == 4 and any(t_.startswith("伸展") for t_ in rbs) and any(t_.startswith("屈曲") for t_ in rbs), f"区間は 4 つ（{list(rbs)}）")
	rb_ext = next(w for t_, w in rbs.items() if t_.startswith("伸展"))
	rb_ext.invoke(); pump(app, 0.5)
	info = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and "区間: 映像の" in str(w.cget("text"))]
	check(info and f"{ft[a0]:.1f}〜{ft[kk]:.1f} 秒" in info[0] and app.ankle_clip_kind.get() == "extension", f"伸展を選ぶと区間の説明（{info[0][:40] if info else '-'}）・覚える")
	pv_ = ctrl["po_vars"]
	pv_["clip_nums_preview"](); pump(app, 0.5)
	nw = pv_["clip_nums_prev_win"]["w"]
	im0 = nw.g["frame"](0); im1 = nw.g["frame"](nw.g["n"] - 1)
	red = lambda im: int(((im[:, :, 0] > 150) & (im[:, :, 1] < 80)).sum())
	check(red(im0) > 50 and red(im1) > 50, f"数字の動画は始めも終わりも赤（伸展中）: {red(im0)}・{red(im1)} 画素")
	try: nw.destroy()
	except Exception: pass
	# くり返し再生
	cbl = [w for w in walk(cf) if w.winfo_class() == "TCheckbutton" and "この区間（上の「区間」）をくり返し再生" in str(w.cget("text"))][0]
	cbl.invoke(); pump(app, 0.4)
	t_now = float(v["time"]())
	v["seek"](float(ft[kk]) + 3.0); pump(app, 0.4)
	t_back = float(v["time"]())
	msg = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and "くり返し再生しています" in str(w.cget("text"))]
	check(abs(t_now - ft[a0]) < 1.0 and t_back < ft[kk] and msg and "伸展の区間" in msg[0], f"伸展の区間をくり返し再生（始め {t_now:.1f} 秒・終わりを越えると {t_back:.1f} 秒へ・{msg[0][:30] if msg else '-'}）")
	cbl.invoke(); pump(app, 0.2)
	v["set_paused"](True)
	# 靭帯の束の「計算する範囲」
	s2 = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p]
	if s2:
		s2 = s2[-1]
		s2.open_window(); pump(app, 0.5)
		crb = {str(w.cget("text")): w for w in walk(s2.win) if w.winfo_class() == "TRadiobutton" and "区間だけ" in str(w.cget("text"))}
		check("伸展の区間だけ" in crb and "屈曲の区間だけ" in crb, f"靭帯の束の「計算する範囲」に伸展・屈曲（{list(crb)}）")
		crb["伸展の区間だけ"].invoke(); pump(app, 0.4)
		rng = s2._calc_frames()
		ea = int(np.searchsorted(ft[:s2.n_view], ft[a0] - 2.0)); eb = int(min(s2.n_view - 1, np.searchsorted(ft[:s2.n_view], ft[kk] + 2.0)))
		check(rng == (s2.s0 + ea, s2.s0 + eb), f"伸展の区間だけ = 伸展の区間の前後 2 秒（{rng}）")
		crb_all = [w for w in walk(s2.win) if w.winfo_class() == "TRadiobutton" and str(w.cget("text")) == "全部のコマ"][0]
		crb_all.invoke(); pump(app, 0.3)
		try: s2.win.destroy()
		except Exception: pass
	else:
		check(False, "段階2の靭帯の束がある")
	next(w for t_, w in rbs.items() if t_.startswith("前方引張")).invoke(); pump(app, 0.2)
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
	# ---- 前後方の試験は今までどおり
	sel(AP_TAB)
	app.on_ankle_animate(); pump(app, 2.0)
	v2 = app._sim_viewers_alive()[-1]
	scn2 = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "pres_overlay" in o and "_t_shift" in o][-1]
	cl2 = scn2["pres_overlay"]["clips"]()
	check(not scn2["pres_overlay"]["fe_mode"]() and set(cl2) == {"anterior", "posterior"}, f"前後方の試験は今までどおり前方・後方引張だけ（{sorted(cl2)}）")
	cw2 = v2["ctrl"]["window"]; cw2.tk.call(cw2.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	shutil.move(STB, STORE)
	app.ankle_lig_mode.set("stage1"); app.ankle_clip_kind.set("anterior")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
