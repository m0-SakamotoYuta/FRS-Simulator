# -*- coding: utf-8 -*-
"""発表用の動画の区間の始め・終わりを手で選ぶ（2026-10-25）: 261007_AP_FE60_軸力100 の前方引張で、
ボタンと説明（はじめは「自動」）・今のコマを始めに → 「マニュアルで選択中（自動より ○ コマ（○ s）分、遅らせた）」・区間の始めへ が
そのコマ（2 秒前は付けない）・区間の説明も・今のコマを終わりに（早めた）・区間の終わりへ・最後のコマは ±200 N でなくそのコマの値・
始めより前を終わりにすると断る・後方引張は自動のまま（種類ごと）・段階2 の区間（clips）は自動のまま・くり返し再生も手で選んだ区間・
タブに保存（状態ファイルにも）・自動に戻す。接触の計算はダミー。1〜2分。"""
import importlib.util, os, sys, time, gc, json
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
def tacts(p):
	out = []
	props = p.renderer.GetViewProps(); props.InitTraversal()
	for _ in range(props.GetNumberOfItems()):
		a_ = props.GetNextProp()
		if isinstance(a_, vtk.vtkTextActor) and a_.GetVisibility():
			out.append(str(a_.GetInput()))
	return out
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
ti = next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_AP_FE60_軸力100")
app.on_ankle_tab_select(ti); pump(app, 0.5)
tab = app._ankle_tabs[ti]
tab.pop("clip_manual", None)
app.ankle_lig_mode.set("stage1"); app.ankle_start_at_load.set(False); app.ankle_clip_kind.set("anterior")
app.ankle_po_force_on.set(True)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]; v["set_paused"](True)
scn = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "pres_overlay" in o and "_t_shift" in o][-1]
po = scn["pres_overlay"]
ft = np.asarray(scn["frame_times"], float)
def now_fi():
	return int(np.searchsorted(ft, float(v["time"]()) + 1e-9, side="right") - 1)
def goto(fi):
	v["seek"](float(ft[int(fi)]) + 0.003); pump(app, 0.3)
cf = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
btns = [w for w in walk(cf) if w.winfo_class() == "TButton"]
bt = {}
for w in btns:
	bt.setdefault(str(w.cget("text")), []).append(w)
def lbl(prefix):
	x = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and str(w.cget("text")).startswith(prefix)]
	return x[0] if x else ""
def msg():
	x = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and ("区間: 映像の" in str(w.cget("text")) or "より" in str(w.cget("text")) and "コマにしてください" in str(w.cget("text")))]
	return x[0] if x else ""
check("今のコマを区間の始めに" in bt and "今のコマを区間の終わりに" in bt and len(bt.get("自動に戻す", [])) == 2,
      f"ボタン: 今のコマを区間の始めに・終わりに・自動に戻す ×2")
l0a, l0b = lbl("区間の始め："), lbl("区間の終わり：")
check(l0a.startswith("区間の始め：自動（") and l0b.startswith("区間の終わり：自動（"), f"はじめは自動（{l0a} ／ {l0b}）")
auto_rg = dict(po["clips"]())
bt["区間の始めへ"][0].invoke(); pump(app, 0.4); fa = now_fi()
bt["区間の終わりへ"][0].invoke(); pump(app, 0.4); fb = now_fi()
check(fa < auto_rg["anterior"][0] and fb == auto_rg["anterior"][1], f"自動: 始め {ft[fa]:.2f} 秒（前方引張の 2 秒前から）・終わり {ft[fb]:.2f} 秒")
# ---- 始めを遅らせる
f1 = int(np.searchsorted(ft, ft[fa] + 3.0))
goto(f1); f1 = now_fi()
bt["今のコマを区間の始めに"][0].invoke(); pump(app, 0.4)
la = lbl("区間の始め：")
d_ = f1 - fa
check("マニュアルで選択中" in la and f"{d_:,} コマ" in la and f"{ft[f1] - ft[fa]:.2f} s" in la and "遅らせた" in la,
      f"始めを手で選ぶ: 「{la}」")
check(abs(float(tab.get("clip_manual", {}).get("anterior", {}).get("a", -1)) - ft[f1]) < 1e-3, f"タブに保存（{tab.get('clip_manual')}）")
goto(fb)
bt["区間の始めへ"][0].invoke(); pump(app, 0.4)
check(now_fi() == f1, f"区間の始めへ → 手で選んだコマ（{ft[now_fi()]:.2f} 秒。2 秒前は付けない）")
check(f"区間: 映像の {ft[f1]:.1f}〜{ft[fb]:.1f} 秒" in msg(), f"区間の説明も（{msg()[:40]}）")
check(po["clips"]()["anterior"] == auto_rg["anterior"], f"靭帯の束（段階2）が使う区間は自動のまま（{po['clips']()['anterior']}）")
# ---- 終わりを早める
f2 = int(np.searchsorted(ft, ft[fb] - 1.0))
goto(f2); f2 = now_fi()
bt["今のコマを区間の終わりに"][0].invoke(); pump(app, 0.4)
lb_ = lbl("区間の終わり：")
check("マニュアルで選択中" in lb_ and f"{fb - f2:,} コマ" in lb_ and "早めた" in lb_, f"終わりを手で選ぶ: 「{lb_}」")
goto(f1)
bt["区間の終わりへ"][0].invoke(); pump(app, 0.4)
txt = " ".join(x for x in tacts(p) if "引張" in x or x.strip().endswith(" N"))
check(now_fi() == f2 and "前方引張中" in txt and "200 N" not in txt, f"区間の終わりへ → 手で選んだコマ（{ft[now_fi()]:.2f} 秒）・右上は ±200 N でなくそのコマの値（{txt[:60]}）")
# ---- 始めより前を終わりにすると断る
goto(f1 - 30)
bt["今のコマを区間の終わりに"][0].invoke(); pump(app, 0.4)
m_ = [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and "より後のコマにしてください" in str(w.cget("text"))]
check(bool(m_) and abs(float(tab["clip_manual"]["anterior"]["b"]) - ft[f2]) < 1e-3, f"始めより前を終わりにすると断る（{m_[0][:40] if m_ else '-'}）・保存はそのまま")
# ---- 種類ごと
rbs = {str(w.cget("text")): w for w in walk(cf) if w.winfo_class() == "TRadiobutton" and "引張" in str(w.cget("text"))}
next(w for t_, w in rbs.items() if t_.startswith("後方引張")).invoke(); pump(app, 0.4)
check(lbl("区間の始め：").startswith("区間の始め：自動") and lbl("区間の終わり：").startswith("区間の終わり：自動"), "後方引張は自動のまま（区間の種類ごと）")
next(w for t_, w in rbs.items() if t_.startswith("前方引張")).invoke(); pump(app, 0.4)
check("マニュアルで選択中" in lbl("区間の始め：") and "マニュアルで選択中" in lbl("区間の終わり："), "前方引張に戻すと手で選んだまま")
# ---- くり返し再生
cbl = [w for w in walk(cf) if w.winfo_class() == "TCheckbutton" and "くり返し再生" in str(w.cget("text"))]
if cbl:
	cbl[0].invoke(); pump(app, 0.4)
	t_now = float(v["time"]())
	v["seek"](float(ft[f2]) + 2.0); pump(app, 0.4)
	t_back = float(v["time"]())
	check(-0.05 < t_now - ft[f1] < 0.8 and t_back < ft[f2], f"くり返し再生も手で選んだ区間（始め {t_now:.2f} 秒・終わりを越えると {t_back:.2f} 秒へ）")
	cbl[0].invoke(); pump(app, 0.2)
	v["set_paused"](True)
else:
	check(False, "くり返し再生のチェックがある")
# ---- 状態ファイル
app._save_ankle_state(save_pose_caches=False)
fp = [f for f in os.listdir(HERE) if f.startswith("frs2015_gui_state_ankle") and f.endswith(".json")]
saved = False
for f in fp:
	try:
		d = json.load(open(os.path.join(HERE, f), encoding="utf-8"))
		saved |= any(isinstance(t, dict) and t.get("name") == tab["name"] and "anterior" in (t.get("clip_manual") or {}) for t in d.get("tabs", []))
	except Exception:
		pass
check(saved, "状態ファイルにも保存（開き直しても残る）")
# ---- 自動に戻す
for w in bt["自動に戻す"]:
	w.invoke(); pump(app, 0.3)
bt["区間の始めへ"][0].invoke(); pump(app, 0.4)
check(lbl("区間の始め：").startswith("区間の始め：自動") and lbl("区間の終わり：").startswith("区間の終わり：自動") and "clip_manual" not in tab
      and now_fi() == fa, f"自動に戻す（始め {ft[now_fi()]:.2f} 秒・タブから消える）")
cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
