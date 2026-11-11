# -*- coding: utf-8 -*-
"""区間のプリセット（名前・メモ）と「グラフは自動の区間の始めから描く」（2026-10-26）: 261007_AP_FE60_軸力100 の前方引張で、
欄（保存した区間・メモ・呼び出す／今の区間を保存／メモを保存／消す）・手で選んでいないと保存を断る・始めを 3 秒遅らせて名前とメモで保存
（タブに・状態ファイルにも）・自動に戻してから呼び出すと元の区間とメモ・メモだけ書き換え・成り立たない区間は断って元のまま・消す・
グラフ（png）は自動の区間の始めから（手で選んでいない png と同じ絵）・外すと手で選んだ始めから（絵が変わる）・
動画のグラフの縦の線は途中から動き出す・3D の動画の始めは手で選んだコマのまま。段階1の靭帯・接触の計算はダミー。2分。"""
import importlib.util, os, sys, time, gc, json, tempfile, shutil
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, cv2
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
def imread(p):
	return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)
D = os.path.join(tempfile.gettempdir(), f"区間の保存テスト_{os.getpid()}"); os.makedirs(D, exist_ok=True)
frs.messagebox.askyesno = lambda *a, **k: True
import tkinter.messagebox as _mb
_mb.askyesno = lambda *a, **k: True
from tkinter import filedialog
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
tab.pop("clip_manual", None); tab.pop("clip_presets", None)
app.ankle_lig_mode.set("stage1"); app.ankle_start_at_load.set(False); app.ankle_side_charts.set("lig"); app.ankle_clip_kind.set("anterior")
app.ankle_clip_chart_x.set("time"); app.ankle_clip_chart_hide.set(""); app.ankle_clip_sec.set(3.0); app.ankle_clip_hold.set(1.0)
app.ankle_clip_chart_auto0.set(True)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; ctrl = v["ctrl"]; v["set_paused"](True)
scn = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "pres_overlay" in o and "_t_shift" in o][-1]
ft = np.asarray(scn["frame_times"], float)
pv = ctrl["po_vars"]
cf = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
def now_fi():
	return int(np.searchsorted(ft, float(v["time"]()) + 1e-9, side="right") - 1)
def goto(fi):
	v["seek"](float(ft[int(fi)]) + 0.003); pump(app, 0.3)
def btn(t):
	return [w for w in walk(cf) if w.winfo_class() == "TButton" and str(w.cget("text")) == t]
def labels(sub):
	return [str(w.cget("text")) for w in walk(cf) if w.winfo_class() == "TLabel" and sub in str(w.cget("text"))]
def png(fp):
	filedialog.asksaveasfilename = lambda **k: fp
	pv["clip_chart_png"](); pump(app, 0.3)
	return imread(fp) if os.path.exists(fp) else None
try:
	cbx = [w for w in walk(cf) if w.winfo_class() == "TCombobox"]
	check(len(cbx) == 1 and all(len(btn(t)) == 1 for t in ("呼び出す", "今の区間を保存", "メモを保存", "消す")) and labels("保存した区間:"),
	      "欄: 保存した区間・メモ・呼び出す／今の区間を保存／メモを保存／消す")
	# 手で選んでいないときは断る
	btn("今の区間を保存")[0].invoke(); pump(app, 0.2)
	check(bool(labels("手で選んだ区間がありません")) and not tab.get("clip_presets"), "手で選んでいないと保存を断る")
	# 自動の区間の png（手で選んでいない）
	img_auto = png(os.path.join(D, "自動.png"))
	# 始めを 3 秒遅らせる
	btn("区間の始めへ")[0].invoke(); pump(app, 0.4); fa = now_fi()
	btn("区間の終わりへ")[0].invoke(); pump(app, 0.4); fb = now_fi()
	f1 = int(np.searchsorted(ft, ft[fa] + 3.0)); goto(f1); f1 = now_fi()
	btn("今のコマを区間の始めに")[0].invoke(); pump(app, 0.3)
	pv["clip_pr_name"].set("見やすさ用の始め"); pv["clip_pr_memo"].set("引っ張り始めの揺れを避ける。グラフは 0 N から")
	btn("今の区間を保存")[0].invoke(); pump(app, 0.3)
	L = (tab.get("clip_presets") or {}).get("anterior") or []
	check(len(L) == 1 and L[0]["name"] == "見やすさ用の始め" and abs(L[0]["a"] - ft[f1]) < 1e-3 and L[0]["b"] is None and "0 N から" in L[0]["memo"],
	      f"名前とメモで保存（{L[0] if L else '-'}）")
	app._save_ankle_state(save_pose_caches=False)
	d = json.load(open(os.path.join(HERE, "frs2015_gui_state_ankle_sim.json"), encoding="utf-8"))
	check(any(t.get("name") == tab["name"] and (t.get("clip_presets") or {}).get("anterior") for t in d.get("tabs", [])), "状態ファイルにも保存")
	# グラフ: 自動の区間の始めから（手で選んでいない png と同じ）
	img_on = png(os.path.join(D, "オン.png"))
	dif_on = float(np.abs(img_on.astype(int) - img_auto.astype(int)).mean()) if (img_on is not None and img_auto is not None and img_on.shape == img_auto.shape) else -1
	check(0 <= dif_on < 0.01, f"グラフは自動の区間の始めから（手で選んでいない png との差 {dif_on:.4f}）")
	cb0 = [w for w in walk(cf) if w.winfo_class() == "TCheckbutton" and "グラフは自動の区間の始め" in str(w.cget("text"))][0]
	cb0.invoke(); pump(app, 0.2)
	img_off = png(os.path.join(D, "オフ.png"))
	dif_off = float(np.abs(img_off.astype(int) - img_auto.astype(int)).mean()) if (img_off is not None and img_off.shape == img_auto.shape) else -1
	check(dif_off > 0.5 and app.ankle_clip_chart_auto0.get() is False, f"外すと手で選んだ始めから（差 {dif_off:.2f}）・設定を覚える")
	cb0.invoke(); pump(app, 0.2)
	# 動画のグラフの縦の線
	def line_xs(fp):
		filedialog.asksaveasfilename = lambda **k: fp
		pv["clip_chart_only"](); pump(app, 0.5)
		tmp = os.path.join(tempfile.gettempdir(), f"frs_t_{os.getpid()}.mp4"); shutil.copyfile(fp, tmp)
		cap = cv2.VideoCapture(tmp); xs = []
		while True:
			ok, fr = cap.read()
			if not ok: break
			m = (fr[:, :, 2] > 180) & (fr[:, :, 1] < 60) & (fr[:, :, 0] < 60)
			c_ = np.where(m.sum(axis=0) > 20)[0]
			xs.append(float(c_.mean()) if len(c_) else np.nan)
		cap.release(); os.remove(tmp)
		return xs
	xs_m = line_xs(os.path.join(D, "手.mp4"))
	# 自動の区間の幅（終わり−始め）のうち、手で遅らせた 3 秒の分だけ右から始まるはず
	frac = (ft[f1] - ft[fa]) / (ft[fb] - ft[fa])
	x_left = xs_m[-1] - (xs_m[-1] - xs_m[0]) / max(1e-9, 1.0 - frac)     # 自動の区間の始めの x（線形の目盛りから逆算）
	check(len(xs_m) == 120 and np.isfinite(xs_m[0]) and 0.05 < frac < 0.2 and abs(x_left - 76) < 15,
	      f"動画のグラフの縦の線は途中から（始め x {xs_m[0]:.0f}・終わり x {xs_m[-1]:.0f}。区間の {100 * frac:.1f} % の所＝目盛りから逆算した 0 の位置 x {x_left:.0f}・グラフの左端は x≈76）")
	btn("区間の始めへ")[0].invoke(); pump(app, 0.4)
	check(now_fi() == f1, f"3D の動画の始めは手で選んだコマのまま（{ft[now_fi()]:.2f} 秒）")
	# 自動に戻してから呼び出す
	for w in btn("自動に戻す"):
		w.invoke(); pump(app, 0.2)
	check("clip_manual" not in tab, "自動に戻した")
	pv["clip_pr_name"].set("見やすさ用の始め"); pv["clip_pr_memo"].set("")
	cbx[0].event_generate("<<ComboboxSelected>>"); pump(app, 0.2)
	check("0 N から" in pv["clip_pr_memo"].get(), f"選ぶとメモが出る（{pv['clip_pr_memo'].get()}）")
	btn("呼び出す")[0].invoke(); pump(app, 0.4)
	btn("区間の始めへ")[0].invoke(); pump(app, 0.4)
	check(now_fi() == f1 and any("マニュアルで選択中" in x for x in labels("区間の始め：")) and labels("の区間にしました"),
	      f"呼び出すと元の区間（{ft[now_fi()]:.2f} 秒）")
	# メモだけ書き換え
	pv["clip_pr_memo"].set("メモを直した"); btn("メモを保存")[0].invoke(); pump(app, 0.2)
	check(tab["clip_presets"]["anterior"][0]["memo"] == "メモを直した" and abs(tab["clip_presets"]["anterior"][0]["a"] - ft[f1]) < 1e-3, "メモだけ書き換え（区間はそのまま）")
	# 成り立たない区間は断って元のまま
	tab["clip_presets"]["anterior"].append({"name": "壊れた", "a": float(ft[fb] + 5.0), "b": None, "memo": "", "time": ""})
	pv["clip_pr_name"].set("壊れた"); btn("呼び出す")[0].invoke(); pump(app, 0.3)
	check(bool(labels("成り立ちません")) and abs(tab["clip_manual"]["anterior"]["a"] - ft[f1]) < 1e-3, "成り立たない区間は断って元のまま")
	# 消す
	btn("消す")[0].invoke(); pump(app, 0.2)
	check([e["name"] for e in tab["clip_presets"]["anterior"]] == ["見やすさ用の始め"], f"消す（残り {[e['name'] for e in tab['clip_presets']['anterior']]}）")
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	tab.pop("clip_manual", None); tab.pop("clip_presets", None)
	shutil.rmtree(D, ignore_errors=True)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
