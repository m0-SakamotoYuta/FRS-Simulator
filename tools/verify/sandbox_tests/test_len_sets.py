# -*- coding: utf-8 -*-
"""初期長のセット（2026-10-24）: 付着部は全タブ共通のまま、初期長 % だけをタブごとに選ぶ「初期長のセット」に。
それまでの初期長（束ごと・全タブ共通）から「前後方用」を作って既定に（計算結果の鍵は変わらない）・窓の欄（このタブのセットの名前・使っているタブ・メモ）・
新しく作る（いまの値を写す・このタブだけ）→ スライダーで変えてもほかのセットは変わらない・ほかのタブは既定のセット・選び直す・名前を変える・削除。
セットを入れる前の束のデータ（backup/lig_bundles_store/g8a86693d_20261024_202701_before_len_sets.json）と計算結果・一覧を写してから回す。2〜3分。"""
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
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)
STORE = os.path.join(HERE, "cache", "ankle_lig_bundles", "g8a86693d.json")
STB = STORE + ".testbak"
shutil.copy2(STORE, STB)
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a[:2])[:200])
import tkinter.simpledialog as _sd, tkinter.messagebox as _mb
_mb.askyesno = lambda *a, **k: True
ANS = {"v": "屈曲伸展用"}
_sd.askstring = lambda *a, **k: ANS["v"]
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
TAB, OTHER = "261007_AP_FE60_軸力100", "261007_FE_軸力100"
try:
	before = json.load(open(STORE, encoding="utf-8"))
	old_pct = {b["id"]: b.get("rest_pct") for b in before["bundles"]}
	check("len_sets" not in before.get("params", {}) and any(v is not None for v in old_pct.values()),
	      f"写した束のデータは前の形（初期長が束に入っている: {[(b['name'], b.get('rest_pct')) for b in before['bundles']]}）")
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
	app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(False)
	app.on_ankle_animate(); pump(app, 2.0)
	v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
	v["set_paused"](True)
	s2 = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
	ready = [b for b in s2.data["bundles"] if s2._ready(b)]
	byname = {b["name"]: b for b in ready}
	sets = s2._len_sets()
	sid1 = s2._len_set_id()
	check(len(sets) == 1 and sets[sid1]["name"] == "前後方用" and all(abs(s2._rest_pct(b) - float(old_pct[b["id"]] or 100.0)) < 1e-9 for b in ready),
	      f"前の初期長から「前後方用」を作って既定に（{[(b['name'], s2._rest_pct(b)) for b in ready]}）")
	saved = json.load(open(STORE, encoding="utf-8"))
	check(saved["params"].get("len_set_default") == sid1 and saved["params"]["len_sets"][sid1]["name"] == "前後方用", "保存される")
	check(s2._tab_name() == TAB, f"タブの名前で覚える（{s2._tab_name()}）")
	# 計算結果の鍵は変わらない（AM 91・PL 100・PCL 未設定 → 本番の lb_ff59…）
	keep = dict(sets[sid1]["pct"])
	sets[sid1]["pct"] = {byname["AM束"]["id"]: 91.0, byname["PL束"]["id"]: 100.0}
	k91 = s2._key(ready)
	sets[sid1]["pct"] = keep
	check(k91 == "ff59d7866047b596fd112c4d", f"計算結果の鍵は前と同じ（{k91}）")
	# ---- 窓
	s2._load_cached_results(); s2._rebuild_actors()
	s2.open_window(); pump(app, 0.6)
	title = str(s2.w["rest_box"].cget("text"))
	cb = s2.w["len_set_cb"]
	lbls = [str(w.cget("text")) for w in walk(s2.w["rest_box"]) if w.winfo_class() == "TLabel"]
	check("前後方用" in title and cb.get() == "前後方用" and any(TAB in t for t in lbls),
	      f"欄の見出しにこのタブのセットの名前・タブの名前（{title[:40]}…）")
	check(any("選んでいないタブ（既定）" in t for t in lbls), "使っているタブ（既定なので「選んでいないタブ（既定）」）")
	tx = s2.w["len_set_memo"]
	check("それまでの初期長" in tx.get("1.0", "end-1c"), f"メモ（{tx.get('1.0', 'end-1c')[:30]}）")
	tx.delete("1.0", "end"); tx.insert("1.0", "前後方の試験で合わせた値（AM 96）")
	tx.event_generate("<FocusOut>"); pump(app, 0.3)
	saved = json.load(open(STORE, encoding="utf-8"))
	check(saved["params"]["len_sets"][sid1]["memo"] == "前後方の試験で合わせた値（AM 96）", "メモを書くと保存される")
	# ---- 新しく作る
	btn = {str(w.cget("text")): w for w in walk(s2.w["rest_box"]) if w.winfo_class() == "TButton"}
	btn["新しく作る…"].invoke(); pump(app, 0.6)
	sets = s2._len_sets()
	sid2 = s2._len_set_id()
	check(len(sets) == 2 and sid2 != sid1 and sets[sid2]["name"] == "屈曲伸展用" and sets[sid2]["pct"] == sets[sid1]["pct"]
	      and s2.data["params"]["len_set_of"].get(f"tab:{TAB}") == sid2,
	      f"「新しく作る…」: いまの値を写した「屈曲伸展用」を作り、このタブで使う（{sets[sid2]['pct']}）")
	check("屈曲伸展用" in str(s2.w["rest_box"].cget("text")) and s2.w["len_set_cb"].get() == "屈曲伸展用", "欄の見出しも「屈曲伸展用」に")
	b0 = byname["AM束"]
	_b, vv0, lab0, now0 = s2.w["rest_w"][b0["id"]]
	vv0.set(80.0); s2._rest_slide(b0, vv0, lab0); pump(app, 0.5)
	sets = s2._len_sets()
	check(abs(sets[sid2]["pct"][b0["id"]] - 80.0) < 1e-9 and abs(sets[sid1]["pct"][b0["id"]] - 96.0) < 1e-9 and abs(s2._rest_pct(b0) - 80.0) < 1e-9,
	      f"スライダーで AM を 80 % に: 「屈曲伸展用」だけ変わり、「前後方用」は 96 % のまま")
	check(b0.get("rest_pct") == old_pct[b0["id"]], f"束そのもの（付着部などの共通の定義）は変えない（{b0.get('rest_pct')}）")
	# ---- ほかのタブは既定のセット
	po = s2.scene["pres_overlay"]
	po["tab_name"] = OTHER
	check(s2._len_set_id() == sid1 and abs(s2._rest_pct(b0) - 96.0) < 1e-9, f"ほかのタブ（{OTHER}）は既定の「前後方用」（AM {s2._rest_pct(b0)} %）")
	po["tab_name"] = TAB
	# ---- 選び直す
	s2.w["len_set_cb"].set("前後方用"); s2._len_set_choose("前後方用"); pump(app, 0.5)
	check(s2._len_set_id() == sid1 and abs(s2._rest_pct(b0) - 96.0) < 1e-9 and len(s2.results) == len(ready),
	      f"セットを選び直すと、その初期長に（AM {s2._rest_pct(b0)} %）・計算結果も探し直す（{len(s2.results)} 束）")
	s2._len_set_choose("屈曲伸展用"); pump(app, 0.5)
	ANS["v"] = "屈曲伸展用（試し）"
	btn = {str(w.cget("text")): w for w in walk(s2.w["rest_box"]) if w.winfo_class() == "TButton"}
	btn["名前を変える…"].invoke(); pump(app, 0.4)
	check(s2._len_set()["name"] == "屈曲伸展用（試し）", "名前を変える")
	btn = {str(w.cget("text")): w for w in walk(s2.w["rest_box"]) if w.winfo_class() == "TButton"}
	btn["削除"].invoke(); pump(app, 0.5)
	check(len(s2._len_sets()) == 1 and s2._len_set_id() == sid1 and f"tab:{TAB}" not in s2.data["params"].get("len_set_of", {}),
	      "削除すると、このタブは既定の「前後方用」に戻る")
	btn = {str(w.cget("text")): w for w in walk(s2.w["rest_box"]) if w.winfo_class() == "TButton"}
	btn["削除"].invoke(); pump(app, 0.3)
	check(len(s2._len_sets()) == 1, "最後の 1 つは削除できない")
	try: s2.win.destroy()
	except Exception: pass
	cw = ctrl["window"]; cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
finally:
	shutil.move(STB, STORE)
	app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
