# -*- coding: utf-8 -*-
"""まとめて解析の一覧: 「④ ArUco検出+PnP」と「接触の計算」を別の欄に（2026-10-14）。
接触は、計算に効く設定の指紋を index.json に残し、一覧ではいまの設定と比べる。
計算そのものは速いダミーに差し替える（一覧の表示の変わり方だけを見る）。"""
import importlib.util, os, sys, time, json
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
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showinfo = lambda *a, **k: None
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a)[:200])
app = frs.MainMenuGUI(); app.update()
app._ankle_show_batch_summary = lambda *a, **k: None

# 計算はダイミーに（各骨・各コマ 1.0mm）
def fake_multi(bd, update_progress=None, cancel_var=None, quiet=False):
	N = len(bd[0][3])
	return [{idx: np.full(m.n_points, 1.0, np.float32) for (idx, n, m, T) in bd} for _ in range(N)]
app._sim_view_precompute_multi_heatmap = fake_multi

idx_path = app._ankle_hm_index_path()
if idx_path.exists(): idx_path.unlink()
TAB = "261007_FE_軸力100_ACL切"

app._ankle_folder_filter = app._ANKLE_FOLDER_ALL      # 一覧は「すべて」で（本番の状態の表示フォルダに引きずられない）

def open_dialog():
	for w in app.winfo_children():
		if isinstance(w, tk.Toplevel) and "まとめて解析" in w.title():
			w.destroy()
	app._ankle_open_batch_dialog(); pump(app, 0.3)
	win = next(w for w in app.winfo_children() if isinstance(w, tk.Toplevel) and "まとめて解析" in w.title())
	return win

def labels(win):
	out = []
	def walk(wd):
		for c in wd.winfo_children():
			try:
				if c.winfo_class() == "TLabel":
					gi = c.grid_info()
					if gi:
						out.append((int(gi["row"]), int(gi["column"]), str(c.cget("text")) or str(c.tk.globalgetvar(str(c.cget("textvariable")))) if str(c.cget("textvariable")) else str(c.cget("text")), c))
			except Exception: pass
			walk(c)
	walk(win)
	return out

def row_of(win, name):
	L = labels(win)
	r = next(r for (r, c, t, w) in L if c == 1 and t == name)
	return {c: (t, w) for (rr, c, t, w) in L if rr == r}

win = open_dialog()
L = labels(win)
heads = [t for (r, c, t, w) in L if r == 0]
check("④ ArUco検出+PnP" in heads and "接触の計算" in heads, f"見出しに ④ と 接触の計算 の2つの欄: {heads}")
rw = row_of(win, TAB)
print(f"   {TAB}: ④={rw[6][0]!r} / 接触={rw[7][0]!r}", flush=True)
check(rw[6][0].startswith("済み") and "接触" not in rw[6][0], "④の欄は④だけ（済み・枚数・実寸）")
check("（" in rw[4][0] and any(k in rw[4][0] for k in ("中央値", "手動", "自動")), f"補正後・計算に使う の欄: {rw[4][0]!r}")
check(not rw[7][0].startswith("計算済み"), "接触の計算をしていないタブは「計算済み」にならない")
# 文字が切れていないか（ラベルに必要な幅 ≦ 実際の幅）
win.update_idletasks()
cut = [(t, w.winfo_reqwidth(), w.winfo_width()) for (r, c, t, w) in labels(win) if r > 0 and c in (4, 6, 7) and w.winfo_ismapped()]
f = tk.font.nametofont("TkDefaultFont") if hasattr(tk, "font") else None
import tkinter.font as tkf
fnt = tkf.nametofont("TkDefaultFont")
over = [t for (t, rq, wd) in cut if fnt.measure(t) > wd + 2]
check(not over, f"④・接触の欄の文字が切れていない（はみ出し: {over[:3]}）")
win.destroy()

# 接触だけを、まとめて解析で計算（ダミー）
tab = next(t for t in app._ankle_tabs if t["name"] == TAB)
res = app._ankle_run_batch([(tab, float(tab["snapshot"].get("ankle_marker_size_true_mm", 15.0)))], do_detect=False, do_contact=True)
print("   ", res, flush=True)
idx = app._ankle_hm_index()        # 2026-10-15 から タブごとに記録のリスト
check(TAB in idx and idx[TAB] and idx[TAB][0].get("fp"), "計算して保存すると index.json にタブと設定の指紋が残る")
win = open_dialog(); rw = row_of(win, TAB)
print(f"   計算のあと: 接触={rw[7][0]!r}", flush=True)
check(rw[7][0].startswith("計算済み"), "計算のあと、一覧の接触の欄が「計算済み」")
win.destroy()

# 設定を変える（このタブの②の補正後）→ 要計算
sn = tab["snapshot"]; old = sn.get("ankle_marker_size_mm")
sn["ankle_marker_size_mm"] = float(old) + 0.1
win = open_dialog(); rw = row_of(win, TAB)
check(rw[7][0].startswith("要計算"), f"②の補正後を変えると「要計算」: {rw[7][0]!r}")
win.destroy()
sn["ankle_marker_size_mm"] = old
# 骨リスト（共有）のキャリブを変える → 要計算、戻すと計算済み
gid = app._ankle_tab_group(tab); grp = app._ankle_bone_groups_dict()[gid]
# 開いているタブが同じ共有なら、画面の骨リスト（一覧を開くと共有へ書き戻される）の方を変える
_bl = app.ankle_bones if app._ankle_tab_shares_with_active(tab) else grp["bones"]
T0 = [list(r) for r in _bl[1]["marker_to_bone_T"]]
_bl[1]["marker_to_bone_T"] = [list(r) for r in T0]; _bl[1]["marker_to_bone_T"][0][3] = float(T0[0][3]) + 0.05
win = open_dialog(); rw = row_of(win, TAB)
check(rw[7][0].startswith("要計算"), f"共有の骨リストのキャリブを変えると「要計算」: {rw[7][0]!r}")
win.destroy()
_bl[1]["marker_to_bone_T"] = T0
win = open_dialog(); rw = row_of(win, TAB)
check(rw[7][0].startswith("計算済み"), f"元に戻すと「計算済み」: {rw[7][0]!r}")
# 「接触が未計算だけチェック」
btn = None
def walkb(wd):
	global btn
	for c in wd.winfo_children():
		try:
			if c.winfo_class() == "TButton" and str(c.cget("text")) == "接触が未計算だけチェック": btn = c
		except Exception: pass
		walkb(c)
walkb(win)
check(btn is not None, "「接触が未計算だけチェック」のボタン")
win.destroy()

# 保存済みを使ったとき（シミュレーション実行で読み込み）も記録される: index を消して、もう一度
idx_path.unlink()
app._ankle_run_batch([(tab, float(tab["snapshot"].get("ankle_marker_size_true_mm", 15.0)))], do_detect=False, do_contact=True)
idx = json.loads(idx_path.read_text(encoding="utf-8")) if idx_path.exists() else {}
check(TAB in idx, "保存済みの結果を使ったときも index.json に記録する（前からある結果も、一度開けば一覧に出る）")

print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
