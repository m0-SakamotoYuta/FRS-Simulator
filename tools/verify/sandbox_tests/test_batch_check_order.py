# -*- coding: utf-8 -*-
"""まとめて解析: チェックを入れた順に解析する・順番の数字を出す（2026-10-15）。解析そのものは呼ばない（順番だけ見る）。"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import tkinter as tk
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
asks = []
frs.messagebox.askyesno = lambda *a, **k: (asks.append(a), True)[1]
frs.messagebox.showinfo = lambda *a, **k: None
frs.messagebox.showwarning = lambda *a, **k: None
app = frs.MainMenuGUI(); app.update()
app._ankle_folder_filter = app._ANKLE_FOLDER_ALL
got = []
app._ankle_run_batch = lambda jobs, **k: (got.extend(str(t.get("name")) for t, _v in jobs), [])[1]
app._ankle_show_batch_summary = lambda *a, **k: None
def walk(w):
	for c in w.winfo_children():
		yield c
		yield from walk(c)
before = set(app.winfo_children())
app._ankle_open_batch_dialog(); pump(app, 0.5)
win = [w for w in app.winfo_children() if w not in before and isinstance(w, tk.Toplevel)][0]
# 行のチェック（やることの2つを除く）とタブ名
cbs = [w for w in walk(win) if w.winfo_class() == "TCheckbutton" and int(w.grid_info().get("column", -1) or -1) == 0] if False else None
rows = {}
for w in walk(win):
	if w.winfo_class() == "TLabel" and w.grid_info() and int(w.grid_info()["column"]) == 1:
		r = int(w.grid_info()["row"])
		if r > 0:
			rows[r] = [str(w.cget("text")), None]
for w in walk(win):
	if w.winfo_class() == "TCheckbutton" and w.grid_info() and int(w.grid_info()["column"]) == 0 and int(w.grid_info()["row"]) in rows:
		rows[int(w.grid_info()["row"])][1] = w
names = ["261007_FE_軸力100_ACL切", "261007_FE_軸力無し", "261007_AP_FE60_軸力100"]
byname = {n: cb for (n, cb) in rows.values()}
for n in names:
	byname[n].invoke(); pump(app, 0.05)
nums = [str(byname[n].cget("text")) for n in names]
check(nums == ["1", "2", "3"], f"チェックした順に 1・2・3 と出る: {nums}")
# 2番目を外すと詰まる
byname[names[1]].invoke(); pump(app, 0.05)
check([str(byname[n].cget("text")) for n in (names[0], names[2])] == ["1", "2"] and str(byname[names[1]].cget("text")) == "", "外すと番号が詰まる")
byname[names[1]].invoke(); pump(app, 0.05)
check(str(byname[names[1]].cget("text")) == "3", "もう一度入れると最後（3）")
btn = [w for w in walk(win) if w.winfo_class() == "Button" and "チェックしたタブを解析" in str(w.cget("text"))][0]
btn.invoke(); pump(app, 0.3)
exp = [names[0], names[2], names[1]]
check(got == exp, f"チェックを入れた順に解析する: {got}")
m = str(asks[-1][1]) if asks else ""
print("   確認の文:", m.replace(chr(10), " / "), flush=True)
exp_eff = {app._ankle_tab_eff_size(next(t for t in app._ankle_tabs if t["name"] == n))[0] for n in names}
check("マーカー実寸（ノギス）" in m and "計算に使う補正後" in m and all(f"{v:g} mm" in m for v in exp_eff),
      f"実行前の確認に「計算に使う補正後」（{sorted(exp_eff)}）")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
