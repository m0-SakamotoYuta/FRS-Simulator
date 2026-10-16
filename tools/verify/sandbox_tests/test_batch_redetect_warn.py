# -*- coding: utf-8 -*-
"""まとめて解析: ④が済んでいるタブで④を選ぶと注意を出す（はい=やり直す／いいえ=④を飛ばす／キャンセル=やめる）。
2026-10-15。解析そのものは呼ばない（渡された内容だけ見る）。画面は隠して動かす。"""
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
warn = {"ans": None, "msgs": []}
frs.messagebox.askyesnocancel = lambda *a, **k: (warn["msgs"].append((a, k)), warn["ans"])[1]
frs.messagebox.askyesno = lambda *a, **k: True
infos = []
frs.messagebox.showinfo = lambda *a, **k: infos.append(a)
frs.messagebox.showwarning = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_folder_filter = app._ANKLE_FOLDER_ALL
calls = []
app._ankle_run_batch = lambda jobs, **k: (calls.append(([str(t.get("name")) for t, _v in jobs], k)), [])[1]
app._ankle_show_batch_summary = lambda *a, **k: None
def walk(w):
	for c in w.winfo_children():
		yield c
		yield from walk(c)
NAMES = ["261007_FE_軸力100_ACL切", "261007_FE_軸力無し"]
def run(ans, contact):
	warn["ans"] = ans; warn["msgs"].clear(); calls.clear(); infos.clear()
	before = set(app.winfo_children())
	app._ankle_open_batch_dialog(); pump(app, 0.3)
	win = [w for w in app.winfo_children() if w not in before and isinstance(w, tk.Toplevel)][0]
	win.withdraw()
	cbs = [w for w in walk(win) if w.winfo_class() == "TCheckbutton"]
	task_det, task_con = cbs[0], cbs[1]
	if contact:
		task_con.invoke()
	for w in walk(win):
		if w.winfo_class() == "TLabel" and w.grid_info() and int(w.grid_info()["column"]) == 1 and str(w.cget("text")) in NAMES:
			r = int(w.grid_info()["row"])
			cb = next(c for c in walk(win) if c.winfo_class() == "TCheckbutton" and c.grid_info() and int(c.grid_info()["row"]) == r and int(c.grid_info()["column"]) == 0)
			cb.invoke()
	btn = [w for w in walk(win) if w.winfo_class() == "Button" and "チェックしたタブを解析" in str(w.cget("text"))][0]
	btn.invoke(); pump(app, 0.2)
	try:
		win.destroy()
	except Exception:
		pass

run(None, contact=True)
check(len(warn["msgs"]) == 1 and "④（ArUco検出+PnP）がすでに済んでいます" in str(warn["msgs"][0][0][1]), "④が済んでいるタブで④を選ぶと注意が出る")
check(warn["msgs"][0][1].get("default") == "no" and warn["msgs"][0][1].get("icon") == "warning", "注意は警告の印・既定は「いいえ」（④を飛ばす）")
check(not calls, "キャンセル: 何もしない")
run(False, contact=True)
check(calls and sorted(calls[0][0]) == sorted(NAMES) and len(calls[0][1].get("skip_detect") or ()) == 2 and calls[0][1].get("do_contact"),
      "いいえ: ④を飛ばして接触だけ計算する")
run(True, contact=True)
check(calls and not calls[0][1].get("skip_detect") and calls[0][1].get("do_detect"), "はい: ④もやり直す")
run(False, contact=False)
check(not calls and infos and "することがありません" in str(infos[-1]), "④だけで「いいえ」: することがない旨を出して何もしない")
# ④を外して接触だけなら、注意は出ない
warn["msgs"].clear(); calls.clear()
before = set(app.winfo_children())
app._ankle_open_batch_dialog(); pump(app, 0.3)
win = [w for w in app.winfo_children() if w not in before and isinstance(w, tk.Toplevel)][0]; win.withdraw()
cbs = [w for w in walk(win) if w.winfo_class() == "TCheckbutton"]
cbs[0].invoke(); cbs[1].invoke()          # ④を外し、接触を入れる
for w in walk(win):
	if w.winfo_class() == "TLabel" and w.grid_info() and int(w.grid_info()["column"]) == 1 and str(w.cget("text")) in NAMES:
		r = int(w.grid_info()["row"])
		next(c for c in walk(win) if c.winfo_class() == "TCheckbutton" and c.grid_info() and int(c.grid_info()["row"]) == r and int(c.grid_info()["column"]) == 0).invoke()
[w for w in walk(win) if w.winfo_class() == "Button" and "チェックしたタブを解析" in str(w.cget("text"))][0].invoke(); pump(app, 0.2)
check(not warn["msgs"] and calls and not calls[0][1].get("do_detect"), "④を外して接触だけなら、注意は出ずにそのまま進む")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
