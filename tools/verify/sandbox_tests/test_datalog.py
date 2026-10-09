# -*- coding: utf-8 -*-
"""ロボットの datalog: 自動で探す・時刻合わせ・グラフ（実データ、接触の計算はしない）。"""
import importlib.util, os, sys, time
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
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", a[:2])
app = frs.MainMenuGUI(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
expect = {"261007_FE_軸力100": ("FE_jikuryoku100", "0000000002.csv", 2.75),
          "261007_FE_軸力100_ACL切": ("FE_zikuryoku100_ACLcut", "0000000001.KKR", 12.40)}
for name, (stem, tail, off_exp) in expect.items():
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == name)); pump(app, 0.3)
	app.ankle_robot_log_path.set("")
	app._ankle_robot_log_autofind_ui(); pump(app, 0.1)
	f = os.path.basename(app.ankle_robot_log_path.get())
	check(f.startswith(stem) and f.endswith(tail), f"{name}: 自動で探す → {f}")
	app.ankle_robot_log_auto.set(True); app.ankle_robot_log_items.set("FE,AP,Fpd")
	t0 = time.time(); app.on_ankle_animate(); pump(app, 1.5)
	off = app.ankle_robot_log_offset_s.get()
	check(abs(off - off_exp) < 0.11, f"{name}: 自動の時刻合わせ {off:.2f} 秒（前回の解析 {off_exp:.2f} 秒）")
	v = app._sim_viewers_alive()[-1]; p = v["plotter"]
	n_ch = len(p.renderer._charts) if getattr(p.renderer, "_charts", None) is not None else -1
	check(n_ch == 3, f"{name}: グラフ 3 つ（{n_ch}）")
	bt = []
	def walk(wd):
		for c in wd.winfo_children():
			if c.winfo_class() == "TButton": bt.append(c)
			walk(c)
	walk(v["ctrl"]["window"])
	check(any(b.cget("text") == "datalog の項目…" for b in bt), f"{name}: 再生コントロールに「datalog の項目…」")
	v["seek"](100.0); pump(app, 0.4)
	p.screenshot(os.path.join(HERE, f"datalog_{'pre' if 'ACL' not in name else 'acl'}.png"))
# 項目を選び直す（最後のウィンドウで）
app.ankle_robot_log_items.set("FE,Fap,Fpd,IE")
st_btn = [b for b in bt if b.cget("text") == "datalog の項目…"][0]
st_btn.invoke(); pump(app, 0.3)
dlg = [w for w in app.winfo_children() if isinstance(w, tk.Toplevel) and w.title() == "datalog の項目"][0]
[c for c in dlg.winfo_children() if c.winfo_class() == "TButton"][0].invoke(); pump(app, 0.4)
n_ch = len(p.renderer._charts)
check(n_ch == 4, f"項目を選び直すとグラフが 4 つに（{n_ch}）")
print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
