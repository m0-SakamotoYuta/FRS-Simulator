# -*- coding: utf-8 -*-
"""靭帯の長さを比べる画面の表に、ロボットの AP・Fap との相関と「AP が増えると 伸びる/縮む」（2026-10-15）。"""
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
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
names = [it["name"] for it in app.ankle_ligaments]
print("   靭帯:", names, flush=True)
before = set(app.winfo_children())
app._ankle_open_lig_compare(); pump(app, 0.5)
cw = [w for w in app.winfo_children() if w not in before and isinstance(w, tk.Toplevel)][0]; cw.withdraw()
c = cw._lig_compare
for t, v in c["tab_vars"]: v.set(t["name"] == TAB)
for n, v in c["lig_vars"]: v.set(True)
c["draw"](); pump(app, 0.5)
cols = list(c["tree"]["columns"])
check("AP との相関 r" in cols and "Fap との相関 r" in cols and "AP が増えると" in cols, "表に AP・Fap との相関と「AP が増えると」")
rows = {c["tree"].item(r)["values"][1]: c["tree"].item(r)["values"] for r in c["tree"].get_children()}
for n, v in rows.items():
	print(f"   {n}: AP r={v[7]}  Fap r={v[8]}  AP が増えると {v[9]}", flush=True)
ok = all(str(v[7]).startswith(("+", "-")) for v in rows.values())
check(ok, "相関が数字で出る（datalog と時刻合わせを使う）")
if "ACL" in rows and "PCL" in rows:
	check(float(rows["ACL"][7]) > 0.3 and float(rows["PCL"][7]) < -0.3, "いまの ACL は AP が増えると伸び、PCL は縮む")
cw.destroy()
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
