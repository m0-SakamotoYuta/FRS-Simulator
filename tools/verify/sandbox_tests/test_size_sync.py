# -*- coding: utf-8 -*-
import importlib.util, os, sys, time, copy
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.update_idletasks(); app.update()
		if time.time() >= t: break
		time.sleep(0.01)
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); pump(app, 0.5)
for i in range(app.notebook.index("end")):
	if "ankle" in app.notebook.tab(i, "text"): app.notebook.select(i)
FOLDER = "261007_ ブタ　ACLcut前後の比較試験"
A = "261007_FE_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == A)); pump(app, 0.5)
before = {t["name"]: copy.deepcopy(t.get("snapshot") or {}) for t in app._ankle_tabs}
# 補正後を手で 14.80 に
af = app._ankle_size_note_lbl.master.master
ent = af.grid_slaves(row=0, column=5)[0]
app.ankle_marker_size_mm.set(14.80); ent.focus_force(); pump(app, 0.1); ent.event_generate("<KeyRelease>", keysym="0", when="now"); pump(app, 0.5)
check(not app.ankle_marker_size_auto.get() and app.ankle_marker_size_mm.get() == 14.8, "補正後を手で 14.80 にすると手動")
btns = [c for c in af.grid_slaves(row=0, column=6)[0].winfo_children() if c.winfo_class() == "TButton"]
check([b.cget("text") for b in btns] == ["深度から推定…", "他のタブと同期…"], "②にボタン: " + " / ".join(b.cget("text") for b in btns))
btns[1].invoke(); pump(app, 0.5)
w = [x for x in app.winfo_children() if isinstance(x, frs.tk.Toplevel) and "同期" in x.title()]
check(len(w) == 1, "同期の窓が開く")
w = w[0]
cbs = []
def walk(wd):
	for c in wd.winfo_children():
		if c.winfo_class() == "TCheckbutton": cbs.append(c)
		walk(c)
walk(w)
on = [c.cget("text") for c in cbs if c.instate(["selected"])]
check(len(on) == 7 and all(any(n in x for n in [t["name"] for t in app._ankle_tabs if t.get("folder") == FOLDER]) for x in on), f"同じフォルダの 7 タブだけ最初からチェック（{len(on)}）")
check(len(cbs) == len(app._ankle_tabs) - 1, f"ほかのフォルダのタブも選べる（全 {len(cbs)} 件）")
from capture_util import capture
capture(w, os.path.join(HERE, "sync.png"))
go = None
def findb(wd):
	global go
	for c in wd.winfo_children():
		if c.winfo_class() == "TButton" and "入れる" in str(c.cget("text")): go = c
		findb(c)
findb(w); check(go is not None and "14.8 mm" in go.cget("text"), "実行ボタン: " + go.cget("text"))
go.invoke(); pump(app, 0.3)
for t in app._ankle_tabs:
	sn = t.get("snapshot") or {}
	n = t["name"]
	if t.get("folder") == FOLDER and n != A:
		tru_before = before[n].get("ankle_marker_size_true_mm", before[n].get("ankle_marker_size_mm"))
		check(sn.get("ankle_marker_size_mm") == 14.8 and sn.get("ankle_marker_size_auto") is False and sn.get("ankle_marker_size_true_mm") == tru_before,
		      f"{n}: 補正後 {sn.get('ankle_marker_size_mm')}・{'自動' if sn.get('ankle_marker_size_auto') else '手動'} / 実寸(ノギス) {sn.get('ankle_marker_size_true_mm')}（前 {tru_before}）")
others_changed = [t["name"] for t in app._ankle_tabs if t.get("folder") != FOLDER and t["name"] != A and (t.get("snapshot") or {}) != before[t["name"]]]
check(not others_changed, f"ほかのフォルダのタブは変わらない（変わった: {others_changed[:3]}）")
check(app.ankle_marker_size_mm.get() == 14.8 and app.ankle_marker_size_true_mm.get() == 15.0, "いまのタブはそのまま（補正後 14.8 / 実寸 15）")
B = "261007_AP_FE60_軸力100_ACL切"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == B)); pump(app, 0.6)
check(app.ankle_marker_size_mm.get() == 14.8 and app.ankle_marker_size_true_mm.get() == 15.0 and not app.ankle_marker_size_auto.get(),
      f"同期したタブを開くと 補正後 {app.ankle_marker_size_mm.get()} / 実寸 {app.ankle_marker_size_true_mm.get()} / 手動")
print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
