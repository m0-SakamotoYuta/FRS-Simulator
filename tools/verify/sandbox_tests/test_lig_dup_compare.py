# -*- coding: utf-8 -*-
"""靭帯の複製（名前を変えられる）と、靭帯の長さを試験どうしで比べる画面（2026-10-15）。接触の計算はダミー。"""
import importlib.util, os, sys, time, copy, csv
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
infos = []
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: None
frs.messagebox.showinfo = lambda *a, **k: infos.append(a)
app = frs.MainMenuGUI(); app.withdraw(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
def walk(w):
	for c in w.winfo_children():
		yield c
		yield from walk(c)
TAB, TAB2 = "261007_AP_FE60_軸力100", "261007_AP_FE60_軸力100_ACL切"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
L0 = copy.deepcopy(app.ankle_ligaments[0]); L0["name"] = "ACL"
app.ankle_ligaments = [L0]
captured = {}
_orig_run = app._sim_engine_run
app._sim_engine_run = lambda scene: (captured.__setitem__("scene", scene), _orig_run(scene))[1]
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]
import gc
lig = next(o for o in gc.get_objects() if isinstance(o, dict) and "open" in o and "update" in o and "close" in o and callable(o.get("open")))
lig["open"](); pump(app, 0.5)
lw = [w for w in walk(app) if isinstance(w, tk.Toplevel) and "靭帯" in w.title()]
lw = lw[-1] if lw else None
dup = [w for w in walk(lw) if w.winfo_class() == "TButton" and str(w.cget("text")) == "複製"] if lw else []
check(bool(dup), "靭帯管理の各行に「複製」")
dup[0].invoke(); pump(app, 1.2)
names = [it["name"] for it in app.ankle_ligaments]
check(names == ["ACL", "ACL（複製）"], f"複製すると同じ付着部の「ACL（複製）」ができる（{names}）")
check(app.ankle_ligaments[1]["a"]["point"] == app.ankle_ligaments[0]["a"]["point"] and app.ankle_ligaments[1]["b"]["point"] == app.ankle_ligaments[0]["b"]["point"], "付着部は同じ")
ent = [w for w in walk(lw) if w.winfo_class() == "TEntry"]
nm_ent = [e for e in ent if e.get() == "ACL（複製）"]
check(bool(nm_ent), "名前の欄で変えられる（「ACL（複製）」の欄がある）")
if nm_ent:
	nm_ent[0].delete(0, "end"); nm_ent[0].insert(0, "ACL_cut位置"); pump(app, 1.2)
	check([it["name"] for it in app.ankle_ligaments] == ["ACL", "ACL_cut位置"], f"名前を変えると保存される（{[it['name'] for it in app.ankle_ligaments]}）")
check(any(str(w.cget("text")) == "長さを比べる…" for w in walk(lw) if w.winfo_class() == "TButton"), "靭帯管理に「長さを比べる…」")
lig["close"]()
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_ligaments = [L0]; app._ankle_tabs[app._ankle_active_tab]["snapshot"] = app._ankle_snapshot_current()

# 比べる画面
before = set(app.winfo_children())
app._ankle_open_lig_compare(); pump(app, 0.5)
cw = [w for w in app.winfo_children() if w not in before and isinstance(w, tk.Toplevel)][0]
c = cw._lig_compare
sel = [t["name"] for t, vv in c["tab_vars"] if vv.get()]
check(sorted(sel) == sorted([TAB, TAB2]), f"最初から、いまの試験と ACL切 の試験が選ばれている（{sel}）")
check([n for n, vv in c["lig_vars"] if vv.get()] == ["ACL"], "靭帯は ACL が選ばれている")
t0 = time.time(); c["draw"](); pump(app, 0.5)
rows = c["tree"].get_children()
vals = [c["tree"].item(r)["values"] for r in rows]
print(f"   描く（映像の時刻）: {time.time() - t0:.1f} 秒", flush=True)
for vv in vals: print("     ", vv, flush=True)
check(len(rows) == 2, "2試験 × ACL の 2 本が描かれる")
check(app._ankle_tabs[app._ankle_active_tab]["name"] == TAB, "計算のあと、開いていたタブに戻る")
# 長さ = 可視化と同じ（靭帯の長さのグラフの値）
sc = captured["scene"]; it = sc["ligaments"]["items"][0]; sb = sc["bones"]
Pa = np.asarray(sb[it["a"]["bone"]]["poses"]); Pb = np.asarray(sb[it["b"]["bone"]]["poses"])
Lv = np.linalg.norm((Pa[:, :3, :3] @ np.asarray(it["a"]["point"]) + Pa[:, :3, 3]) - (Pb[:, :3, :3] @ np.asarray(it["b"]["point"]) + Pb[:, :3, 3]), axis=1)
mine = next(r for r in c["last"]["rows"] if r[0] == TAB)
check(np.allclose(mine[4], Lv[:len(mine[4])], atol=1e-3), f"長さは可視化（靭帯の長さのグラフ）と同じ（{Lv.min():.2f}〜{Lv.max():.2f} mm）")
# 横軸 = ロボットの AP
c["x_var"].set("ロボット AP [mm]"); c["draw"](); pump(app, 0.3)
xr = [(r[0], np.nanmin(r[3]), np.nanmax(r[3])) for r in c["last"]["rows"]]
print("   横軸 AP の範囲:", [(a, round(float(b), 1), round(float(cc), 1)) for a, b, cc in xr], flush=True)
check(len(c["last"]["rows"]) == 2 and all(np.isfinite(b) and cc - b > 5 for a, b, cc in xr), "横軸をロボットの AP にできる（datalog の時刻合わせを使う）")
# CSV
out = os.path.join(HERE, "lig_compare_test.csv")
frs.filedialog.asksaveasfilename = lambda *a, **k: out
c["export"](); rr = list(csv.reader(open(out, encoding="utf-8-sig")))
check(rr[0] == ["試験", "靭帯", "横軸", "x", "長さ mm"] and len(rr) > 1000, f"CSV に書き出せる（{len(rr) - 1} 行）")
try:
	c["ax"].figure.savefig(os.path.join(HERE, "lig_compare.png"), dpi=90)
except Exception:
	pass
cw.destroy()
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
