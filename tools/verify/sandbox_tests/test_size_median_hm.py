# -*- coding: utf-8 -*-
"""②の「補正後」(2026-10-15):
A) 他のタブと同期: チェックしたタブ（いまのタブも含む）の深度推定の中央値を全部に入れ、「中央値」と表示する
B) 接触の結果をタブごとに3つまで残し、②に「この値なら計算済み」を出す。押すとその値に戻り、計算し直さない
接触の計算はダミーに差し替える。"""
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
msgs = []
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showinfo = lambda *a, **k: msgs.append(a)
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a)[:200])
app = frs.MainMenuGUI(); app.update()
ncalc = {"n": 0}
def fake_multi(bd, update_progress=None, cancel_var=None, quiet=False):
	ncalc["n"] += 1
	N = len(bd[0][3])
	return [{idx: np.full(m.n_points, 1.0, np.float32) for (idx, n, m, T) in bd} for _ in range(N)]
app._sim_view_precompute_multi_heatmap = fake_multi
def sel(name):
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == name)); pump(app, 0.6)
def walk(w):
	for c in w.winfo_children():
		yield c
		yield from walk(c)

# ===== A) 中央値 =====
print("== A) 中央値", flush=True)
NIKI = "niki 2872 intact 0DF-IE"
sel(NIKI)
cur = app._ankle_active_tab_dict(); folder = cur.get("folder")
gid = app._ankle_tab_group(cur)
same = [t for t in app._ankle_tabs if t is not cur and app._ankle_tab_group(t) == gid]       # 骨リストを共有（niki）
rest = [t for t in app._ankle_tabs if t is not cur and t not in same and t.get("folder") == folder]
rest_before = {t["name"]: t["snapshot"].get("ankle_marker_size_mm") for t in rest}
est = [app._ankle_tab_depth_estimate(t) for t in [cur] + same]
exp = round(float(np.median([e for e in est if e])), 2)
print(f"   骨リストを共有 {len(same) + 1} タブ（同じフォルダのほかのタブ {len(rest)}）、推定のあるもの {sum(1 for e in est if e)}、中央値 {exp}", flush=True)
check(all("niki" in t["name"] for t in same) and len(same) + 1 == 19, "最初から選ばれるのは骨リストを共有している niki の 19 タブ")
before = set(app.winfo_children())
app._ankle_open_size_sync_dialog(); pump(app, 0.3)
win = [w for w in app.winfo_children() if w not in before and isinstance(w, tk.Toplevel)][0]
rads = [w for w in walk(win) if w.winfo_class() == "TRadiobutton"]
lbls = [str(w.tk.globalgetvar(str(w.cget("textvariable")))) for w in walk(win) if w.winfo_class() == "TLabel" and str(w.cget("textvariable"))]
check(any("中央値" in str(r.cget("text")) for r in rads) and any(f"中央値 = {exp:.2f}" in x for x in lbls), f"同期の画面に中央値（{exp:.2f}）が出る: {[x for x in lbls if '中央値' in x]}")
btn = [w for w in walk(win) if w.winfo_class() == "TButton" and "中央値" in str(w.cget("text"))]
check(bool(btn), "ボタンが「中央値 … mm を入れる」")
btn[0].invoke(); pump(app, 0.3)
vals = [float(t["snapshot"]["ankle_marker_size_mm"]) for t in same]
check(all(abs(v - exp) < 1e-9 for v in vals) and abs(float(app.ankle_marker_size_mm.get()) - exp) < 1e-9,
      f"同じフォルダの全タブといまのタブに中央値 {exp} が入る")
check(all(not t["snapshot"].get("ankle_marker_size_auto", True) and "中央値" in t["snapshot"].get("ankle_marker_size_basis", "") for t in same),
      "入れたタブは手動・由来は「中央値」")
check(all(t["snapshot"].get("ankle_marker_size_mm") == rest_before[t["name"]] for t in rest), "同じフォルダでも、骨リストを共有していないタブは変わらない")
note = str(app.ankle_marker_size_note.get())
check(note.startswith("中央値") and f"{exp:.2f}" in note, f"②の表示: {note}")
# ほかのタブに切り替えて戻っても「中央値」のまま／関係ないタブに由来が漏れない
sel("261007_FE_軸力100"); n2 = str(app.ankle_marker_size_note.get())
check("中央値" not in n2, f"関係ないタブには出ない: {n2}")
sel(NIKI); n3 = str(app.ankle_marker_size_note.get())
check(n3.startswith("中央値"), f"戻ると「中央値」のまま: {n3}")
# 手で書き換えると由来は消える
class _E: keysym = "5"
app.ankle_marker_size_mm.set(exp + 0.01); app._ankle_size_mark_manual(_E()); app._ankle_size_refresh_auto()
check(not app.ankle_marker_size_basis.get() and str(app.ankle_marker_size_note.get()).startswith("手動"), f"手で変えると「中央値」は消える: {app.ankle_marker_size_note.get()}")

# ===== B) 計算済みの値 =====
print("== B) 計算済みの値", flush=True)
TAB = "261007_FE_軸力100_ACL切"
sel(TAB)
idxp = app._ankle_hm_index_path()
d = app._ankle_hm_index(); d.pop(TAB, None); app._ankle_hm_index_write(d)
# このタブの前の結果（サンドボックスに残っているもの）を消してから始める
for f in app._ankle_heatmap_dir().glob("hm_*.npz"):
	try:
		with np.load(str(f), allow_pickle=False) as z:
			m = json.loads(str(z["meta_json"]))
		if m.get("tab") == TAB:
			f.unlink()
	except Exception:
		pass
def compute(v):
	app.ankle_marker_size_auto.set(False); app.ankle_marker_size_mm.set(v); app.ankle_marker_size_basis.set(f"由来{v}")
	n0 = ncalc["n"]; r = app.on_ankle_animate(heatmap_only=True); pump(app, 0.6)
	return r, ncalc["n"] - n0
r1, c1 = compute(14.76)
r2, c2 = compute(14.70)
check(c1 == 1 and c2 == 1, f"値を変えると計算する（{c1}, {c2}）")
ents = app._ankle_hm_index().get(TAB) or []
files = [e["file"] for e in ents]
check(len(ents) == 2 and all((app._ankle_heatmap_dir() / f).exists() for f in files), f"前の値の結果も残る（{[(e['size_mm'], e['file'][:10]) for e in ents]}）")
app._ankle_refresh_hm_sizes(); pump(app, 0.2)
fr = app._ankle_hm_sizes_fr
txt = [str(w.cget("text")) for w in fr.winfo_children()]
print("   ②の表示:", txt, flush=True)
b76 = [w for w in fr.winfo_children() if w.winfo_class() == "TButton" and "14.76" in str(w.cget("text"))]
check(any("14.7 mm" in t and "いまの値" in t for t in txt) and bool(b76), "②に「14.76 mm（…）にする」と「14.7 mm ← いまの値」")
b76[0].invoke(); pump(app, 0.6)
check(abs(float(app.ankle_marker_size_mm.get()) - 14.76) < 1e-9 and app.ankle_marker_size_basis.get() == "由来14.76",
      f"押すと 14.76 に戻り、由来もそのときのもの（{app.ankle_marker_size_basis.get()}）")
n0 = ncalc["n"]; r = app.on_ankle_animate(heatmap_only=True); pump(app, 0.3)
check(ncalc["n"] == n0 and r.get("cached"), f"戻した値では計算し直さない（{r.get('msg')}）")
# キャリブを変えると、その値は「計算済み」に出ない
T0 = [list(x) for x in app.ankle_bones[1]["marker_to_bone_T"]]
app.ankle_bones[1]["marker_to_bone_T"] = [list(x) for x in T0]; app.ankle_bones[1]["marker_to_bone_T"][0][3] += 0.05
check(app._ankle_hm_sizes_ready(app._ankle_active_tab_dict()) == [], "キャリブが変わると、計算済みの値は出ない")
app.ankle_bones[1]["marker_to_bone_T"] = T0
check(len(app._ankle_hm_sizes_ready(app._ankle_active_tab_dict())) == 2, "戻すと2つとも出る")
# 3つまで: さらに2つの値で計算 → いちばん古く使ったもの（14.70）が消える
f70 = next(e["file"] for e in app._ankle_hm_index()[TAB] if abs(e["size_mm"] - 14.70) < 1e-9)
compute(14.80); compute(14.85)
ents = app._ankle_hm_index().get(TAB) or []
sz = [round(e["size_mm"], 2) for e in ents]
check(len(ents) == 3 and 14.70 not in sz and not (app._ankle_heatmap_dir() / f70).exists(),
      f"タブごとに3つまで（新しく使った順 {sz}、14.70 の結果は消えた）")
# まとめて解析の一覧の欄も、複数の記録で判定
st = app._ankle_hm_status_for_tab(app._ankle_active_tab_dict())
check(st[0].startswith("計算済み"), f"まとめて解析の一覧: {st[0]}")

print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
