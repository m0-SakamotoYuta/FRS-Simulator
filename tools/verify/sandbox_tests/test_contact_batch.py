# -*- coding: utf-8 -*-
"""接触（ヒートマップ）の保存・再利用と、まとめて解析での一括計算（実データ）。"""
import importlib.util, os, json, time, copy, glob
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c:
		fails.append(l)
def pump(app, n=3):
	for _ in range(n):
		app.update_idletasks(); app.update()
def walk(w):
	yield w
	for c in w.winfo_children():
		yield from walk(c)
dialogs = []
for kind in ("showwarning", "showerror", "showinfo"):
	setattr(frs.messagebox, kind, lambda *a, _k=kind, **k: dialogs.append((_k, a[:2])))
frs.messagebox.askyesno = lambda *a, **k: (dialogs.append(("askyesno", a[:2])), True)[1]

HM = os.path.join(HERE, "cache", "ankle_heatmap")
for f in glob.glob(os.path.join(HM, "*.npz")):
	os.remove(f)

app = frs.MainMenuGUI(); pump(app, 5)
app._ankle_folder_filter = app._ANKLE_FOLDER_ALL; app._ankle_rebuild_tabbar()
def tidx(name):
	return [i for i, t in enumerate(app._ankle_tabs) if t["name"] == name][0]
TAB = "heal strike 100N"
app.on_ankle_tab_select(tidx(TAB)); pump(app)
print(f"   {TAB}: 骨 {[b['name'] for b in app.ankle_bones]} / モード {app.ankle_workflow_mode.get()} / "
      f"④ {int((app._ankle_get_current_cache() or {}).get('frame_count', 0))}枚")

# ---------- 1. まとめて計算（接触だけ）→ 保存 ----------
cap = {}
orig_bc = app._ankle_heatmap_batch_compute
def spy_bc(bones_data, bones_for_key, N, n_bones, from_cache, *a, **k):
	cap.update(bd=bones_data, bfk=bones_for_key, N=N, n=n_bones)
	return orig_bc(bones_data, bones_for_key, N, n_bones, from_cache, *a, **k)
app._ankle_heatmap_batch_compute = spy_bc
t0 = time.time()
dialogs.clear()
res = app.on_ankle_animate(heatmap_only=True, title_prefix="[test] ")
dt = time.time() - t0
print(f"   まとめて計算: {res}  ({dt:.0f}秒)")
check(isinstance(res, dict) and res.get("status") == "ok" and not res.get("cached"), "接触だけを計算して ok")
check(not [d for d in dialogs if d[0] != "askyesno"], f"ダイアログは出ない {dialogs}")
files = glob.glob(os.path.join(HM, "hm_*.npz"))
check(len(files) == 1, f"結果が1ファイル保存される {[os.path.basename(f) for f in files]}")
size_mb = os.path.getsize(files[0]) / 1e6 if files else 0
print(f"   ファイルの大きさ: {size_mb:.1f} MB（{cap['n']}骨 × {cap['N']}フレーム）")

# 保存した値 = その場で計算し直した値 か
key = app._ankle_heatmap_key(cap["bfk"], 15000)
loaded = app._ankle_heatmap_cache_load(key, cap["bd"], 15000, cap["N"])
check(loaded is not None, "保存した結果を読み込める")
bd_s = [(i, n, app._sim_simplify_for_heatmap(n, m, 15000), T) for (i, n, m, T) in cap["bd"]]
cv = frs.tk.BooleanVar(value=False)
fresh = app._sim_view_precompute_multi_heatmap(bd_s, update_progress=lambda *a: True, cancel_var=cv, quiet=True)
worst = 0.0; worst_contact = 0.0
for t in range(cap["N"]):
	for idx, d in fresh[t].items():
		a = np.clip(np.nan_to_num(d, nan=300, posinf=300, neginf=-300), -300, 300)
		b = loaded[1][t][idx]
		worst = max(worst, float(np.max(np.abs(a - b))))
		m = a < 5.0                                 # 接触の色が付く付近（5mm 以内）
		if m.any():
			worst_contact = max(worst_contact, float(np.max(np.abs(a[m] - b[m]))))
check(worst_contact <= 0.0051, f"5mm以内（色が付く範囲）で、保存した値と計算し直した値の差 ≤ 0.005mm（最大 {worst_contact:.4f} mm）")
print(f"   （全範囲の最大差 {worst:.4f} mm ※±300mm で打ち切り）")

# ---------- 2. シミュレーション実行 → 保存済みを使い、計算開始のダイアログを出さない ----------
called = {"dialog": 0, "engine": None}
app._show_precompute_dialog = lambda *a, **k: (called.__setitem__("dialog", called["dialog"] + 1), (_ for _ in ()).throw(RuntimeError("dialog")))[1]
app._sim_engine_run = lambda scene: called.__setitem__("engine", scene)
dialogs.clear()
app.on_ankle_animate(); pump(app)
sc = called["engine"]
check(called["dialog"] == 0 and sc is not None, "シミュレーション実行: 計算開始のダイアログを出さずに表示へ進む")
if sc is not None:
	check(bool(sc["heatmap"]["enabled"]) and all(b["scalars"] is not None for b in sc["bones"]), "表示に接触の色が入っている")
	b0 = sc["bones"][0]
	same = np.allclose(b0["scalars"][3], loaded[1][3][b0["_idx"]])
	check(same, "表示に使われた値 = 保存した値")

# ---------- 4. 骨を固定しても、保存済みの結果がそのまま使われる ----------
app.ankle_bones[0]["fixed"] = True
called.update(dialog=0, engine=None)
app.on_ankle_animate(); pump(app)
check(called["dialog"] == 0 and called["engine"] is not None, "骨を固定しても計算し直さない（骨どうしの距離は変わらないため）")
app.ankle_bones[0]["fixed"] = False

# ---------- 3. 条件を変えたら、古い結果は使わない ----------
T_old = copy.deepcopy(app.ankle_bones[1].get("marker_to_bone_T"))
if T_old is not None:
	Tn = np.asarray(T_old, float); Tn[0, 3] += 0.5
	app.ankle_bones[1]["marker_to_bone_T"] = Tn.tolist()
	called.update(dialog=0, engine=None)
	try:
		app.on_ankle_animate(); pump(app)
	except RuntimeError:
		pass
	check(called["dialog"] == 1, "キャリブを変えると、保存済みの結果を使わず「計算開始」のダイアログが出る")
	app.ankle_bones[1]["marker_to_bone_T"] = T_old

# ---------- 6. 骨が2本未満 / ④の結果が無い ----------
keep = app.ankle_bones
app.ankle_bones = keep[:1]
r = app.on_ankle_animate(heatmap_only=True)
check(r.get("status") == "skip", f"骨が1本: 計算なし（skip） {r}")
app.ankle_bones = keep
c0 = app._ankle_pose_cache.pop(TAB)
r = app.on_ankle_animate(heatmap_only=True)
check(r.get("status") == "error" and "④" in r.get("msg", ""), f"④の結果が無い: 理由つきエラー {r}")
app._ankle_pose_cache[TAB] = c0

# ---------- キャンセル: 保存しない ----------
for f in glob.glob(os.path.join(HM, "*.npz")):
	os.remove(f)
orig_pre = app._sim_view_precompute_multi_heatmap
def cancel_pre(bd, update_progress, cancel_var, quiet=False):
	cancel_var.set(True); return []
app._sim_view_precompute_multi_heatmap = cancel_pre
r = app.on_ankle_animate(heatmap_only=True)
check(r.get("status") == "cancel" and not glob.glob(os.path.join(HM, "hm_*.npz")), f"キャンセル: 保存しない {r}")
app._sim_view_precompute_multi_heatmap = orig_pre

# ---------- 5. まとめて解析の画面から（接触だけ・2タブ）----------
TAB2 = "足底設置 100Nまで"
app.on_ankle_tab_select(tidx(TAB)); pump(app)
app._ankle_heatmap_batch_compute = orig_bc
# 1タブ目は先に計算して保存済みにしておく（2回目は「保存済み」になるはず）
app.on_ankle_animate(heatmap_only=True)
before = set(app.winfo_children())
app._ankle_open_batch_dialog(); pump(app, 5)
win = [w for w in app.winfo_children() if w not in before and w.winfo_class() == "Toplevel"][0]
cbs = [w for w in walk(win) if w.winfo_class() == "TCheckbutton"]
task_det, task_con = cbs[0], cbs[1]
rows = cbs[2:]
task_det.invoke(); task_con.invoke(); pump(app)          # ④を外し、接触を入れる
rows[tidx(TAB)].invoke(); rows[tidx(TAB2)].invoke(); pump(app)
btn = [w for w in walk(win) if w.winfo_class() == "Button" and "チェックしたタブを解析" in str(w.cget("text"))][0]
dialogs.clear()
t0 = time.time()
btn.invoke(); pump(app, 5)
print(f"   画面から実行: {time.time() - t0:.0f}秒  ダイアログ {[d[0] for d in dialogs]}")
summary = [d for d in dialogs if d[0] in ("showinfo", "showwarning")]
print("   結果まとめ:", summary[-1][1][1].replace("\n", " / ") if summary else "")
labels = {str(w.cget("text")) for w in walk(win) if w.winfo_class() == "TLabel"}
r1 = [x for x in labels if "接触 保存済み" in x]
r2 = [x for x in labels if "接触 計算済み" in x or "接触なし" in x]
check(bool(r1), f"1タブ目は「接触 保存済み」（計算を省いた） {r1}")
check(bool(r2), f"2タブ目は計算した {r2}")
win.destroy(); pump(app)
app.destroy()
print("RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL: {fails}")
