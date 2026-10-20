# -*- coding: utf-8 -*-
"""靭帯の束（段階2）の「面を描く」（2026-10-19）:
面があるところで描き直すと、新しく打った点だけで楕円が決まる（前の点と混ざらない）・描いている間は前の面の輪（赤）を出したまま・
①「楕円を当てはめる」で赤い輪が消えて黄色の輪（骨の面へ落とした形）と長径・短径（2×半径）が出る・ノギスの値との差・
点を直すと楕円と長径・短径がその場で変わる・Delete キーで選んだ点を消す・②で決めると黄色と同じ形の輪・やめると前の輪に戻る・
前とほぼ同じなら知らせる・黄色の輪は打った点を通る（楕円の面の法線の向きに骨の面へ落とす。10/20）・
「前の点を読み込む」で直せる・打った点を 3D の画面で左クリックすると選べる（一覧も選ばれ、点の色が変わる）・
選んだ点を消す／打ち直す・左ドラッグ（回す）では選ばない・一覧で選んでも 3D の点の色が変わる。
右クリック・左クリックは VTK のイベントを本当に起こして確かめる。接触の計算はダミー。1〜2分。"""
import importlib.util, os, sys, time, shutil, gc
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
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a[:2])[:300])
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
app._sim_view_precompute_multi_heatmap = lambda bd, update_progress=None, cancel_var=None, quiet=False: [
	{idx: np.ones(len(np.asarray(m.points)), np.float32) for (idx, n, m, T) in bd} for t in range(len(bd[0][3]))]
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)

TAB = "261007_FE_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
shutil.rmtree(os.path.join(HERE, "cache", "ankle_lig_bundles"), ignore_errors=True)
app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(True); app.ankle_alt_model.set(False)
app.on_ankle_animate(); pump(app, 1.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
v["set_paused"](True)
ctx = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
ctx.open_window(); pump(app, 0.4)
G = [x for x in app.ankle_ligaments if x["name"] == "ACL"][0]
si_t = ctx.names.index("脛骨")
bones = ctx._ensure_bones(); idx_t = ctx.idx_of[si_t]
cpp, ns, _ = bones.closest(idx_t, np.asarray(G["b"]["point"], float)[None]); nrm = ns[0]
t1 = np.cross(nrm, [1, 0, 0]); t1 /= np.linalg.norm(t1); t2 = np.cross(nrm, t1)
def ring(c, ra, rb_, n=8, ph=0.3):
	th = np.linspace(0, 2 * np.pi, n, endpoint=False) + ph
	return bones.closest(idx_t, c + np.outer(np.cos(th) * ra, t1) + np.outer(np.sin(th) * rb_, t2) + 2 * nrm)[0]
ctx._add_bundle(); pump(app, 0.2)
b = ctx.data["bundles"][-1]; b["name"] = "選ぶテスト"
# 1回目: 脛骨の上に 4 × 2 mm の外周
ctx._ol_start(b, "b")
for q in ring(cpp[0], 4.0, 2.0):
	ctx.add_point(si_t, q)
ctx._ol_commit(); pump(app, 0.3)
e1 = dict(b["b"]["ellipse"])
check(abs(e1["ra"] - 4.0) < 0.6, f"1回目の楕円（長半径 {e1['ra']:.2f} mm）")

# 画面: 付着部の中心を真上から
T = ctx.poses[si_t][ctx.fi]
def W(q):
	return T[:3, :3] @ np.asarray(q, float) + T[:3, 3]
c2 = bones.closest(idx_t, (cpp[0] + 6.0 * t1)[None])[0][0]           # 2回目は 6 mm 横へずらす
p.camera.focal_point = W(c2); p.camera.position = W(c2) + 25 * (T[:3, :3] @ nrm); p.camera.up = tuple(T[:3, :3] @ t2)
# 大腿骨が手前で脛骨を隠すので、確かめる間は脛骨だけ見せる（右クリックは見えている骨に当たる）
hidden = []
for k_, a_ in p.renderer.actors.items():
	if str(k_).startswith("bone_") and not str(k_).startswith(f"bone_{si_t}") and a_.GetVisibility():
		a_.VisibilityOff(); hidden.append(a_)
p.render(); pump(app, 0.3)
ren = p.renderer; it = p.iren.interactor
def disp(wp):
	ren.SetWorldPoint(float(wp[0]), float(wp[1]), float(wp[2]), 1.0); ren.WorldToDisplay(); return ren.GetDisplayPoint()[:2]
def click(btn, x, y, drag=0):
	it.SetEventInformation(int(x), int(y)); it.InvokeEvent(f"{btn}ButtonPressEvent")
	it.SetEventInformation(int(x) + drag, int(y)); it.InvokeEvent(f"{btn}ButtonReleaseEvent")
	pump(app, 0.15)

# 2回目: 「面を描く」→ 前の点は読み込まれない・前の楕円は残る
btn_draw = [w for w in walk(ctx.win) if w.winfo_class() == "TButton" and str(w.cget("text")) == "面を描く"]
ctx._ol_start(b, "b"); pump(app, 0.2)
m = ctx.st["mode"]
check(m is not None and len(m["pts"]) == 0 and m.get("prev") is not None and len(m["prev"][1]) == 8,
      f"面があるところで「面を描く」→ 点は空から（前の {len(m['prev'][1]) if m and m.get('prev') else 0} 点は「前の点を読み込む」用に取っておく）")
a = ctx.actors.get(b["id"])
check(a is not None and a.get("eb_act") is not None and a["eb_act"].GetVisibility(), "描いている間は、前の面の輪（赤）を出したまま")
msg = ctx.w["ol_msg"].cget("text")
check("前の点を読み込む" in msg, f"描く欄にお知らせ（{msg[:60]}…）")
# 右クリックで 8 点（6 mm 横・3 × 1.5 mm）
new_local = ring(c2, 3.0, 1.5)
for q in new_local:
	x, y = disp(W(q + 0.5 * nrm))
	click("Right", x, y)
check(len(m["pts"]) == 8 and m["si"] == si_t, f"右クリックで 8 点打てた（{len(m['pts'])} 点・{ctx.names[m['si']] if m['si'] is not None else '-'}）")
import ankle_lig_bundles as _alb
from vtk.util.numpy_support import vtk_to_numpy as _v2n
def yellow_line():
	L_ = [act for act, _si in ctx.pend if act.GetMapper().GetInput().GetNumberOfLines() > 0]
	return _v2n(L_[0].GetMapper().GetInput().GetPoints().GetData()).astype(float) if L_ else None
check(yellow_line() is None and a["eb_act"].GetVisibility() and ctx.w["ol_dims"].cget("text") == "",
      "①を押す前は、黄色の輪も長径・短径も出さない（赤い輪はそのまま）")
[w for w in walk(ctx.win) if w.winfo_class() == "TButton" and "楕円を当てはめる" in str(w.cget("text"))][0].invoke(); pump(app, 0.3)
ef = _alb.fit_ellipse([q.copy() for q in m["pts"]])
dims = ctx.w["ol_dims"].cget("text")
check(not a["eb_act"].GetVisibility() and yellow_line() is not None and ctx.st["mode"] is not None and "fitted" in ctx.st["mode"],
      "①「楕円を当てはめる」→ 赤い輪が消えて黄色の輪が出る（まだ保存しない・描き続けられる）")
check(f"長径 {2 * ef['ra']:.1f} mm" in dims and f"短径 {2 * ef['rb']:.1f} mm" in dims, f"長径・短径（端から端 = 2×半径）を出す（{dims}）")
from scipy.spatial import cKDTree as _KD
def ring_dev(Q, R_):
	Rf = np.concatenate([R_[k] + np.linspace(0, 1, 30)[:, None] * (R_[(k + 1) % len(R_)] - R_[k]) for k in range(len(R_))])
	return _KD(Rf).query(np.asarray(Q, float))[0]
dv = ring_dev(m["pts"], yellow_line())
check(dv.max() < 0.3, f"黄色の輪は打った点を通る（点から輪までの最大 {dv.max():.2f} mm）")
# 凹凸のある所でも: 脛骨のいろいろな所に 5 点（骨の面の上の楕円）→ 当てはめた輪が点を通るか
rng_ = np.random.default_rng(3)
Vt = np.asarray(ctx.bones_full[[k for k, b_ in enumerate(ctx.bones_full) if int(b_[0]) == idx_t][0]][2].points, float)
devs = []
for _k in range(25):
	q0 = Vt[rng_.integers(len(Vt))]
	cp_, ns_, _ = bones.closest(idx_t, q0[None]); n0 = ns_[0]
	a1 = np.cross(n0, [1, 0, 0]); a1 /= np.linalg.norm(a1); a2 = np.cross(n0, a1)
	th_ = np.linspace(0, 2 * np.pi, 5, endpoint=False) + rng_.uniform(0, 1)
	Q = bones.onto(idx_t, cp_[0] + np.outer(np.cos(th_) * rng_.uniform(3, 6), a1) + np.outer(np.sin(th_) * rng_.uniform(1.5, 3), a2), n0)[0]
	try:
		ring_, _ = ctx._foot_ring({"ellipse": _alb.fit_ellipse(Q), "bone_name": "脛骨"}, 96)
	except Exception:
		continue
	devs.append(ring_dev(Q, ring_["out_calc"]).max())
devs = np.array(devs)
check(len(devs) >= 15 and np.median(devs) < 0.1 and np.percentile(devs, 90) < 0.3,
      f"脛骨の {len(devs)} か所で、当てはめた輪が打った点を通る（ずれの中央値 {np.median(devs):.2f} mm・90% {np.percentile(devs, 90):.2f} mm。直す前は 0.89・1.97）")
ctx.w["cal_L"].delete(0, "end"); ctx.w["cal_L"].insert(0, "7"); ctx.w["cal_S"].delete(0, "end"); ctx.w["cal_S"].insert(0, "3")
ctx.w["cal_L"].focus_force(); pump(app, 0.1); ctx.w["cal_L"].event_generate("<Return>"); pump(app, 0.1)
dims = ctx.w["ol_dims"].cget("text")
check(f"ノギス 7 mm、差 {2 * ef['ra'] - 7:+.1f}" in dims and f"ノギス 3 mm、差 {2 * ef['rb'] - 3:+.1f}" in dims, f"ノギスの値を入れると差を出す（{dims}）")
# 左クリックで 3 番目の点を選ぶ
_sp = [disp(W(q)) for q in m["pts"]]
print("   画面の上での点の間隔の最小", round(min(np.hypot(a_[0] - b_[0], a_[1] - b_[1]) for i_, a_ in enumerate(_sp) for b_ in _sp[i_ + 1:]), 1), "px")
x3, y3 = disp(W(m["pts"][2]))
click("Left", x3 + 2, y3 - 1)
lb = ctx.w["ol_list"]
check(ctx.st["sel"] == 2 and lb.curselection() == (2,), f"打った点を左クリックすると選べる（選んだ {ctx.st['sel']}・一覧 {lb.curselection()}）")
cols = [tuple(round(c, 2) for c in act.GetProperty().GetColor()) for act, _si in ctx.pend[:8]]
check(cols[2] == (1.0, 0.0, 1.0) and all(c_ == (1.0, 0.83, 0.0) for k, c_ in enumerate(cols) if k != 2),
      "選んだ点だけ色が変わる（赤紫）")
# 左ドラッグ（回す）では選びが変わらない
x5, y5 = disp(W(m["pts"][4]))
click("Left", x5, y5, drag=30)
check(ctx.st["sel"] == 2, "左ドラッグ（回す）では選ばない")
# 何もない所の左クリックでは選びは変わらない
click("Left", 5, 5)
check(ctx.st["sel"] == 2, "点から離れた所の左クリックでは選びは変わらない")
# 一覧で選ぶと 3D の色も変わる
lb.selection_clear(0, "end"); lb.selection_set(5); lb.event_generate("<<ListboxSelect>>"); pump(app, 0.2)
cols = [tuple(round(c, 2) for c in act.GetProperty().GetColor()) for act, _si in ctx.pend[:8]]
check(ctx.st["sel"] == 5 and cols[5] == (1.0, 0.0, 1.0), "一覧で選んでも 3D の点の色が変わる")
# 選んだ点を消す（3D で選んだ点）
click("Left", *disp(W(m["pts"][2])))
p_sel = m["pts"][2].copy()
[w for w in walk(ctx.win) if w.winfo_class() == "TButton" and str(w.cget("text")) == "選んだ点を消す"][0].invoke(); pump(app, 0.2)
check(len(m["pts"]) == 7 and not any(np.allclose(q, p_sel) for q in m["pts"]), "3D で選んだ点を「選んだ点を消す」で消せる")
ef7 = _alb.fit_ellipse([q.copy() for q in m["pts"]])
check(f"長径 {2 * ef7['ra']:.1f} mm" in ctx.w["ol_dims"].cget("text") and "（7 点）" in ctx.w["ol_dims"].cget("text"),
      f"点を消すと、楕円と長径・短径がその場で変わる（{ctx.w['ol_dims'].cget('text')[:40]}…）")
# Delete キー（3D の画面）で選んだ点を消す
click("Left", *disp(W(m["pts"][5])))
p_del = m["pts"][5].copy()
it.SetKeySym("Delete"); it.InvokeEvent("KeyPressEvent"); pump(app, 0.2)
check(len(m["pts"]) == 6 and not any(np.allclose(q, p_del) for q in m["pts"]) and "（6 点）" in ctx.w["ol_dims"].cget("text"),
      "3D の画面で Delete キー → 選んだ点を消す（長径・短径も変わる）")
# 選んだ点を打ち直す
click("Left", *disp(W(m["pts"][0])))
[w for w in walk(ctx.win) if w.winfo_class() == "TButton" and str(w.cget("text")) == "選んだ点を打ち直す"][0].invoke(); pump(app, 0.2)
p0_before = m["pts"][0].copy(); others = [q.copy() for q in m["pts"][1:]]
x, y = disp(W(new_local[2] + 0.5 * nrm))
click("Right", x, y)
check(len(m["pts"]) == 6 and np.linalg.norm(m["pts"][0] - p0_before) > 0.5 and all(np.allclose(a_, b_) for a_, b_ in zip(m["pts"][1:], others))
      and ctx.st["sel"] == 0, "3D で選んだ点を「選んだ点を打ち直す」→ 右クリックでその点だけ置き換わる（選んだまま）")
# 決める → 新しい点だけの楕円（前の面と混ざらない）
exp = _alb.fit_ellipse([q.copy() for q in m["pts"]])
PL = yellow_line()
d_surf = np.abs(bones.closest(idx_t, PL)[2]).max() if PL is not None else 99
check(PL is not None and len(PL) == 48 and d_surf < 0.2, f"黄色の見本の輪は骨の面へ落としてある（48 点・面からの距離の最大 {d_surf:.3f} mm）")
[w for w in walk(ctx.win) if w.winfo_class() == "TButton" and "この楕円で決める" in str(w.cget("text"))][0].invoke(); pump(app, 0.3)
e2 = b["b"]["ellipse"]
check(b["b"].get("caliper") == [7.0, 3.0], f"ノギスの値も束の端に覚える（{b['b'].get('caliper')}）")
check(ctx.st["mode"] is None and len(b["b"]["outline"]) == 6 and abs(e2["ra"] - exp["ra"]) < 1e-6 and abs(e2["rb"] - exp["rb"]) < 1e-6
      and float(np.linalg.norm(np.asarray(e2["center"]) - np.asarray(e1["center"]))) > 2.0,
      f"②で、新しく打った 6 点だけの楕円に決まる（{2 * e2['ra']:.1f}×{2 * e2['rb']:.1f} mm・前の面の中心から {float(np.linalg.norm(np.asarray(e2['center']) - np.asarray(e1['center']))):.1f} mm）")
a = ctx.actors.get(b["id"])
RN = _v2n(a["eb_pd"].GetPoints().GetData()).astype(float)
check(a["eb_act"].GetVisibility() and PL is not None and np.abs(RN - PL).max() < 0.05,
      f"決めると新しい輪（赤）が出て、黄色の見本と同じ形（差の最大 {np.abs(RN - PL).max() if PL is not None else -1:.3f} mm）")
# 「前の点を読み込む」で直せる
ctx._ol_start(b, "b"); pump(app, 0.2)
[w for w in walk(ctx.win) if w.winfo_class() == "TButton" and "前の点を読み込む" in str(w.cget("text"))][0].invoke(); pump(app, 0.2)
check(len(ctx.st["mode"]["pts"]) == 6 and lb.size() == 6 and ctx.w["cal_L"].get() == "7", "「前の点を読み込む」で前の外周（6 点）を読み込める・ノギスの値も戻る")
ctx._ol_fit(); pump(app, 0.2)
check(not ctx.actors[b["id"]]["eb_act"].GetVisibility(), "①で赤い輪が隠れる")
ctx._ol_cancel(); pump(app, 0.2)
check(ctx.actors[b["id"]]["eb_act"].GetVisibility(), "やめると前の面の輪（赤）が戻る")
# 前の点を読み込んでそのまま決める → 「ほぼ同じ」と知らせる
ctx._ol_start(b, "b"); pump(app, 0.2); ctx._ol_load_prev(); pump(app, 0.2); ctx._ol_commit(); pump(app, 0.3)
check("ほぼ同じ" in ctx.w["ol_msg"].cget("text"), f"前とほぼ同じ楕円なら知らせる（{ctx.w['ol_msg'].cget('text')[:70]}…）")
check(ctx.st["sel"] is None and not ctx.pend, "やめると選びも黄色の点も消える")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
