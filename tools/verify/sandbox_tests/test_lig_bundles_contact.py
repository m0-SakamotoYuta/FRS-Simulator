# -*- coding: utf-8 -*-
"""靭帯の束（段階2）（2026-10-20）: 束どうしの当たり（外形どうし）と自然長の基準のコマ。
画面の欄・既定で当たり ON・基準のコマで自然長＝その時の長さ・戻す・切り替えると計算の結果が外れる・当たりで外形の重なりが減る。
当たりで外形が跳ばない（10/21）。接触の計算はダミー。4〜6分。"""
import importlib.util, os, sys, time, json, glob, shutil, gc
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
ctx = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
ctx.open_window(); pump(app, 0.4)
# 束を1つ（いまの ACL の点のまわりに付着部の面）
G = [x for x in app.ankle_ligaments if x["name"] == "ACL"][0]
def ring(si, center, ra, rb_, n=7):
	bones = ctx._ensure_bones(); idx = ctx.idx_of[si]
	cpp, ns, _ = bones.closest(idx, np.asarray(center, float)[None]); nrm = ns[0]
	t1 = np.cross(nrm, [1, 0, 0]); t1 /= np.linalg.norm(t1); t2 = np.cross(nrm, t1)
	th = np.linspace(0, 2 * np.pi, n, endpoint=False) + 0.3
	return bones.closest(idx, cpp[0] + np.outer(np.cos(th) * ra, t1) + np.outer(np.sin(th) * rb_, t2) + 2 * nrm)[0]
def make(name, lig, si_a, si_b, off=(0.0, 0.0, 0.0)):
	ctx._add_bundle(); pump(app, 0.2)
	b = ctx.data["bundles"][-1]; b["name"] = name; b["n_fibers"] = 8
	G = [x for x in app.ankle_ligaments if x["name"] == lig][0]
	for end, si, key in (("a", si_a, "a"), ("b", si_b, "b")):
		ctx._ol_start(b, end)
		for q in ring(si, np.asarray(G[key]["point"], float) + np.asarray(off, float), 3.0, 1.8):
			ctx.add_point(si, q)
		ctx._ol_commit(); pump(app, 0.2)
	return b
si_f, si_t = ctx.names.index("大腿骨"), ctx.names.index("脛骨")
b1 = make("ACL(当たり)", "ACL", si_f, si_t)
b2 = make("PCL(当たり)", "PCL", si_f, si_t)
b3 = make("ACL(隣)", "ACL", si_f, si_t, off=(2.5, 0.0, 0.0))     # ACL の隣（跳びが出やすい、隣り合う束）
check(ctx._ready(b1) and ctx._ready(b2) and ctx._ready(b3), "束 3 本の面ができた")
ctx.data["params"]["stride"] = 10
lbl = ctx.w["ref_lbl"]
check(lbl.cget("text").startswith(("表示の始まりのコマ", "初期状態（自動）")), f"自然長の基準の欄（{lbl.cget('text')}）")
# 自動: 試験の段階から初期状態の終わりの 1 秒前（人工の段階で確かめる）
po_save = ctx.scene.get("pres_overlay")
times_ = np.asarray(ctx.scene.get("frame_times") or [], float)
fps_ = 1.0 / float(np.median(np.diff(times_)))
ctx.scene["pres_overlay"] = {"phase_at": lambda fi: (0 if fi < 300 else (1 if fi < 600 else -1), 0.0)}
ctx._auto_rest_memo = "none"
exp_ = ctx.s0 + 300 - 1 - int(round(fps_))
check(ctx._auto_rest() == exp_ and ctx._rest_frame() == exp_, f"自動: 初期状態の終わりの 1 秒前（{ctx._auto_rest()}・期待 {exp_}）")
ctx._rest_label()
check(lbl.cget("text").startswith("初期状態（自動）"), f"欄に「初期状態（自動）」（{lbl.cget('text')}）")
ctx.scene["pres_overlay"] = {"phase_at": lambda fi: None}
ctx._auto_rest_memo = "none"
check(ctx._auto_rest() is None, "記録が無ければ自動は無し（表示の始まりのコマ）")
ctx.scene["pres_overlay"] = {"phase_at": lambda fi: (0, 0.0)}
ctx._auto_rest_memo = "none"
check(ctx._auto_rest() is None, "引張が無い記録（FE など）も自動は無し")
if po_save is None: ctx.scene.pop("pres_overlay", None)
else: ctx.scene["pres_overlay"] = po_save
ctx._auto_rest_memo = "none"; ctx._rest_label()
btns = {str(w.cget("text")): w for w in walk(ctx.win) if w.winfo_class() == "TButton"}
check("いまのコマを基準にする" in btns and "自動（初期状態）に戻す" in btns, "基準のボタン")
cv = [w for w in walk(ctx.win) if w.winfo_class() == "TCheckbutton" and "束どうしの当たり" in str(w.cget("text"))]
check(len(cv) == 1 and bool(ctx.w["contact_var"].get()), "「束どうしの当たり」のチェック（既定で ON）")
import ankle_lig_bundles as _alb
key_on = ctx._key([b1, b2, b3])
# 当たりあり（既定）で計算
t0 = time.time(); ctx.compute(parent=ctx.win); pump(app, 0.3); dt_on = time.time() - t0
check(all(ctx.results.get(b["id"]) is not None for b in (b1, b2, b3)), f"当たりありで計算した（{dt_on:.0f} 秒）")
def env_overlap():
	"""計算したコマごとに、束 B の線維の点が束 A の外形（楕円の断面＋太さ）の中にある数（両端 8% を除く）。"""
	from scipy.spatial import cKDTree
	ra, rb = ctx.results[b1["id"]], ctx.results[b2["id"]]
	T_a = ctx.poses[ra["sia"]]; T_b = ctx.poses[rb["sia"]]
	n = 0
	for j in range(0, len(ra["frames"]), 3):
		fi = int(ra["frames"][j]) - ctx.s0
		if not (0 <= fi < ctx.n_view): continue
		Xa = ra["local"][j].astype(float) @ T_a[fi][:3, :3].T + T_a[fi][:3, 3]
		Xb = rb["local"][j].astype(float) @ T_b[fi][:3, :3].T + T_b[fi][:3, 3]
		for X, Y, rfy in ((Xa, Xb, ctx._rf(b2)), (Xb, Xa, ctx._rf(b1))):
			C, t, e1, e2, r1, r2 = _alb._envelopes(X, ctx._rf(b1) if X is Xa else ctx._rf(b2))
			K = len(C); k0 = max(1, int(round(0.08 * (K - 1))))
			P = Y[:, 1:-1].reshape(-1, 3)
			k = cKDTree(C).query(P)[1]
			q = P - C[k]; al = np.einsum("ij,ij->i", q, t[k]); qp = q - al[:, None] * t[k]
			s = np.sqrt((np.einsum("ij,ij->i", qp, e1[k]) / (r1[k] + rfy)) ** 2 + (np.einsum("ij,ij->i", qp, e2[k]) / (r2[k] + rfy)) ** 2)
			ok = (k >= k0) & (k <= K - 1 - k0) & (np.abs(al) <= 0.75 * np.linalg.norm(np.diff(C, axis=0), axis=1).mean())
			n += int(((s < 0.9) & ok).sum())
	return n
ov_on = env_overlap()
def jumps():
	"""外形の中心線（21 点に取り直し）の 2 階差分が、骨そのものの動き（2点目の付着部の線維の端の重心）の 3 倍 + 0.5mm を超えるコマの数と最大。"""
	n_bad, worst = 0, 0.0
	for b in (b1, b2, b3):
		r = ctx.results[b["id"]]
		Cs, En = [], []
		for j in range(len(r["frames"])):
			X = r["local"][j].astype(float)
			C = _alb._envelopes(X, ctx._rf(b))[0]
			sl = np.r_[0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))]
			u = np.linspace(0, sl[-1], 21)
			Cs.append(np.stack([np.interp(u, sl, C[:, i]) for i in range(3)], axis=1)); En.append(X[:, -1].mean(axis=0))
		Cs, En = np.asarray(Cs), np.asarray(En)
		acc = np.linalg.norm(Cs[2:] - 2 * Cs[1:-1] + Cs[:-2], axis=2).max(axis=1)
		base = np.linalg.norm(En[2:] - 2 * En[1:-1] + En[:-2], axis=1)
		n_bad += int((acc > 3 * base + 0.5).sum()); worst = max(worst, float((acc - 3 * base).max()))
	return n_bad, worst
jb, jw = jumps()
check(jb == 0, f"当たりありでも外形が跳ばない（骨の動きの 3 倍 + 0.5mm を超えるコマ {jb}・最大の超え {jw:.2f} mm）")
# 当たりを切る → 結果が外れる・鍵が変わる・保存
cv[0].invoke(); pump(app, 0.3)
check(not bool(ctx.data["params"]["bundle_contact"]) and all(ctx.results.get(b["id"]) is None for b in (b1, b2))
      and json.load(open(ctx.store, encoding="utf-8"))["params"]["bundle_contact"] is False, "当たりを切ると計算の結果が外れる（保存）")
check(ctx._key([b1, b2, b3]) != key_on, "当たりの有無は計算の鍵に入る")
ctx.compute(parent=ctx.win); pump(app, 0.3)
ov_off = env_overlap()
check(ov_on < 0.5 * ov_off or (ov_off == 0 and ov_on == 0), f"当たりで外形の重なりが減る（当たりなし {ov_off} 点 → あり {ov_on} 点）")
cv[0].invoke(); pump(app, 0.3)
# 自然長の基準のコマ
fi_ref = ctx.n_view // 2
v["ctrl"].get("seek", lambda f: None)
ctx.update(fi_ref, render=True); pump(app, 0.2)
btns["いまのコマを基準にする"].invoke(); pump(app, 0.3)
g = ctx._rest_frame()
check(g == ctx.s0 + fi_ref and lbl.cget("text").startswith(f"コマ {fi_ref}") and "手で選んだ" in lbl.cget("text"), f"いまのコマを基準にする（{lbl.cget('text')}）")
rf_saved = json.load(open(ctx.store, encoding="utf-8"))["params"].get("rest_frames") or {}
check(rf_saved == {ctx._rec_id(): g}, f"基準のコマは記録ごとに覚える（{rf_saved}）")
check(all(ctx.results.get(b["id"]) is None for b in (b1, b2)), "基準を変えると計算の結果が外れる")
ctx.compute(parent=ctx.win); pump(app, 0.3)
r1_ = ctx.results[b1["id"]]
bones = ctx._ensure_bones()
Ts = {int(i): np.asarray(T, float) for (i, n_, m_, T) in ctx.bones_full}
rt = ctx._runtime(b1)
A = (Ts[rt["a"]][g][:3, :3] @ rt["pa"][0] + Ts[rt["a"]][g][:3, 3])[None]
B = (Ts[rt["b"]][g][:3, :3] @ rt["pb"][0] + Ts[rt["b"]][g][:3, 3])[None]
X = _alb._taut_paths(bones, Ts, g, A, B, 33, np.array([rt["rf"]]), maxit=3000)
Lg = float(np.linalg.norm(np.diff(X[0], axis=0), axis=1).sum())
check(abs(float(r1_["L0"][0]) - Lg * float(ctx.params()["rest_scale"])) < 0.05, f"自然長＝基準のコマの長さ（自然長 {float(r1_['L0'][0]):.2f}・そのコマ {Lg:.2f} mm）")
btns["自動（初期状態）に戻す"].invoke(); pump(app, 0.3)
check(ctx._rest_manual() is None and ctx._rest_frame() == ctx._auto_rest() and "手で選んだ" not in lbl.cget("text"), f"自動に戻す（{lbl.cget('text')}）")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
