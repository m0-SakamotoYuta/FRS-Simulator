# -*- coding: utf-8 -*-
"""靭帯の束（段階2・別ファイル ankle_lig_bundles.py。2026-10-16）。
⑤の段階1/段階2・段階2のときは段階1の管を隠す・付着部の点のファイルを目安に表示・外周を打つ（取り消し・打ち直し・
ほかの骨は断る）・楕円を当てはめる・計算（たるむ・骨に入らない・線維どうし重ならない）・右のグラフ・保存して次は計算しない・
段階1に戻せる・段階2のファイルが壊れても段階1は動く。接触の計算はダミー。数分かかる。"""
import importlib.util, os, sys, time, json, glob, shutil
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, gc
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
warns = []
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: (warns.append(a), print("   [warning]", str(a[:2])[:300]))
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
def close(v):
	w = v["ctrl"]["window"]; w.tk.call(w.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
def ligs_of(plotter):
	"""いま開いている可視化の、点と点の靭帯（段階1）。"""
	col = plotter.renderer.GetActors(); col.InitTraversal()
	acts = set()
	for _k in range(col.GetNumberOfItems()):
		acts.add(col.GetNextActor().GetAddressAsString("vtkActor"))
	for o in gc.get_objects():
		try:
			if isinstance(o, dict) and "_ligs" in o and "update" in o and o["_ligs"] 					and o["_ligs"][0]["actor"].GetAddressAsString("vtkActor") in acts:
				return o
		except Exception:
			pass
	return None
def latest(pred):
	hit = None
	for o in gc.get_objects():
		try:
			if pred(o): hit = o
		except Exception:
			pass
	return hit

TAB = "261007_FE_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
shutil.rmtree(os.path.join(HERE, "cache", "ankle_lig_bundles"), ignore_errors=True)
rb = [w for w in walk(app) if w.winfo_class() == "TRadiobutton" and str(w.cget("variable")) == str(app.ankle_lig_mode)]
check(len(rb) == 2, "⑤に「靭帯の表し方: 段階1 / 段階2」")
app.ankle_lig_wrap.set(False); app.ankle_start_at_load.set(True)
# 段階1の靭帯（点と点）は 2 本だけ見えるようにしておく
for it in app.ankle_ligaments:
	it["visible"] = it["name"] in ("ACL", "PCL")
app.ankle_lig_mode.set("stage2")
app.on_ankle_animate(); pump(app, 1.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
btns = [w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TButton" and "靭帯の束（段階2）" in str(w.cget("text"))]
check(len(btns) == 1, "再生コントロールに「靭帯の束（段階2）…」")
ctx = latest(lambda o: type(o).__name__ == "Stage2")
lig1 = ligs_of(v["plotter"])
check(ctx is not None and all(not l["actor"].GetVisibility() for l in lig1["_ligs"]), "段階2のときは、点と点の靭帯（段階1）の管を描かない")
btns[0].invoke(); pump(app, 0.5)
check(ctx.win is not None and ctx.win.winfo_exists(), "靭帯の束の画面が開く")
# 付着部の点のファイル（バックアップ）を目安に
bk = sorted(glob.glob(os.path.join(r"C:\Users\yutas\Desktop\ankle simulator用ファイル\261007_ブタ膝ACL切除前後のアルコマーカーによる可視化試験", "靭帯の付着部_バックアップ_*.json")))
check(len(bk) >= 1, f"付着部の点のバックアップのファイルがある（{os.path.basename(bk[-1]) if bk else ''}）")
import tkinter.filedialog as _fd
_fd.askopenfilename = lambda *a, **k: bk[-1]
[w for w in walk(ctx.win) if w.winfo_class() == "TButton" and "付着部の点のファイル" in str(w.cget("text"))][0].invoke(); pump(app, 0.5)
check(len(ctx.guides) >= 10, f"付着部の点を目安に表示（{len(ctx.guides) // 2} 点）")
# 束を 2 つ（ACL のまわり・PCL のまわり）
G = json.load(open(bk[-1], encoding="utf-8"))["ligaments"]
def ring(si, center, ra, rb_, n=7, ang=0.3):
	bones = ctx._ensure_bones(); idx = ctx.idx_of[si]
	cpp, ns, _ = bones.closest(idx, np.asarray(center, float)[None]); nrm = ns[0]
	t1 = np.cross(nrm, [1, 0, 0]); t1 /= np.linalg.norm(t1); t2 = np.cross(nrm, t1)
	th = np.linspace(0, 2 * np.pi, n, endpoint=False) + ang
	P = cpp[0] + np.outer(np.cos(th) * ra, t1) + np.outer(np.sin(th) * rb_, t2) + 2 * nrm
	return bones.closest(idx, P)[0]
si_f = ctx.names.index("大腿骨"); si_t = ctx.names.index("脛骨")
for nm, gname in (("ACL(試験)", "ACL"), ("PCL(試験)", "PCL")):
	g = next(x for x in G if x["name"] == gname)
	[w for w in walk(ctx.win) if w.winfo_class() == "TButton" and "束を追加" in str(w.cget("text"))][0].invoke(); pump(app, 0.2)
	b = ctx.data["bundles"][-1]; b["name"] = nm; b["n_fibers"] = 6
	for end, si, key in (("a", si_f, "a"), ("b", si_t, "b")):
		ctx._ol_start(b, end); pump(app, 0.1)
		pts = ring(si, g[key]["point"], 3.0, 1.5)
		for q in pts[:3]:
			ctx.add_point(si, q)
		if nm.startswith("ACL") and end == "a":
			check(ctx.add_point(si_t if si == si_f else si_f, pts[0]) is False, "ほかの骨の点は断る（最初の点と同じ骨だけ）")
			ctx.add_point(si, pts[0] + 5.0)               # 間違えた点
			ctx._ol_undo()
			check(len(ctx.st["mode"]["pts"]) == 3, "「最後の点を取り消す」")
			ctx.add_point(si, pts[0] + 3.0)               # 間違えた点 → 打ち直す
			ctx.w["ol_list"].selection_clear(0, "end"); ctx.w["ol_list"].selection_set(3); ctx._ol_replace()
			ctx.add_point(si, pts[3])
			check(len(ctx.st["mode"]["pts"]) == 4 and np.allclose(ctx.st["mode"]["pts"][3], pts[3]), "「選んだ点を打ち直す」")
			for q in pts[4:]:
				ctx.add_point(si, q)
		else:
			for q in pts[3:]:
				ctx.add_point(si, q)
		ctx._ol_commit(); pump(app, 0.2)
	for end in ("a", "b"):
		e = b[end].get("ellipse", {}); O = np.asarray(b[end]["outline"], float) - np.asarray(e["center"], float)
		val = ((O @ np.asarray(e["u"])) / e["ra"]) ** 2 + ((O @ np.asarray(e["v"])) / e["rb"]) ** 2
		check(np.mean(np.abs(np.sqrt(val) - 1.0)) < 0.15,
		      f"{nm} {end}: 打った外周の点が、当てはめた楕円の上にある（{2 * e['ra']:.1f}×{2 * e['rb']:.1f} mm・ずれ {np.mean(np.abs(np.sqrt(val) - 1.0)):.3f}）")
# 平らな楕円の上の点なら、ちょうどその楕円になる
fit_ellipse = type(ctx).__init__.__globals__["fit_ellipse"]
th = np.linspace(0, 2 * np.pi, 6, endpoint=False) + 0.4
u0 = np.array([0.6, 0.8, 0.0]); v0 = np.array([0.0, 0.0, 1.0]); c0 = np.array([5.0, -3.0, 610.0])
ee = fit_ellipse(c0 + np.outer(4.0 * np.cos(th), u0) + np.outer(1.25 * np.sin(th), v0))
check(abs(2 * ee["ra"] - 8.0) < 1e-3 and abs(2 * ee["rb"] - 2.5) < 1e-3 and abs(abs(np.dot(ee["u"], u0)) - 1) < 1e-6,
      f"楕円の当てはめ（平らな 8×2.5 の楕円の 6 点 → {2 * ee['ra']:.3f}×{2 * ee['rb']:.3f}）")
check(all(ctx._ready(b) for b in ctx.data["bundles"]), "2 束とも両方の面がある")
check(os.path.exists(ctx.store), "束の定義はファイルに保存（cache/ankle_lig_bundles/）")
# 計算
t0 = time.time()
[w for w in walk(ctx.win) if w.winfo_class() == "TButton" and "計算する" in str(w.cget("text"))][0].invoke(); pump(app, 0.5)
check(len(ctx.results) == 2, f"計算した（{time.time() - t0:.0f} 秒）")
# いくつかのコマで: 骨に入らない・たるみの長さ・線維どうし
import open3d as o3d
from scipy.spatial import cKDTree
worst_bone, worst_pair, slack_ratio = [], [], []
for fi in (0, 300, 900, 1500):
	ctx.update(fi)
	for b in ctx.data["bundles"]:
		a = ctx.actors[b["id"]]; r = a["r"]
		T = ctx.poses[r["sia"]][fi]; W = ctx._local_at(r, fi) @ T[:3, :3].T + T[:3, 3]
		# 骨まで（両端を除く）
		Q = W[:, 1:-1].reshape(-1, 3); md = np.inf
		bones = ctx._ensure_bones()
		for si in range(len(ctx.names)):
			Tb = ctx.poses[si][fi]; q = (Q - Tb[:3, 3]) @ Tb[:3, :3]
			_c, _n, sd = bones.closest(ctx.idx_of[si], q); md = min(md, float(sd.min()))
		worst_bone.append(md - a["rf"])
		# 線維の長さ / 自然長（たるんでいる線維）
		arc = np.linalg.norm(np.diff(W, axis=1), axis=2).sum(1)
		chord = np.linalg.norm(W[:, -1] - W[:, 0], axis=1)
		sl = chord < r["L0"] - 0.3
		if sl.any():
			slack_ratio += list(arc[sl] / r["L0"][sl])
	# 線維どうし（別の線維、両端を除く）
	allW = []; fid = []
	for j, b in enumerate(ctx.data["bundles"]):
		a = ctx.actors[b["id"]]; r = a["r"]; T = ctx.poses[r["sia"]][fi]
		W = ctx._local_at(r, fi) @ T[:3, :3].T + T[:3, 3]
		allW.append(W[:, 2:-2].reshape(-1, 3)); fid += list(np.repeat(np.arange(W.shape[0]) + 100 * j, W.shape[1] - 4))
	P = np.concatenate(allW); fid = np.array(fid)
	pr = cKDTree(P).query_pairs(2.0, output_type="ndarray"); pr = pr[fid[pr[:, 0]] != fid[pr[:, 1]]] if len(pr) else pr
	if len(pr):
		worst_pair.append(float(np.linalg.norm(P[pr[:, 0]] - P[pr[:, 1]], axis=1).min()))
check(min(worst_bone) > -0.15, f"線維は骨に入らない（骨の面から半径を引いた距離の最小 {min(worst_bone):.2f} mm）")
check(not worst_pair or min(worst_pair) > 0.55, f"線維どうしは重ならない（別の線維の点どうしの最小 {min(worst_pair) if worst_pair else 'なし'} mm、太さ 0.8）")
check(len(slack_ratio) == 0 or abs(np.median(slack_ratio) - 1.0) < 0.05,
      f"たるんだ線維は伸びない（長さ/自然長 中央値 {np.median(slack_ratio) if slack_ratio else 'たるみなし'}・{len(slack_ratio)} 本）")
ser = ctx.series()
check(len(ser) == 2 and all(np.all(np.isfinite(L)) for _t, L, _c, _n in ser), f"右のグラフに 2 束の中心の線維の長さ（{[round(float(L[0]), 2) for _t, L, _c, _n in ser]} mm）")
try:
	p.screenshot(os.path.join(HERE, "bundles_view.png"))
except Exception as e:
	print("   スクリーンショット失敗", e)
ctx._close_window(); close(v)
# 次に開くと、保存した結果を使う（計算しない）
app.on_ankle_animate(); pump(app, 1.0)
v = app._sim_viewers_alive()[-1]
ctx2 = latest(lambda o: type(o).__name__ == "Stage2" and o is not ctx)
check(ctx2 is not None and len(ctx2.results) == 2, "次に開くと、保存した結果で束を表示（計算しない）")
close(v)
# 段階1に戻す（見えるはずの 2 本を表示にしておく。本物の状態では隠してある）
app.ankle_lig_mode.set("stage1")
for it in app.ankle_ligaments:
	it["visible"] = it["name"] in ("ACL", "PCL")
app.on_ankle_animate(); pump(app, 1.0)
v = app._sim_viewers_alive()[-1]
btns = [w for w in walk(v["ctrl"]["window"]) if w.winfo_class() == "TButton" and "靭帯の束（段階2）" in str(w.cget("text"))]
lig1 = ligs_of(v["plotter"])
vis = {l["name"]: bool(l["actor"].GetVisibility()) for l in lig1["_ligs"]}
check(not btns and vis.get("ACL") and vis.get("PCL"), f"段階1に戻すと、束のボタンは無く、点と点の靭帯が見える（{vis}）")
close(v)
# 段階2のファイルが壊れていても、段階1は動く
mod_fp = os.path.join(HERE, "ankle_lig_bundles.py"); bak = mod_fp + ".bak"
shutil.copy2(mod_fp, bak)
open(mod_fp, "a", encoding="utf-8").write("\nraise RuntimeError('テスト用に壊した')\n")
try:
	app.ankle_lig_mode.set("stage2"); warns.clear()
	app.on_ankle_animate(); pump(app, 1.0)
	v = app._sim_viewers_alive()[-1]
	lig1 = ligs_of(v["plotter"])
	check(any("段階2を準備できませんでした" in str(w) for w in warns) and lig1 is not None and len(lig1["_ligs"]) > 0
	      and any(l["actor"].GetVisibility() for l in lig1["_ligs"]),
	      "段階2のファイルが壊れても、知らせて、段階1の靭帯はそのまま表示される")
	close(v)
finally:
	shutil.move(bak, mod_fp)
app.ankle_lig_mode.set("stage1")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
