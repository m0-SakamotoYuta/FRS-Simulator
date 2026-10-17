# -*- coding: utf-8 -*-
"""靭帯を骨に沿わせる（段階1。2026-10-16）。
⑤のチェック・開く前に計算・折れ線で描く・骨の中を通らない・骨に当たらない靭帯は直線と同じ長さ・
保存して次は計算しない・管理画面で切り替え・半径を変えると計算し直し・右のグラフと比べる画面も沿わせた長さ。
接触の計算はダミー。3本・1試験の計算に数十秒かかる。"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, open3d as o3d
from vtk.util.numpy_support import vtk_to_numpy
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
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a[:2])[:200])
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
def fake_multi(bd, update_progress=None, cancel_var=None, quiet=False):
	N = len(bd[0][3])
	return [{idx: np.ones(len(np.asarray(m.points)), np.float32) for (idx, n, m, T) in bd} for t in range(N)]
app._sim_view_precompute_multi_heatmap = fake_multi
solves = {"n": 0}
_orig_solve = app._ankle_lig_wrap_solve
def counting_solve(*a, **k):
	solves["n"] += 1
	return _orig_solve(*a, **k)
app._ankle_lig_wrap_solve = counting_solve
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)
def close(v):
	w = v["ctrl"]["window"]; w.tk.call(w.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)

TAB = "261007_FE_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
# 靭帯は 3 本に絞る（時間を短く）。サンドボックスの状態なので本物には書かない
keep = []
for nm in ("ACL", "PCL", "靭帯6"):
	it = next((x for x in app.ankle_ligaments if x.get("name") == nm), None)
	if it is not None:
		keep.append(it)
app.ankle_ligaments[:] = keep
print("   靭帯:", [x["name"] for x in keep], flush=True)
# このテストの前の結果を消す
import shutil
shutil.rmtree(app._ankle_lig_wrap_dir(), ignore_errors=True)
cbs = [w for w in walk(app) if w.winfo_class() == "TCheckbutton" and str(w.cget("variable")) == str(app.ankle_lig_wrap)]
check(len(cbs) == 1, "⑤に「靭帯を骨に沿わせる」のチェック")
app.ankle_lig_wrap.set(True); app.ankle_start_at_load.set(True)

t0 = time.time()
app.on_ankle_animate(); pump(app, 1.0)
t_first = time.time() - t0
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
check(solves["n"] == 1, f"開く前に 1 回だけ計算した（{t_first:.0f} 秒）")
eng = None
# エンジンの靭帯（_ligs）を取り出す
import gc
for o in gc.get_objects():
	if isinstance(o, dict) and "_ligs" in o and "update" in o and "open" in o:
		eng = o
ligs = eng["_ligs"]
check(all(l.get("wrap_res") is not None for l in ligs), "全部の靭帯に沿わせた経路がある")
fi = 300
v["seek"](float(fi) / 15.0); pump(app, 0.3)
from vtk.util.numpy_support import vtk_to_numpy as v2n
npts = [l["line"].GetNumberOfPoints() for l in ligs]
check(all(n == app._LIG_WRAP_K for n in npts), f"折れ線で描く（点の数 {npts}）")
# 骨の中を通らない（描いている点で、骨の面までの符号つき距離 ≥ −0.05mm。両端を除く）
acts = p.renderer.actors
bone_keys = [k for k in acts.keys() if k.startswith("bone_") and k.count("_") == 1]
def world_mesh(k):
	a = acts[k]; m = a.GetUserMatrix(); M = np.array([[m.GetElement(r, c) for c in range(4)] for r in range(4)])
	import pyvista as pv
	tri = pv.wrap(a.GetMapper().GetInput()).triangulate()
	P = np.asarray(tri.points, float) @ M[:3, :3].T + M[:3, 3]
	F = np.asarray(tri.faces).reshape(-1, 4)[:, 1:].astype(np.int32)
	tri2 = pv.PolyData(P, np.hstack([np.full((len(F), 1), 3), F]).ravel())
	nm = tri2.compute_normals(cell_normals=True, point_normals=False, consistent_normals=True, auto_orient_normals=True)
	sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(P.astype(np.float32)), o3d.core.Tensor(F)))
	return sc, np.asarray(nm.cell_data["Normals"], float)
scs = [world_mesh(k) for k in bone_keys]
worst = []
for l in ligs:
	pts = v2n(l["line"].GetPoints().GetData()).astype(float)[1:-1]
	md = np.inf
	for sc, tn in scs:
		cp = sc.compute_closest_points(o3d.core.Tensor(pts.astype(np.float32)))
		d = pts - cp["points"].numpy(); s = np.einsum("ij,ij->i", d, tn[cp["primitive_ids"].numpy().astype(np.int64)])
		sd = np.where(s >= 0, np.linalg.norm(d, axis=1), -np.linalg.norm(d, axis=1))
		md = min(md, float(sd.min()))
	worst.append(md)
check(all(w > -0.05 for w in worst), f"沿わせた靭帯は骨の中を通らない（骨までの最小の符号つき距離 {np.round(worst, 2).tolist()} mm）")
# 骨に当たらない靭帯は直線と同じ長さ
straight = []
for l in ligs:
	wa = l["sa"][0].GetCenter(); wb = l["sb"][0].GetCenter()
	straight.append(float(np.linalg.norm(np.subtract(wa, wb))))
wl = [float(l["lengths"][fi]) for l in ligs]
check(all(w >= s - 0.02 for w, s in zip(wl, straight)), f"沿わせた長さ ≥ 直線（{np.round(wl, 2).tolist()} / 直線 {np.round(straight, 2).tolist()}）")
# 右のグラフも沿わせた長さ（スクリプトから同じ口で取り出して比べる）
sc_ = None
for o in gc.get_objects():
	if isinstance(o, dict) and "_lig_wrap" in o and "frame_times" in o:
		sc_ = o
prov = sc_["_lig_wrap"]
r0 = prov["get"](ligs[0]["a_bone"], ligs[0]["a_pt"], ligs[0]["b_bone"], ligs[0]["b_pt"], ligs[0]["radius"])
check(r0 is not None and np.allclose(r0["len"], ligs[0]["lengths"]), "右のグラフに渡す長さ＝靭帯の長さ（沿わせたもの）")
# 管理画面で切り替え
v["ctrl"]["window"]; eng["open"](); pump(app, 0.4)
st = eng["_st"]
check(st.get("wrap_var") is not None and bool(st["wrap_var"].get()), "靭帯管理の画面に「骨に沿わせる」（いまはオン）")
wcb = [w for w in walk(st["win"]) if w.winfo_class() == "TCheckbutton" and "沿わせる" in str(w.cget("text"))][0]
wcb.invoke(); pump(app, 0.3)          # オン → オフ
check(not app.ankle_lig_wrap.get() and all(l["line"].GetNumberOfPoints() == 2 for l in ligs), "オフにすると直線に戻る（タブにも覚える）")
n0 = solves["n"]
wcb.invoke(); pump(app, 0.5)          # オフ → オン
check(app.ankle_lig_wrap.get() and solves["n"] == n0 and all(l.get("wrap_res") is not None for l in ligs),
      "オンに戻すと、保存済みの結果で沿わせる（計算し直さない）")
# 半径を変えると、その靭帯は計算し直しが要る
l0 = ligs[0]; vars_ = l0["row"]["vars"]; vars_[3].set("2.5"); pump(app, 0.3)
check(l0.get("wrap_res") is None and l0["line"].GetNumberOfPoints() == 2, "半径を変えると、その靭帯は直線（計算し直しが要る）")
lbl = st.get("wrap_lbl")
check(lbl is not None and "2 / 3" in str(lbl.cget("text")), f"状態の表示（{lbl.cget('text') if lbl else ''}）")
btn = [w for w in walk(st["win"]) if w.winfo_class() == "TButton" and "まだの靭帯を計算" in str(w.cget("text"))][0]
btn.invoke(); pump(app, 0.5)
check(solves["n"] == n0 + 1 and l0.get("wrap_res") is not None, "「まだの靭帯を計算」で、その靭帯だけ計算する")
vars_[3].set("1.5"); pump(app, 0.3)
close(v)
# 次に開くと計算しない
n1 = solves["n"]; t0 = time.time()
app.on_ankle_animate(); pump(app, 1.0)
v = app._sim_viewers_alive()[-1]
check(solves["n"] == n1, f"次に開くときは計算しない（保存した結果。{time.time() - t0:.1f} 秒）")
# 比べる画面の長さ（全コマ）＝可視化（軸力から始めた分ずらす）
eng2 = None
for o in gc.get_objects():
	if isinstance(o, dict) and "_ligs" in o and "update" in o and o is not eng:
		eng2 = o
lv = eng2["_ligs"][0]["lengths"]
res = app._ankle_lig_lengths_current_tab()
check(res.get("wrapped") is True and solves["n"] == n1, "比べる画面も沿わせた長さ（保存した結果を使い、計算しない）")
Lc = res["lengths"][eng2["_ligs"][0]["name"]]
s0 = len(Lc) - len(lv)
check(s0 >= 0 and np.allclose(Lc[s0:], lv, atol=1e-4), f"比べる画面の長さ＝可視化の長さ（軸力から始めた {s0} コマをずらすと一致）")
close(v)
app.ankle_lig_wrap.set(False)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
