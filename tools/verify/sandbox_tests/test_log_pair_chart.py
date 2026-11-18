# -*- coding: utf-8 -*-
"""datalog と 2点の距離を並べたグラフ（2026-11-17）:
⑤の「datalog と並べたグラフ…」は datalog が入っているタブだけ押せる・datalog が無ければ断る／
261007_AP_FE60_軸力100 に脛骨と大腿骨のいちばん近い所の 2 点の組を（画面の中だけで）置き、
2点の距離は姿勢から独立に計算した値と一致・datalog の時刻のずれは「靭帯の長さを比べる」と同じ・区間（前方引張の行き）で
AP がほぼ増える一方・横軸 AP／縦軸 2点の初期値・赤い線（クリックの位置にいちばん近いコマ・← → で 1 コマ）・値の文字が配列と一致・
区間を変えると描き直す・CSV（行数・値）・PNG／2 点の量は「骨A に対する点B の移動距離」だけ（2026-11-18）。状態ファイル・キャッシュは書かない。2〜3 分。"""
import importlib.util, os, sys, time, tempfile, csv
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
frs.messagebox.showinfo = lambda *a, **k: msgs.append(("info", a[:2]))
frs.messagebox.showwarning = lambda *a, **k: msgs.append(("warning", a[:2]))
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._schedule_state_autosave = lambda *a, **k: None
app._save_ankle_state = lambda *a, **k: None
TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
btn = app._ankle_log_pair_btn
check(str(btn.cget("state")) == "normal", f"datalog のあるタブではボタンが押せる（{btn.cget('state')}）")
keep_log = app.ankle_robot_log_path.get()
app.ankle_robot_log_path.set(""); pump(app, 0.1)
check(str(btn.cget("state")) == "disabled", "datalog を外すとボタンは押せない")
msgs.clear(); n_top = len([w for w in app.winfo_children() if isinstance(w, tk.Toplevel)])
app._ankle_open_log_pair_chart(); pump(app, 0.3)
check(msgs and "datalog が入っていない" in str(msgs[-1][1][1]) and
      len([w for w in app.winfo_children() if isinstance(w, tk.Toplevel)]) == n_top, "datalog が無いと理由を出して開かない")
app.ankle_robot_log_path.set(keep_log); pump(app, 0.1)
check(str(btn.cget("state")) == "normal", "datalog を戻すと押せる")

# 2点の組を置く（脛骨と大腿骨のいちばん近い頂点どうし・最初のコマ）
cache = app._ankle_cache_for_current_size(app._ankle_get_current_cache())
anim, N, _w = app._ankle_build_bone_transforms(cache)
names = {i: str(app.ankle_bones[i].get("name", "")) for (i, m, T) in anim}
it = next(i for i in names if "脛骨" in names[i]); jf = next(i for i in names if "大腿骨" in names[i])
mt = next(m for (i, m, T) in anim if i == it); mf = next(m for (i, m, T) in anim if i == jf)
Tt = np.asarray(next(T for (i, m, T) in anim if i == it)); Tf = np.asarray(next(T for (i, m, T) in anim if i == jf))
vt = np.asarray(mt.points); vf = np.asarray(mf.points)
wt = vt @ Tt[0, :3, :3].T + Tt[0, :3, 3]; wf = vf @ Tf[0, :3, :3].T + Tf[0, :3, 3]
from scipy.spatial import cKDTree
dd, jj = cKDTree(wf).query(wt[::5]); k = int(np.argmin(dd)); ia_v, ib_v = 5 * k, int(jj[k])
pa = app._ankle_scan_point(app.ankle_bones[it], vt[ia_v], to_scaled=False)
pb = app._ankle_scan_point(app.ankle_bones[jf], vf[ib_v], to_scaled=False)
pairs_keep = app.ankle_bones[it].get("disp_pairs")
app.ankle_bones[it]["disp_pairs"] = [{"id": "t1", "name": "脛骨–大腿骨（テスト）", "color": "#2ca02c", "visible": True,
                                      "point_a": [float(x) for x in pa], "bone_b": names[jf], "point_b": [float(x) for x in pb]}]
print(f"   2点: 脛骨 {np.round(vt[ia_v], 1)} ↔ 大腿骨 {np.round(vf[ib_v], 1)}（最初のコマの距離 {dd[k]:.2f} mm）", flush=True)

t0 = time.time()
S = app._ankle_log_pair_series()
print(f"   値を作るのに {time.time() - t0:.1f} 秒・{len(S['times'])} コマ・ずれ {S['offset']:.2f} 秒", flush=True)
check(len(S["pairs"]) == 1 and len(S["pairs"][0]["dist"]) == len(S["times"]) and np.isfinite(S["pairs"][0]["dist"]).all(),
      "2点の距離が全コマで出る")
check(abs(float(S["pairs"][0]["dist"][0]) - float(dd[k])) < 1.0, f"最初のコマの距離 {S['pairs'][0]['dist'][0]:.2f} mm（置いた所 {dd[k]:.2f} mm・平滑化の差 1 mm 以内）")
L = app._ankle_lig_lengths_current_tab()
check(L["offset"] is not None and abs(float(L["offset"]) - S["offset"]) < 1e-9, f"datalog のずれは「靭帯の長さを比べる」と同じ（{L['offset']} / {S['offset']}）")
# 独立に計算した距離（平滑化した姿勢を同じ関数で作り直して）
bd = [(i, names[i], m, np.asarray(T, float)) for (i, m, T) in anim]
fps = app._ankle_fps_of(S["times"])
det = {i: np.asarray(cache["bones"][int(app.ankle_bones[i]["aruco_id"])]["detected"], bool) for i in names}
bd2, _ = app._ankle_smooth_pose_series(bd, cutoff_hz=float(app.ankle_smooth_cutoff_hz.get()), frame_rate_hz=fps,
                                       detected_map=det, reject_outliers=bool(app.ankle_reject_outliers.get()))
T2 = {i: np.asarray(T) for (i, nm, m, T) in bd2}
qa = app._ankle_scan_point(app.ankle_bones[it], pa, to_scaled=True); qb = app._ankle_scan_point(app.ankle_bones[jf], pb, to_scaled=True)
d_ind = np.linalg.norm((np.einsum("nij,j->ni", T2[jf][:, :3, :3], qb) + T2[jf][:, :3, 3])
                       - (np.einsum("nij,j->ni", T2[it][:, :3, :3], qa) + T2[it][:, :3, 3]), axis=1)
check(np.max(np.abs(d_ind - S["pairs"][0]["dist"])) < 1e-9, "2点の距離は、平滑化した姿勢から独立に計算した値と一致")
ap = S["log"]["AP"]; rr = app._ankle_log_ranges(S["times"], S["log"])
print("   区間:", {k: v for k, v in rr.items()}, flush=True)
check("anterior" in rr and "posterior" in rr and "load" in rr, "区間: 前方引張・後方引張・軸力からが求まる")
a, b = rr["anterior"]
seg = ap[a:b + 1]
check(np.nanmax(seg) - seg[0] > 1.0 and np.mean(np.diff(seg) >= -0.05) > 0.9,
      f"前方引張（行き）の区間で AP はほぼ増える一方（{seg[0]:.2f} → {np.nanmax(seg):.2f} mm）")
fap = S["log"]["Fap"]
check(abs(fap[a]) < 5 and fap[b] > np.nanmax(fap) - 6, f"前方引張の始め Fap {fap[a]:.1f} N・終わり {fap[b]:.1f} N（最大 {np.nanmax(fap):.1f}）")

# 画面
before = set(app.winfo_children())
app._ankle_open_log_pair_chart(); pump(app, 1.0)
win = [w for w in app.winfo_children() if w not in before and isinstance(w, tk.Toplevel)][0]
G = win._log_pair_state
check(G["x_var"].get() == "ロボット AP" and (G["y_var"].get().startswith("#1 ") and "点B の移動距離" in G["y_var"].get()), f"初期: 横軸 {G['x_var'].get()}・縦軸 {G['y_var'].get()}")
check(G["r_var"].get() == "前方引張（行き）", f"初期の区間は前方引張（行き）: {G['r_var'].get()}")
st = G["st"]
check(st["a"] == a and st["b"] == b and abs(st["y"][0]) < 1e-12, "区間の始めのコマで 点B の移動距離が 0")
check(abs(st["x"][0]) < 1e-12 and abs(np.nanmax(st["x"]) - (np.nanmax(seg) - seg[0])) < 1e-9,
      f"横軸の AP は区間の始めを 0 にした前方変位（最大 {np.nanmax(st['x']):.2f} mm）")
G["rel_var"].set(False); pump(app, 0.3)
check(abs(st["x"][0] - seg[0]) < 1e-9, f"チェックを外すと AP の絶対値（{st['x'][0]:.2f} mm）")
G["rel_var"].set(True); pump(app, 0.3)
xm = float(np.nanmin(st["x"]) + 0.6 * (np.nanmax(st["x"]) - np.nanmin(st["x"])))
G["pick"](xm); pump(app, 0.2)
kk = st["k"]; j = kk - st["a"]
d_all = np.abs(st["x"] - xm)
check(abs(st["x"][j] - xm) <= np.nanmin(d_all) + 0.004 * (np.nanmax(st["x"]) - np.nanmin(st["x"])) + 1e-12,
      f"クリックの位置 {xm:.3f} mm にいちばん近いコマ {kk}（AP {st['x'][j]:.3f}）")
txt = G["info"].get()
check(f"{st['x'][j]:+.3f}" in txt and f"{st['y'][j]:+.3f}" in txt and f"コマ {kk}" in txt, f"値の文字が配列と一致: {txt}")
lx = G["st"]["line"].get_xdata()
check(abs(float(lx[0]) - st["x"][j]) < 1e-12, "赤い縦線がそのコマの横軸の位置にある")
from matplotlib.backend_bases import KeyEvent
cv_ = G["fig"].canvas
def key(k):
	ev = KeyEvent("key_press_event", cv_, k); cv_.callbacks.process("key_press_event", ev)
key("right"); pump(app, 0.2)
check(st["k"] == kk + 1, f"→ キーで 1 コマ進む（{kk} → {st['k']}）")
key("left"); key("left"); pump(app, 0.2)
check(st["k"] == kk - 1, f"← キーで 1 コマ戻る（{st['k']}）")
G["r_var"].set("全体"); pump(app, 0.5)
check(st["a"] == 0 and st["b"] == len(S["times"]) - 1, "区間を全体にすると描き直す")
G["x_var"].set("映像の時刻"); G["y_var"].set("ロボット Fap"); pump(app, 0.5)
check(abs(st["x"][0]) < 1e-12 and np.nanmax(np.abs(st["y"])) > 50, "横軸 時刻・縦軸 Fap にも切り替えられる")
G["x_var"].set("ロボット AP"); G["y_var"].set([q[0] for q in G["Q"] if q[3] == "fn"][0]); G["r_var"].set("前方引張（行き）"); pump(app, 0.5)
tmp = tempfile.mkdtemp()
frs.filedialog.asksaveasfilename = lambda **k: os.path.join(tmp, k.get("initialfile", "x"))
for w in (win,):
	pass
def walk(w):
	yield w
	for c in w.winfo_children():
		yield from walk(c)
bt = {c.cget("text"): c for c in walk(win) if c.winfo_class() == "TButton"}
bt["CSV を保存…"].invoke(); bt["画像（PNG）を保存…"].invoke(); pump(app, 0.5)
pcsv = os.path.join(tmp, "datalog_2点の距離.csv"); ppng = os.path.join(tmp, "datalog_2点の距離.png")
rows = list(csv.reader(open(pcsv, encoding="utf-8-sig")))
check(len(rows) == (b - a + 1) + 1 and rows[0][2].startswith("ロボット AP"), f"CSV: 見出し＋{len(rows) - 1} 行（区間 {b - a + 1} コマ）")
check(abs(float(rows[1 + j][3]) - st["y"][j]) < 1e-4, "CSV の値がグラフと一致")
check(os.path.exists(ppng) and os.path.getsize(ppng) > 10000, f"PNG を保存（{os.path.getsize(ppng) if os.path.exists(ppng) else 0} バイト）")
win.destroy()
# ---- 点B の移動距離（2026-11-18） ----
p0 = S["pairs"][0]
check(p0.get("pbA") is not None and p0.get("bone_a") == names[it], f"2 点の組に、骨A（{p0.get('bone_a')}）の座標での点B が入る")
pbA_ind = np.einsum("nij,j->ni", np.einsum("nij,njk->nik", np.linalg.inv(T2[it]), T2[jf])[:, :3, :3], qb) \
          + np.einsum("nij,njk->nik", np.linalg.inv(T2[it]), T2[jf])[:, :3, 3]
check(np.max(np.abs(pbA_ind - p0["pbA"])) < 1e-9, "骨A の座標での点B は、平滑化した姿勢から独立に計算した値と一致")
before = set(app.winfo_children())
app._ankle_open_log_pair_chart(); pump(app, 1.0)
win = [w for w in app.winfo_children() if w not in before and isinstance(w, tk.Toplevel)][0]
G = win._log_pair_state
ks = [q[0] for q in G["Q"] if q[0].startswith("#")]
print("   2点の選択肢:", ks, flush=True)
check(len(ks) == 1 and "点B の移動距離" in ks[0] and f"{names[it]}に対して" in ks[0],
      "2 点の量の選択肢は「点B の移動距離（骨A に対して）」だけ")
check(not any(w_ in q[0] for q in G["Q"] for w_ in ("開き", "ずれ", "回転", "距離の変化")),
      "開き・ずれ・回転・2点の距離の変化は選択肢に無い")
a, b = rr["anterior"]; idx = np.arange(a, b + 1)
mv = G["Q"][[q[0] for q in G["Q"]].index(ks[0])][2](idx)
check(abs(mv[0]) < 1e-12 and np.all(mv >= 0) and np.max(np.abs(mv - np.linalg.norm(pbA_ind[idx] - pbA_ind[a], axis=1))) < 1e-9,
      f"移動距離 = 区間の始めからの点B の動きの大きさ（独立に計算した値と一致・最大 {np.max(mv):.3f} mm）")
d_chg = S["pairs"][0]["dist"][idx] - S["pairs"][0]["dist"][a]
check(np.all(mv + 1e-9 >= np.abs(d_chg) - 0.05),
      f"移動距離は 2点の距離の変化以上（距離の変化 最大 {np.max(np.abs(d_chg)):.3f}・移動距離 最大 {np.max(mv):.3f} mm）")
win.destroy()
# 方向の偏りの確認（合成）: 点A と点B が 10 mm 離れ、点B を 2 点を結ぶ向きに垂直に 0.5 mm ずらす
qa_ = np.zeros(3); pb0 = np.array([0.0, 0.0, 10.0]); pb1 = pb0 + np.array([0.5, 0.0, 0.0])
dist_chg = np.linalg.norm(pb1 - qa_) - np.linalg.norm(pb0 - qa_)
check(dist_chg < 0.02 and abs(np.linalg.norm(pb1 - pb0) - 0.5) < 1e-12,
      f"横に 0.5 mm ずれると: 2点の距離の変化 {dist_chg:.4f} mm（ほぼ見えない）・点B の移動距離 0.5 mm")

app.ankle_bones[it]["disp_pairs"] = pairs_keep

print(f"\n{'すべて OK' if not fails else f'NG {len(fails)} 件'}", flush=True)
try:
	app.destroy()
except Exception:
	pass
sys.exit(1 if fails else 0)
