# -*- coding: utf-8 -*-
"""マーカーの向きの裏返り（IPPE の逆の解）と、マーカーの大きさの見張り（2026-11-12）。
仁木先生の報告（骨がときどき大きく跳ねる・骨が映らない／骨どうしが 60〜70 cm 離れる）への対策。
・保存済みの④の結果の裏返りを数える: 実際のタブの姿勢の一部のコマを逆の解に入れ替えると、跳びの回数を
  数え、④の状態の文字に ⚠ と「④をやり直すと直ります」・版 2 の結果はやり直しを勧める・可視化に使う向きは
  勝手に直さない（直すと、ほとんど逆の解のときに逆の向きにそろってしまう。2935 の ID2・ID4 で実際に起きた）
・大きさの見張り: 深度の推定がノギスと 10% 以上違えば「自動」でもノギスを使う／まとめて解析の一覧の
  「補正後」も同じ／④の大きさと深度の推定が 10% 以上違えば④の状態の文字に ⚠
・IPPE の解の選び方: 深度の 1 か所だけの乱れでは、前のコマの向きのまま（3 コマ続いたら切り替える）
キャッシュは画面の中だけ差し替える（ファイルは書かない）。1 分ほど。"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, cv2
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def ang(Ra, Rb):
	return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1))))
def n_jumps(P, D):
	idx = np.where(D)[0]
	return int(sum(ang(P[a][:3, :3], P[b][:3, :3]) > 20 for a, b in zip(idx[:-1], idx[1:]) if b - a <= 3))
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_autosave_pose_cache = lambda *a, **k: None

# ---- 1. 保存済みの④の結果の裏返りを直す ----
name = next(t["name"] for t in app._ankle_tabs
            if int((app._ankle_pose_cache.get(t["name"]) or {}).get("frame_count", 0)) > 1000
            and (app._ankle_pose_cache[t["name"]].get("intrinsics") or {}).get("fx"))
orig = app._ankle_pose_cache[name]
aid = next(a for a, b in sorted(orig["bones"].items()) if np.asarray(b["detected"]).mean() > 0.95)
N = 1200
P0 = np.array(orig["bones"][aid]["poses"][:N], dtype=float); D0 = np.asarray(orig["bones"][aid]["detected"][:N], bool)
base = dict(orig, bones={aid: dict(orig["bones"][aid], poses=P0, detected=D0)}, frame_count=N)
cnt0 = app._ankle_count_pose_flips(base)
print(f"   使う結果: {name} の ID={aid}・{N} コマ（もともとの裏返り {cnt0}）", flush=True)
intr = orig["intrinsics"]
K = np.array([[intr["fx"], 0, intr["cx"]], [0, intr["fy"], intr["cy"]], [0, 0, 1.0]]); dist = np.array(intr.get("dist") or [0] * 5, float)
obj = np.asarray(app._ankle_marker_obj_points(float(orig["marker_size_mm"])), float)
rng = np.random.default_rng(0)
cand = [i for i in np.where(D0)[0] if 5 < i < N - 5]
flip_at = []
for st in rng.choice(cand, 25, replace=False):           # 1〜6 コマ続く裏返りを 25 か所
	flip_at += list(range(int(st), int(st) + int(rng.integers(1, 7))))
flip_at = sorted(set(i for i in flip_at if D0[i]))
P1 = P0.copy(); n_ok = 0
for i in flip_at:
	rv, _ = cv2.Rodrigues(P0[i, :3, :3])
	img, _ = cv2.projectPoints(obj, rv, P0[i, :3, 3].reshape(3, 1), K, dist)
	n, rvs, tvs, _e = cv2.solvePnPGeneric(obj, img.reshape(-1, 2), K, dist, flags=cv2.SOLVEPNP_IPPE)
	Rs = [cv2.Rodrigues(r)[0] for r in rvs]
	k = int(np.argmax([ang(R, P0[i, :3, :3]) for R in Rs]))
	if ang(Rs[k], P0[i, :3, :3]) > 5:
		P1[i, :3, :3] = Rs[k]; P1[i, :3, 3] = np.asarray(tvs[k]).ravel(); n_ok += 1
cache1 = dict(orig, bones={aid: dict(orig["bones"][aid], poses=P1, detected=D0)}, frame_count=N, detect_version=2)
nj = n_jumps(P1, D0)
print(f"   入れ替えたコマ {n_ok}・跳び {nj} 回", flush=True)
cnt = app._ankle_count_pose_flips(cache1)
check(cnt.get(int(aid)) == nj, f"裏返りの跳びを数える: {cnt}（期待 {nj}）")
check(sum(cnt0.values()) < app._ANKLE_FLIP_REDO_MIN, f"元の結果の跳びは {cnt0}（やり直しの目安 {app._ANKLE_FLIP_REDO_MIN} 回未満）")
viz = app._ankle_cache_for_current_size(cache1)
check(np.array_equal(np.asarray(viz["bones"][aid]["poses"])[:, :3, :3], P1[:, :3, :3]),
      "可視化に使う④の結果の向きは勝手に直さない（直すと、ほとんど逆の解のときに逆の向きにそろってしまう）")
check(app._ankle_hm_fingerprint({}, [], cache1) == app._ankle_hm_fingerprint({}, [], dict(cache1, bones={aid: dict(orig["bones"][aid], poses=P0, detected=D0)})),
      "接触の指紋は裏返りの数では変わらない")
st = app._ankle_cache_status_text(cache1)
check("⚠ マーカーの向きが裏返って" in st and f"ID={aid}: {nj}回" in st and "④をやり直すと直ります" in st,
      "④の状態の文字に ⚠ 裏返りの回数と「④をやり直すと直ります」")
check("⚠ マーカーの向きが裏返って" not in app._ankle_cache_status_text(dict(base, detect_version=2)), "裏返りの無い結果には ⚠ が出ない")
need2, low2 = app._ankle_cache_needs_rescue(cache1)
need3, _ = app._ankle_cache_needs_rescue(dict(cache1, detect_version=3))
check(need2 and low2 is None and not need3, f"版 2 で裏返りがあれば④のやり直しを勧める（{need2}）・版 3 なら勧めない（{need3}）")

# ---- 2. 大きさの見張り ----
pl = app._ankle_depth_size_plausible
check(pl(14.76, 15.0) and pl(15.0, 15.0) and not pl(148.0, 15.0) and not pl(20.0, 15.0) and not pl(14.6, 20.0),
      "深度の推定がノギスに近いか: 14.76/15 はよい・148/15・20/15・14.6/20 はだめ")
tab = next(t for t in app._ankle_tabs if t["name"] == name)
sn = tab.setdefault("snapshot", {})
keep = (sn.get("ankle_marker_size_auto"), sn.get("ankle_marker_size_true_mm"))
_orig_est = app._ankle_tab_depth_estimate
other = next(t for t in app._ankle_tabs if t is not app._ankle_active_tab_dict())
osn = other.setdefault("snapshot", {})
okeep = (osn.get("ankle_marker_size_auto"), osn.get("ankle_marker_size_true_mm"))
osn["ankle_marker_size_auto"] = True; osn["ankle_marker_size_true_mm"] = 15.0
app._ankle_tab_depth_estimate = lambda t: 148.0
v, kind, basis = app._ankle_tab_eff_size(other)
check(v == 15.0 and "外れ" in basis, f"「自動」でも推定 148 mm（ノギス 15）は使わずノギス: {v}（{basis}）")
app._ankle_tab_depth_estimate = lambda t: 14.76
v, kind, basis = app._ankle_tab_eff_size(other)
check(abs(v - 14.76) < 1e-9 and basis == "深度の推定", f"推定 14.76 mm（ノギス 15）は今までどおり使う: {v}（{basis}）")
app._ankle_tab_depth_estimate = _orig_est
osn["ankle_marker_size_auto"], osn["ankle_marker_size_true_mm"] = okeep
# 開いているタブの②の欄（自動）
_orig_get = app._ankle_depth_size_get
app._ankle_depth_size_get = lambda s: {"all": {"median": 148.0, "p25": 140.0, "p75": 150.0, "n": 30}}
app.ankle_marker_size_true_mm.set(15.0); app.ankle_marker_size_auto.set(True)
app._ankle_size_refresh_auto()
check(abs(float(app.ankle_marker_size_mm.get()) - 15.0) < 1e-9 and "⚠" in app.ankle_marker_size_note.get(),
      f"②の補正後（自動）は 15 のまま・メモに ⚠: {app.ankle_marker_size_mm.get()} ／ {app.ankle_marker_size_note.get()[:40]}…")
# ④の状態の文字: ④で 20 mm、深度では 14.6 mm
app._ankle_depth_size_get = lambda s: {"all": {"median": 14.6, "p25": 14.3, "p75": 14.9, "n": 40}}
st = app._ankle_cache_status_text(dict(base, marker_size_mm=20.0))
check("⚠ ④で使ったマーカーの大きさ 20 mm" in st, "④の大きさ 20 mm と深度 14.6 mm が違うと④の状態の文字に ⚠")
st = app._ankle_cache_status_text(dict(base, marker_size_mm=15.0))
check("⚠ ④で使ったマーカー" not in st, "15 mm なら ⚠ は出ない")
app._ankle_depth_size_get = _orig_get
# 手動で 150 と入れたとき
app.ankle_marker_size_auto.set(False); app.ankle_marker_size_mm.set(150.0)
app._ankle_size_refresh_auto()
check("⚠ 補正後 150" in app.ankle_marker_size_note.get(), f"手動で 150 mm（ノギス 15）なら②のメモに ⚠: {app.ankle_marker_size_note.get()[:30]}…")
app.ankle_marker_size_auto.set(True); app._ankle_size_refresh_auto()

# ---- 3. IPPE の解の選び方: 1 コマだけの深度の乱れでは裏返らない ----
size = 15.0; obj15 = np.asarray(app._ankle_marker_obj_points(size), float)
K2 = np.array([[650.0, 0, 640], [0, 650.0, 360], [0, 0, 1.0]]); dist2 = np.zeros(5)
R_true = cv2.Rodrigues(np.array([0.35, 0.0, 0.0]))[0]; t_true = np.array([0.0, 0.0, 150.0])
img = cv2.projectPoints(obj15, cv2.Rodrigues(R_true)[0], t_true, K2, dist2)[0].reshape(-1, 2)
n, rvs, tvs, rep = cv2.solvePnPGeneric(obj15, img, K2, dist2, flags=cv2.SOLVEPNP_IPPE)
true_k = int(np.argmin([ang(cv2.Rodrigues(r)[0], R_true) for r in rvs]))
def depth_map(R, t, noise_at=None):
	dm = np.zeros((720, 1280), np.uint16)
	c = img.mean(0)
	for k, (u, v) in enumerate(c + 0.7 * (img - c)):
		z = ((R @ (0.7 * obj15[k])) + t)[2]
		if noise_at is not None and k == noise_at:
			z = ((R @ (0.7 * obj15[k])) + t)[2] - 4.0 * np.sign(((R @ (0.7 * obj15[k])))[2] + 1e-9)
		u, v = int(round(u)), int(round(v)); dm[v - 3:v + 4, u - 3:u + 4] = int(round(z / 0.1))
	return dm
# 逆の解の深度の形を作る（1 コマだけ「逆の傾き」に見える深度）
R_bad = cv2.Rodrigues(rvs[1 - true_k])[0]; t_bad = np.asarray(tvs[1 - true_k]).ravel()
app._ankle_prev_pose = {}; app._ankle_pick_calls = {}; app._ankle_pick_disagree = {}
seq = []
for f in range(8):
	dm = depth_map(R_bad, t_bad) if f in (4,) else depth_map(R_true, t_true)
	k, _s = app._ankle_pick_ippe_solution(7, rvs, tvs, rep, int(n), img, obj15, dm, 0.1)
	seq.append(k)
check(all(k == true_k for k in seq), f"1 コマだけ逆の傾きの深度でも、前のコマの向きのまま: {seq}（正解 {true_k}）")
app._ankle_prev_pose = {}; app._ankle_pick_calls = {}; app._ankle_pick_disagree = {}
seq = []
for f in range(8):
	dm = depth_map(R_bad, t_bad) if f >= 3 else depth_map(R_true, t_true)
	k, _s = app._ankle_pick_ippe_solution(7, rvs, tvs, rep, int(n), img, obj15, dm, 0.1)
	seq.append(k)
check(seq[:5] == [true_k] * 5 and seq[5:] == [1 - true_k] * 3,
      f"逆の傾きの深度が 3 コマ続いたら切り替える: {seq}")
app._ankle_prev_pose = {}; app._ankle_pick_calls = {}; app._ankle_pick_disagree = {}
k, _s = app._ankle_pick_ippe_solution(7, rvs, tvs, rep, int(n), img, obj15 * (20.0 / 15.0), depth_map(R_true, t_true), 0.1)
check(k == true_k, "最初のコマ: ④の大きさが実物と違っても（20 mm で計算）、深度の傾きの形で正しい解を選ぶ")

print(f"\n{'すべて OK' if not fails else f'NG {len(fails)} 件'}", flush=True)
try:
	app.destroy()
except Exception:
	pass
sys.exit(1 if fails else 0)
