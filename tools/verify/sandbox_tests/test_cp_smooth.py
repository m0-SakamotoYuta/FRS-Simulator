# -*- coding: utf-8 -*-
"""接触圧の平滑化・いちばん高い値・発表用の動画の欄の共有（2026-10-22）:
接触圧を時間で平滑化（チェック・σ 秒・既定 4 秒。接触しているコマだけのガウス）が 3D の右上と数字の動画に効く、
「数字の動画だけ書き出す…」の下に接触圧のいちばん高い値（区間・試験全体。平滑化後も）、
骨リストを共有しているタブどうしで発表用の表示・動画の欄の設定を共有。
接触の計算はダミー（コマごとに揺らした距離）、datalog は本物（261007_AP_FE60_軸力100）。2〜3分。"""
import importlib.util, os, sys, time, gc
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, vtk
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a[:2])[:200])
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
WIG = lambda t: 0.25 * np.sin(t / 40.0) + 0.15 * np.sin(t / 7.0 + 1.0)      # コマごとに揺らす（ガタガタの代わり）
MESH = {}
def _fake(bd, update_progress=None, cancel_var=None, quiet=False):
	for (idx, n, m, T) in bd:
		MESH[idx] = np.asarray(m.points, float)
	return [{idx: ((np.asarray(m.points)[:, 0] % 3.0) - 1.0 + WIG(t)).astype(np.float32) for (idx, n, m, T) in bd} for t in range(len(bd[0][3]))]
app._sim_view_precompute_multi_heatmap = _fake
app._ankle_heatmap_cache_load = lambda *a, **k: None
app._ankle_heatmap_cache_save = lambda *a, **k: None
TAB, TAB2 = "261007_AP_FE60_軸力100", "261007_AP_FE60_軸力100_ACL切"
def sel(name):
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == name)); pump(app, 0.5)
sel(TAB)
g1, g2 = app._ankle_tab_group(app._ankle_tabs[next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)]), \
	app._ankle_tab_group(app._ankle_tabs[next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB2)])
check(g1 and g1 == g2, f"2 つのタブは骨リストを共有（{g1} / {g2}）")
DIL, HA, H = 0.15, 2.8, 1.6
app.ankle_dilate_mm.set(DIL); app.ankle_cp_ha.set(HA); app.ankle_cp_h.set(H); app.ankle_lig_mode.set("stage1")
app.ankle_po_force_on.set(True); app.ankle_po_contact_on.set(True); app.ankle_po_cp_kind.set("max"); app.ankle_po_area_on.set(False)
app.ankle_po_cp_smooth_on.set(False); app.ankle_po_cp_smooth_s.set(4.0); app.ankle_start_at_load.set(False); app.ankle_hm_show.set(True)
app.ankle_clip_sec.set(3.0); app.ankle_clip_hold.set(0.0); app.ankle_clip_kind.set("anterior"); app.ankle_clip_size.set(400)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
v["set_paused"](True)
cw = ctrl["window"]; pv_ = ctrl["po_vars"]
scn = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "heatmap" in o and "_t_shift" in o][-1]
ft = np.asarray(scn["frame_times"], float)
tib = next(i for i, b in enumerate(scn["bones"]) if "脛骨" in str(b.get("name", "")))
# 独立に: 最大接触圧 = H_A/h × max(めり込み)、めり込み = −(距離 − 2δ)
P0 = MESH[tib]
dmin = float(((P0[:, 0] % 3.0) - 1.0).min())
raw = np.array([HA / H * max(0.0, -(dmin + WIG(t) - 2 * DIL)) for t in range(len(ft))])
dt = float(np.median(np.diff(ft)))
def smooth(sig):
	sf = sig / dt; r = int(np.ceil(3 * sf)); k = np.exp(-0.5 * (np.arange(-r, r + 1) / sf) ** 2)
	m = raw > 0
	return np.where(m, np.convolve(np.where(m, raw, 0), k, "same") / np.maximum(np.convolve(m.astype(float), k, "same"), 1e-12), np.nan)
def tacts():
	out = []
	props = p.renderer.GetViewProps(); props.InitTraversal()
	for _ in range(props.GetNumberOfItems()):
		a_ = props.GetNextProp()
		if isinstance(a_, vtk.vtkTextActor) and a_.GetVisibility():
			out.append(a_)
	return out
def go(f):
	v["seek"](float(ft[f]) + 0.003); pump(app, 0.3)
	return int(np.searchsorted(ft, float(v["time"]()) + 1e-9, side="right") - 1)
def shown_val():
	a = [x for x in tacts() if str(x.GetInput()).startswith("接触圧：")]          # 最大も「接触圧」と出す（2026-10-22）
	if not a:
		return None
	try:
		return float(str(a[0].GetInput()).split("：")[1].split(" ")[0])
	except Exception:
		return None
# ---- 欄
cbs = [w for w in walk(cw) if w.winfo_class() == "TCheckbutton" and "接触圧を時間で平滑化" in str(w.cget("text"))]
sps = [w for w in walk(cw) if w.winfo_class() == "TSpinbox" and str(w.cget("textvariable")) == str(pv_.get("cp_smooth_s"))]
check(len(cbs) == 1 and len(sps) == 1 and sps[0].get() == "4" and app.ankle_po_cp_smooth_on.get() is False,
      f"「接触圧を時間で平滑化する」のチェック（既定オフ）と σ（既定 4 秒: {sps[0].get() if sps else '-'}）")
fr = [int(len(ft) * q) for q in (0.3, 0.55)]
offs = []
for f in fr:
	fi = go(f)
	offs.append((fi, shown_val()))
check(all(sv is not None and abs(sv - raw[fi]) <= 0.011 for fi, sv in offs), f"平滑化なし: 表示 = 生の最大接触圧（{[(sv, round(raw[fi], 3)) for fi, sv in offs]}）")
cbs[0].invoke(); pump(app, 0.5)
check(app.ankle_po_cp_smooth_on.get() is True, "チェックを入れると覚える")
ons = []
for f in fr:
	fi = go(f)
	ons.append((fi, shown_val()))
s4 = smooth(4.0)
check(all(sv is not None and abs(sv - s4[fi]) <= 0.011 for fi, sv in ons) and any(abs(raw[fi] - s4[fi]) > 0.03 for fi, _ in ons),
      f"平滑化あり（σ 4 秒）: 表示 = 接触しているコマのガウス平均（{[(sv, round(s4[fi], 3), round(raw[fi], 3)) for fi, sv in ons]}）")
sps[0].set("1"); sps[0].event_generate("<FocusOut>"); pump(app, 0.5)
fi = go(fr[0]); sv = shown_val(); s1 = smooth(1.0)
check(sv is not None and abs(sv - s1[fi]) <= 0.011 and abs(s1[fi] - s4[fi]) > 0.03 and abs(app.ankle_po_cp_smooth_s.get() - 1.0) < 1e-9,
      f"σ を 1 秒に: 表示 {sv}・期待 {s1[fi]:.3f}（覚える {app.ankle_po_cp_smooth_s.get()}）")
sps[0].set("4"); sps[0].event_generate("<FocusOut>"); pump(app, 0.5)
# ---- 数字の動画にも効く
cf = [w for w in walk(cw) if w.winfo_class() == "TLabelframe" and "発表用の動画" in str(w.cget("text"))][0]
pv_["clip_nums_preview"](); pump(app, 0.5)
nw = pv_["clip_nums_prev_win"]["w"]; g_on = nw.g
k_mid = g_on["n"] // 2
im_on = g_on["frame"](k_mid)
cbs[0].invoke(); pump(app, 0.6)
g_off = nw.g; im_off = g_off["frame"](k_mid)
diff = float(np.abs(im_on.astype(int) - im_off.astype(int)).mean()) if im_on.shape == im_off.shape else 999
check(diff > 0.05, f"数字の動画（プレビュー）も平滑化で数字が変わる・チェックを変えると開いている窓も描き直す（平均の差 {diff:.3f}）")
cbs[0].invoke(); pump(app, 0.6)
# ---- 接触圧のいちばん高い値（数字の動画だけ書き出すの下）
lab = pv_.get("clip_cp_summary_lab")
pump(app, 0.5)
txt = str(lab.cget("text")) if lab is not None else ""
order = [w for w in cf.pack_slaves()]
i_btn = next((i for i, w in enumerate(order) if any("数字の動画だけ書き出す" in str(c.cget("text")) for c in w.winfo_children() if c.winfo_class() == "TButton")), -1)
check(lab is not None and order.index(lab) > i_btn >= 0, "「数字の動画だけ書き出す…」の下に接触圧のいちばん高い値")
check("前方引張の区間" in txt and "試験全体" in txt and f"最大接触圧 {raw.max():.2f} MPa" in txt.split("試験全体")[1],
      f"試験全体の最大接触圧のいちばん高い値 = 独立に求めた {raw.max():.2f} MPa（{txt.splitlines()[-2] if txt.count(chr(10)) else txt}）")
check("平滑化（σ 4 秒）のあと" in txt and f"最大接触圧 {np.nanmax(s4):.2f} MPa" in txt.split("試験全体")[1],
      f"平滑化を使っているので、平滑化後のいちばん高い値も（{np.nanmax(s4):.2f}）")
# ---- 共有: 発表用の動画の欄・発表用の表示
sp_sec = [w for w in walk(cf) if w.winfo_class() == "TSpinbox" and str(w.cget("textvariable")) == str(pv_["clip_sec"])][0]
sp_sec.set("7"); sp_sec.event_generate("<FocusOut>"); pump(app, 0.3)
cbh = [w for w in walk(cf) if w.winfo_class() == "TCheckbutton" and "ヒートマップも映す" in str(w.cget("text"))][0]
h0 = bool(app.ankle_clip_heat_on.get()); cbh.invoke(); pump(app, 0.3)
view = app._ankle_bone_groups_dict()[g1]["view"]
check(abs(float(view.get("ankle_clip_sec", -1)) - 7.0) < 1e-9 and bool(view.get("ankle_clip_heat_on")) == (not h0) and view.get("ankle_po_cp_smooth_on") is True,
      f"動画の欄（秒数・ヒートマップ）と平滑化の設定が共有の中身へ（{view.get('ankle_clip_sec')}・{view.get('ankle_clip_heat_on')}・{view.get('ankle_po_cp_smooth_on')}）")
try: nw.destroy()
except Exception: pass
cw.tk.call(cw.protocol("WM_DELETE_WINDOW")); pump(app, 0.5)
sel(TAB2)
check(abs(app.ankle_clip_sec.get() - 7.0) < 1e-9 and app.ankle_clip_heat_on.get() == (not h0) and app.ankle_po_cp_smooth_on.get() is True
      and abs(app.ankle_po_cp_smooth_s.get() - 4.0) < 1e-9 and app.ankle_po_cp_kind.get() == "max",
      f"骨リストを共有している別のタブを開くと同じ設定（秒数 {app.ankle_clip_sec.get()}・ヒートマップ {app.ankle_clip_heat_on.get()}・平滑化 {app.ankle_po_cp_smooth_on.get()}）")
sel(TAB)
app.ankle_po_cp_smooth_on.set(False); app.ankle_clip_heat_on.set(h0); app.ankle_clip_sec.set(10.0)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
