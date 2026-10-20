# -*- coding: utf-8 -*-
"""発表用の表示（2026-10-20）: 右上に試験の段階（初期状態＝緑・前方引張中：Fap＝赤・後方引張中：Fap＝青）。
段階は試験の手順どおり 5 つ（途中で行き来しても増やさない。前方→後方は 0 N 付近を下回った所・最後の −200→0 N は前方引張）。
その下に大腿骨-脛骨の接触圧（黒・既定は平均。最大にも切り替え）、
接触面積は既定で出さない（チェックで出す）。再生コントロールで出し入れ・文字の大きさ（覚える）。色の帯の「dilated …」は既定で出さない。
右のグラフ（y 0.74 より下）と重ならない位置・日本語のフォント。接触の計算はダミー（距離が分かっている）。datalog は本物
（261007_AP_FE60_軸力100 の KKR）。1〜2分。"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk, vtk, pyvista as pv, gc
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
def fake_multi(bd, update_progress=None, cancel_var=None, quiet=False):
	N = len(bd[0][3])
	base = {idx: ((np.asarray(m.points)[:, 0] % 3.0) - 1.0).astype(np.float32) for (idx, n, m, T) in bd}   # −1.0〜+2.0 mm
	return [{idx: base[idx] for idx in base} for t in range(N)]
app._sim_view_precompute_multi_heatmap = fake_multi
app._ankle_heatmap_cache_load = lambda *a, **k: None
app._ankle_heatmap_cache_save = lambda *a, **k: None

# --- 段階の分け方（関数だけ）: 初期 → 前方 → 後方（途中で 0 付近に長くいる）→ 前方 → 初期 ---
t_ = np.arange(0, 300, 0.05)
f_ = np.interp(t_, [0, 30, 60, 70, 110, 150, 180, 220, 230, 280, 300], [0, 0, 195, 195, 5, -3, -195, -195, -150, -2, 0])
f_ = f_ + np.random.default_rng(0).normal(0, 1.5, len(t_))
ph = frs.MainMenuGUI._ankle_ap_phases(t_, f_)
seq = [int(ph[0])] + [int(ph[i]) for i in range(1, len(ph)) if ph[i] != ph[i - 1]]
check(seq == [0, 1, -1, 1, 0], f"段階は 初期→前方→後方→前方→初期 の順（{seq}・ちらつきなし）")
check(ph[np.searchsorted(t_, 130)] == -1, "後方引張の途中で Fap が 0 N 付近に長くいても後方引張のまま")
check(ph[np.searchsorted(t_, 65)] == 1 and ph[np.searchsorted(t_, 200)] == -1, "±200 N で止めている間は、その前の引張のまま")
check(ph[np.searchsorted(t_, 250)] == 1, "最後の −200→0 N に戻す区間は前方引張（Fap は負）")
check(ph[np.searchsorted(t_, 100)] == 1 and ph[np.searchsorted(t_, 112)] == -1, "前方→後方は 0 N 付近（+6 N）を下回った所で切り替わる")
# 途中で前後に行き来する記録（ACL 切除後のよう）でも 5 つの段階だけ
t2 = np.arange(0, 600, 0.05)
f2 = np.interp(t2, [0, 30, 80, 120, 160, 220, 260, 300, 340, 380, 450, 500, 560, 600], [0, 0, 70, 70, -25, 190, 190, -60, 45, 40, -190, -190, -3, 0])
f2 = f2 + np.random.default_rng(1).normal(0, 1.5, len(t2))
ph2 = frs.MainMenuGUI._ankle_ap_phases(t2, f2)
seq2 = [int(ph2[0])] + [int(ph2[i]) for i in range(1, len(ph2)) if ph2[i] != ph2[i - 1]]
check(seq2 == [0, 1, -1, 1, 0], f"途中で前後に行き来しても 段階は 初期→前方→後方→前方→初期 の 5 つ（{seq2}）")

TAB = "261007_AP_FE60_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
# 本物の ACL 切除後の記録（前後に何度か行き来している）
_kp = os.path.join(os.path.dirname(str(app.ankle_robot_log_path.get())), "AP_FE60_軸力100_ACLcut.KKR")
if os.path.exists(_kp):
	_klog = app._ankle_robot_log_load(_kp)
	ph3 = frs.MainMenuGUI._ankle_ap_phases(_klog["t"], _klog["cols"]["Fap"])
	seq3 = [int(ph3[0])] + [int(ph3[i]) for i in range(1, len(ph3)) if ph3[i] != ph3[i - 1]]
	check(seq3 == [0, 1, -1, 1, 0], f"本物の ACL 切除後の記録（前後に何度か行き来している）も 5 つ（{seq3}）")
app.ankle_dilate_mm.set(0.15); app.ankle_dil_title_on.set(False); app.ankle_cp_ha.set(2.8); app.ankle_cp_h.set(1.6)
app.ankle_po_force_on.set(True); app.ankle_po_contact_on.set(True); app.ankle_po_force_fs.set(22); app.ankle_po_contact_fs.set(18)
app.ankle_po_area_on.set(False); app.ankle_po_cp_kind.set("mean")
app.ankle_lig_mode.set("stage1"); app.ankle_start_at_load.set(False); app.ankle_hm_show.set(True)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]; ctrl = v["ctrl"]
v["set_paused"](True)
def tacts():
	out = []
	props = p.renderer.GetViewProps(); props.InitTraversal()
	for _ in range(props.GetNumberOfItems()):
		a_ = props.GetNextProp()
		if isinstance(a_, vtk.vtkTextActor):
			out.append(a_)
	return out
def go(f):
	"""コマ f の時刻へ。実際に出したコマの番号を返す。"""
	v["seek"](float(ft[f]) + 0.003); pump(app, 0.3)
	return int(np.searchsorted(ft, float(v["time"]()) + 1e-9, side="right") - 1)
def shown(key):
	return [a for a in tacts() if a.GetVisibility() and key in str(a.GetInput())]
NL = chr(10)
# 色の帯の題名
check(not any("dilated" in str(k) for k in p.scalar_bars.keys()) and not any("dilated" in str(a.GetInput()) for a in tacts() if a.GetVisibility()),
      f"色の帯に膨張の値（dilated …）は既定で出さない（{list(p.scalar_bars.keys())}）")
dlf = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and "膨張" in str(w.cget("text"))][0]
dcb = [w for w in walk(dlf) if w.winfo_class() == "TCheckbutton"]
check(len(dcb) == 1 and "色の帯" in str(dcb[0].cget("text")), "「骨の膨張」の欄に「色の帯に値を出す」のチェック")
dcb[0].invoke(); pump(app, 0.3)
check(any("dilated 0.15" in str(k) for k in p.scalar_bars.keys()) or any("dilated 0.15" in str(a.GetInput()) for a in tacts() if a.GetVisibility()),
      "チェックを入れると題名に膨張の値が出る")
check(app.ankle_dil_title_on.get() is True, "その設定はタブに覚える")
dcb[0].invoke(); pump(app, 0.3)
check(not any("dilated" in str(k) for k in p.scalar_bars.keys()), "外すと消える")
# datalog の Fap（独立に計算）
scn = [o for o in gc.get_objects() if isinstance(o, dict) and "bones" in o and "heatmap" in o and "_t_shift" in o][-1]
ft = np.asarray(scn["frame_times"], float)
log = app._ankle_robot_log_load(str(app.ankle_robot_log_path.get()))
off = scn["_log_offset"] if scn.get("_log_offset") is not None else float(app.ankle_robot_log_offset_s.get())
fap = np.interp(ft, np.asarray(log["t"]) + off - float(scn["_t_shift"]), np.asarray(log["cols"]["Fap"], float), left=np.nan, right=np.nan)
fP, fN = int(np.nanargmax(fap)), int(np.nanargmin(fap))
RED, BLUE, GREEN = (0.85, 0.0, 0.0), (0.0, 0.25, 0.9), (0.0, 0.55, 0.15)
def num(x):
	return "0 N" if round(x) == 0 else f"{x:+.0f} N".replace("-", "−")
def stage_check(f, lab, col, why):
	fi = go(f)
	a = [x for x in tacts() if x.GetVisibility() and ("引張中" in str(x.GetInput()) or "初期状態" in str(x.GetInput()))]
	exp = lab if lab == "初期状態" else f"{lab}：{num(fap[fi])}"
	check(len(a) == 1 and str(a[0].GetInput()) == exp and np.allclose(a[0].GetTextProperty().GetColor(), col, atol=0.01),
	      f"{why}: 「{exp}」（表示「{a[0].GetInput() if a else '-'}」・Fap {fap[fi]:+.1f} N）")
fin = np.where(np.isfinite(fap))[0]
stage_check(fP, "前方引張中", RED, "前方に引っぱるコマで赤")
stage_check(fN, "後方引張中", BLUE, "後方に引っぱるコマで青")
stage_check(fin[5], "初期状態", GREEN, "始め（引っぱる前）は緑で初期状態")
mid = np.arange(fP, fN)
stage_check(int(mid[np.argmin(np.abs(fap[mid] - 25))]), "前方引張中", RED, "最大のあと +25 N まで戻した所はまだ前方引張")
stage_check(int(mid[np.argmin(np.abs(fap[mid] + 10))]), "後方引張中", BLUE, "0 N を下回った後（−10 N）は後方引張")
last = np.arange(fN, fin[-1])
stage_check(int(last[np.argmin(np.abs(fap[last] + 100))]), "前方引張中", RED, "最後の −200→0 N に戻す区間は前方引張（Fap が負でも）")
stage_check(fin[-3], "初期状態", GREEN, "終わりに 0 N 付近へ戻ると初期状態")
# 映像は記録より長い（前後）。記録の外でも初期状態のまま（最後のコマまで消えない）
if fin[-1] < len(ft) - 1:
	go(len(ft) - 1)
	a = [x for x in tacts() if x.GetVisibility() and "初期状態" in str(x.GetInput())]
	check(len(a) == 1 and np.allclose(a[0].GetTextProperty().GetColor(), GREEN, atol=0.01),
	      f"記録が終わった後（映像の最後のコマ・記録より {ft[-1] - ft[fin[-1]]:.1f} 秒あと）も初期状態のまま")
if fin[0] > 0:
	go(0)
	a = [x for x in tacts() if x.GetVisibility() and "初期状態" in str(x.GetInput())]
	check(len(a) == 1, f"記録が始まる前（映像の最初のコマ・記録より {ft[fin[0]] - ft[0]:.1f} 秒前）も初期状態")
# 接触圧（脛骨・独立に計算）
fP = go(fP)
names = [b["name"] for b in scn["bones"]]; ti = names.index("脛骨")
m_ = scn["bones"][ti]["mesh"].triangulate()
F_ = np.asarray(m_.faces).reshape(-1, 4)[:, 1:]; P_ = np.asarray(m_.points, float)
A_ = 0.5 * np.linalg.norm(np.cross(P_[F_[:, 1]] - P_[F_[:, 0]], P_[F_[:, 2]] - P_[F_[:, 0]]), axis=1)
ar = np.zeros(len(P_)); [np.add.at(ar, F_[:, c], A_ / 3) for c in range(3)]
d = np.asarray(scn["bones"][ti]["scalars"][fP], float) - 0.3
pen = np.clip(-d, 0, None); c = pen > 0; pp = 2.8 * pen / 1.6
pmax, pmean, area = pp.max(), (pp[c] * ar[c]).sum() / ar[c].sum(), ar[c].sum()
a = shown("接触圧")
check(len(a) == 1 and str(a[0].GetInput()) == f"接触圧：{pmean:.2f} MPa" and np.allclose(a[0].GetTextProperty().GetColor(), (0, 0, 0)),
      f"既定は大腿骨-脛骨の平均接触圧を「接触圧」として黒で（「{a[0].GetInput() if a else '-'}」・期待 {pmean:.2f}）")
_notes = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabel" and "＝ 平均接触圧" in str(w.cget("text"))]
check(len(_notes) == 1 and "接触力 ÷ 接触面積" in str(_notes[0].cget("text")), "再生コントロールに「接触圧 ＝ 平均接触圧（接触力 ÷ 接触面積）」の説明")
check(not shown("接触面積"), "接触面積は既定で出さない")
fa = shown("引張中")[0]
check(fa.GetPosition()[1] > 0.75 and a[0].GetPosition()[1] > 0.75 and fa.GetPosition()[0] > 0.9,
      f"右上（右のグラフ y 0.74 より上）に置く（{fa.GetPosition()} / {a[0].GetPosition()}）")
check(fa.GetPosition()[1] > a[0].GetPosition()[1], "段階の行の下に接触圧の行")
jf = app._sim_ligament_font_file()
check(jf is not None and fa.GetTextProperty().GetFontFile() == jf and fa.GetTextProperty().GetFontFamily() == vtk.VTK_FONT_FILE,
      f"日本語のフォントで描く（{jf}）")
# 再生コントロール
fr = [w for w in walk(ctrl["window"]) if w.winfo_class() == "TLabelframe" and str(w.cget("text")).startswith("発表用の表示")]
_clipf = [w for w in walk(fr[0]) if w.winfo_class() == "TLabelframe"] if fr else []
def walk_po(wd):
	"""「発表用の表示」の欄の中（中の「発表用の動画」の欄は除く）。"""
	for c in wd.winfo_children():
		if c in _clipf:
			continue
		yield c
		yield from walk_po(c)
check(len(fr) == 1, "再生コントロールに「発表用の表示」の欄")
cbs = {str(w.cget("text")): w for w in walk_po(fr[0]) if w.winfo_class() == "TCheckbutton"}
rbs = {str(w.cget("text")): w for w in walk_po(fr[0]) if w.winfo_class() == "TRadiobutton"}
sps = [w for w in walk_po(fr[0]) if w.winfo_class() == "TSpinbox"]
check(len(cbs) == 3 and set(rbs) == {"最大", "平均"} and len(sps) == 2, "段階・接触圧・接触面積のチェック、最大／平均、文字の大きさ 2 つ")
cb_stage = [w for k, w in cbs.items() if "段階" in k][0]
cb_cp = [w for k, w in cbs.items() if k.startswith("接触圧")][0]
cb_area = [w for k, w in cbs.items() if "接触面積" in k][0]
cb_area.invoke(); pump(app, 0.2)
a = shown("接触圧")
check(len(a) == 1 and str(a[0].GetInput()) == f"接触圧：{pmean:.2f} MPa{NL}接触面積：{area:.0f} mm²" and app.ankle_po_area_on.get() is True,
      f"「接触面積も出す」で接触面積の行が増える（{str(a[0].GetInput()).replace(NL, ' / ') if a else '-'}・覚える）")
rbs["最大"].invoke(); pump(app, 0.2)
a = shown("接触圧")
check(len(a) == 1 and str(a[0].GetInput()).startswith(f"最大接触圧：{pmax:.2f} MPa") and app.ankle_po_cp_kind.get() == "max",
      f"「最大」に切り替えると最大接触圧（{str(a[0].GetInput()).split(NL)[0] if a else '-'}・覚える）")
rbs["平均"].invoke(); pump(app, 0.2)
cb_area.invoke(); pump(app, 0.2)
cb_stage.invoke(); pump(app, 0.2)
check(not shown("引張中") and shown("接触圧") and app.ankle_po_force_on.get() is False, "段階のチェックを外すと消える（接触圧は出たまま・覚える）")
cb_stage.invoke(); pump(app, 0.2)
cb_cp.invoke(); pump(app, 0.2)
check(not shown("接触圧") and shown("引張中") and app.ankle_po_contact_on.get() is False, "接触圧のチェックを外すと消える（覚える）")
cb_cp.invoke(); pump(app, 0.2)
for sp_, key, val in ((sps[0], "force", 36), (sps[1], "contact", 26)):
	sp_.focus_force(); pump(app, 0.1)
	sp_.delete(0, "end"); sp_.insert(0, str(val)); sp_.event_generate("<Return>"); pump(app, 0.3)
	a_ = shown("引張中" if key == "force" else "接触圧")[0]
	check(a_.GetTextProperty().GetFontSize() == val and int(getattr(app, f"ankle_po_{key}_fs").get()) == val,
	      f"文字の大きさを {val} に（{a_.GetTextProperty().GetFontSize()}・覚える）")
fa = shown("引張中")[0]; ca = shown("接触圧")[0]
H = p.renderer.GetSize()[1]
check((fa.GetPosition()[1] - ca.GetPosition()[1]) * H >= 1.4 * 36, "大きくしても、接触圧の行は段階の行の下へずれる（重ならない）")
w_ = ctrl["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.on_ankle_animate(); pump(app, 2.0)
v = app._sim_viewers_alive()[-1]; ctrl = v["ctrl"]
check(ctrl["po_vars"]["force_fs"].get() == "36" and ctrl["po_vars"]["contact_fs"].get() == "26", "次に開くと前の文字の大きさ")
w_ = ctrl["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_po_force_fs.set(22); app.ankle_po_contact_fs.set(18); app.ankle_po_area_on.set(False); app.ankle_po_cp_kind.set("mean")
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
