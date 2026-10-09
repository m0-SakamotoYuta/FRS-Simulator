# -*- coding: utf-8 -*-
"""実際の ankle タブ 2 つ（ACL 切除前・後）を可視化し、比較コントロールが使えるか（サンドボックスで実行）。

使い方: python test_multi_viewer_real.py <前のタブ番号> <後のタブ番号> [見つからないモデルの代用品]
接触の計算ダイアログは「計算しない」で閉じる（保存済みの接触結果があればそれを使う）。
"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import tkinter as tk
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c:
		fails.append(l)
def pump(app, sec=0.0):
	# app.update() は、描画が 5ms を超えると次々に来る再生ループのタイマーを処理し続けて戻らない
	# （実データで 60 秒戻らなかった）。1 件ずつ処理して時間で切る。
	t_end = time.time() + sec
	app.update_idletasks()
	while True:
		app.tk.dooneevent(tk._tkinter.DONT_WAIT)
		if time.time() >= t_end:
			break
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", a[:2])
frs.messagebox.showinfo = lambda *a, **k: print("   [info]", a[:2])

app = frs.MainMenuGUI(); pump(app, 0.5)
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
	def grab_release(self): pass
def _fake_dialog(N, has_cartilage=False):
	return (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
app._show_precompute_dialog = _fake_dialog

# エクスプローラで移動したモデルを、ファイル名と親フォルダ名で探す（サンドボックスの中だけで書き換える）
ROOT = os.path.join(os.path.expanduser("~"), "Desktop", "ankle simulator用ファイル")
_index = {}
for dp, dn, fn in os.walk(ROOT):
	for f in fn:
		_index.setdefault(f.lower(), []).append(os.path.join(dp, f))
def _find(path):
	if not path or os.path.exists(path):
		return path
	hits = _index.get(os.path.basename(path).lower(), [])
	par = os.path.basename(os.path.dirname(path)).lower()
	best = [h for h in hits if os.path.basename(os.path.dirname(h)).lower() == par] or hits
	if best:
		return best[0]
	# 見つからないモデルは代用品で（本番の経路を通すことが目的。見た目は確かめない）
	if path.lower().endswith((".obj", ".stl", ".ply")) and STANDIN:
		print("   [代用]", os.path.basename(path), "→", os.path.basename(STANDIN))
		return STANDIN
	return path
STANDIN = (sys.argv[3] if len(sys.argv) > 3 else "")

a, b = int(sys.argv[1]), int(sys.argv[2])
for ti in (a, b):
	app.on_ankle_tab_select(ti)
	pump(app, 0.3)
	for bone in app.ankle_bones:
		for k, v in list(bone.items()):
			if isinstance(v, str) and k.endswith("path") and v:
				nv = _find(v)
				if nv != v:
					bone[k] = nv
	t0 = time.time()
	app.on_ankle_animate()
	pump(app, 1.0)
	print(f"   タブ{ti} 「{app._ankle_tabs[ti]['name']}」 を開くのに {time.time() - t0:.1f} 秒")
vs = app._sim_viewers_alive()
check(len(vs) == 2, f"2 つ開いた ({len(vs)})")
for v, ti in zip(vs, (a, b)):
	check(app._ankle_tabs[ti]["name"] in v["title"], "題名にタブ名: " + v["title"])
check(getattr(app, "_sim_compare_win", None) is not None and app._sim_compare_win.winfo_exists(), "比較コントロールが出る")
if len(vs) == 2:
	v1, v2 = vs
	print("   長さ:", v1["max_time"], v2["max_time"])
	v2["seek"](1.0); app._sim_compare_shift(2.0); pump(app, 0.2)
	check(abs((v2["time"]() - v1["time"]()) - 1.0) < 1e-6, f"ずれを保って動く {v1['time']():.3f} / {v2['time']():.3f}")
	shown = {1: set(), 2: set()}
	app._sim_compare_toggle_play()
	t0 = time.time()
	while time.time() - t0 < 3.0:
		pump(app, 0.01)
		for v in (v1, v2):
			shown[v["no"]].add(v["ctrl"]["frame_label"].cget("text"))
	app._sim_compare_toggle_play(); pump(app, 0.2)
	el = v1["time"]() - 2.0
	check(2.8 < el < 3.3, f"3 秒で約 3 秒進む（実時間どおり）: {el:.2f} 秒")
	for k in (1, 2):
		print(f"   #{k}: 3 秒間に描いたコマ数 {len(shown[k])}（{len(shown[k]) / 3.0:.1f} コマ/秒）")
	d = v2["time"]() - v1["time"]()
	check(abs(d - 1.0) < 0.006, f"実データでも再生後にずれが保たれる（差 {d:.3f}s）")
	c = v1["plotter"].renderer.GetActiveCamera(); c.Azimuth(30); v1["plotter"].render(); pump(app, 0.3)
	check(app._sim_cam_get(v1["plotter"]) == app._sim_cam_get(v2["plotter"]), "視点が連動する")
	try:
		from PIL import ImageGrab
		pump(app, 0.5); ImageGrab.grab().save(os.path.join(HERE, "multi_viewer_real.png"))
	except Exception as e:
		print("   画面を撮れませんでした:", e)
for v in vs:
	try:
		app._sim_viewer_unregister(v); v["plotter"].close(); v["ctrl"]["window"].destroy()
	except Exception:
		pass
pump(app, 0.3)
print(f"\n合計: NG {len(fails)} 件")
for f in fails:
	print("  NG:", f)
os._exit(1 if fails else 0)
