# -*- coding: utf-8 -*-
"""可視化ウィンドウを複数開いて比較する機能のテスト（サンドボックスで実行）。

合成した骨（動く箱と球）のシーンを 2 つ開き、比較コントロールで
共通の再生・巻き戻し・コマ送り・速度・視点の連動・末尾で止まる・連動を外す・閉じる を確かめる。
"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np
import pyvista as pv
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c:
		fails.append(l)
def pump(app, sec=0.0, n=3):
	t_end = time.time() + sec
	while True:
		for _ in range(n):
			app.update_idletasks(); app.update()
		if time.time() >= t_end:
			break
		time.sleep(0.005)

def make_scene(title, dur, fps):
	N = int(dur * fps) + 1
	times = [i / fps for i in range(N)]
	box = pv.Cube(x_length=30, y_length=20, z_length=10)
	ball = pv.Sphere(radius=12)
	P1, P2 = [], []
	for t in times:
		T = np.eye(4); T[0, 3] = 20 * np.sin(t); P1.append(T)
		a = 0.5 * t
		R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
		T2 = np.eye(4); T2[:3, :3] = R; T2[:3, 3] = [0, 40, 0]; P2.append(T2)
	return {"window_title": title,
	        "bones": [{"name": "box", "mesh": box, "poses": np.array(P1), "color": "#DEB887", "opacity": 1.0, "scalars": None},
	                  {"name": "ball", "mesh": ball, "poses": np.array(P2), "color": "#ADD8E6", "opacity": 1.0, "scalars": None}],
	        "frame_times": times, "heatmap": {"enabled": False},
	        "features": {"csv": False, "export_model": True, "screenshot": True}}

app = frs.MainMenuGUI(); pump(app, n=5)

# --- 1つ目: 比較コントロールはまだ出ない ---
app._sim_engine_run(make_scene("test: ACL前", 10.0, 30)); pump(app, 0.3)
vs = app._sim_viewers_alive()
check(len(vs) == 1 and vs[0]["no"] == 1, "1つ目は #1")
check(vs[0]["title"] == "#1 test: ACL前", "題名に番号: " + vs[0]["title"])
check(vs[0]["ctrl"]["window"].title() == "再生コントロール — #1 test: ACL前", "再生コントロールの題名: " + vs[0]["ctrl"]["window"].title())
check(getattr(app, "_sim_compare_win", None) is None, "1つだけなら比較コントロールは出ない")
check(vs[0].get("_cmp_btn") is None, "1つだけなら「比較コントロール」ボタンは足さない")

# --- 2つ目（長さとコマの間隔が違う） ---
app._sim_engine_run(make_scene("test: ACL後", 12.0, 15)); pump(app, 0.5)
vs = app._sim_viewers_alive()
v1, v2 = vs
check(len(vs) == 2 and v2["no"] == 2 and v2["title"] == "#2 test: ACL後", "2つ目は #2: " + v2["title"])
cw = app._sim_compare_win
check(cw is not None and cw.winfo_exists(), "2つ目を開くと比較コントロールが出る")
check(all(v.get("_cmp_btn") is not None for v in vs), "両方の再生コントロールに「比較コントロール」ボタンが付く")
sw = app.winfo_screenwidth()
p1 = v1["plotter"].render_window.GetPosition(); p2 = v2["plotter"].render_window.GetPosition()
s1 = v1["plotter"].render_window.GetSize()
print("   配置:", p1, p2, "大きさ", s1, "画面幅", sw)
check(abs(p1[0] - 0) <= 20 and abs(p2[0] - sw // 2) <= 20, "左右に並ぶ")
check(abs(s1[0] - sw // 2) <= 40, "幅は画面の半分")

st = app._sim_compare_state
def times():
	return round(v1["time"](), 3), round(v2["time"](), 3)

# --- 手で合わせる（#2 だけ 2 秒進める）→ 共通の操作でずれが保たれる ---
v2["seek"](2.0); pump(app, 0.1)
check(times() == (0.0, 2.0), f"手で合わせる: {times()}")
app._sim_compare_shift(1.0); pump(app, 0.1)
check(times() == (1.0, 3.0), f"+1秒 → 両方とも 1 秒進む {times()}")
app._sim_compare_step(+1); pump(app, 0.1)
t = times()
check(abs(t[0] - (1.0 + 1 / 30)) < 0.006 and abs(t[1] - t[0] - 2.0) < 0.006, f"1コマ（#1 の 1/30 秒）進み、ずれは 2 秒のまま {t}")
check(v2["ctrl"]["frame_label"].cget("text").startswith("Frame: 45/"), "#2 は 15fps なので 3.03 秒 = 45 コマ目: " + v2["ctrl"]["frame_label"].cget("text"))
app._sim_compare_shift(-5.0); pump(app, 0.1)
check(times() == (0.0, 2.0), f"−5秒: #1 が先頭で止まり、ずれは保つ {times()}")
app._sim_compare_shift(100.0); pump(app, 0.1)
check(times() == (10.0, 12.0), f"+100秒: #1 が末尾で止まり、ずれは保つ {times()}")
app._sim_compare_to_start(); pump(app, 0.1)
check(times() == (0.0, 2.0), f"⏮ 先頭へ: 手前の #1 が 0 秒に {times()}")

# --- 1つだけ動かす（ずれを合わせる） ---
app._sim_compare_nudge(v2, sec=1.0); pump(app, 0.1)
check(times() == (0.0, 3.0), f"行の +1秒 は #2 だけ動く {times()}")
app._sim_compare_nudge(v2, frames=-15); pump(app, 0.1)
check(times() == (0.0, 2.0), f"行の −1コマ×15 は #2 の 15fps で 1 秒戻る {times()}")

# --- VTK の題名が日本語で出る ---
import ctypes, re
def native_title(v):
	m = re.search(r"([0-9a-fA-F]{6,})", str(v["plotter"].render_window.GetGenericWindowId()))
	buf = ctypes.create_unicode_buffer(256)
	u = ctypes.windll.user32; u.GetWindowTextW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_int]; u.GetWindowTextW(int(m.group(1), 16), buf, 256)
	return buf.value
pump(app, 0.3)
check(native_title(v1) == "#1 test: ACL前" and native_title(v2) == "#2 test: ACL後",
      f"ウィンドウの題名が化けない: {native_title(v1)} / {native_title(v2)}")

# --- 位置のスライダー ---
st["dragging"][0] = True; st["pos"].set(4.0); pump(app, 0.1); st["dragging"][0] = False
check(times() == (4.0, 6.0), f"位置のスライダーを 4 秒へ → 全体が同じだけ動く {times()}")
pump(app, 0.15)
check(abs(float(st["pos"].get()) - 4.0) < 0.011, "スライダーは #1 の時刻を示す")
lbl = st["row_labels"][id(v2)].cget("text")
check("差 +2.000 s" in lbl, "一覧に差が出る: " + lbl.strip())

# --- 共通の再生・停止 ---
app._sim_compare_toggle_play(); pump(app, 0.6)
check(not v1["is_paused"]() and not v2["is_paused"](), "再生 → 両方とも再生中")
check(st["play_btn"].cget("text") == "⏸ 一時停止", "ボタンは「一時停止」に変わる")
app._sim_compare_toggle_play(); pump(app, 0.1)
t = times()
check(v1["is_paused"]() and v2["is_paused"](), "一時停止 → 両方とも止まる")
check(4.4 < t[0] < 5.0 and abs(t[1] - t[0] - 2.0) < 0.006, f"約0.6秒進み、ずれは 2 秒のまま {t}")

# --- 速度 ---
app._sim_compare_set_speed(2.0); pump(app, 0.1)
check(v1["speed"]() == 2.0 and v2["speed"]() == 2.0, "速度 2 倍 → 両方とも 2 倍")
check(float(v2["ctrl"]["speed_scale"].get()) == 2.0, "各ウィンドウの速度スライダーも合う")

# --- 連動中は末尾で止まる（先頭に戻らない） ---
app._sim_compare_shift(9.6 - v1["time"]()); pump(app, 0.1)
app._sim_compare_toggle_play(); pump(app, 0.8)
t = times()
check(v1["is_paused"]() and v2["is_paused"](), "#1 が末尾に着くと両方止まる")
check(max(t) in (10.0, 12.0) and abs(t[1] - t[0] - 2.0) < 0.006, f"先頭には戻らず、ずれも保つ（5ms 刻み） {t}")
check("最後まで" in st["msg"].get(), "知らせが出る: " + st["msg"].get())
app._sim_compare_set_speed(1.0)

# --- 視点の連動 ---
pump(app, 0.1)
c1 = v1["plotter"].renderer.GetActiveCamera()
c1.Azimuth(35); c1.Elevation(20); v1["plotter"].render()
pump(app, 0.2)
check(app._sim_cam_get(v1["plotter"]) == app._sim_cam_get(v2["plotter"]), "#1 を回すと #2 も同じ視点になる")
c2 = v2["plotter"].renderer.GetActiveCamera()
c2.Zoom(1.5); v2["plotter"].render(); pump(app, 0.2)
check(app._sim_cam_get(v1["plotter"]) == app._sim_cam_get(v2["plotter"]), "#2 をズームすると #1 も合う（どちらからでも）")
st["cam_link"].set(False); pump(app, 0.1)
c1.Azimuth(-50); v1["plotter"].render(); pump(app, 0.2)
check(app._sim_cam_get(v1["plotter"]) != app._sim_cam_get(v2["plotter"]), "視点の連動を外すと、別々に動かせる")
st["cam_link"].set(True); pump(app, 0.2)

# --- 連動を外したウィンドウは共通の操作で動かない ---
app._sim_compare_to_start(); pump(app, 0.1)
before2 = v2["time"]()
v2["linked"].set(False); pump(app, 0.1)
app._sim_compare_shift(1.0); pump(app, 0.1)
check(abs(v2["time"]() - before2) < 1e-9 and abs(v1["time"]() - 1.0) < 1e-9, f"連動を外した #2 は動かない {times()}")
check("連動しない" in st["row_labels"][id(v2)].cget("text"), "一覧に「連動しない」")
v2["linked"].set(True); pump(app, 0.1)

# --- スクリーンショット（確認用） ---
try:
	from PIL import ImageGrab
	pump(app, 0.3)
	ImageGrab.grab().save(os.path.join(HERE, "multi_viewer_screen.png"))
	print("   画面:", os.path.join(HERE, "multi_viewer_screen.png"))
except Exception as e:
	print("   画面を撮れませんでした:", e)

# --- 閉じる: 1つになったら比較コントロールも閉じ、残りは従来どおり（先頭に戻ってくり返す） ---
v2["ctrl"]["window"].protocol("WM_DELETE_WINDOW")  # 存在確認
app._sim_viewer_unregister(v2)
try:
	v2["plotter"].close(); v2["ctrl"]["window"].destroy()
except Exception:
	pass
pump(app, 0.3)
check(len(app._sim_viewers_alive()) == 1, "閉じると一覧から外れる")
check(not app._sim_compare_win.winfo_exists(), "1つになると比較コントロールは閉じる")
check(not app._sim_compare_stop_at_end(v1), "1つだけなら末尾で止めない（従来どおり くり返す）")
v1["seek"](9.7); v1["set_paused"](False); pump(app, 0.6)
check(not v1["is_paused"]() and v1["time"]() < 1.0, f"従来どおり先頭に戻って再生を続ける t={v1['time']():.3f}")
v1["set_paused"](True)

# --- 3つ目を開くと番号は空いた #2 ---
app._sim_engine_run(make_scene("test: 3", 5.0, 30)); pump(app, 0.4)
vs = app._sim_viewers_alive()
check([v["no"] for v in vs] == [1, 2] and vs[1]["title"].startswith("#2 "), "閉じた番号は使い回す: " + vs[1]["title"])
check(app._sim_compare_win.winfo_exists(), "また 2 つになると比較コントロールが出る")

for v in vs:
	try:
		v["ctrl"]["window"].protocol("WM_DELETE_WINDOW")
		app._sim_viewer_unregister(v); v["plotter"].close(); v["ctrl"]["window"].destroy()
	except Exception:
		pass
pump(app, 0.2)
print(f"\n合計: NG {len(fails)} 件")
for f in fails:
	print("  NG:", f)
try:
	app.destroy()
except Exception:
	pass
sys.exit(1 if fails else 0)
