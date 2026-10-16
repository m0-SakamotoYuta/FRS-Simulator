# -*- coding: utf-8 -*-
"""再生コントロールの「ヒートマップを表示」: 付け外し・計算し直し不要・前回の選択を覚える・比較中は連動（合成データ）。"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, pyvista as pv
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(frs.tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
app = frs.MainMenuGUI(); app.update()

times = [i / 30 for i in range(60)]
box = pv.Cube(x_length=30, y_length=20, z_length=10).triangulate().subdivide(2)
P = np.array([np.eye(4) for _ in times])
sc = [(np.linspace(-3, 3, box.n_points) + 0.01 * t).astype(np.float32) for t in range(len(times))]
COLOR = "#3366CC"

def scene(title, cfg, enabled=True):
	return {"window_title": title, "bones": [{"name": "box", "mesh": box.copy(), "poses": P, "color": COLOR,
	                                          "opacity": 1.0, "scalars": (sc if enabled else None)}],
	        "frame_times": times, "heatmap": {"enabled": enabled, "title": "distance [mm]", **cfg},
	        "features": {"csv": False, "export_model": True, "screenshot": True},
	        "bone_style": {}}      # ankle と同じく「骨の表示」の枠を出す

def open_view(sc_):
	app._sim_engine_run(sc_); pump(app, 0.5)
	return app._sim_viewers_alive()[-1]

def close(v):
	w = v["ctrl"]["window"]
	w.tk.call(w.protocol("WM_DELETE_WINDOW")); pump(app, 0.3)

def actor(v):
	return v["plotter"].renderer.actors["bone_0"]

def toggle_cb(v):
	out = []
	def walk(wd):
		for c in wd.winfo_children():
			if c.winfo_class() == "TCheckbutton" and "ヒートマップを表示" in str(c.cget("text")):
				out.append(c)
			walk(c)
	walk(v["ctrl"]["window"])
	return out[0] if out else None

def scalars_now(v):
	return np.asarray(actor(v).GetMapper().GetInput().GetPointData().GetScalars())[:3].copy()

def has_bar(v):
	return len(v["plotter"].scalar_bars) > 0

# ---- 1. 1つのウィンドウで付け外し ----
app.ankle_hm_show.set(True)
v = open_view(scene("test: toggle", app._ankle_hm_scene_cfg()))
cb = toggle_cb(v)
check(cb is not None, "再生コントロールに「ヒートマップを表示」のチェックがある")
check(cb is not None and "disabled" not in cb.state() and v["ctrl"]["hm_show_var"].get(), "最初はオン（押せる）")
check(bool(actor(v).GetMapper().GetScalarVisibility()) and has_bar(v), "オン: 距離の色と色の帯")
# 骨の表示の枠の下にあるか
kids = v["ctrl"]["window"].pack_slaves()
i_style = next((i for i, c in enumerate(kids) if c.winfo_class() == "TLabelframe" and "骨の表示" in str(c.cget("text"))), None)
i_tog = next((i for i, c in enumerate(kids) if cb is not None and str(cb).startswith(str(c) + ".")), None)
check(i_style is not None and i_tog == i_style + 1, f"「骨の表示」の枠のすぐ下（{i_style} → {i_tog}）")
_win = v["ctrl"]["window"]; pump(app, 0.2)
_f = kids[i_tog] if i_tog is not None else None
check(_f is not None and _f.winfo_y() + _f.winfo_height() <= _win.winfo_height(),
      f"窓の中に収まっている（下端 {(_f.winfo_y() + _f.winfo_height()) if _f else '-'} / 窓 {_win.winfo_height()}）")
_last = _win.pack_slaves()[-1]
check(_last.winfo_y() + _last.winfo_height() <= _win.winfo_height(),
      f"いちばん下の行（範囲）も窓に収まる（下端 {_last.winfo_y() + _last.winfo_height()} / 窓 {_win.winfo_height()}）")

cb.invoke(); pump(app, 0.3)      # invoke はチェックを反転してから command を呼ぶ → オフ
check(not v["ctrl"]["hm_show_var"].get(), "チェックを外せる")
check(not actor(v).GetMapper().GetScalarVisibility(), "オフ: 距離の色を使わない")
check(np.allclose(actor(v).GetProperty().GetColor(), app._sim_hex_to_rgb01(COLOR), atol=0.01), "オフ: 骨の色で塗る")
check(not has_bar(v), "オフ: 色の帯を消す")
check("--" in str(v["ctrl"]["max_pent_label"].cget("text")), "オフ: Max Pent は --")
check(app.ankle_hm_show.get() is False, "選んだ状態はタブ（⑤のチェック）に覚える")
# 画面でも骨の色になっているか（真ん中の色）
img = v["plotter"].screenshot(return_img=True)
h, w_ = img.shape[:2]
mid = img[h // 2 - 20:h // 2 + 20, w_ // 2 - 20:w_ // 2 + 20].reshape(-1, 3).astype(float)
bone = np.array(app._sim_hex_to_rgb01(COLOR)) * 255
dirn = mid.mean(axis=0) / max(mid.mean(axis=0).max(), 1)
check(dirn[2] > dirn[0] + 0.2 and dirn[2] > dirn[1], f"画面の真ん中は骨の色（青系）: {np.round(mid.mean(axis=0))}")
# オフの間に動かしても塗り替えない → オンに戻すと いまのコマの色
before = scalars_now(v)
v["seek"](times[40]); pump(app, 0.2)
check(np.allclose(scalars_now(v), before), "オフの間はコマが進んでも塗り替えない（軽い）")
# オフの間に範囲を変えても帯は出ない
hr = [c for c in v["ctrl"]["window"].winfo_children() if c.winfo_class() == "TLabelframe" and "範囲" in str(c.cget("text"))][0]
ents = [c for c in hr.winfo_children() if c.winfo_class() == "TEntry"]
ents[0].delete(0, "end"); ents[0].insert(0, "1"); ents[1].delete(0, "end"); ents[1].insert(0, "0.5")
[c for c in hr.winfo_children() if c.winfo_class() == "TButton" and c.cget("text") == "適用"][0].invoke(); pump(app, 0.2)
check(not has_bar(v) and not actor(v).GetMapper().GetScalarVisibility(), "オフの間に範囲を変えても、色は出ない")
cb.invoke(); pump(app, 0.3)
fi = 40
check(bool(actor(v).GetMapper().GetScalarVisibility()) and has_bar(v), "オンに戻す: 距離の色と色の帯が戻る")
check(np.allclose(scalars_now(v), sc[fi][:3]), "オンに戻す: いまのコマの距離で塗る（計算し直しなし）")
check(tuple(np.round(actor(v).mapper.scalar_range, 3)) == (-1.0, 0.5), "オフの間に変えた範囲が効いている")
check("Max Pent: 2." in str(v["ctrl"]["max_pent_label"].cget("text")), f"Max Pent が戻る: {v['ctrl']['max_pent_label'].cget('text')}")
check(app.ankle_hm_show.get() is True, "オンもタブに覚える")
cb.invoke(); pump(app, 0.2)    # オフにして閉じる
close(v)

# ---- 2. 前回の選択を覚える（オフで開く） ----
check(app.ankle_hm_show.get() is False, "閉じたあともオフのまま")
check("ankle_hm_show" in app._ankle_state_vars(), "状態ファイルに保存される項目に入っている")
snap = app._ankle_state_vars()
v = open_view(scene("test: remember", app._ankle_hm_scene_cfg()))
cb = toggle_cb(v)
check(not v["ctrl"]["hm_show_var"].get(), "次に開くとオフのまま")
check(not actor(v).GetMapper().GetScalarVisibility() and not has_bar(v), "開いた最初からヒートマップなし（帯もなし）")
check("--" in str(v["ctrl"]["max_pent_label"].cget("text")), "開いた最初の Max Pent は --")
cb.invoke(); pump(app, 0.3)
check(bool(actor(v).GetMapper().GetScalarVisibility()) and np.allclose(scalars_now(v), sc[0][:3]),
      "オンにすると 1 コマ目の距離で塗る")
close(v)

# ---- 3. 比較中はそろえる ----
app.ankle_hm_show.set(True)
v1 = open_view(scene("test: 前", app._ankle_hm_scene_cfg()))
v2 = open_view(scene("test: 後", app._ankle_hm_scene_cfg()))
check(len(app._sim_viewers_alive()) == 2, "2つ開いた")
toggle_cb(v1).invoke(); pump(app, 0.3)
check(not v2["ctrl"]["hm_show_var"].get() and not actor(v2).GetMapper().GetScalarVisibility(),
      "#1 で外すと #2 も外れる")
toggle_cb(v2).invoke(); pump(app, 0.3)
check(v1["ctrl"]["hm_show_var"].get() and bool(actor(v1).GetMapper().GetScalarVisibility()) and has_bar(v1),
      "#2 で付けると #1 も付く")
v2["linked"].set(False); pump(app, 0.1)
toggle_cb(v1).invoke(); pump(app, 0.3)
check(not v1["ctrl"]["hm_show_var"].get() and v2["ctrl"]["hm_show_var"].get()
      and bool(actor(v2).GetMapper().GetScalarVisibility()), "連動を外したウィンドウは、そろえない")
close(v2); close(v1)

# ---- 4. 接触を計算していないとき ----
app.ankle_hm_show.set(True)
v = open_view(scene("test: no heatmap", app._ankle_hm_scene_cfg(), enabled=False))
cb = toggle_cb(v)
check(cb is not None and "disabled" in cb.state() and not v["ctrl"]["hm_show_var"].get(), "計算していないときは灰色で押せない")
check(app.ankle_hm_show.get() is True, "押せないときは、覚えている選択を変えない")
close(v)

# ---- 5. hip/knee（show を渡さない）は従来どおり ----
v = open_view(scene("test: hip-like", {"clim": (-10.0, 0.0)}))
check(toggle_cb(v) is None and v["ctrl"]["hm_show_var"] is None, "hip/knee: チェックは出ない")
check(bool(actor(v).GetMapper().GetScalarVisibility()) and has_bar(v), "hip/knee: いつもどおりヒートマップあり")
v["seek"](times[30]); pump(app, 0.2)
check(np.allclose(scalars_now(v), sc[30][:3]), "hip/knee: コマごとに塗り替える")
close(v)

print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
