# -*- coding: utf-8 -*-
"""「左端へ」とタブを選ぶスライドバーのテスト（サンドボックスで実行）。"""
import importlib.util, os, time
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c:
		fails.append(l)
def pump(app, n=3):
	for _ in range(n):
		app.update_idletasks(); app.update()

app = frs.MainMenuGUI(); pump(app, 5)
app._ankle_folder_filter = app._ANKLE_FOLDER_ALL
app._ankle_rebuild_tabbar()
for i in range(app.notebook.index("end")):
	if "ankle" in app.notebook.tab(i, "text"):
		app.notebook.select(i)
pump(app, 5)
for _ in range(10):
	time.sleep(0.1); pump(app, 3)
ui = app._tabbar_ui["ankle"]; cv = ui["canvas"]; sc = ui["slider"]
n = len(app._ankle_tabs)

# --- 並び（左端へ は ◀ の左） ---
order = [w.cget("text") for w in ui["row"].pack_slaves() if w.winfo_class() == "Button"]
print("   ボタンの並び(pack順):", order)
check(order == ["＋", "▶▶ 右端へ", "▶", "◀", "◀◀ 左端へ"], "左端へ は ◀ の左に並ぶ（画面上は 左端へ ◀ ▶ 右端へ ＋）")
for key in ("ankle", "knee", "av"):
	u = app._tabbar_ui[key]
	check(u.get("start") is not None and u.get("slider") is not None, f"[{key}] 左端へ とスライドバーがある")

# --- 左端へ ---
ui["end"].invoke(); pump(app)
check(cv.xview()[1] > 0.999, f"（準備）右端へ {cv.xview()}")
check(str(ui["start"].cget("state")) == "normal", "右端にいるとき 左端へ は押せる")
ui["start"].invoke(); pump(app, 5)
check(cv.xview()[0] < 0.001, f"左端へ → 左端まで行き、引き戻されない {cv.xview()}")
check(str(ui["start"].cget("state")) == "disabled", "左端にいるとき 左端へ は押せない")
active_before = app._ankle_active_tab
check(app._ankle_active_tab == active_before, "左端へ でタブは切り替わらない")

# --- スライドバー: 範囲と位置 ---
btns = app._tabbar_buttons("ankle")
check(len(btns) == n, f"スライドバーが数えるタブ = 帯のタブ {len(btns)}")
check(int(float(sc.cget("to"))) == n - 1, f"範囲 0〜{n - 1}")
check(int(round(float(sc.get()))) == app._ankle_active_tab, f"つまみの位置 = 開いているタブ ({app._ankle_active_tab})")
check("開いているタブ" in ui["slider_info"].get(), "表示: " + ui["slider_info"].get())

# --- 動かす（まだ切り替わらない）→ 離すと切り替わる ---
target = 3
sc.set(target); pump(app)
check(app._ankle_active_tab == active_before, "動かしただけでは切り替わらない")
check(app._ankle_tabs[target]["name"] in ui["slider_info"].get() and "離すと" in ui["slider_info"].get(),
      "動かすと、その位置のタブ名が出る: " + ui["slider_info"].get())
check(str(btns[target].cget("bg")) == "#fff3b0", "その位置のタブが黄色で示される")
lo, hi = cv.xview(); tot = ui["inner"].winfo_reqwidth()
check(lo * tot - 1 <= btns[target].winfo_x() <= hi * tot + 1, f"帯がそのタブの位置へ動く {cv.xview()}")
sc.set(10.4); pump(app)
check(str(btns[target].cget("bg")) != "#fff3b0", "さらに動かすと、前の黄色は元に戻る")
app._tabbar_slider_commit("ankle"); pump(app, 5)
check(app._ankle_active_tab == 10, f"離すと、つまみの位置（10.4 → 10）のタブに切り替わる (active={app._ankle_active_tab})")
check(int(round(float(ui["slider"].get()))) == 10 and "開いているタブ" in ui["slider_info"].get(),
      "切り替わったあと、つまみと表示が新しいタブに合う")
check(all(str(b.cget("bg")) != "#fff3b0" for b in app._tabbar_buttons("ankle")), "黄色の印は残らない")

# --- ホイール: 1つずつ進み、0.5 秒止まると切り替わる ---
app._tabbar_slider_step("ankle", +1); app._tabbar_slider_step("ankle", +1); pump(app)
check(app._ankle_active_tab == 10, "ホイール中はまだ切り替わらない")
time.sleep(0.7); pump(app, 5)
check(app._ankle_active_tab == 12, f"止まって 0.5 秒後に、2つ先のタブへ切り替わる (active={app._ankle_active_tab})")

# --- 端で止まる ---
sc.set(-5); pump(app); app._tabbar_slider_commit("ankle"); pump(app, 5)
check(app._ankle_active_tab == 0, "左へ振り切ると最初のタブ")
sc.set(n + 20); pump(app); app._tabbar_slider_commit("ankle"); pump(app, 5)
check(app._ankle_active_tab == n - 1, "右へ振り切ると最後のタブ")

# --- フォルダ表示中は、そのフォルダのタブだけをたどる ---
idx = [i for i, t in enumerate(app._ankle_tabs) if "15*15" in t["name"]]
app._ankle_set_tab_folder(idx, "15mm")
app._ankle_folder_combo.current(app._ankle_folder_combo_keys.index("15mm"))
app._ankle_on_folder_filter_changed(); pump(app, 5)
check(int(float(sc.cget("to"))) == len(idx) - 1, f"フォルダ表示中: 範囲は そのフォルダの {len(idx)} タブ")
sc.set(2); pump(app); app._tabbar_slider_commit("ankle"); pump(app, 5)
check(app._ankle_active_tab == idx[2], f"フォルダの3番目のタブ（全体では {idx[2]}）に切り替わる")

# --- 精度検証（av）のタブ帯でも使える ---
avu = app._tabbar_ui["av"]
nav = len(app._tabbar_buttons("av"))
if nav > 1:
	k = 1 if app._av_active_tab != 1 else 0
	avu["slider"].set(k); pump(app); app._tabbar_slider_commit("av"); pump(app, 5)
	check(app._av_active_tab == k, f"精度検証のタブ帯でも切り替わる (active={app._av_active_tab})")
app.destroy()
print("RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL: {fails}")
