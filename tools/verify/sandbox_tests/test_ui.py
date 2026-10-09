# -*- coding: utf-8 -*-
"""フォルダ・右端へ・画面構築・保存/復元のテスト（サンドボックス内のコピーで実行）。"""
import importlib.util, json, os, sys, traceback
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(frs)

fails = []
def check(cond, label):
	print(("  OK  " if cond else "  NG  ") + label)
	if not cond:
		fails.append(label)

def pump(app, n=3):
	for _ in range(n):
		app.update_idletasks(); app.update()

def make_app():
	app = frs.MainMenuGUI()
	pump(app, 5)
	return app

app = make_app()
assert os.path.dirname(os.path.abspath(frs.__file__)) == HERE
ALL = app._ANKLE_FOLDER_ALL
n_tabs = len(app._ankle_tabs)
print(f"タブ数 {n_tabs}, 開いているタブ = {app._ankle_tabs[app._ankle_active_tab]['name']}")

# --- 1. 初期表示: フォルダなし → 全部並ぶ ---
check(app._ankle_folder_filter == ALL, "初期のフォルダ表示は「すべて」")
check(len(app._ankle_tab_buttons) == n_tabs, f"帯に全タブ {n_tabs} 個が並ぶ")
check(app._ankle_tab_button_idx == list(range(n_tabs)), "ボタン位置とタブ番号が一致")
vals = list(app._ankle_folder_combo.cget("values"))
check(vals[0].startswith("すべて表示") and vals[-1].startswith("（未分類）"), f"フォルダの選択肢 {vals}")
check("開いているタブ" in app._ankle_current_tab_label_var.get(), "開いているタブの表示: " + app._ankle_current_tab_label_var.get())

# --- 2. 右端へボタン（ankle / knee / av すべて） ---
for key in ("ankle", "knee", "av"):
	ui = app._tabbar_ui.get(key)
	check(ui is not None and ui.get("end") is not None, f"[{key}] 右端へボタンがある")
	pack_order = [w for w in ui["row"].pack_slaves()]
	names = [w.cget("text") if hasattr(w, "cget") and w.winfo_class() == "Button" else w.winfo_class() for w in pack_order]
	print(f"     [{key}] 並び(pack順): {names}")
app.ankle_notebook_select = None
try:
	# ankle タブを前面に出して幅を確定させる
	for i in range(app.notebook.index("end")):
		if "ankle" in app.notebook.tab(i, "text"):
			app.notebook.select(i)
except Exception as e:
	print("notebook select:", e)
pump(app, 5)
import time as _t
for _ in range(10):
	_t.sleep(0.1); pump(app, 3)   # window settles
cv = app._tabbar_ui["ankle"]["canvas"]
cv.xview_moveto(0.0); app._tabbar_update_arrows("ankle"); pump(app)
lo, hi = cv.xview()
check(hi < 0.999, f"左端にいるとき 右端が見えていない (xview={lo:.3f},{hi:.3f})")
check(str(app._tabbar_ui["ankle"]["end"].cget("state")) == "normal", "右端へボタンは押せる")
app._tabbar_ui["ankle"]["end"].invoke(); pump(app)
lo, hi = cv.xview()
check(hi > 0.999, f"右端へ を押すと右端まで行く (xview={lo:.3f},{hi:.3f})")
check(str(app._tabbar_ui["ankle"]["end"].cget("state")) == "disabled", "右端にいるときは右端へボタンが押せない")
active_before = app._ankle_active_tab
check(app._ankle_active_tab == active_before, "右端へ を押してもタブは切り替わらない")

# --- 3. フォルダへ移す・絞り込み ---
A = [i for i, t in enumerate(app._ankle_tabs) if "15*15" in t["name"]]
B = [i for i, t in enumerate(app._ankle_tabs) if "10*10" in t["name"] or "10×10" in t["name"]]
print(f"  15mm系 {len(A)} タブ, 10mm系 {len(B)} タブ")
app._ankle_set_tab_folder(A, "15mm検証")
app._ankle_set_tab_folder(B, "10mm検証")
check(app._ankle_all_folders() == ["15mm検証", "10mm検証"], f"フォルダ一覧 {app._ankle_all_folders()}")
cnt = app._ankle_folder_counts()
check(cnt.get("15mm検証") == len(A) and cnt.get("10mm検証") == len(B), f"件数 {cnt}")
# 絞り込み: 15mm検証 を選ぶ
keys = app._ankle_folder_combo_keys
app._ankle_folder_combo.current(keys.index("15mm検証"))
app._ankle_on_folder_filter_changed(); pump(app)
check(app._ankle_folder_filter == "15mm検証", "フォルダ表示が 15mm検証 になる")
check(app._ankle_tab_button_idx == A, "帯には 15mm検証 のタブだけが並ぶ")
check(app._ankle_active_tab == A[-1], f"開いていたタブがフォルダ外 → フォルダの右端を開く (active={app._ankle_active_tab}, 期待 {A[-1]})")
check(app._ankle_tabs[app._ankle_active_tab]["name"] in app._ankle_current_tab_label_var.get(), "表示ラベルが追随")

# 右クリックメニューの左右移動: 見えている隣と入れ替わる
i0, i1 = A[0], A[1]
name0, name1 = app._ankle_tabs[i0]["name"], app._ankle_tabs[i1]["name"]
vis = app._ankle_visible_indices(); pos = vis.index(i0)
app._ankle_tab_move(i0, vis[pos + 1]); pump(app)
names_vis = [app._ankle_tabs[k]["name"] for k in app._ankle_visible_indices()]
check(names_vis[0] == name1 and names_vis[1] == name0, "→右へ移動: 見えている隣と入れ替わる")
app._ankle_tab_move(app._ankle_visible_indices()[1], app._ankle_visible_indices()[0]); pump(app)
names_vis = [app._ankle_tabs[k]["name"] for k in app._ankle_visible_indices()]
check(names_vis[0] == name0 and names_vis[1] == name1, "←左へ移動で元に戻る")
check(len(app._ankle_tabs) == n_tabs, "移動でタブ数は変わらない")

# ドラッグの落下位置 → タブ番号への変換
check(app._ankle_tab_full_index(0) == app._ankle_tab_button_idx[0], "ボタン位置0 → タブ番号")
check(app._ankle_tab_full_index(2) == app._ankle_tab_button_idx[2], "ボタン位置2 → タブ番号")

# 右クリックメニューが例外なく作れる（表示はしない）
import tkinter as tk
orig_popup = tk.Menu.tk_popup
tk.Menu.tk_popup = lambda self, *a, **k: None
try:
	class E: x_root = 10; y_root = 10
	app._ankle_tab_context_menu(E(), A[0])
	check(True, "右クリックメニューを作れる")
except Exception as e:
	traceback.print_exc(); check(False, f"右クリックメニュー例外 {e}")
finally:
	tk.Menu.tk_popup = orig_popup

# --- 4. ＋（フォルダ表示中）→ そのフォルダに、フォルダ内右端の複製で作られる ---
src_snap = app._ankle_tabs[app._ankle_visible_indices()[-1]]["snapshot"]
app.on_ankle_tab_add(); pump(app)
new = app._ankle_tabs[-1]
check(new.get("folder") == "15mm検証", "＋で作ったタブは 15mm検証 に入る")
check(new["snapshot"].get("ankle_marker_size_mm") == src_snap.get("ankle_marker_size_mm"), "複製元はフォルダ内右端のタブ")
check(new["snapshot"].get("ankle_depth", "") in ("", None) or new["snapshot"].get("ankle_depth") == app._ankle_default_snap.get("ankle_depth"), "④(録画)は複製しない（既存仕様）")
check(app._ankle_active_tab == len(app._ankle_tabs) - 1 and app._ankle_tab_button_idx[-1] == len(app._ankle_tabs) - 1, "新しいタブが帯の右端で開く")

# --- 5. 削除: 次に開くのは表示中フォルダのタブ ---
import tkinter.messagebox as mb
orig_ask = mb.askyesno
frs.messagebox.askyesno = lambda *a, **k: True
try:
	app.on_ankle_tab_delete(len(app._ankle_tabs) - 1); pump(app)
finally:
	frs.messagebox.askyesno = orig_ask
check(len(app._ankle_tabs) == n_tabs, "削除で元のタブ数に戻る")
check(app._ankle_tab_visible(app._ankle_active_tab), "削除後に開くタブは表示中フォルダの中")

# --- 6. 未分類 表示 ---
app._ankle_folder_combo.current(app._ankle_folder_combo_keys.index(""))
app._ankle_on_folder_filter_changed(); pump(app)
unf = [i for i, t in enumerate(app._ankle_tabs) if not t.get("folder")]
check(app._ankle_tab_button_idx == unf, f"未分類表示 {len(unf)} タブ")

# --- 7. 画面: フォルダ整理・まとめて解析 が例外なく開く ---
for fn, label in ((app._ankle_open_folder_manager, "フォルダ整理画面"), (app._ankle_open_batch_dialog, "まとめて解析画面")):
	before = set(app.winfo_children())
	try:
		fn(); pump(app, 5)
		wins = [w for w in app.winfo_children() if w not in before and w.winfo_class() == "Toplevel"]
		check(len(wins) == 1, f"{label}が開く")
		if label == "まとめて解析画面":
			try:
				from PIL import ImageGrab
				w = wins[0]; w.lift(); pump(app, 5)
				x, y, ww, hh = w.winfo_rootx(), w.winfo_rooty(), w.winfo_width(), w.winfo_height()
				ImageGrab.grab(bbox=(x, y, x + ww, y + hh)).save(os.path.join(HERE, "shot_batch.png"))
			except Exception as e:
				print("   screenshot:", e)
		if label == "フォルダ整理画面":
			try:
				from PIL import ImageGrab
				w = wins[0]; w.lift(); pump(app, 5)
				x, y, ww, hh = w.winfo_rootx(), w.winfo_rooty(), w.winfo_width(), w.winfo_height()
				ImageGrab.grab(bbox=(x, y, x + ww, y + hh)).save(os.path.join(HERE, "shot_folder.png"))
			except Exception as e:
				print("   screenshot:", e)
		for w in wins:
			w.destroy()
		pump(app)
	except Exception as e:
		traceback.print_exc(); check(False, f"{label} 例外 {e}")

# 本体のツールバーのスクショ
app._ankle_folder_combo.current(app._ankle_folder_combo_keys.index("15mm検証"))
app._ankle_on_folder_filter_changed(); pump(app, 5)
try:
	from PIL import ImageGrab
	row = app._tabbar_ui["ankle"]["row"]; top = app._ankle_folder_combo.master
	x, y = top.winfo_rootx(), top.winfo_rooty()
	ImageGrab.grab(bbox=(x, y, x + row.winfo_width(), row.winfo_rooty() + row.winfo_height() + 4)).save(os.path.join(HERE, "shot_toolbar.png"))
except Exception as e:
	print("   screenshot:", e)

# --- 8. 保存 → 別インスタンスで復元 ---
active_name = app._ankle_tabs[app._ankle_active_tab]["name"]
app._save_ankle_state(save_pose_caches=False)
with open("frs2015_gui_state_ankle_sim.json", encoding="utf-8") as f:
	data = json.load(f)
check(data.get("folders") == ["15mm検証", "10mm検証"], f"保存: folders {data.get('folders')}")
check(data.get("folder_filter") == "15mm検証", "保存: folder_filter")
check(sum(1 for t in data["tabs"] if t.get("folder") == "15mm検証") == len(A), "保存: 各タブの folder")
app.destroy()

app2 = make_app()
check(app2._ankle_folder_filter == "15mm検証", "復元: フォルダ表示")
check(app2._ankle_all_folders() == ["15mm検証", "10mm検証"], "復元: フォルダ一覧と順番")
check(app2._ankle_tab_button_idx == [i for i, t in enumerate(app2._ankle_tabs) if t.get("folder") == "15mm検証"], "復元: 帯は 15mm検証 だけ")
check(app2._ankle_tabs[app2._ankle_active_tab]["name"] == active_name, "復元: 開いていたタブ")
check(len(app2._ankle_tabs) == n_tabs, "復元: タブ数")

# --- 9. フォルダ削除相当: 中身は未分類に戻る（タブは消えない） ---
for t in app2._ankle_tabs:
	if t.get("folder") == "10mm検証":
		t.pop("folder", None)
app2._ankle_folders = [x for x in app2._ankle_all_folders() if x != "10mm検証"]
app2._ankle_rebuild_tabbar()
check(app2._ankle_all_folders() == ["15mm検証"] and len(app2._ankle_tabs) == n_tabs, "フォルダを消してもタブは残る")
app2.destroy()

print()
print("RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL: {fails}")
