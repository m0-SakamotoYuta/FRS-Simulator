# -*- coding: utf-8 -*-
"""まとめて解析の「マーカー実寸を一括指定」が、各タブの②マーカー実寸に反映されるか。"""
import importlib.util, os, sys, json, copy, time
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
EXPECT = sys.argv[1] if len(sys.argv) > 1 else "new"     # old = 修正前の動作を確かめる
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c:
		fails.append(l)
def pump(app, n=3):
	for _ in range(n):
		app.update_idletasks(); app.update()
def walk(w):
	yield w
	for c in w.winfo_children():
		yield from walk(c)
dialogs = []
for kind in ("showwarning", "showerror", "showinfo"):
	setattr(frs.messagebox, kind, lambda *a, _k=kind, **k: dialogs.append(_k))
frs.messagebox.askyesno = lambda *a, **k: (dialogs.append("askyesno"), True)[1]

app = frs.MainMenuGUI(); pump(app, 5)
app._ankle_folder_filter = app._ANKLE_FOLDER_ALL; app._ankle_rebuild_tabbar()
tabs = app._ankle_tabs
def size_of(i):
	if i == app._ankle_active_tab:
		return float(app.ankle_marker_size_mm.get())
	return float(tabs[i]["snapshot"].get("ankle_marker_size_mm", 20.0))

# 録画のあるタブ2つ（チェックできる）＋ 録画の無いタブ1つ（チェックできない）を選ぶ
has_src = [i for i, t in enumerate(tabs)
           if os.path.exists(str(t["snapshot"].get("ankle_depth", "") or "")) and i != app._ankle_active_tab]
no_src = [i for i, t in enumerate(tabs)
          if not os.path.exists(str(t["snapshot"].get("ankle_depth", "") or "")) and i != app._ankle_active_tab
          and not app._ankle_pose_cache.get(t["name"])]   # no recording and no pose result
A, B = has_src[0], has_src[1]
C = no_src[0]
act = app._ankle_active_tab
print(f"   A={tabs[A]['name']} ({size_of(A)}mm)  B={tabs[B]['name']} ({size_of(B)}mm)  C(録画なし)={tabs[C]['name']} ({size_of(C)}mm)")
print(f"   開いているタブ={tabs[act]['name']} ({size_of(act)}mm)")

def open_dialog():
	before = set(app.winfo_children())
	app._ankle_open_batch_dialog(); pump(app, 5)
	win = [w for w in app.winfo_children() if w not in before and w.winfo_class() == "Toplevel"][0]
	entries = [w for w in walk(win) if w.winfo_class() == "TEntry"]
	checks = [w for w in walk(win) if w.winfo_class() == "TCheckbutton"][2:]   # [0][1] = do-what checkboxes
	buttons = {str(w.cget("text")): w for w in walk(win) if w.winfo_class() in ("TButton", "Button")}
	return win, entries, checks, buttons

# --- 1) 一括指定して、解析せずに閉じる ---
win, entries, checks, buttons = open_dialog()
bulk = entries[1]
checks[A].invoke(); checks[B].invoke()
bulk.delete(0, "end"); bulk.insert(0, "15.5")
buttons["チェックしたタブに入れる"].invoke(); pump(app)
row_entries = entries[2:]
check(row_entries[A].get() == "15.5" and row_entries[B].get() == "15.5", "画面の実寸欄は 15.5 になる")
# 1行だけ手で書き換え（開いているタブの行）
row_entries[act].delete(0, "end"); row_entries[act].insert(0, "18"); pump(app)
win.destroy(); pump(app)
sA, sB, sAct = size_of(A), size_of(B), size_of(act)
print(f"   閉じたあとの各タブの②: A={sA} B={sB} 開いているタブ={sAct}")
if EXPECT == "old":
	check(sA != 15.5 and sB != 15.5, "【修正前】一括指定して閉じても、各タブの②は変わらない（ご指摘どおり）")
else:
	check(sA == 15.5 and sB == 15.5, "一括指定は、解析しなくても各タブの②に入る")
	check(sAct == 18.0, "画面で手で書き換えた実寸も、そのタブ（開いているタブ）の②に入る")
	check(abs(float(app.ankle_marker_size_mm.get()) - 18.0) < 1e-9, "開いているタブは②の入力欄そのものが変わる")

# --- 2) 録画の無いタブにも入れられるか（キャリブだけするタブ） ---
win, entries, checks, buttons = open_dialog()
check(str(checks[C].cget("state")) == "disabled", "（前提）録画の無いタブはチェックできない")
if EXPECT != "old":
	bulk = entries[1]
	bulk.delete(0, "end"); bulk.insert(0, "15")
	check("表示中のタブすべてに入れる" in buttons, "「表示中のタブすべてに入れる」ボタンがある")
	# 名前で C だけに絞り込んでから入れる
	q = entries[0]; q.delete(0, "end"); q.insert(0, tabs[C]["name"]); pump(app)
	buttons["表示中のタブすべてに入れる"].invoke(); pump(app)
	win.destroy(); pump(app)
	check(size_of(C) == 15.0, f"録画の無いタブにも入る（キャリブ用） {size_of(C)}")
	check(size_of(A) == 15.5, "絞り込みの外のタブは変わらない")
	# 不正な値は入れない
	win, entries, checks, buttons = open_dialog()
	entries[2:][A].delete(0, "end"); entries[2:][A].insert(0, "abc"); pump(app)
	entries[2:][B].delete(0, "end"); entries[2:][B].insert(0, "-3"); pump(app)
	win.destroy(); pump(app)
	check(size_of(A) == 15.5 and size_of(B) == 15.5, "数字でない値・0以下は入れない（元のまま）")
	# 状態ファイルに保存される（自動保存を待つ）
	time.sleep(2.0); pump(app, 5)
	app._save_ankle_state(save_pose_caches=False)
	d = json.load(open("frs2015_gui_state_ankle_sim.json", encoding="utf-8"))
	check(float(d["tabs"][C]["snapshot"]["ankle_marker_size_mm"]) == 15.0, "状態ファイルにも保存される")
	# タブを切り替えて開くと、②に出る
	app.on_ankle_tab_select(A); pump(app)
	check(float(app.ankle_marker_size_mm.get()) == 15.5, "A を開くと、②のマーカー実寸が 15.5")
else:
	win.destroy()

# --- 3) 解析を実行した場合（修正前から入る）---
app._ankle_detect_from_bag = lambda *a, **k: {"frame_count": 1, "bones": {}, "timestamps": [0.0],
                                              "marker_size_mm": a[2], "source": a[0], "aruco_dict": a[1]}
win, entries, checks, buttons = open_dialog()
checks[B].invoke()
entries[2:][B].delete(0, "end"); entries[2:][B].insert(0, "16.5"); pump(app)
buttons["▶ チェックしたタブを解析"].invoke(); pump(app, 5)
win.destroy(); pump(app)
check(size_of(B) == 16.5, f"解析を実行したタブは②に入る {size_of(B)}")
app.destroy()
print("RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL: {fails}")
