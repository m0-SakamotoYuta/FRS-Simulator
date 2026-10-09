# -*- coding: utf-8 -*-
"""まとめて解析のテスト。  python test_batch.py fast | real"""
import importlib.util, json, os, sys, time, copy, traceback
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(frs)
MODE = sys.argv[1] if len(sys.argv) > 1 else "fast"

fails = []
def check(cond, label):
	print(("  OK  " if cond else "  NG  ") + label, flush=True)
	if not cond:
		fails.append(label)

def pump(app, n=3):
	for _ in range(n):
		app.update_idletasks(); app.update()

# ダイアログが出たら記録する（まとめて解析中は 1 回も出てはいけない）
dialogs = []
def _rec(kind, ret=None):
	def f(*a, **k):
		dialogs.append((kind, a[:2]))
		return ret
	return f
for kind in ("showwarning", "showerror", "showinfo"):
	setattr(frs.messagebox, kind, _rec(kind))
frs.filedialog.askopenfilename = _rec("askopenfilename", "")
ask_answer = [False]
frs.messagebox.askyesno = lambda *a, **k: (dialogs.append(("askyesno", a[:2])), ask_answer[0])[1]

app = frs.MainMenuGUI()
pump(app, 5)

def tab_by(sub):
	for t in app._ankle_tabs:
		if sub in t["name"]:
			return t
	raise KeyError(sub)

def poses_of(cache):
	return {aid: (np.array(b["poses"]), np.array(b["detected"])) for aid, b in (cache.get("bones") or {}).items()}

if MODE == "fast":
	T_ok = tab_by("AP_FE90")                 # 録画あり・以前の結果あり
	T_missing = tab_by("15*15cm_X")          # 録画ファイルが見つからない
	T_other = tab_by("15*15cm_斜めから_Z")   # 録画あり
	orig_active = app._ankle_tabs[app._ankle_active_tab]
	old_cache = copy.deepcopy(app._ankle_pose_cache.get(T_ok["name"]))
	check(old_cache is not None and int(old_cache.get("frame_count", 0)) == 1818, "以前の結果 (AP_FE90, 1818枚) が読めている")

	# 検出の重い部分を差し替える（呼ばれた引数を記録して、すぐ返す）
	calls = []
	real_bag = app._ankle_detect_from_bag
	def fake_bag(bag_path, aruco_dict_name, marker_size_mm, target_ids, stride, upd, cancel_check):
		calls.append(dict(bag=os.path.basename(bag_path), size=marker_size_mm, ids=sorted(target_ids),
		                  stride=stride, dict=aruco_dict_name))
		c = copy.deepcopy(old_cache); c["marker_size_mm"] = marker_size_mm; c["source"] = bag_path
		return c
	app._ankle_detect_from_bag = fake_bag

	# --- (a) 正常 + ファイル無し: ダイアログを出さずに最後まで進む ---
	dialogs.clear()
	res = app._ankle_run_batch([(T_ok, 14.5), (T_missing, 15.0), (T_other, 15.0)], parent=app)
	st = [r[1] for r in res]
	print("   結果:", [(r[0][:28], r[1], r[2].splitlines()[0][:50]) for r in res])
	check(st == ["ok", "error", "ok"], f"状態 = {st}（期待 ok/error/ok）")
	check("見つかりません" in res[1][2], "ファイル無しのタブは理由つきでエラー")
	check(dialogs == [], f"実行中にダイアログが1つも出ない（出たもの: {dialogs}）")
	check([c["size"] for c in calls] == [14.5, 15.0], f"各タブに指定した実寸で検出が呼ばれる {[c['size'] for c in calls]}")
	check(calls[0]["ids"] == [0, 1, 2] and calls[1]["ids"] == [1, 2], f"各タブの骨リストのIDが使われる {[c['ids'] for c in calls]}")
	check(calls[0]["bag"].startswith("ankle_d405_20260910_185656") and calls[1]["bag"].startswith("ankle_d405_20260908_173332"),
	      f"各タブの②の録画が使われる {[c['bag'] for c in calls]}")
	check(app._ankle_tabs[app._ankle_active_tab] is orig_active, "終わったら元のタブに戻る")
	check(float(T_ok["snapshot"]["ankle_marker_size_mm"]) == 14.5, "指定した実寸がタブに保存される (14.5)")
	check(float(app._ankle_pose_cache[T_ok["name"]]["marker_size_mm"]) == 14.5, "結果がそのタブのキャッシュに入る")
	p = app._ankle_pose_cache_path(T_ok["name"])
	check(p.exists() and time.time() - p.stat().st_mtime < 60, f"結果がファイルに自動保存される ({p.name})")
	with open("frs2015_gui_state_ankle_sim.json", encoding="utf-8") as f:
		saved = json.load(f)
	st_tab = [t for t in saved["tabs"] if t["name"] == T_ok["name"]][0]
	check(float(st_tab["snapshot"]["ankle_marker_size_mm"]) == 14.5, "状態ファイルにも実寸が保存される")
	check(not app._ankle_batch_running, "実行中フラグが戻る")

	# --- (b) キャンセル: 結果を上書きせず、「続けない」で残りは未実行 ---
	app._ankle_pose_cache[T_ok["name"]] = copy.deepcopy(old_cache)
	def cancel_bag(*a, **k):
		app._ankle_detect_cancel = True
		c = copy.deepcopy(old_cache); c["frame_count"] = 7; c["marker_size_mm"] = 99.0
		return c
	app._ankle_detect_from_bag = cancel_bag
	dialogs.clear(); ask_answer[0] = False
	res = app._ankle_run_batch([(T_ok, 15.0), (T_other, 15.0)], parent=app)
	check([r[1] for r in res] == ["cancel", "skip"], f"キャンセル → 残りは未実行 {[r[1] for r in res]}")
	check(int(app._ankle_pose_cache[T_ok["name"]]["frame_count"]) == 1818, "キャンセルしたタブの結果は以前のまま (1818枚)")
	check([d[0] for d in dialogs] == ["askyesno"], f"出るのは「続けますか？」だけ {[d[0] for d in dialogs]}")
	# 「続ける」を選んだ場合
	calls.clear(); dialogs.clear(); ask_answer[0] = True
	seq = [cancel_bag, fake_bag]
	app._ankle_detect_from_bag = lambda *a, **k: seq.pop(0)(*a, **k)
	res = app._ankle_run_batch([(T_ok, 15.0), (T_other, 15.0)], parent=app)
	check([r[1] for r in res] == ["cancel", "ok"], f"「続ける」なら次のタブを実行 {[r[1] for r in res]}")

	# --- (c) 手動の ④ はこれまでどおり（ファイル選択 → 完了ダイアログ） ---
	app._ankle_detect_from_bag = fake_bag
	idx = app._ankle_tabs.index(T_ok)
	app.on_ankle_tab_select(idx); pump(app)
	dialogs.clear()
	r = app.on_ankle_detect_markers()
	kinds = [d[0] for d in dialogs]
	check(kinds == ["askopenfilename", "showinfo"], f"手動の④: ファイル選択 → 完了ダイアログ {kinds}")
	check(r is None, "手動の④の戻り値は従来どおり None")
	# 手動の④でファイルが見つからないとき → 警告ダイアログ（従来どおり）
	app.on_ankle_tab_select(app._ankle_tabs.index(T_missing)); pump(app)
	dialogs.clear()
	app.on_ankle_detect_markers()
	check([d[0] for d in dialogs] == ["askopenfilename", "showwarning"], f"手動の④: 見つからない → 警告 {[d[0] for d in dialogs]}")

	# --- (d) まとめて解析の画面から実行（チェック → 実行ボタン） ---
	app.on_ankle_tab_select(app._ankle_tabs.index(orig_active)); pump(app)
	app._ankle_detect_from_bag = fake_bag
	before = set(app.winfo_children())
	app._ankle_open_batch_dialog(); pump(app, 5)
	win = [w for w in app.winfo_children() if w not in before and w.winfo_class() == "Toplevel"][0]
	# 画面の中の部品を探す
	def walk(w):
		yield w
		for c in w.winfo_children():
			yield from walk(c)
	entries = [w for w in walk(win) if w.winfo_class() == "TEntry"]
	checks = [w for w in walk(win) if w.winfo_class() == "TCheckbutton"][2:]   # [0][1] = do-what checkboxes
	buttons = {w.cget("text"): w for w in walk(win) if w.winfo_class() in ("TButton", "Button")}
	print("   ボタン:", list(buttons))
	n_rows = len(app._ankle_tabs)
	check(len(checks) == n_rows, f"全タブぶんの行がある ({len(checks)})")
	# 名前で絞り込み → 表示中をすべてチェック
	q_entry = entries[1]        # [0]=一括指定? 並びを確認
	print("   Entry 数:", len(entries))
	dialogs.clear()
	ask_answer[0] = True
	# AP_FE90 の行だけチェックし、実寸を 15.2 にする
	k = app._ankle_tabs.index(T_ok)
	checks[k].invoke()
	size_entries = entries[2:]   # [0]=名前絞り込み, [1]=一括指定, 以降=各行
	size_entries[k].delete(0, "end"); size_entries[k].insert(0, "15.2")
	pump(app)
	calls.clear()
	buttons["▶ チェックしたタブを解析"].invoke(); pump(app, 5)
	kinds = [d[0] for d in dialogs]
	check(kinds == ["askyesno", "showinfo"], f"画面から実行: 確認 → 結果まとめ {kinds}")
	check(len(calls) == 1 and calls[0]["size"] == 15.2, f"画面で入れた実寸で実行 {[c['size'] for c in calls]}")
	print("   結果まとめ:", dialogs[-1][1][1].replace("\n", " / ") if dialogs else "")
	win.destroy(); pump(app)
	app._ankle_detect_from_bag = real_bag

elif MODE == "real":
	# 本物の検出で「手動の④」と「まとめて解析」を同じ録画に走らせ、結果が完全に一致するか
	T = tab_by("AP_FE90")
	old = copy.deepcopy(app._ankle_pose_cache.get(T["name"]))
	size = float(old.get("marker_size_mm"))
	app.on_ankle_tab_select(app._ankle_tabs.index(T)); pump(app)
	app.ankle_marker_size_mm.set(size)
	t0 = time.time(); dialogs.clear()
	app.on_ankle_detect_markers()             # 手動の④（ファイル選択は "" = ②の設定を使う）
	manual = copy.deepcopy(app._ankle_pose_cache.get(T["name"]))
	print(f"  手動の④: {time.time() - t0:.0f}秒, ダイアログ {[d[0] for d in dialogs]}", flush=True)
	app.on_ankle_tab_select(0 if app._ankle_tabs.index(T) != 0 else 1); pump(app)
	t0 = time.time(); dialogs.clear()
	res = app._ankle_run_batch([(T, size)], parent=app)
	print(f"  まとめて解析: {time.time() - t0:.0f}秒  結果 {res[0][1]}", flush=True)
	check(res[0][1] == "ok", "まとめて解析が成功")
	check(dialogs == [], f"まとめて解析ではダイアログが出ない {dialogs}")
	new = app._ankle_pose_cache.get(T["name"])
	for label, A, B in (("手動の④ と まとめて解析", manual, new), ("（参考）9/10 の結果 と まとめて解析", old, new)):
		print("  --", label)
		po, pn = poses_of(A), poses_of(B)
		ok = int(A["frame_count"]) == int(B["frame_count"]) and sorted(po) == sorted(pn)
		worst = 0.0; det_same = True
		for aid in sorted(set(po) & set(pn)):
			(Pa, Da), (Pb, Db) = po[aid], pn[aid]
			det_same &= bool(np.array_equal(Da, Db))
			m = Da & Db
			if m.any():
				worst = max(worst, float(np.nanmax(np.abs(Pa[m] - Pb[m]))))
			print(f"     ID{aid}: 検出 {int(Da.sum())} / {int(Db.sum())} 枚, 姿勢の最大差 {float(np.nanmax(np.abs(Pa[m] - Pb[m]))) if m.any() else 0:.3e}")
		ts_same = np.array_equal(np.asarray(A["timestamps"]), np.asarray(B["timestamps"]))
		if label.startswith("手動"):
			check(ok and det_same and ts_same and worst == 0.0,
			      f"手動の④ と まとめて解析 が完全一致（枚数・ID・検出・時刻・姿勢 差 {worst:.1e}）")
		else:
			print(f"     枚数一致={ok}, 検出一致={det_same}, 時刻一致={ts_same}, 姿勢の最大差 {worst:.3e}")

app.destroy()
print()
print("RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL: {fails}")
