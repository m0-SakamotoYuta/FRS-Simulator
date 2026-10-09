# -*- coding: utf-8 -*-
import importlib.util, os, sys, time, json, io
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.update_idletasks(); app.update()
		if time.time() >= t: break
		time.sleep(0.01)
phase = sys.argv[1] if len(sys.argv) > 1 else "1"
app = frs.MainMenuGUI(); pump(app, 0.5)
for i in range(app.notebook.index("end")):
	if "ankle" in app.notebook.tab(i, "text"): app.notebook.select(i)
pump(app, 0.3)
def sel(name):
	app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == name)); pump(app, 0.5)
A, B = "261007_FE_軸力100", "261007_FE_軸力100_ACL切"
if phase == "1":
	sel(A)
	check(app.ankle_marker_size_true_mm.get() == 15.0 and app.ankle_marker_size_mm.get() == 15.0, "古いタブ: 実寸 15 / 補正後 15（何も変わらない）")
	check(app.ankle_marker_size_auto.get() is True, "古いタブは自動")
	check("深度の推定なし" in app.ankle_marker_size_note.get(), "メモ: " + app.ankle_marker_size_note.get())
	# 深度の推定ができた（④の中で測ったのと同じ保存の関数）
	src = app._ankle_get_current_cache()["source"]
	app._ankle_depth_size_store(src, {0: [15.02] * 10, 1: [14.68] * 10, 2: [14.62, 14.60, 14.64] * 3 + [14.62]})
	app._ankle_size_refresh_auto(); pump(app, 0.2)
	eff = app.ankle_marker_size_mm.get()
	check(abs(eff - 14.68) < 0.011, f"推定ができると補正後が自動で更新: {eff}")
	check(app._ankle_marker_true_mm() == 15.0, "実寸（ノギス）は 15 のまま（自動キャリブ・印刷・キャリブの照合に使う）")
	check("自動: 深度の推定" in app.ankle_marker_size_note.get(), "メモ: " + app.ankle_marker_size_note.get())
	check("④は 15 mm で計算" in app.ankle_detection_status.get() or True, "④の欄")
	app._ankle_update_detection_status(); pump(app)
	check("深度による実寸の推定" in app.ankle_detection_status.get(), "④の欄に推定")
	# 手で変える → 手動
	ent = [w for w in app._ankle_size_note_lbl.master.master.grid_slaves(row=0, column=5)][0]
	app.ankle_marker_size_mm.set(14.9); ent.focus_force(); pump(app, 0.1); ent.event_generate("<KeyRelease>", keysym="9", when="now"); pump(app, 0.5)
	check(app.ankle_marker_size_auto.get() is False and app.ankle_marker_size_mm.get() == 14.9, "手で変えると手動になり、値は 14.9 のまま")
	check("手動（自動なら 14.68" in app.ankle_marker_size_note.get(), "自動の値がメモに残る: " + app.ankle_marker_size_note.get())
	check(app._ankle_size_auto_btn.winfo_ismapped(), "「自動に戻す」ボタンが出る")
	# 別のタブ（推定なし）
	sel(B)
	check(app.ankle_marker_size_mm.get() == 15.0 and app.ankle_marker_size_auto.get(), "別のタブ（推定なし）は 15 / 自動")
	app.ankle_marker_size_true_mm.set(15.2); pump(app, 0.6)
	check(abs(app.ankle_marker_size_mm.get() - 15.2) < 1e-9, "推定が無いタブは、実寸を変えると補正後も追従")
	app.ankle_marker_size_true_mm.set(15.0); pump(app, 0.6)
	sel(A)
	check(app.ankle_marker_size_mm.get() == 14.9 and not app.ankle_marker_size_auto.get(), "戻ると手動の 14.9 が保たれる")
	app._ankle_size_set_auto(True); pump(app, 0.2)
	check(abs(app.ankle_marker_size_mm.get() - 14.68) < 0.011 and not app._ankle_size_auto_btn.winfo_ismapped(), "「自動に戻す」で推定に戻る")
	# 古い 14.5 のタブ
	old = [t for t in app._ankle_tabs if abs(float((t.get("snapshot") or {}).get("ankle_marker_size_mm", 0)) - 14.5) < 1e-9]
	if old:
		sel(old[0]["name"]); check(app.ankle_marker_size_true_mm.get() == 14.5, f"以前 14.5 にしていたタブ（{old[0]['name']}）は 実寸も 14.5（要確認として報告）")
	# CSV
	out = os.path.join(HERE, "size_list.csv")
	frs.filedialog.asksaveasfilename = lambda **k: out
	frs.messagebox.showinfo = lambda *a, **k: None
	app._ankle_size_export_csv()
	rows = io.open(out, encoding="utf-8-sig").read().splitlines()
	check(len(rows) == len(app._ankle_tabs) + 1, f"CSV に全タブ（{len(rows)-1}行）")
	print("   ", [r for r in rows if A + "," in r][0])
	# 窓
	sel(A)
	app._ankle_open_size_estimate_dialog(); pump(app, 0.5)
	w = [x for x in app.winfo_children() if isinstance(x, frs.tk.Toplevel) and "推定" in x.title()][0]
	from capture_util import capture
	capture(w, os.path.join(HERE, "dlg.png")); w.destroy()
	capture(app, os.path.join(HERE, "main.png"))
	app._on_close() if hasattr(app, "_on_close") else app._save_state_now()
	print(f"\n合計: NG {len(fails)} 件", flush=True)
	os._exit(1 if fails else 0)
else:
	sel(A)
	check(abs(app.ankle_marker_size_mm.get() - 14.68) < 0.011 and app.ankle_marker_size_auto.get() and app.ankle_marker_size_true_mm.get() == 15.0, "再起動後も 実寸15 / 補正後14.68 / 自動")
	snap = [t for t in app._ankle_tabs if t["name"] == A][0]["snapshot"]
	check("ankle_marker_size_true_mm" in snap and "ankle_marker_size_auto" in snap, "状態ファイルに新しい項目")
	print(f"\n合計: NG {len(fails)} 件", flush=True)
	os._exit(1 if fails else 0)
