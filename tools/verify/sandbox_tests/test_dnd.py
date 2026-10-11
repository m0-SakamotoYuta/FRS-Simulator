# -*- coding: utf-8 -*-
"""ドラッグ＆ドロップ: tkdnd の Tcl 側（OS から受け取った直後）からドロップを再現して確かめる。"""
import importlib.util, os, sys, ctypes
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
msgs = []
frs.messagebox.showwarning = lambda t, m, **k: msgs.append(m)
frs.messagebox.showinfo = lambda t, m, **k: msgs.append(m)
u32 = ctypes.windll.user32
u32.GetPropW.restype = ctypes.c_void_p; u32.GetPropW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p]
app = frs.MainMenuGUI(); app.update()
for i in range(app.notebook.index("end")):
	if "ankle" in app.notebook.tab(i, "text"): app.notebook.select(i)
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_FE_軸力100"))
for _ in range(5): app.update()
def find_entry_by_label(text):
	hit = []
	def walk(w):
		for c in w.winfo_children():
			try:
				if c.winfo_class() == "TLabel" and text in str(c.cget("text")): hit.append(c)
			except Exception: pass
			walk(c)
	walk(app)
	lab = hit[0]
	r = int(lab.grid_info()["row"])
	cands = [w for w in lab.master.grid_slaves(row=r) if w.winfo_class() in ("TEntry", "TFrame")]
	e = [w for w in cands if w.winfo_class() == "TEntry"] or [w.winfo_children()[0] for w in cands]
	return e[0]
def drop(entry, path):
	f = path.replace("\\", "/")
	x = entry.winfo_rootx() + 10; y = entry.winfo_rooty() + 5
	data = app.tk.call("list", f)
	app.tk.call("tkdnd::olednd::HandleDragEnter", str(entry), ("CF_HDROP", "DND_Files"), ("copy",), (), x, y, (), data)
	app.tk.call("tkdnd::olednd::HandleDragOver", str(entry), (), x, y)
	r = app.tk.call("tkdnd::olednd::HandleDrop", str(entry), (), x, y, "DND_Files", data)
	app.update()
	return r
def ole(w):
	return bool(u32.GetPropW(int(w.winfo_id()), "OleDropTargetInterface"))
obj = os.path.join(HERE, "drop files", "テスト モデル.obj")
db3 = os.path.join(HERE, "drop files", "テスト 録画.db3")
kkr = os.path.join(HERE, "drop files", "ログ 1.KKR")
app._ankle_selected_bone = 1; app._ankle_load_editor_from_bone(); app.update()
b = app._ankle_current_bone()
before = b.get("model_path")
e_model = find_entry_by_label("骨モデル (可視化用フル)")
check(ole(e_model), "③の骨モデルの欄に Windows のドロップ受け付けの印")
drop(e_model, obj)
check(b.get("model_path", "").replace("\\", "/") == obj.replace("\\", "/"), f"③の骨モデル: 選択中の骨（{b.get('name')}）に保存")
check(os.path.basename(app._ankle_bone_editor_widgets["model_var"].get()) == "テスト モデル.obj" or "テスト モデル.obj" in app._ankle_bone_editor_widgets["model_var"].get(), "欄の表示も変わる")
e_cal = find_entry_by_label("キャリブ用モデル (省略可)")
drop(e_cal, obj)
check(b.get("calib_model_path", "").replace("\\", "/") == obj.replace("\\", "/"), "③のキャリブ用モデルにも入る")
msgs.clear(); prev = b.get("model_path"); drop(e_model, db3)
check(b.get("model_path") == prev and any("3Dモデル" in m for m in msgs), "③に .db3 を落とすと入れずに知らせる")
e_post = find_entry_by_label("初期スキャン側 骨領域")
drop(e_post, obj)
check(b.get("post_scan_path", "").replace("\\", "/") == obj.replace("\\", "/"), "③の初期スキャン側 骨領域にも入る")
# ② と datalog
e_depth = find_entry_by_label("深度データ")
drop(e_depth, db3)
check(app.ankle_depth_path.get().replace("\\", "/") == db3.replace("\\", "/"), "②の深度データ（従来どおり）")
hit = []
def walk(w):
	for c in w.winfo_children():
		if c.winfo_class() == "TEntry" and str(c.cget("textvariable")) == str(app.ankle_robot_log_path): hit.append(c)
		walk(c)
walk(app)
check(len(hit) == 1 and ole(hit[0]), "⑤の datalog の欄に受け付けの印")
drop(hit[0], kkr)
check(app.ankle_robot_log_path.get().replace("\\", "/") == kkr.replace("\\", "/"), "⑤の datalog の欄に入る")
print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
