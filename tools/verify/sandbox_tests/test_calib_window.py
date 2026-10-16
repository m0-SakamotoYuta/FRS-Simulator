# -*- coding: utf-8 -*-
"""手動キャリブの窓: ガイドを × で閉じても進む、クリック画面が前面・日本語の題名、2回目も出る（本物の流れ）。"""
import importlib.util, os, sys, time, threading, ctypes, ctypes.wintypes as W, re
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import tkinter as tk, pyvista as pv
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
u32 = ctypes.windll.user32; PID = os.getpid()
EnumProc = ctypes.WINFUNCTYPE(W.BOOL, W.HWND, W.LPARAM)
def wins():
	out = []
	def cb(h, l):
		pid = W.DWORD(); u32.GetWindowThreadProcessId(h, ctypes.byref(pid))
		if pid.value == PID and u32.IsWindowVisible(h):
			buf = ctypes.create_unicode_buffer(256); u32.GetWindowTextW(h, buf, 256)
			cls = ctypes.create_unicode_buffer(64); u32.GetClassNameW(h, cls, 64)
			out.append((h, buf.value, cls.value))
		return True
	u32.EnumWindows(EnumProc(cb), 0); return out
seen = {}
def close_dialog_soon():
	def run():
		t0 = time.time()
		while time.time() - t0 < 30:
			d = next((h for h, t, c in wins() if c == "#32770"), None)
			if d:
				seen["dialog_fg"] = (u32.GetForegroundWindow() == d); time.sleep(0.3)
				u32.PostMessageW(d, 0x0010, 0, 0); return
			time.sleep(0.2)
	threading.Thread(target=run, daemon=True).start()
# クリック画面: 本物の show（閉じるまで待つ）。最初の描画のあとで確かめて閉じる
_orig_show = pv.Plotter.show
def _show(self, *a, **k):
	st = {"n": 0}
	def obs(*_):

		st["n"] += 1
		if st["n"] == 2:
			m = re.search(r"([0-9a-fA-F]{6,})", str(self.render_window.GetGenericWindowId()))
			h = int(m.group(1), 16)
			buf = ctypes.create_unicode_buffer(256); u32.GetWindowTextW(ctypes.c_void_p(h), buf, 256)
			seen["pick_title"] = buf.value; seen["pick_fg"] = (u32.GetForegroundWindow() == h)
			seen["pick_vis"] = bool(u32.IsWindowVisible(h))
			self.iren.interactor.TerminateApp()
	self.iren.add_observer("RenderEvent", obs)
	def kick():
		for _ in range(3):
			time.sleep(0.8)
	return _orig_show(self, *a, **k)
pv.Plotter.show = _show
app = frs.MainMenuGUI(); app.update()
for i in range(app.notebook.index("end")):
	if "ankle" in app.notebook.tab(i, "text"): app.notebook.select(i)
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_FE_軸力100"))
app._ankle_selected_bone = 2; app.update()
def guide_win():
	return next((w for w in app.winfo_children() if isinstance(w, tk.Toplevel) and "マーカー-骨キャリブ:" in w.title()), None)
def on_guide(action):
	def f():
		g = guide_win()
		if g is None:
			app.after(200, f); return
		seen.setdefault("guides", 0); seen["guides"] += 1
		if action == "x":
			g.tk.call("wm", "protocol", str(g), "WM_DELETE_WINDOW"); g.tk.eval(g.tk.call("wm", "protocol", str(g), "WM_DELETE_WINDOW"))
		elif action == "esc":
			g.event_generate("<Escape>")
		elif action == "cancel":
			def walk(x):
				for c in x.winfo_children():
					if c.winfo_class() == "TButton" and c.cget("text") == "やめる": c.invoke(); return True
					if walk(c): return True
			walk(g)
	app.after(800, f)
# 1) ガイドを × → クリック画面 → 点なしで閉じる → 知らせ
on_guide("x"); close_dialog_soon()
app.on_ankle_calibrate_marker_to_bone(); app.update()
check(seen.get("pick_vis") is True, "ガイドを × で閉じると、クリック画面が開く")
check(seen.get("pick_fg") is True, "クリック画面は前面に出る")
check(seen.get("pick_title", "").startswith("膝蓋骨: マーカー4隅"), f"クリック画面の題名が日本語: {seen.get('pick_title')}")
check(seen.get("dialog_fg") is True, "点が足りない知らせの窓も前面に出る")
# 2) もう一度押す → ガイドが出る（今度は Esc で中止）
seen.pop("pick_vis", None); n0 = seen.get("guides", 0)
on_guide("esc"); app.on_ankle_calibrate_marker_to_bone(); app.update()
check(seen.get("guides", 0) == n0 + 1 and "pick_vis" not in seen, "2回目もガイドが出る / Esc では中止（クリック画面は開かない）")
on_guide("cancel"); app.on_ankle_calibrate_marker_to_bone(); app.update()
check(seen.get("guides", 0) == n0 + 2 and "pick_vis" not in seen, "「やめる」でも中止")
check(app.grab_current() is None, "操作できない状態（grab）が残っていない")
print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
