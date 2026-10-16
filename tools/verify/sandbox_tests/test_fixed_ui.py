# -*- coding: utf-8 -*-
"""③の「固定」の分かりやすさと、方眼紙・移動量が2回目も出るか（実データ・261007）。
2026-10-13: 2回目に方眼紙が出ない報告 → 大腿骨の固定が「固定/解除」で外れていた（骨リストの 🔒 が見えなかった）。"""
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np, tkinter as tk
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
def pump(app, sec=0.0):
	t = time.time() + sec
	while True:
		app.tk.dooneevent(tk._tkinter.DONT_WAIT)
		if time.time() >= t: break
frs.messagebox.askyesno = lambda *a, **k: True
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", a[:2])
frs.messagebox.showinfo = lambda *a, **k: print("   [info]", a[:2])
app = frs.MainMenuGUI(); app.update()
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, False), tk.BooleanVar(app, True))
TAB = "261007_AP_FE60_軸力なし"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.3)
for b in app.ankle_bones: b["fixed"] = False            # お手元と同じ: 固定が外れている
app._ankle_refresh_bone_listbox(); app.ankle_grid_show.set(True); app.ankle_disp_show.set(True); pump(app, 0.1)

def lb_items():
	lb = app._ankle_bones_listbox
	return [lb.get(i) for i in range(lb.size())]

def viz():
	app.on_ankle_animate(); pump(app, 1.5)
	v = app._sim_viewers_alive()[-1]
	v["set_paused"](False); pump(app, 0.8); v["set_paused"](True); pump(app, 0.2)
	p = v["plotter"]
	names = list(p.renderer.actors.keys())
	rens = p.render_window.GetRenderers(); rens.InitTraversal(); rl = [rens.GetNextItem() for _ in range(rens.GetNumberOfItems())]
	ov = sum(r.GetActors().GetNumberOfItems() for r in rl if r.GetLayer() == 1)
	texts = []
	for a in p.renderer.actors.values():
		try:
			if hasattr(a, "GetText"):
				texts.append(str(a.GetText(0)))
		except Exception:
			pass
	return v, names, ov, " ".join(texts)

def close(v):
	w = v["ctrl"]["window"]; w.tk.call(w.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)

# 1. 固定が無いとき
lbl = str(app._ankle_grid_fix_lbl.cget("text"))
check("固定" in lbl and "ありません" in lbl, f"⑤に「固定している骨がありません」と出る: {lbl[:40]}")
check(str(app._ankle_fix_btn.cget("text")) == "この骨を固定する", f"ボタン: {app._ankle_fix_btn.cget('text')}")
v, names, ov, txt = viz()
check(not any(n.startswith("grid") for n in names) and "固定" in txt, f"可視化の画面に理由が出る（左下）: {txt[:50]}")
close(v)

# 2. 大腿骨を固定
fi = next(i for i, b in enumerate(app.ankle_bones) if b.get("name") == "大腿骨")
app._ankle_selected_bone = fi; app._ankle_load_editor_from_bone(); pump(app, 0.1)
app.on_ankle_bone_toggle_fixed(); pump(app, 0.2)
items = lb_items()
print("    骨リスト:", items, flush=True)
check(items[fi].startswith("【固定】") and sum(x.startswith("【固定】") for x in items) == 1, "骨リストに【固定】の印（文字なので必ず見える）")
check(str(app._ankle_fix_btn.cget("text")) == "固定を外す（いま固定中）", f"ボタンの文字が状態に合わせて変わる: {app._ankle_fix_btn.cget('text')}")
check("大腿骨" in str(app._ankle_grid_fix_lbl.cget("text")), f"⑤: {app._ankle_grid_fix_lbl.cget('text')}")
# ほかの骨を選ぶとボタンは「固定する」
oi = next(i for i, b in enumerate(app.ankle_bones) if i != fi)
app._ankle_selected_bone = oi; app._ankle_load_editor_from_bone(); pump(app, 0.1)
check(str(app._ankle_fix_btn.cget("text")) == "この骨を固定する", "固定していない骨を選ぶと「この骨を固定する」")
# 共有しているほかのタブにも
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_FE_軸力100")); pump(app, 0.3)
check(any(b.get("fixed") and b.get("name") == "大腿骨" for b in app.ankle_bones), "骨リストを共有しているタブでも大腿骨が固定")
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.3)

# 3. 2回続けて可視化
for k in (1, 2, 3):
	v, names, ov, txt = viz()
	check("grid_minor" in names and "grid_major" in names and ov >= 4 and "移動" in txt,
	      f"{k}回目の可視化: 方眼紙 {[n for n in names if n.startswith('grid')]}・移動の印 {ov} 個・文字 {txt[:24]}")
	if k < 3:
		close(v)
# 開いたままもう1つ
v2, names2, ov2, txt2 = viz()
check("grid_minor" in names2 and ov2 >= 4, f"開いたままもう1つ開いても出る（印 {ov2} 個）")
close(v2); close(v)

# 4. もう一度押すと外れる（知らせとボタン）
app._ankle_selected_bone = fi; app._ankle_load_editor_from_bone(); app.on_ankle_bone_toggle_fixed(); pump(app, 0.2)
check(not app.ankle_bones[fi].get("fixed") and str(app._ankle_fix_btn.cget("text")) == "この骨を固定する"
      and "ありません" in str(app._ankle_grid_fix_lbl.cget("text")), "もう一度押すと固定が外れ、ボタンと⑤も戻る")
print(f"\n合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(1 if fails else 0)
