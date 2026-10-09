# -*- coding: utf-8 -*-
import importlib.util, os, sys, time
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
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); pump(app, 0.5)
names = [t["name"] for t in app._ankle_tabs if t.get("folder") == "261007_ ブタ　ACLcut前後の比較試験"]
A = "261007_FE_軸力100"
check(all("ankle_marker_size_true_mm" not in (t.get("snapshot") or {}) for t in app._ankle_tabs if t["name"] != app._ankle_tabs[app._ankle_active_tab]["name"]),
      "（準備）古い状態: 開いていないタブには実寸（ノギス）の項目が無い")
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == A)); pump(app, 0.5)
src = app._ankle_get_current_cache()["source"]
app._ankle_depth_size_store(src, {0: [14.76] * 9, 1: [14.76] * 9})
app._ankle_size_refresh_auto(); pump(app, 0.2)
# 「補正後に入れる」の共通の関数（②の「他のタブと同期…」が使う）で、同じフォルダの全タブに入れる
app._ankle_size_apply_to_tabs([t for t in app._ankle_tabs if t.get("folder") == "261007_ ブタ　ACLcut前後の比較試験"], 14.76)
pump(app, 0.3)
for n in names:
	sn = [t for t in app._ankle_tabs if t["name"] == n][0].get("snapshot") or {}
	if n == A: continue
	check(sn.get("ankle_marker_size_true_mm") == 15.0 and abs(sn.get("ankle_marker_size_mm") - 14.76) < 1e-9 and sn.get("ankle_marker_size_auto") is False,
	      f"{n}: 実寸(ノギス) {sn.get('ankle_marker_size_true_mm')} / 補正後 {sn.get('ankle_marker_size_mm')} / {'自動' if sn.get('ankle_marker_size_auto') else '手動'}")
check(app.ankle_marker_size_true_mm.get() == 15.0 and abs(app.ankle_marker_size_mm.get() - 14.76) < 1e-9, "開いているタブも 実寸15 / 補正後14.76")
B = "261007_FE_軸力100_ACL切"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == B)); pump(app, 0.6)
check(app.ankle_marker_size_true_mm.get() == 15.0 and abs(app.ankle_marker_size_mm.get() - 14.76) < 1e-9 and not app.ankle_marker_size_auto.get(),
      f"別のタブを開くと 画面も 実寸(ノギス) {app.ankle_marker_size_true_mm.get()} / 補正後 {app.ankle_marker_size_mm.get()} / 手動")
# まとめて解析の実寸の欄は実寸（ノギス）
app._ankle_open_batch_dialog(); pump(app, 0.8)
bw = [x for x in app.winfo_children() if isinstance(x, frs.tk.Toplevel) and "まとめて" in x.title()]
check(len(bw) == 1, "まとめて解析の窓")
ents = []
def walk(wd):
	for c in wd.winfo_children():
		if c.winfo_class() == "TEntry": ents.append(c)
		walk(c)
walk(bw[0])
vals = [e.get() for e in ents]
check("14.76" not in vals and vals.count("15") + vals.count("15.0") >= 8, f"実寸の欄はノギスの値（14.76 が出ていない）: 15 が {vals.count('15') + vals.count('15.0')} 個")
from capture_util import capture
capture(bw[0], os.path.join(HERE, "batch.png"))
print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
