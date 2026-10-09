# -*- coding: utf-8 -*-
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
app = frs.MainMenuGUI(); app.update()
ti = next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_FE_軸力100")
app.on_ankle_tab_select(ti); app.update()
cache = app._ankle_get_current_cache(); src = cache["source"]
check(app._ankle_depth_size_get(src) is None, "最初は推定が無い")
t0 = time.time()
r = app._ankle_depth_size_measure(src, stride=15)
print(f"   測定 {time.time()-t0:.0f} 秒", flush=True)
check(r is not None and r.get("all"), "測れた")
if r:
	for k, v in sorted(r["ids"].items()):
		print(f"   ID{k}: {v['median']:.3f} mm (25-75% {v['p25']:.3f}-{v['p75']:.3f}, n={v['n']})", flush=True)
	a = r["all"]; print(f"   全体: {a['median']:.3f} mm (25-75% {a['p25']:.3f}-{a['p75']:.3f}, n={a['n']})", flush=True)
	check(14.3 < a["median"] < 15.2, "全体の中央値が妥当な範囲")
check(os.path.exists(os.path.join(HERE, "cache", "ankle_depth_size.json")), "サンドボックスの cache/ankle_depth_size.json に保存")
check(app._ankle_get_current_cache() is cache, "④の結果は変わらない")
app._ankle_update_detection_status()
st = app.ankle_detection_status.get()
check("深度による実寸の推定" in st, "④の欄に表示: " + [l for l in st.splitlines() if "深度" in l][0] if "深度" in st else st)
# 接触で確かめる（FE が動いている 45 秒以降）
B = app.ankle_bones
t0 = time.time()
rc = app._ankle_contact_check(cache, B[0], B[1], 45.0, None, sizes=[15.0, round(r["all"]["median"], 2) if r else 14.8])
print(f"   接触の確認 {time.time()-t0:.0f} 秒: {rc['frames']}コマ {rc['t0']:.1f}〜{rc['t1']:.1f}s", flush=True)
for sz, g in rc["by_size"].items():
	print(f"     実寸 {sz}: 中央値 {g['median']:+.2f} (5-95% {g['p5']:+.2f}〜{g['p95']:+.2f})", flush=True)
print(f"     0 になる実寸: {rc.get('zero_size')}", flush=True)
check(abs(rc["by_size"][15.0]["median"] - 0.93) < 0.25, "15mm のすき間は先の解析（約 +0.9mm）と合う")
check(rc.get("zero_size") and 14.6 < rc["zero_size"] < 14.95, "0 になる実寸は約 14.8")
# 窓を開いて撮る
app._ankle_open_size_estimate_dialog(); app.update(); time.sleep(0.5); app.update()
w = [x for x in app.winfo_children() if isinstance(x, frs.tk.Toplevel) and "推定" in x.title()]
check(len(w) == 1, "窓が開く")
try:
	sys.path.insert(0, HERE); from capture_util import capture
	capture(w[0], os.path.join(HERE, "size_dialog.png"))
except Exception as e:
	print("   撮影できず:", e)
print(f"\n合計: NG {len(fails)} 件", flush=True)
os._exit(1 if fails else 0)
