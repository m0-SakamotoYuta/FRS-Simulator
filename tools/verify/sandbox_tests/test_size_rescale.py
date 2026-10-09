# -*- coding: utf-8 -*-
import importlib.util, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
import numpy as np
from scipy.spatial.transform import Rotation as Rot
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c: fails.append(l)
app = frs.MainMenuGUI(); app.update()
ti = next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == "261007_FE_軸力100")
app.on_ankle_tab_select(ti); app.update()
raw = app._ankle_get_current_cache()
check(raw is not None and abs(float(raw["marker_size_mm"]) - 15.0) < 1e-9, "④の結果は 15mm で計算されている")
# --- ②が同じなら、そのまま返す ---
app.ankle_marker_size_mm.set(15.0)
check(app._ankle_cache_for_current_size(raw) is raw, "②が同じなら写しを作らない（従来と同じ）")
# --- ②を 14.8 に ---
app.ankle_marker_size_mm.set(14.8); app.update(); time.sleep(0.4); app.update()
sc = app._ankle_cache_for_current_size(raw)
P0 = np.asarray(raw["bones"][1]["poses"]); P1 = np.asarray(sc["bones"][1]["poses"])
ok = np.all(np.isfinite(P0.reshape(len(P0), -1)), axis=1)
check(np.allclose(P1[ok][:, :3, :3], P0[ok][:, :3, :3]), "向きは変わらない")
check(np.allclose(P1[ok][:, :3, 3], P0[ok][:, :3, 3] * 14.8 / 15.0), "位置は 14.8/15 倍")
check(np.asarray(app._ankle_get_current_cache()["bones"][1]["poses"])[ok][0, 0, 3] == P0[ok][0, 0, 3], "保存してある④の結果は変わらない")
st = app.ankle_detection_status.get()
check("②の 14.8 mm に合わせます" in st, "④の欄に表示: " + st.splitlines()[1] if len(st.splitlines()) > 1 else st)
# --- 可視化の経路（骨の位置の計算）で反映されるか ---
anim15, N15, _ = app._ankle_build_bone_transforms(raw)
anim148, N148, _ = app._ankle_build_bone_transforms(sc)
print("   build の戻り値の例:", type(anim148), len(anim148), flush=True)
# --- ④を 14.8 でやり直したものと比べる（録画の一部） ---
if len(sys.argv) > 1 and sys.argv[1] == "redetect":
	bag = raw["source"]
	stride = 200
	res = {}
	for size in (15.0, 14.8):
		t0 = time.time()
		c = app._ankle_detect_from_bag(bag, "DICT_4X4_100", size, {0, 1, 2}, stride, lambda *a, **k: None, lambda: False)
		res[size] = c; print(f"   ④ 実寸 {size} mm（{stride}枚おき）: {time.time()-t0:.0f} 秒", flush=True)
	A = res[15.0]["bones"]; Bc = res[14.8]["bones"]
	for aid in (0, 1, 2):
		pa = np.asarray(A[aid]["poses"]); pb = np.asarray(Bc[aid]["poses"])
		m = np.all(np.isfinite(pa.reshape(len(pa), -1)), axis=1) & np.all(np.isfinite(pb.reshape(len(pb), -1)), axis=1)
		dR = max(np.degrees(np.linalg.norm(Rot.from_matrix(pb[i][:3, :3] @ pa[i][:3, :3].T).as_rotvec())) for i in np.where(m)[0])
		dt = np.abs(pb[m][:, :3, 3] - pa[m][:, :3, 3] * 14.8 / 15.0).max()
		check(dR < 1e-6 and dt < 1e-6, f"ID={aid}: ④を14.8でやり直した結果 = 15の結果×14.8/15（向きの差 最大 {dR:.2e}°、位置の差 最大 {dt:.2e} mm、{m.sum()}枚）")
print(f"\n合計: NG {len(fails)} 件")
os._exit(1 if fails else 0)
