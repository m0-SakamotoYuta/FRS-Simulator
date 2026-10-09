# -*- coding: utf-8 -*-
"""マーカー-骨キャリブに使った実寸の記録と、②との食い違いの知らせ。"""
import importlib.util, os, json, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
fails = []
def check(c, l):
	print(("  OK  " if c else "  NG  ") + l, flush=True)
	if not c:
		fails.append(l)
def pump(app, n=3):
	for _ in range(n):
		app.update_idletasks(); app.update()
dialogs = []
for kind in ("showwarning", "showerror", "showinfo"):
	setattr(frs.messagebox, kind, lambda *a, _k=kind, **k: dialogs.append((_k, a[:2])))
ask = [False]
frs.messagebox.askyesno = lambda *a, **k: (dialogs.append(("askyesno", a[:2])), ask[0])[1]

app = frs.MainMenuGUI(); pump(app, 5)
app._ankle_folder_filter = app._ANKLE_FOLDER_ALL; app._ankle_rebuild_tabbar()
MODEL = r"C:/Users/yutas/Desktop/ankle simulator用ファイル/260930_使える3D計測モデル/2_hikotsu/hikotsu.obj"
# 新しいタブに骨を2本作る
app.on_ankle_tab_add(); pump(app)
app.ankle_bones = [app._ankle_default_bone(0), app._ankle_default_bone(1)]
for b in app.ankle_bones:
	b["model_path"] = MODEL
app._ankle_selected_bone = 0; app._ankle_refresh_bone_listbox()
app.ankle_marker_size_mm.set(15.0); time.sleep(0.4); pump(app)
lb = app._ankle_bones_listbox
status = app._ankle_bone_editor_widgets["status_lbl"]

# --- 手動4点キャリブ（ガイドとピックは差し替え）---
app._ankle_calib_guide_dialog = lambda *a, **k: True
app._ankle_visualize_calibration = lambda *a, **k: None
def fake_pick(mesh, bone, size, title, texture=None):
	return np.asarray(app._ankle_marker_obj_points(15.0), float) + np.array([10.0, 20.0, 30.0])
app._ankle_pick_marker_corners = fake_pick
dialogs.clear()
app.on_ankle_calibrate_marker_to_bone(); pump(app)
b0 = app.ankle_bones[0]
check(b0.get("marker_to_bone_T") is not None, "手動キャリブの結果が入る")
check(b0.get("marker_to_bone_size_mm") == 15.0 and b0.get("marker_to_bone_method") == "manual",
      f"手動キャリブに使った実寸と方式を記録 ({b0.get('marker_to_bone_size_mm')}, {b0.get('marker_to_bone_method')})")
check("実寸 15 mm で実施" in str(status.cget("text")), "③に「実寸 15 mm で実施」: " + str(status.cget("text")))

# --- 自動キャリブ（中身は差し替え・旧方式）---
app.ankle_calib_v2.set(False)
app._ankle_auto_calibrate_from_mesh_impl = lambda b, progress=None: (np.eye(4).tolist(), 0.3, 1000.0, 12, int(b.get("aruco_id", 1)))
app._ankle_selected_bone = 1; app._ankle_refresh_bone_listbox()
app.on_ankle_auto_calibrate_from_mesh(); pump(app)
b1 = app.ankle_bones[1]
check(b1.get("marker_to_bone_size_mm") == 15.0 and b1.get("marker_to_bone_method") == "auto",
      f"自動キャリブに使った実寸と方式を記録 ({b1.get('marker_to_bone_size_mm')}, {b1.get('marker_to_bone_method')})")

# --- ②の実寸を 20 に変える → ③の表示が変わる ---
app.ankle_marker_size_mm.set(20.0); time.sleep(0.5); pump(app, 5)
line0, line1 = lb.get(0), lb.get(1)
print("   骨リスト:", [line0, line1])
check("⚠" not in line0, "手動キャリブの骨は、骨リストで警告しない（結果が実寸に左右されないため）")
check("⚠実寸15≠20" in line1 and str(lb.itemcget(1, "fg")) == "#c62828", "自動キャリブの骨は、骨リストで赤字の ⚠実寸15≠20")
app._ankle_selected_bone = 1; app._ankle_refresh_bone_listbox(); pump(app)
check("⚠" in str(status.cget("text")) and str(status.cget("foreground")) == "#c62828",
      "自動の骨を選ぶと赤字の警告: " + str(status.cget("text")).replace("\n", " "))
app._ankle_selected_bone = 0; app._ankle_refresh_bone_listbox(); pump(app)
check("そのまま使えます" in str(status.cget("text")) and "⚠" not in str(status.cget("text")),
      "手動の骨は「そのまま使えます」のお知らせ: " + str(status.cget("text")))

# --- 可視化の前の確認（新プランのときだけ）---
app.ankle_workflow_mode.set("self_pose")
dialogs.clear(); ask[0] = False
r = app._ankle_confirm_calib_sizes("シミュレーション")
check(r is False and [d[0] for d in dialogs] == ["askyesno"], "新プラン: 確認が出て「いいえ」なら表示しない")
check("骨2" in dialogs[0][1][1] and "骨1" not in dialogs[0][1][1], "確認に出るのは自動キャリブの骨だけ")
ask[0] = True; dialogs.clear()
check(app._ankle_confirm_calib_sizes("シミュレーション") is True, "「はい」なら表示する")
app.ankle_workflow_mode.set("original"); dialogs.clear()
check(app._ankle_confirm_calib_sizes("シミュレーション") is True and not dialogs, "原プラン（キャリブ結果を使わない）では確認しない")
app.ankle_workflow_mode.set("self_pose")
# on_ankle_animate から呼ばれていること
app._ankle_get_current_cache = lambda: {"frame_count": 1}
ask[0] = False; dialogs.clear()
called = []
app._ankle_build_bone_transforms = lambda cache: (called.append(1), ([], 0, []))[1]
app.on_ankle_animate(); pump(app)
check(dialogs and dialogs[0][0] == "askyesno" and not called, "「シミュレーション実行」で確認が出て、いいえ なら先へ進まない")

# --- ②を 15 に戻すと警告が消える ---
app.ankle_marker_size_mm.set(15.0); time.sleep(0.5); pump(app, 5)
check("⚠" not in lb.get(1) and str(lb.itemcget(1, "fg")) == "black", "②を 15 に戻すと警告が消える")

# --- 保存と復元 ---
app._save_ankle_state(save_pose_caches=False)
d = json.load(open("frs2015_gui_state_ankle_sim.json", encoding="utf-8"))
sb = d["tabs"][app._ankle_active_tab]["snapshot"]["_bones"]
check(sb[0].get("marker_to_bone_size_mm") == 15.0 and sb[1].get("marker_to_bone_method") == "auto", "状態ファイルに記録が残る")

# --- キャリブをクリア → 記録も消える ---
app._ankle_selected_bone = 1; app._ankle_refresh_bone_listbox()
app._ankle_apply_editor_field("marker_to_bone_T", None); pump(app)
check("marker_to_bone_size_mm" not in app.ankle_bones[1] and "marker_to_bone_method" not in app.ankle_bones[1], "キャリブをクリアすると記録も消える")

# --- 記録を始める前のキャリブ ---
app.ankle_bones[1]["marker_to_bone_T"] = np.eye(4).tolist()
app._ankle_refresh_bone_listbox(); pump(app)
check("記録なし" in str(status.cget("text")) and "⚠" not in lb.get(1), "記録の無い古いキャリブは「記録なし」と出し、警告はしない")
app.destroy()
print("RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL: {fails}")
