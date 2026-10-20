# -*- coding: utf-8 -*-
"""靭帯の束（段階2）（2026-10-20）: バックアップの書き出し・読み込み（zip に束の定義・計算の結果・目安の点のファイル・説明。
読み込みは置き換えで、前の束を自動で書き出す。結果は同じタブなら戻し、骨の動きが違えば外す）。接触の計算はダミー。2〜3分。"""
import importlib.util, os, sys, time, json, glob, shutil, gc
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
frs.messagebox.showwarning = lambda *a, **k: print("   [warning]", str(a[:2])[:300])
frs.messagebox.showinfo = lambda *a, **k: None
app = frs.MainMenuGUI(); app.withdraw(); app.update()
app._ankle_skip_debug_images = True
class _W:
	def winfo_exists(self): return False
	def destroy(self): pass
app._show_precompute_dialog = lambda N, has_cartilage=False: (_W(), (lambda *a, **k: True), tk.BooleanVar(app, False), {}, tk.BooleanVar(app, True), tk.BooleanVar(app, False))
app._sim_view_precompute_multi_heatmap = lambda bd, update_progress=None, cancel_var=None, quiet=False: [
	{idx: np.ones(len(np.asarray(m.points)), np.float32) for (idx, n, m, T) in bd} for t in range(len(bd[0][3]))]
def walk(wd):
	for c in wd.winfo_children():
		yield c
		yield from walk(c)

TAB = "261007_FE_軸力100"
app.on_ankle_tab_select(next(i for i, t in enumerate(app._ankle_tabs) if t["name"] == TAB)); pump(app, 0.5)
shutil.rmtree(os.path.join(HERE, "cache", "ankle_lig_bundles"), ignore_errors=True)
app.ankle_lig_mode.set("stage2"); app.ankle_start_at_load.set(True); app.ankle_alt_model.set(False)
app.on_ankle_animate(); pump(app, 1.0)
v = app._sim_viewers_alive()[-1]; p = v["plotter"]
ctx = [o for o in gc.get_objects() if type(o).__name__ == "Stage2" and o.plotter is p][-1]
ctx.open_window(); pump(app, 0.4)
# 束を1つ（いまの ACL の点のまわりに付着部の面）
G = [x for x in app.ankle_ligaments if x["name"] == "ACL"][0]
def ring(si, center, ra, rb_, n=7):
	bones = ctx._ensure_bones(); idx = ctx.idx_of[si]
	cpp, ns, _ = bones.closest(idx, np.asarray(center, float)[None]); nrm = ns[0]
	t1 = np.cross(nrm, [1, 0, 0]); t1 /= np.linalg.norm(t1); t2 = np.cross(nrm, t1)
	th = np.linspace(0, 2 * np.pi, n, endpoint=False) + 0.3
	return bones.closest(idx, cpp[0] + np.outer(np.cos(th) * ra, t1) + np.outer(np.sin(th) * rb_, t2) + 2 * nrm)[0]
ctx._add_bundle(); pump(app, 0.2)
b = ctx.data["bundles"][-1]; b["name"] = "ACL(表示)"; b["n_fibers"] = 6
si_f, si_t = ctx.names.index("大腿骨"), ctx.names.index("脛骨")
for end, si, key in (("a", si_f, "a"), ("b", si_t, "b")):
	ctx._ol_start(b, end)
	for q in ring(si, G[key]["point"], 3.0, 1.5):
		ctx.add_point(si, q)
	ctx._ol_commit(); pump(app, 0.2)
check(ctx._ready(b), "付着部の面が両方ある")
t0 = time.time(); ok = ctx.compute(parent=ctx.win); pump(app, 0.3)
check(ok and ctx.results.get(b["id"]) is not None, f"計算した（{time.time() - t0:.0f} 秒）")
import zipfile, tkinter.filedialog as _fd
OUT = os.path.join(HERE, "_bk_out"); shutil.rmtree(OUT, ignore_errors=True); os.makedirs(OUT)
topf = [w for w in walk(ctx.win) if w.winfo_class() == "TLabelframe" and "段階1の付着部の点" in str(w.cget("text"))][0]
tb = {str(w.cget("text")): w for w in walk(topf) if w.winfo_class() == "TButton"}
check("💾 バックアップを書き出す…" in tb and "バックアップから読み込む…" in tb, "書き出す・読み込むのボタン（上の欄）")
# 目安の点のファイル（段階1）を入れておく
gsrc = os.path.join(OUT, "目安の点.json"); open(gsrc, "w", encoding="utf-8").write(json.dumps({"ligaments": []}))
ctx.data["guide_file"] = gsrc; ctx._save()
# 1) 書き出す（ボタンから。保存のダイアログは差し替え）
fp1 = os.path.join(OUT, "bk1.zip")
_fd.asksaveasfilename = lambda **k: fp1
tb["💾 バックアップを書き出す…"].invoke(); pump(app, 0.3)
z = zipfile.ZipFile(fp1); nm = set(z.namelist())
info = json.loads(z.read("info.json").decode("utf-8"))
check({"info.json", "bundles.json", "results.npz", "guide/目安の点.json"} <= nm, f"zip の中身（{sorted(nm)}）")
check(info["has_results"] and info["n_ready"] == 1 and info["bundles"][0]["name"] == "ACL(表示)" and info["bundles"][0]["faces"] == 2,
      "説明: 束の名前・面 2/2・計算の結果あり")
bj = json.loads(z.read("bundles.json").decode("utf-8")); z.close()
check(bj["bundles"][0]["b"]["ellipse"]["center"] == b["b"]["ellipse"]["center"] and len(bj["bundles"][0]["a"]["outline"]) == 7,
      "束の定義（楕円・外周の点）が入っている")
check(ctx.data["params"].get("backup_dir") == OUT and "書き出しました" in ctx.w["status"].cget("text"), "書き出した場所を覚える・知らせる")
# 2) 変える: 楕円を動かす・束を足す・目安の点のファイルを消す
c0 = list(b["b"]["ellipse"]["center"]); bid = b["id"]
ctx.move_ellipse(b, "b", 1, 0, 2.0); ctx._add_bundle(); pump(app, 0.2)
os.remove(gsrc)
check(ctx.results.get(bid) is None and len(ctx.data["bundles"]) == 2, "（変えた: 楕円を動かして結果が外れた・束が 2 個）")
# 3) 読み込む（ボタンから）
_fd.askopenfilename = lambda **k: fp1
asked = []
frs.messagebox.askyesno = lambda *a, **k: (asked.append(a), True)[1]
t0 = time.time()
tb["バックアップから読み込む…"].invoke(); pump(app, 0.5)
dt = time.time() - t0
check(asked and "置き換えます" in asked[-1][1] and "ACL(表示)" in asked[-1][1], "読み込む前に、置き換える束を見せて確かめる")
b2 = ctx.data["bundles"]
check(len(b2) == 1 and b2[0]["id"] == bid and b2[0]["b"]["ellipse"]["center"] == c0, "束が書き出したときに戻る（1 個・楕円の位置）")
check(ctx.results.get(bid) is not None and dt < 20, f"計算の結果も戻る（計算し直さない・{dt:.1f} 秒）")
check(ctx.actors.get(bid) is not None and ctx.actors[bid].get("fib_act") is not None, "線維が描かれる")
gf = ctx.data.get("guide_file", "")
check(os.path.isfile(gf) and os.path.basename(os.path.dirname(gf)) == "guides", f"消えた目安の点のファイルは zip の中のものを使う（{gf}）")
autos = glob.glob(os.path.join(os.path.dirname(ctx.store), "backup", "自動_読み込む前_*.zip"))
check(len(autos) == 1, "読み込む前の束を自動で書き出す")
za = json.loads(zipfile.ZipFile(autos[0]).read("bundles.json").decode("utf-8"))
check(len(za["bundles"]) == 2 and za["bundles"][0]["b"]["ellipse"]["center"] != c0, "自動の書き出しは、読み込む前（動かした・2 個）の束")
check(ctx.win is not None and ctx.win.winfo_exists() and "読み込みました" in ctx.w["status"].cget("text")
      and "結果も戻しました" in ctx.w["status"].cget("text"), "画面を開き直して知らせる")
saved = json.load(open(ctx.store, encoding="utf-8"))
check(len(saved["bundles"]) == 1 and saved["bundles"][0]["b"]["ellipse"]["center"] == c0, "読み込んだ束を保存")
# 4) 骨の動きが違うタブの結果（鍵が違う）なら結果は使わない
fp2 = os.path.join(OUT, "bk2.zip")
with zipfile.ZipFile(fp1) as zi, zipfile.ZipFile(fp2, "w") as zo:
	for n_ in zi.namelist():
		by = zi.read(n_)
		if n_ == "info.json":
			d_ = json.loads(by.decode("utf-8")); d_["result_key"] = "ほかのタブ"; by = json.dumps(d_, ensure_ascii=False).encode("utf-8")
		zo.writestr(n_, by)
shutil.rmtree(ctx.res_dir, ignore_errors=True)
info2, got2, auto2, miss2 = ctx.import_backup(fp2); pump(app, 0.3)
check(not got2 and ctx.results.get(bid) is None and len(ctx.data["bundles"]) == 1, "結果が合わなければ外す（束は読み込む）")
# 5) バックアップでないファイルは読み込まない（いまの束はそのまま）
fp3 = os.path.join(OUT, "bad.zip")
with zipfile.ZipFile(fp3, "w") as zo:
	zo.writestr("x.txt", "x")
try:
	ctx.import_backup(fp3); ok3 = False
except ValueError:
	ok3 = True
check(ok3 and len(ctx.data["bundles"]) == 1, "バックアップでない zip は断る（いまの束はそのまま）")
# 6) 結果のキャッシュが無くても（別の PC・消した）、zip の中の結果で戻る
shutil.rmtree(ctx.res_dir, ignore_errors=True)
info6, got6, auto6, miss6 = ctx.import_backup(fp1); pump(app, 0.3)
check(got6 and ctx.results.get(bid) is not None and len(glob.glob(os.path.join(ctx.res_dir, "lb_*.npz"))) == 1,
      "キャッシュが無くても zip の中の結果で戻る")
w_ = v["ctrl"]["window"]; w_.tk.call(w_.protocol("WM_DELETE_WINDOW")); pump(app, 0.4)
app.ankle_lig_mode.set("stage1")
shutil.rmtree(OUT, ignore_errors=True)
print(f"合計: NG {len(fails)} 件", flush=True)
sys.stdout.flush(); os._exit(0)
