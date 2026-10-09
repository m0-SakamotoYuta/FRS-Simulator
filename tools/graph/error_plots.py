"""発表用の誤差グラフを作る（本体から独立したモジュール）。

【何を作るか】
  1. 集合棒グラフ  … 軸 × (RMSE / かたより / ばらつき) を 1 枚に。軸は 1 本だけ。
                     3枚に分けるとスケールの取り方で誤解が起きるので 1 枚にまとめる。
  2. 寄与率の帯    … かたより² と ばらつき² の内訳（100% 積み上げ）。
                     RMSE² = かたより² + ばらつき² なので、積み上げが厳密に成り立つ
                     唯一の表し方。棒の高さを足し算しても RMSE にはならない
                     （例: 1.63 + 0.88 = 2.51 に対し RMSE は 1.85 で 35% 過大）。
  3. 条件比較      … マーカーサイズ・撮影角度など、条件ごとに横並び。

【設計方針】
  ・本体 (FRS-SIMULATOR.py) からは「データと設定を渡して呼ぶ」だけ。
    本体が壊れても、このファイルが壊れても、互いに影響しない。
  ・タイトル / 縦軸タイトル / 凡例名 / 数値ラベル は cfg で全部差し替えられる。
  ・コマンドラインからも使える（Excel の「誤差まとめ」シートを読む）。

使い方（コマンドライン）:
    set PYTHONIOENCODING=utf-8
    python tools/graph/error_plots.py 出力先フォルダ まとめ1.xlsx [まとめ2.xlsx ...]
"""
from __future__ import annotations

import os
import sys

# --------------------------------------------------------------------------
# 見た目の既定値（発表スライド向け）
# --------------------------------------------------------------------------
COLORS = {"RMSE": "#3b6ea5", "bias": "#c1666b", "sd": "#7a9e7e"}
LABELS = {"RMSE": "RMSE（総合誤差）", "bias": "かたより（系統誤差）",
          "sd": "ばらつき（偶然誤差）"}
JP_FONT_CANDIDATES = ("Meiryo", "Yu Gothic", "MS Gothic", "Noto Sans CJK JP")
LIN_AXES = ("X", "Y", "Z")
ROT_AXES = ("U", "V", "W")

# 数値ラベルの出し方
LABEL_MODES = ("縦書き", "横書き", "なし")

# 作る図の一覧 (キー, 既定タイトル, 既定の縦軸タイトル)
# キーはそのままファイル名になり、GUI の設定画面の行にもなる。
FIGURES = (
	("直動_誤差3指標", "直動軸の計測誤差", "誤差 [mm]"),
	("直動_誤差の内訳", "直動軸の誤差の内訳", "誤差への寄与率 [%]"),
	("直動_条件比較_RMSE", "条件別の RMSE（直動）", "RMSE [mm]"),
	("直動_条件比較_ばらつき", "条件別のばらつき（直動）", "ばらつき [mm]"),
	("回転_誤差3指標", "回転軸の計測誤差", "誤差 [°]"),
	("回転_誤差の内訳", "回転軸の誤差の内訳", "誤差への寄与率 [%]"),
	("回転_条件比較_RMSE", "条件別の RMSE（回転）", "RMSE [°]"),
	("回転_条件比較_ばらつき", "条件別のばらつき（回転）", "ばらつき [°]"),
)


def default_cfg() -> dict:
	"""設定の既定値。GUI 側はこれを埋めて渡す。"""
	return {
		"titles": {k: t for k, t, _y in FIGURES},
		"ylabels": {k: y for k, _t, y in FIGURES},
		"ymax": {k: "" for k, _t, _y in FIGURES},   # 空文字 = 自動
		"legend": {},              # {条件名: 表示名}
		"label_mode": "縦書き",     # 数値ラベル: 縦書き / 横書き / なし
		"decimals": 3,
		"font_size": 12,
		"fig_w": 9.0,
		"fig_h": 5.0,
		"dpi": 200,
		"data": "生",              # 生 / 平滑後
	}


def _get(cfg, group, key, fallback=""):
	try:
		v = (cfg.get(group) or {}).get(key)
		return v if (v is not None and str(v).strip() != "") else fallback
	except Exception:
		return fallback


def _setup_matplotlib(base_size: int = 12):
	"""日本語が出るように設定した matplotlib.pyplot を返す（画面は使わない）。"""
	import matplotlib
	matplotlib.use("Agg")
	import matplotlib.pyplot as plt
	from matplotlib import font_manager
	for cand in JP_FONT_CANDIDATES:
		try:
			font_manager.findfont(cand, fallback_to_default=False)
			plt.rcParams["font.family"] = cand
			break
		except Exception:
			continue
	plt.rcParams["axes.unicode_minus"] = False        # 全角マイナスを防ぐ
	plt.rcParams["font.size"] = base_size
	plt.rcParams["axes.grid"] = True
	plt.rcParams["grid.alpha"] = 0.3
	plt.rcParams["grid.linewidth"] = 0.6
	return plt


# --------------------------------------------------------------------------
# データの取り出し
# --------------------------------------------------------------------------
def err_stats(ideal, meas) -> dict:
	"""理想と計測から RMSE / かたより / ばらつき を出す。

	本体の _av_err_stats と同じ定義（誤差 = 計測 − 理想、ばらつきは不偏標準偏差）。
	Excel の =SQRT(SUMSQ()/COUNT()) / =AVERAGE() / =STDEV.S() に一致する。
	"""
	import numpy as np
	a = np.asarray(ideal, dtype=float)
	b = np.asarray(meas, dtype=float)
	if a.shape != b.shape:
		return {}
	m = np.isfinite(a) & np.isfinite(b)
	if int(m.sum()) < 3:
		return {}
	a, b = a[m], b[m]
	e = b - a
	return {"n": int(m.sum()),
	        "RMSE": float(np.sqrt(np.mean(e ** 2))),
	        "bias": float(np.mean(e)),
	        "sd": float(np.std(e, ddof=1))}


def rows_from_av_result(res: dict, axis: str, method: str = "rgb") -> list:
	"""ArUco精度検証の解析結果 1 軸ぶんから、生/平滑後の行を作る。

	Excel の「誤差まとめ」シートと同じ計算（等速区間だけ、区間先頭を 0 に置き直す）。
	送り速度が未入力（feed<=0）の軸は理想値が作れないので空を返す。
	"""
	import numpy as np
	out = []
	try:
		kind = res.get("kind", "lin")
		feed = float(res.get("feed") or 0.0)
		if feed <= 0:
			return out
		ts = np.asarray(res["timestamps"], dtype=float)
		series = (res.get("series") or {}).get(method) or {}
		sm = series.get(kind) or {}
		al = sm.get("along")
		bm = sm.get("plot_mask")
		if al is None or bm is None:
			return out
		bm = np.asarray(bm, dtype=bool)
		if not np.any(bm):
			return out
		bi = np.where(bm)[0]
		al = np.asarray(al, dtype=float)
		fin = [int(j) for j in bi if np.isfinite(al[j])]
		if len(fin) < 2:
			return out
		a0 = float(al[fin[0]])
		sgn = 1.0 if float(al[fin[-1]]) >= a0 else -1.0
		ideal = feed * (ts[bi] - float(ts[bi[0]]))
		st = err_stats(ideal, sgn * (al[bi] - a0))
		if st:
			out.append(dict(st, axis=axis, kind=kind, data="生",
			                 unit=("mm" if kind == "lin" else "°"),
			                 commanded=float(res.get("commanded") or 0.0)))
	except Exception as e:
		print(f"[誤差グラフ] {axis}軸 の集計に失敗: {e}")
	return out


def rows_from_xlsx(path: str, method: str = "rgb") -> list:
	"""Excel の「誤差まとめ」シートから行を読む（コマンドライン用）。"""
	from openpyxl import load_workbook
	ws = load_workbook(path, data_only=True)["誤差まとめ"]
	rows, cur, head = [], None, None
	for r in ws.iter_rows(values_only=True):
		a = r[0]
		if a is None:
			continue
		s = str(a)
		if s.startswith("■"):
			cur = s[1:].split()[0]
			head = None
			continue
		if s == "軸":
			head = {str(v): i for i, v in enumerate(r) if v is not None}
			continue
		if s.startswith("【"):
			break
		if cur != method or head is None or s not in (LIN_AXES + ROT_AXES):
			continue

		def g(name):
			i = head.get(name)
			return r[i] if (i is not None and i < len(r)) else None

		try:
			rows.append({"axis": s, "kind": "lin" if g("種別") == "直動" else "rot",
			              "data": str(g("データ") or "生"), "unit": str(g("単位") or ""),
			              "commanded": float(g("指令値") or 0.0),
			              "n": int(g("n") or 0), "RMSE": float(g("RMSE")),
			              "bias": float(g("かたより")), "sd": float(g("ばらつき"))})
		except Exception:
			continue
	return rows


# --------------------------------------------------------------------------
# 描画の共通部品
# --------------------------------------------------------------------------
def _pick(rows, kind, data):
	order = LIN_AXES if kind == "lin" else ROT_AXES
	by = {r["axis"]: r for r in rows if r["kind"] == kind and r["data"] == data}
	return [by[a] for a in order if a in by]


def _annotate(ax, xs, vals, cfg, n_bars: int):
	"""棒の上に数値を書く。

	条件数が増えると棒が細くなり、横書きだと数字どうしが必ず重なる
	（実測: 7条件で 0.113 と 0.106 が完全に重なった）。
	既定を **縦書き** にして、重なりを構造的に無くす。
	それでも邪魔なら cfg["label_mode"] = "なし" で消せる。
	"""
	import numpy as np
	mode = str(cfg.get("label_mode") or "縦書き")
	if mode == "なし":
		return 1.0
	dec = int(cfg.get("decimals", 3) or 3)
	rot = 90 if mode == "縦書き" else 0
	# 棒が細いほど字を小さくする（横書きのときだけ効かせる）
	base = float(cfg.get("font_size", 12) or 12)
	fs = base * 0.72 if rot else max(6.0, min(base * 0.72, base * 2.6 / max(n_bars, 1)))
	for x, v in zip(xs, vals):
		if v is None or not np.isfinite(v):
			continue
		# 数値は **常に棒の上** に置く。負の棒で下に置くと、横軸の目盛ラベルや
		# 枠線と重なる（実測: かたより -0.003 の表示が「Y軸」に重なった）。
		# 負の棒は 0 の位置（＝棒の上端）に置く。符号は文字に出ているので
		# 向きは読み取れる。
		ax.annotate(f"{v:.{dec}f}", (x, max(float(v), 0.0)), ha="center",
		             va="bottom", rotation=rot, fontsize=fs,
		             xytext=(0, 3), textcoords="offset points")
	# 縦書きは字が上へ伸びるので、上端の余裕を多めに返す
	return 1.42 if rot else 1.20


def _make_fig(cfg, wmul=1.0, hmul=1.0, preview=False, fig=None):
	"""図を1枚用意する。

	preview=True のときは pyplot を通さず Figure を直接作る。
	pyplot で作ると図がグローバルな管理表に溜まり、GUI に貼ったまま
	close もできないので、プレビューを繰り返すと確実にメモリを食う。

	fig を渡すと **その図を中身だけ消して使い回す**。
	GUI のプレビューで毎回キャンバスを作り直すと、matplotlib の
	FigureCanvasTk が生成時に focus_set() するため入力欄からフォーカスが
	奪われ、日本語入力が途切れる（実測で確認）。使い回せばそれが起きない。
	"""
	_setup_matplotlib(int(cfg.get("font_size", 12) or 12))   # rcParams (日本語など)
	w = float(cfg.get("fig_w", 9.0)) * wmul
	h = float(cfg.get("fig_h", 5.0)) * hmul
	if fig is not None:
		fig.clear()
		try:
			fig.set_size_inches(w, h, forward=False)
		except Exception:
			pass
		return fig, fig.add_subplot(111)
	if preview:
		from matplotlib.figure import Figure
		fig = Figure(figsize=(w, h), dpi=100)
		return fig, fig.add_subplot(111)
	import matplotlib.pyplot as plt
	return plt.subplots(figsize=(w, h))


def _finish(fig, ax, name, cfg, out_path, unit="", legend_kw=None):
	"""タイトル・縦軸・凡例・保存をまとめて行う。

	out_path が None のときは保存せず Figure をそのまま返す（プレビュー用）。
	"""
	dflt_t = dict((k, t) for k, t, _y in FIGURES).get(name, name)
	dflt_y = dict((k, y) for k, _t, y in FIGURES).get(name, "")
	if unit:
		dflt_y = dflt_y.replace("[mm]", f"[{unit}]").replace("[°]", f"[{unit}]")
	ax.set_title(_get(cfg, "titles", name, dflt_t))
	ax.set_ylabel(_get(cfg, "ylabels", name, dflt_y))
	if legend_kw:
		ax.legend(**legend_kw)
	try:
		fig.tight_layout()
	except Exception:
		pass
	if out_path is None:
		return fig
	fig.savefig(out_path, dpi=int(cfg.get("dpi", 200) or 200),
	             facecolor="white", bbox_inches="tight")
	import matplotlib.pyplot as plt
	plt.close(fig)
	return out_path


def _ymax_of(cfg, name):
	v = (cfg.get("ymax") or {}).get(name)
	try:
		v = float(str(v).strip())
		return v if v > 0 else None
	except Exception:
		return None


# --------------------------------------------------------------------------
# 図
# --------------------------------------------------------------------------
def plot_grouped(rows, kind, out_path, cfg=None, name=None, abs_bias=False, fig=None):
	"""集合棒グラフ。軸 × (RMSE / かたより / ばらつき) を 1 枚・1 軸で描く。

	3 枚に分けると、パネルごとに縦軸の尺度が変わって誤解を招く。
	1 枚にすれば軸が 1 本しかないので、その問題が原理的に起きない。
	"""
	import numpy as np
	cfg = cfg or default_cfg()
	name = name or ("直動_誤差3指標" if kind == "lin" else "回転_誤差3指標")
	sub = _pick(rows, kind, cfg.get("data", "生"))
	if not sub:
		return None
	u = sub[0]["unit"]
	rmse = np.array([r["RMSE"] for r in sub])
	bias = np.array([abs(r["bias"]) if abs_bias else r["bias"] for r in sub])
	sd = np.array([r["sd"] for r in sub])
	x = np.arange(len(sub))
	w = 0.26
	fig, ax = _make_fig(cfg, preview=(out_path is None), fig=fig)
	ax.bar(x - w, rmse, w, label=LABELS["RMSE"], color=COLORS["RMSE"])
	ax.bar(x, bias, w,
	        label=(LABELS["bias"] + "（絶対値）" if abs_bias else LABELS["bias"]),
	        color=COLORS["bias"])
	ax.bar(x + w, sd, w, label=LABELS["sd"], color=COLORS["sd"])
	head = 1.0
	for dx, vals in ((-w, rmse), (0, bias), (w, sd)):
		head = max(head, _annotate(ax, x + dx, vals, cfg, 3))
	# 上端は「最大値 × 係数」ではなく「値の範囲 × 余裕率」で決める。
	# こうしないと、値が全部小さいときや全部負のときに余白が足りなくなる。
	# 数値ラベルは常に上に出すので、上に head 分、下はわずかで足りる。
	vmax = float(max(rmse.max(), bias.max(), sd.max(), 0.0))
	vmin = float(min(0.0, bias.min()))
	span = max(vmax - vmin, 1e-12)
	lo = vmin - span * 0.04
	hi = _ymax_of(cfg, name) or (vmax + span * (head - 1.0))
	ax.set_ylim(lo, hi)
	if lo < 0:
		ax.axhline(0, color="#444", lw=0.8)
	ax.set_xticks(x)
	ax.set_xticklabels([f"{r['axis']}軸" for r in sub])
	return _finish(fig, ax, name, cfg, out_path, unit=u,
	                legend_kw=dict(fontsize=9, ncol=3, loc="upper center",
	                                bbox_to_anchor=(0.5, -0.09), frameon=False))


def plot_contribution(rows, kind, out_path, cfg=None, name=None, fig=None):
	"""寄与率の帯グラフ。かたより² と ばらつき² の内訳を 100% 積み上げで描く。

	RMSE² = かたより² + ばらつき² なので、2 乗の世界でだけ積み上げが厳密に成り立つ。
	「誤差の何割が系統誤差か」を正しく示せる唯一の形。
	"""
	import numpy as np
	cfg = cfg or default_cfg()
	name = name or ("直動_誤差の内訳" if kind == "lin" else "回転_誤差の内訳")
	sub = _pick(rows, kind, cfg.get("data", "生"))
	if not sub:
		return None
	b2 = np.array([r["bias"] ** 2 for r in sub])
	s2 = np.array([r["sd"] ** 2 for r in sub])
	tot = np.where((b2 + s2) > 0, b2 + s2, 1.0)
	pb, ps = 100.0 * b2 / tot, 100.0 * s2 / tot
	x = np.arange(len(sub))
	fig, ax = _make_fig(cfg, wmul=0.78, hmul=0.92, preview=(out_path is None), fig=fig)
	ax.bar(x, pb, 0.55, label="かたより（系統誤差）", color=COLORS["bias"])
	ax.bar(x, ps, 0.55, bottom=pb, label="ばらつき（偶然誤差）", color=COLORS["sd"])
	if str(cfg.get("label_mode") or "縦書き") != "なし":
		for i in range(len(sub)):
			if pb[i] > 8:
				ax.text(i, pb[i] / 2, f"{pb[i]:.0f}%", ha="center", va="center",
				         color="white", fontsize=10, fontweight="bold")
			if ps[i] > 8:
				ax.text(i, pb[i] + ps[i] / 2, f"{ps[i]:.0f}%", ha="center",
				         va="center", color="white", fontsize=10, fontweight="bold")
	ax.set_xticks(x)
	ax.set_xticklabels([f"{r['axis']}軸" for r in sub])
	ax.set_ylim(0, 100)
	return _finish(fig, ax, name, cfg, out_path,
	                legend_kw=dict(fontsize=9, loc="lower right", framealpha=0.9))


def plot_compare(sets, kind, out_path, metric="RMSE", cfg=None, name=None, fig=None):
	"""条件比較。sets = [(条件名, rows), ...] を軸ごとに横並びで描く。

	凡例は cfg["legend"] で条件名 → 表示名に置き換えられる
	（既定はタブ名そのままだが、発表では長すぎるため）。
	"""
	import numpy as np
	cfg = cfg or default_cfg()
	tag = "直動" if kind == "lin" else "回転"
	name = name or f"{tag}_条件比較_{'RMSE' if metric == 'RMSE' else 'ばらつき'}"
	order = LIN_AXES if kind == "lin" else ROT_AXES
	data = cfg.get("data", "生")
	series = []
	for cond, rows in sets:
		by = {r["axis"]: r for r in rows if r["kind"] == kind and r["data"] == data}
		if not by:
			continue
		vals = [abs(by[a][metric]) if a in by else np.nan for a in order]
		series.append((_get(cfg, "legend", cond, cond), vals,
		                next(iter(by.values()))["unit"]))
	if not series:
		return None
	x = np.arange(len(order))
	n = len(series)
	w = min(0.82 / n, 0.28)
	# 条件が多いほど横に広げる (棒が細くなりすぎると数字が読めない)
	wmul = 1.0 if n <= 4 else min(1.5, 1.0 + 0.08 * (n - 4))
	fig, ax = _make_fig(cfg, wmul=wmul, preview=(out_path is None), fig=fig)
	import matplotlib.pyplot as _plt
	cmap = _plt.get_cmap("tab10")
	head = 1.0
	for i, (cond, vals, _u) in enumerate(series):
		off = (i - (n - 1) / 2.0) * w
		ax.bar(x + off, vals, w, label=cond, color=cmap(i % 10))
		head = max(head, _annotate(ax, x + off, vals, cfg, n))
	finite = [v for _c, vs, _u in series for v in vs if np.isfinite(v)]
	hi = _ymax_of(cfg, name) or (max(finite) * head if finite else 1.0)
	ax.set_ylim(0, hi)
	ax.set_xticks(x)
	ax.set_xticklabels([f"{a}軸" for a in order])
	return _finish(fig, ax, name, cfg, out_path, unit=series[0][2],
	                legend_kw=dict(fontsize=8, ncol=min(n, 3), loc="upper center",
	                                bbox_to_anchor=(0.5, -0.09), frameon=False))


# --------------------------------------------------------------------------
# まとめて作る
# --------------------------------------------------------------------------
def make_all(out_dir, rows=None, sets=None, cfg=None, prefix="", abs_bias=False):
	"""必要な図をまとめて作り、作ったファイルのパスを返す。

	rows … 1 条件ぶんの行（集合棒 + 寄与率 を作る）
	sets … [(条件名, rows), ...]（条件比較を作る。2 条件以上のときだけ）
	cfg  … default_cfg() を埋めたもの（タイトル・凡例・数値ラベルなど）
	"""
	cfg = cfg or default_cfg()
	os.makedirs(out_dir, exist_ok=True)
	made = []
	for kind, tag in (("lin", "直動"), ("rot", "回転")):
		# (図のキー, 描く関数) を並べる。キーはファイル名にも
		# cfg のタイトル/縦軸/上限のキーにもなる。
		jobs = []
		if rows:
			jobs.append((f"{tag}_誤差3指標",
			              lambda p, n, k=kind: plot_grouped(
				              rows, k, p, cfg=cfg, name=n, abs_bias=abs_bias)))
			jobs.append((f"{tag}_誤差の内訳",
			              lambda p, n, k=kind: plot_contribution(
				              rows, k, p, cfg=cfg, name=n)))
		if sets and len(sets) >= 2:
			for metric, mtag in (("RMSE", "RMSE"), ("sd", "ばらつき")):
				jobs.append((f"{tag}_条件比較_{mtag}",
				              lambda p, n, k=kind, m=metric: plot_compare(
					              sets, k, p, metric=m, cfg=cfg, name=n)))
		for fig_name, fn in jobs:
			path = os.path.join(out_dir, f"{prefix}{fig_name}.png")
			try:
				if fn(path, fig_name):
					made.append(path)
			except Exception as e:
				print(f"[誤差グラフ] {fig_name} の作成に失敗: {e}")
	return made


def _main(argv):
	if len(argv) < 3:
		print(__doc__)
		return 1
	out_dir, files = argv[1], argv[2:]
	sets = []
	for f in files:
		nm = os.path.splitext(os.path.basename(f))[0]
		try:
			sets.append((nm, rows_from_xlsx(f)))
		except Exception as e:
			print(f"[誤差グラフ] {f} を読めませんでした: {e}")
	if not sets:
		print("読み込めたファイルがありません")
		return 1
	made = make_all(out_dir, rows=sets[0][1], sets=sets)
	for p in made:
		print("  作成:", p)
	return 0


if __name__ == "__main__":
	sys.exit(_main(sys.argv))
