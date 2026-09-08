"""録画済み .db3 の欠落を、時刻・書き込み量・ストリーム別に洗い出す。

知りたいこと:
  1) 欠落は先頭だけか、中盤にもあるか
  2) 周期性があるか（あればフラッシュ/コミットのような定期処理が犯人）
  3) color と depth が同時に落ちているか（同時＝書き込み側 / 片方＝取得側）
  4) 何 GB 書いた所で起きているか（SSD のキャッシュ枯渇なら書き込み量に相関する）
"""
import sqlite3
import sys
from pathlib import Path

import numpy as np

ROOT = Path(r"C:\Users\yutas\Desktop\ankle simulator用ファイル\260903_テスト_ArUco精度検証")


def topics(con):
	cur = con.cursor()
	return cur.execute("SELECT id, name FROM topics").fetchall()


def stamps(con, topic_id):
	cur = con.cursor()
	return np.asarray([r[0] for r in cur.execute(
		"SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
		(topic_id,))], dtype=float) / 1e9


def analyse(p: Path):
	size = p.stat().st_size
	con = sqlite3.connect(str(p))
	try:
		tp = topics(con)
		img = [(i, n) for i, n in tp if n.endswith("image/data")]
		if not img:
			print(f"  (画像トピックなし) {p.name}")
			return
		series = {}
		for tid, name in img:
			key = "color" if "Color" in name else ("depth" if "Depth" in name else name)
			series[key] = stamps(con, tid)
	finally:
		con.close()
	c = series.get("color")
	if c is None or len(c) < 10:
		print(f"  (色フレームが少なすぎる) {p.name}")
		return
	t0 = c[0]
	dur = c[-1] - c[0]
	nom = float(np.median(np.diff(c)))
	print(f"\n=== {p.parent.name}/{p.name}")
	print(f"    {size / 1e9:.2f} GB / {dur:.1f} s = {size / 1e6 / max(dur, 1e-9):.0f} MB/s "
	      f"(color {len(c)} 枚 = {len(c) / max(dur, 1e-9):.2f} fps, 公称 {1 / nom:.1f} fps)")
	for key, s in series.items():
		if s is None or len(s) < 10:
			continue
		d = np.diff(s)
		nm = float(np.median(d))
		miss = np.round(d / nm) - 1
		idx = np.where(miss > 0)[0]
		tot = int(miss[idx].sum()) if len(idx) else 0
		print(f"    [{key:5s}] 欠落 {tot:4d} 枚 / {len(idx):3d} 箇所")
		if not len(idx):
			continue
		# 時間帯ごとの分布
		rel = s[idx] - t0
		bins = [(0, 1), (1, 3), (3, 5), (5, 10), (10, 20), (20, 999)]
		dist = []
		for lo, hi in bins:
			m = (rel >= lo) & (rel < hi)
			if m.any():
				dist.append(f"{lo}-{hi if hi < 999 else '末'}s:{int(miss[idx][m].sum())}枚")
		print(f"            時間分布 " + "  ".join(dist))
		order = idx[np.argsort(-miss[idx])][:6]
		for j in order:
			gb = size / 1e9 * (s[j] - t0) / max(dur, 1e-9)
			print(f"            t={s[j] - t0:6.2f}s ({gb:4.2f} GB書込時点) "
			      f"{int(miss[j]):3d}枚 {d[j] * 1000:6.0f} ms")
		# 周期性: 欠落イベントの間隔
		if len(idx) >= 3:
			iv = np.diff(np.sort(rel))
			print(f"            欠落イベントの間隔: 中央 {np.median(iv):.2f}s "
			      f"(最小 {iv.min():.2f} / 最大 {iv.max():.2f})")
	# color と depth が同時か
	if "depth" in series and series["depth"] is not None and len(series["depth"]) > 10:
		dd = series["depth"]
		nc = float(np.median(np.diff(c)))
		nd = float(np.median(np.diff(dd)))
		gc_ = c[np.where(np.round(np.diff(c) / nc) - 1 > 0)[0]] - t0
		gd_ = dd[np.where(np.round(np.diff(dd) / nd) - 1 > 0)[0]] - dd[0]
		if len(gc_) and len(gd_):
			both = sum(1 for x in gc_ if np.min(np.abs(gd_ - x)) < 0.3)
			print(f"    color の欠落 {len(gc_)} 箇所のうち {both} 箇所は depth も同時 "
			      f"(depth 側は {len(gd_)} 箇所)")


files = sorted(ROOT.rglob("*.db3"), key=lambda q: -q.stat().st_size)
for p in files:
	if p.stat().st_size < 1e6:
		continue
	try:
		analyse(p)
	except Exception as e:
		print(f"  !! {p.name}: {e}")
