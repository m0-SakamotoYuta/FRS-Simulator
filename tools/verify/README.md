# 検証用スクリプト（2026-09-04〜07 の調査で使ったもの）

いずれも `.venv/Scripts/python.exe tools/verify/<名前>.py` で単体実行できる。
GUI は起動しないので状態ファイルには触らない（`hw_final.py` を除き D405 も不要）。

文字化けする場合は `PYTHONIOENCODING=utf-8` を付ける。

| ファイル | 何を測るか | D405 |
|---|---|---|
| `gapscan.py` | 録画済み `.db3` を全部走査してコマ落ちの位置・大きさ・color/depth の同時性を出す | 不要 |
| `noise.py` | 保存済み解析結果から、軸ごとの残差RMS・白色/低周波の内訳・ローパスの効き方を出す | 不要 |
| `t_synth.py` | 合成データ（真値既知）で等速区間の切り出し精度を検証。従来方式との比較つき | 不要 |
| `diskbench2.py` | 録画と同じ粒度・レートでディスクに書き、詰まりの位置と長さを測る | 不要 |
| `hw_queue.py` | フレーム取得方式（wait_for_frames / frame_queue / callback）の取りこぼしを比較 | **必要** |
| `hw_final.py` | 現行方式とRAM録画を同条件で比較（`... hw_final.py <秒数> [A\|B\|both]`） | **必要** |
| `t_ring.py` | RAM録画のリングを、ディスクに妨害を掛けながら長時間検証 | 不要 |
| `quietzone.py` | マーカー周囲の白余白の幅を振り、検出率・角誤差・距離の偏りを合成で測る（`... quietzone.py <距離mm> grey/dark/clutter <最大傾き°> <試行数>`） | 不要 |
| `patch_lib.py` | `FRS-SIMULATOR.py` を安全に編集するヘルパー（CRLF維持・一意アンカー必須） | — |

## patch_lib の使い方

本体は約30,000行あるので、直接編集せずパッチスクリプトを書く。
アンカーの出現回数が 1 でなければ失敗するので、取り違えが起きない。

```python
import patch_lib as P
s = P.read()
CR = "\r\n"
old = "\t\t古い行" + CR
new = "\t\t新しい行" + CR
s = P.replace_once(s, old, new, "何の変更か")
P.write(s)
```

※ パッチスクリプト自体は **Write ツールで書くこと**。
Bash のヒアドキュメント経由だと日本語が化ける。
