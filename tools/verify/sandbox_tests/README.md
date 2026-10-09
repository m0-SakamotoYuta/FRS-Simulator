# 画面まわりの自動テスト（サンドボックスで実行する）

ankle simulator の「まとめて解析・フォルダ・右端へ・骨リストの共有・パスの付け替え・
キャリブのガイド・タブ帯のスクロール」を、実際に GUI を起動して確かめるテスト。
2026-10-01 に作成。合計 約130項目。

## ⚠ 必ず本番とは別のフォルダ（サンドボックス）で実行すること

テストは状態ファイルを書き換える（フォルダを作る・タブを消す・共有を作る など）。
本体と状態ファイルとキャッシュを丸ごと別のフォルダに複製し、その中で実行する。
テストは「自分と同じフォルダにある FRS-SIMULATOR.py」を読み込むので、複製先で動く。

```bash
SB=<作業用フォルダ>/sbx
mkdir -p "$SB/cache"
cp FRS-SIMULATOR.py frs2015_gui_state*.json "$SB/"
cp -r cache/ankle_pose cache/av_result cache/av_plot_templates.json "$SB/cache/"
cp tools/verify/sandbox_tests/*.py "$SB/"
cd "$SB"
```

`test_ui.py` / `test_tabbar_scroll.py` は「フォルダ未使用・すべて表示」の状態を前提にしているので、
複製した `frs2015_gui_state_ankle_sim.json` から `folders` `folder_filter` `bone_groups` と
各タブの `folder` `bone_group` を外してから実行する。

## 一覧

| ファイル | 確かめること |
|---|---|
| `test_ui.py` | フォルダの絞り込み・移動・＋・削除・保存と復元、右端へ、画面が開くか |
| `test_tabbar_scroll.py` | ◀ ▶ と「右端へ」で、開いているタブへ引き戻されないこと |
| `test_batch.py fast` | まとめて解析（ダイアログ0回・キャンセルで上書きしない・手動の④は従来どおり）|
| `test_batch.py real` | 本物の録画で 手動の④ と まとめて解析 の結果が完全一致するか（数分かかる・録画が必要）|
| `test_share_relink.py share` | 骨リストの共有（反映・保存と復元・解除・自動解除）|
| `test_share_relink.py relink` | パスの付け替え（テスト用フォルダを作って移動 → 探す → 付け替え）|
| `test_share_relink.py realdata` | 実際のタブで、見つからないパスがいくつ見つかるか（書き換えはしない）|
| `test_share_relink.py guide` | キャリブのガイド（図の 1左上〜4左下 = ArUco の角0〜3、U で戻す）|
| `capture_util.py` | 隠れていても撮れる PrintWindow のスクリーンショット |

`PYTHONIOENCODING=utf-8` を付けて実行する。
