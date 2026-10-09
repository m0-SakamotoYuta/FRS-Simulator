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
| `test_multi_viewer.py` | 可視化ウィンドウを複数開いて比較（番号つきの題名・左右に並ぶ・共通の再生/巻き戻し/コマ送り/速度・ずれを保つ・末尾で止まる・視点の連動・連動を外す・閉じる）。合成データで 43 項目 |
| `test_multi_viewer_real.py <前> <後> [代用モデル]` | 実際の ankle タブ 2 つで同じことを確かめる。見つからないモデルは代用品で読み込む（本番の経路を通すのが目的）|
| `test_size_rescale.py [redetect]` | ②の実寸を変えると④なしで反映（位置だけ拡縮・保存済みの④は不変）。redetect で④のやり直しと完全一致を確認（録画が必要。④は録画フォルダに debug_frame0 を書くので注意）|
| `test_depth_size.py` | 深度から実寸を推定（録画を15コマおきに読む・約2分）、接触で確かめる、窓 |
| `test_true_size.py 1` → `2` | 実寸（ノギス）と補正後（自動/手動）の分離、メモ、自動に戻す、CSV、再起動後の保存（2 は 1 のあとに実行）|
| `test_apply_fix.py` | 「補正後」を入れる共通の関数（`_ankle_size_apply_to_tabs`）で、開いていない古いタブの実寸（ノギス）が推定値で上書きされないこと／まとめて解析の実寸欄がノギスの値（修正前のコードでは 9 件 NG で再現）|
| `test_size_sync.py` | ②の「他のタブと同期…」: 同じフォルダを最初からチェック、補正後だけ入れて手動、実寸（ノギス）とほかのフォルダは変えない |
| `capture_util.py` | 隠れていても撮れる PrintWindow のスクリーンショット |

`PYTHONIOENCODING=utf-8` を付けて実行する。
