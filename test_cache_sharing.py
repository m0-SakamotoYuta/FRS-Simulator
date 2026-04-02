"""
クロスPC共有キャッシュ テストスクリプト
使い方:
  1. WindowsでNASパスを指定して実行 → キャッシュ書き込み
  2. MacでNASパスを指定して実行 → 同じキャッシュが読めるか確認
"""

import sys
import numpy as np
from pathlib import Path

# NASパスをコマンドライン引数 or ここに直接書く
if len(sys.argv) > 1:
    NAS_BASE = sys.argv[1]
else:
    NAS_BASE = input("NASベースパスを入力 (例: Z:/FRS_cache または /Volumes/NAS/FRS_cache): ").strip()

sys.path.insert(0, str(Path(__file__).parent))
from shared_cache_manager import SharedCacheManager, compute_content_hash

print(f"\n{'='*50}")
print(f"OS: {sys.platform}")
print(f"NASパス: {NAS_BASE}")
print(f"{'='*50}\n")

# --- テスト用固定メッシュ（Mac/Win共通） ---
TEST_VERTICES = np.array([
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

TEST_DATA = {
    "stress": np.array([100.0, 200.0, 150.0]),
    "displacement": np.array([0.01, 0.02, 0.015]),
    "description": "cross-platform cache test"
}

# --- SharedCacheManager初期化 ---
scm = SharedCacheManager(
    local_dir=str(Path(__file__).parent / ".test_cache"),
    nas_dir=NAS_BASE + "/.fem_cache",
    namespace="test",
)

print(f"[NAS接続] {'OK' if scm.is_nas_available() else 'NG（ローカルのみ）'}")

# --- ハッシュ計算 ---
cache_key = compute_content_hash(TEST_VERTICES)
print(f"[ハッシュ] {cache_key}  ← Mac/Winで同じ値になるはず\n")

# --- 読み込みテスト ---
print("--- 読み込みテスト ---")
result = scm.get(cache_key)
if result is not None:
    print(f"[OK] キャッシュHIT!")
    print(f"     stress: {result['stress']}")
    print(f"     description: {result['description']}")
    print("\n→ 別PCで書き込んだキャッシュが正常に読めました！")
else:
    print("[MISS] キャッシュなし → 書き込みます")
    scm.put(cache_key, TEST_DATA)
    print(f"[OK] NASに書き込み完了: {cache_key}")
    print("\n→ 別PC（Mac or Windows）でこのスクリプトを実行してキャッシュが読めるか確認してください")

# --- 統計情報 ---
print(f"\n--- 統計情報 ---")
stats = scm.get_stats()
print(f"ローカルキャッシュ: {stats['local_count']}件 ({stats['local_size_mb']:.1f} MB)")
print(f"NASキャッシュ:     {stats['nas_count']}件 ({stats['nas_size_mb']:.1f} MB)")
print(f"NAS接続:          {'OK' if stats['nas_available'] else 'NG'}")
