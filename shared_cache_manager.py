# ============================================================================
# SharedCacheManager — NAS共有 + ローカルフォールバック 2層キャッシュ
# ============================================================================
# 複数PC（Mac/Windows/Linux）間でFEM・ヒートマップ等の計算結果を共有する。
#
# ■ 動作原理:
#   1. NASパスが設定済み & アクセス可能 → NASを優先的に使用
#   2. NASが使えない場合 → ローカルキャッシュにフォールバック
#   3. ローカルで計算した結果は、次回NAS接続時に自動同期（sync）
#
# ■ ハッシュ計算:
#   ファイルパスではなくファイル内容（メッシュ頂点座標等）に基づくため、
#   異なるPC・OSで同じデータなら同じハッシュになる。
#
# ■ 使い方:
#   from shared_cache_manager import SharedCacheManager
#   cache = SharedCacheManager(
#       local_dir=".fem_cache",
#       nas_dir="Z:/FRS_cache/fem_cache",  # 任意
#       namespace="fem",
#   )
#   result = cache.get(hash_key)
#   cache.put(hash_key, result)
#   cache.sync()  # ローカル→NAS同期
# ============================================================================

import os
import pickle
import shutil
import time
import hashlib
import json
import numpy as np
from pathlib import Path
from typing import Optional, Any, Dict, List


# ============================================================================
# ハッシュヘルパー（パスに依存しない、内容ベースのハッシュ）
# ============================================================================

def compute_content_hash(*args, precision: int = 3) -> str:
    """任意のデータからコンテンツベースのSHA-256ハッシュを計算する。

    ファイルパスではなくデータの中身に基づくため、OS・PCを問わず同一。

    Args:
        *args: ハッシュに含めるデータ（numpy配列、数値、文字列など）
        precision: numpy配列の丸め桁数（デフォルト3 = 1μm精度）

    Returns:
        16文字のハッシュ文字列
    """
    hasher = hashlib.sha256()
    for arg in args:
        if isinstance(arg, np.ndarray):
            rounded = np.round(arg, decimals=precision)
            hasher.update(rounded.tobytes())
        elif isinstance(arg, (int, float)):
            hasher.update(str(arg).encode('utf-8'))
        elif isinstance(arg, str):
            hasher.update(arg.encode('utf-8'))
        elif isinstance(arg, (list, tuple)):
            for item in arg:
                hasher.update(str(item).encode('utf-8'))
        elif isinstance(arg, dict):
            for k, v in sorted(arg.items()):
                hasher.update(f"{k}={v}".encode('utf-8'))
        elif arg is None:
            hasher.update(b"__none__")
        else:
            hasher.update(str(arg).encode('utf-8'))
    return hasher.hexdigest()[:16]


# ============================================================================
# SharedCacheManager 本体
# ============================================================================

class SharedCacheManager:
    """NAS共有 + ローカルフォールバックの2層キャッシュマネージャ。

    Args:
        local_dir: ローカルキャッシュディレクトリ（相対パスの場合スクリプトからの相対）
        nas_dir: NAS上のキャッシュディレクトリ（Noneまたは空文字でNAS無効）
        namespace: キャッシュの名前空間（"fem", "overlap"等、ログ用）
        max_local_gb: ローカルキャッシュの最大サイズ（GB）
        max_nas_gb: NASキャッシュの最大サイズ（GB）
    """

    def __init__(
        self,
        local_dir: str = ".cache",
        nas_dir: Optional[str] = None,
        namespace: str = "general",
        max_local_gb: float = 2.0,
        max_nas_gb: float = 10.0,
    ):
        self.namespace = namespace
        self.max_local_bytes = int(max_local_gb * 1024**3)
        self.max_nas_bytes = int(max_nas_gb * 1024**3)

        # ローカルディレクトリの解決
        if os.path.isabs(local_dir):
            self._local_dir = Path(local_dir)
        else:
            try:
                base = Path(os.path.dirname(os.path.abspath(__file__)))
            except Exception:
                base = Path.cwd()
            self._local_dir = base / local_dir

        # NASディレクトリ
        self._nas_dir: Optional[Path] = None
        if nas_dir and nas_dir.strip():
            self._nas_dir = Path(nas_dir.strip())

        # メモリキャッシュ（LRU簡易実装）
        self._memory: Dict[str, Any] = {}
        self._memory_order: List[str] = []
        self._memory_max = 10

    # ================================================================
    # NAS接続チェック
    # ================================================================

    def is_nas_available(self) -> bool:
        """NASが設定されていてアクセス可能かどうか"""
        if self._nas_dir is None:
            return False
        try:
            # ディレクトリが存在するか、作成できるか
            if self._nas_dir.exists():
                # 書き込みテスト
                test_file = self._nas_dir / ".write_test"
                test_file.write_text("ok")
                test_file.unlink()
                return True
            else:
                self._nas_dir.mkdir(parents=True, exist_ok=True)
                return True
        except (OSError, PermissionError):
            return False

    # ================================================================
    # 取得（メモリ → NAS → ローカル の順に検索）
    # ================================================================

    def get(self, key: str) -> Optional[Any]:
        """キャッシュから結果を取得する。

        検索順: メモリ → NAS → ローカル
        NASにあってローカルにない場合、ローカルにもコピーする。
        """
        # 1. メモリキャッシュ
        if key in self._memory:
            self._log(f"メモリキャッシュヒット (key={key})")
            return self._memory[key]

        # 2. NASキャッシュ
        if self.is_nas_available():
            nas_file = self._nas_dir / f"{key}.pkl"
            if nas_file.exists():
                try:
                    result = self._load_pickle(nas_file)
                    self._memory_put(key, result)
                    # ローカルにもコピー（高速アクセス用）
                    self._save_to_local(key, result)
                    self._log(f"NASキャッシュヒット (key={key})")
                    return result
                except Exception as e:
                    self._log(f"NAS読み込みエラー (key={key}): {e}")

        # 3. ローカルキャッシュ
        local_file = self._local_dir / f"{key}.pkl"
        if local_file.exists():
            try:
                result = self._load_pickle(local_file)
                self._memory_put(key, result)
                self._log(f"ローカルキャッシュヒット (key={key})")
                return result
            except Exception as e:
                self._log(f"ローカル読み込みエラー (key={key}): {e}")

        return None

    # ================================================================
    # 保存（NAS + ローカル 両方に書き込み）
    # ================================================================

    def put(self, key: str, data: Any) -> None:
        """キャッシュに結果を保存する。

        NASが使える場合: NAS + ローカル 両方に保存
        NASが使えない場合: ローカルのみ + 同期待ちリストに追加
        """
        # メモリキャッシュ
        self._memory_put(key, data)

        # ローカル保存（常に実行）
        self._save_to_local(key, data)

        # NAS保存（可能な場合）
        if self.is_nas_available():
            try:
                self._save_to_nas(key, data)
            except Exception as e:
                self._log(f"NAS保存エラー (key={key}): {e}")
                self._add_to_pending_sync(key)
        else:
            self._add_to_pending_sync(key)

    # ================================================================
    # 同期（ローカル → NAS）
    # ================================================================

    def sync(self) -> Dict[str, int]:
        """ローカルの未同期キャッシュをNASに同期する。

        Returns:
            {'synced': 同期成功数, 'failed': 失敗数, 'skipped': スキップ数}
        """
        stats = {'synced': 0, 'failed': 0, 'skipped': 0}

        if not self.is_nas_available():
            self._log("NASが利用できないため同期をスキップ")
            return stats

        pending = self._get_pending_sync_list()
        if not pending:
            self._log("同期待ちなし")
            return stats

        self._log(f"同期開始: {len(pending)}件")
        for key in pending:
            local_file = self._local_dir / f"{key}.pkl"
            nas_file = self._nas_dir / f"{key}.pkl"

            if not local_file.exists():
                stats['skipped'] += 1
                continue

            if nas_file.exists():
                # NASの方が新しい場合はスキップ
                if nas_file.stat().st_mtime >= local_file.stat().st_mtime:
                    stats['skipped'] += 1
                    continue

            try:
                shutil.copy2(str(local_file), str(nas_file))
                stats['synced'] += 1
                self._log(f"  同期完了: {key}")
            except Exception as e:
                stats['failed'] += 1
                self._log(f"  同期失敗: {key} ({e})")

        # 成功した分は同期待ちリストから削除
        if stats['synced'] > 0:
            self._clear_pending_sync(
                [k for k in pending if (self._nas_dir / f"{k}.pkl").exists()]
            )

        self._log(f"同期完了: {stats}")
        return stats

    # ================================================================
    # キャッシュ情報・管理
    # ================================================================

    def stats(self) -> Dict[str, Any]:
        """キャッシュの統計情報を返す"""
        local_files = list(self._local_dir.glob("*.pkl")) if self._local_dir.exists() else []
        local_size = sum(f.stat().st_size for f in local_files)

        nas_files = []
        nas_size = 0
        if self.is_nas_available():
            nas_files = list(self._nas_dir.glob("*.pkl"))
            nas_size = sum(f.stat().st_size for f in nas_files)

        pending = self._get_pending_sync_list()

        return {
            'namespace': self.namespace,
            'local_count': len(local_files),
            'local_size_mb': local_size / (1024**2),
            'nas_available': self.is_nas_available(),
            'nas_count': len(nas_files),
            'nas_size_mb': nas_size / (1024**2),
            'pending_sync': len(pending),
            'memory_count': len(self._memory),
        }

    def clear_local(self) -> None:
        """ローカルキャッシュを全削除"""
        self._memory.clear()
        self._memory_order.clear()
        if self._local_dir.exists():
            shutil.rmtree(str(self._local_dir), ignore_errors=True)
        self._log("ローカルキャッシュを削除しました")

    def clear_all(self) -> None:
        """ローカル + NAS のキャッシュを全削除"""
        self.clear_local()
        if self.is_nas_available() and self._nas_dir.exists():
            shutil.rmtree(str(self._nas_dir), ignore_errors=True)
            self._log("NASキャッシュを削除しました")

    def set_nas_dir(self, nas_dir: Optional[str]) -> None:
        """NASディレクトリを変更する"""
        if nas_dir and nas_dir.strip():
            self._nas_dir = Path(nas_dir.strip())
        else:
            self._nas_dir = None

    # ================================================================
    # 内部メソッド
    # ================================================================

    def _save_to_local(self, key: str, data: Any) -> None:
        try:
            self._local_dir.mkdir(parents=True, exist_ok=True)
            filepath = self._local_dir / f"{key}.pkl"
            with open(filepath, 'wb') as f:
                pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
            self._enforce_size_limit(self._local_dir, self.max_local_bytes)
        except Exception as e:
            self._log(f"ローカル保存エラー (key={key}): {e}")

    def _save_to_nas(self, key: str, data: Any) -> None:
        self._nas_dir.mkdir(parents=True, exist_ok=True)
        filepath = self._nas_dir / f"{key}.pkl"
        with open(filepath, 'wb') as f:
            pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
        self._enforce_size_limit(self._nas_dir, self.max_nas_bytes)

    def _load_pickle(self, filepath: Path) -> Any:
        with open(filepath, 'rb') as f:
            return pickle.load(f)

    def _memory_put(self, key: str, data: Any) -> None:
        if key in self._memory:
            self._memory_order.remove(key)
        self._memory[key] = data
        self._memory_order.append(key)
        while len(self._memory_order) > self._memory_max:
            old_key = self._memory_order.pop(0)
            self._memory.pop(old_key, None)

    def _enforce_size_limit(self, cache_dir: Path, max_bytes: int) -> None:
        """LRUでサイズ制限を強制（最終更新が古いものから削除）"""
        try:
            files = sorted(cache_dir.glob("*.pkl"), key=lambda f: f.stat().st_mtime)
            total = sum(f.stat().st_size for f in files)
            while total > max_bytes and files:
                old = files.pop(0)
                total -= old.stat().st_size
                old.unlink()
        except Exception:
            pass

    # --- 同期待ちリスト ---

    def _pending_sync_file(self) -> Path:
        return self._local_dir / "_pending_sync.json"

    def _get_pending_sync_list(self) -> List[str]:
        f = self._pending_sync_file()
        if f.exists():
            try:
                with open(f, 'r') as fp:
                    return json.load(fp)
            except Exception:
                pass
        return []

    def _add_to_pending_sync(self, key: str) -> None:
        pending = self._get_pending_sync_list()
        if key not in pending:
            pending.append(key)
            self._local_dir.mkdir(parents=True, exist_ok=True)
            with open(self._pending_sync_file(), 'w') as f:
                json.dump(pending, f)

    def _clear_pending_sync(self, synced_keys: List[str]) -> None:
        pending = self._get_pending_sync_list()
        remaining = [k for k in pending if k not in synced_keys]
        with open(self._pending_sync_file(), 'w') as f:
            json.dump(remaining, f)

    def _log(self, msg: str) -> None:
        print(f"[SharedCache:{self.namespace}] {msg}")
