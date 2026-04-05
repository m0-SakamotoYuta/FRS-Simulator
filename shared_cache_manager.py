# ============================================================================
# SharedCacheManager — NAS共有キャッシュ
# ============================================================================
# 複数PC（Mac/Windows）間でFEM・ヒートマップ等の計算結果を共有する。
#
# ■ 動作原理:
#   1. NASパスが設定済み & アクセス可能 → NASに保存・NASから読み込み
#   2. NASが使えない場合 → メモリキャッシュのみ（セッション内限定）
#
# ■ ハッシュ計算:
#   ファイルパスではなくファイル内容（メッシュ頂点座標等）に基づくため、
#   異なるPC・OSで同じデータなら同じハッシュになる。
# ============================================================================

import pickle
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
    """NAS共有キャッシュマネージャ。

    NASが使える場合はNASに保存・読み込み。
    NASが使えない場合はメモリキャッシュのみ（セッション内限定）。

    Args:
        nas_dir: NAS上のキャッシュディレクトリ（Noneまたは空文字でNAS無効）
        namespace: キャッシュの名前空間（"fem", "overlap"等、ログ用）
        max_nas_gb: NASキャッシュの最大サイズ（GB）
    """

    def __init__(
        self,
        nas_dir: Optional[str] = None,
        namespace: str = "general",
        max_nas_gb: float = 10.0,
    ):
        self.namespace = namespace
        self.max_nas_bytes = int(max_nas_gb * 1024**3)

        # NASディレクトリ
        self._nas_dir: Optional[Path] = None
        if nas_dir and nas_dir.strip():
            self._nas_dir = Path(nas_dir.strip())

        # メモリキャッシュ（セッション内高速アクセス用、LRU簡易実装）
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
            if self._nas_dir.exists():
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
    # 取得（メモリ → NAS の順に検索）
    # ================================================================

    def get(self, key: str) -> Optional[Any]:
        """キャッシュから結果を取得する。検索順: メモリ → NAS"""
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
                    self._log(f"NASキャッシュヒット (key={key})")
                    return result
                except Exception as e:
                    self._log(f"NAS読み込みエラー (key={key}): {e}")

        return None

    # ================================================================
    # 保存（メモリ + NAS に書き込み）
    # ================================================================

    def put(self, key: str, data: Any) -> None:
        """キャッシュに結果を保存する。NASが使えない場合はメモリのみ。"""
        self._memory_put(key, data)

        if self.is_nas_available():
            try:
                self._save_to_nas(key, data)
            except Exception as e:
                self._log(f"NAS保存エラー (key={key}): {e}")
        else:
            self._log(f"NAS未接続のため保存スキップ (key={key})")

    # ================================================================
    # キャッシュ情報・管理
    # ================================================================

    def stats(self) -> Dict[str, Any]:
        """キャッシュの統計情報を返す"""
        nas_files = []
        nas_size = 0
        if self.is_nas_available():
            nas_files = list(self._nas_dir.glob("*.pkl"))
            nas_size = sum(f.stat().st_size for f in nas_files)

        return {
            'namespace': self.namespace,
            'nas_available': self.is_nas_available(),
            'nas_count': len(nas_files),
            'nas_size_mb': nas_size / (1024**2),
            'memory_count': len(self._memory),
        }

    def clear_all(self) -> None:
        """メモリ + NAS のキャッシュを全削除"""
        self._memory.clear()
        self._memory_order.clear()
        if self.is_nas_available() and self._nas_dir.exists():
            import shutil
            shutil.rmtree(str(self._nas_dir), ignore_errors=True)
            self._log("NASキャッシュを削除しました")

    # ================================================================
    # メタデータ管理（キャッシュ一覧表示用）
    # ================================================================

    def save_metadata(self, meta_key: str, metadata: dict) -> None:
        """キャッシュエントリのメタデータをNASに保存する。"""
        if not self.is_nas_available():
            return
        try:
            meta_file = self._nas_dir / f"_meta_{meta_key}.json"
            with open(meta_file, 'w', encoding='utf-8') as f:
                json.dump(metadata, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self._log(f"メタデータ保存エラー: {e}")

    def get_all_metadata(self) -> List[dict]:
        """NAS上の全メタデータを取得（更新日時の新しい順）。"""
        if not self.is_nas_available():
            return []
        result = []
        try:
            for meta_file in sorted(
                self._nas_dir.glob("_meta_*.json"),
                key=lambda f: f.stat().st_mtime, reverse=True
            ):
                try:
                    with open(meta_file, 'r', encoding='utf-8') as f:
                        meta = json.load(f)
                    meta['_meta_key'] = meta_file.stem[6:]  # "_meta_" を除去
                    result.append(meta)
                except Exception:
                    pass
        except Exception:
            pass
        return result

    def delete_dataset(self, meta_key: str) -> None:
        """メタデータとそれに紐づくPKLファイルを削除する。"""
        if not self.is_nas_available():
            return
        try:
            # メタデータ読み込み（frame_keysがあればPKLも削除）
            meta_file = self._nas_dir / f"_meta_{meta_key}.json"
            if meta_file.exists():
                with open(meta_file, 'r', encoding='utf-8') as f:
                    meta = json.load(f)
                meta_file.unlink()
                # FEM: frame_keysリストのPKL削除
                for key in meta.get('frame_keys', []):
                    pkl = self._nas_dir / f"{key}.pkl"
                    if pkl.exists():
                        pkl.unlink()
                    self._memory.pop(key, None)
                # Overlap: meta_keyと同名のPKL削除
                if meta.get('type') == 'overlap':
                    pkl = self._nas_dir / f"{meta_key}.pkl"
                    if pkl.exists():
                        pkl.unlink()
                    self._memory.pop(meta_key, None)
        except Exception as e:
            self._log(f"データセット削除エラー: {e}")

    def set_nas_dir(self, nas_dir: Optional[str]) -> None:
        """NASディレクトリを変更する"""
        if nas_dir and nas_dir.strip():
            self._nas_dir = Path(nas_dir.strip())
        else:
            self._nas_dir = None

    # ================================================================
    # 内部メソッド
    # ================================================================

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

    def _log(self, msg: str) -> None:
        print(f"[SharedCache:{self.namespace}] {msg}")
