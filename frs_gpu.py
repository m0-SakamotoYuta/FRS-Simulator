# ============================================================================
# frs_gpu.py — GPU/ベクトル化 計算バックエンド（PyTorch）
# ============================================================================
# ヒートマップ等の重い距離計算を、PyTorch で一括ベクトル化して高速化する。
#
# ■ 方針
#   - GPU(CUDA/Apple MPS) があれば自動で使い、無ければ CPU で実行（いずれも
#     Python の for ループより大幅に高速）。
#   - PyTorch が未インストールなら _HAS_TORCH=False を返し、呼び出し側は
#     従来の CPU 実装にフォールバックする。
#   - 距離は「点→三角形（メッシュ表面）」の厳密距離。VTK の find_closest_cell と
#     同じ意味なので、結果値がブレず、内容ベースのキャッシュとも整合する。
#
# ■ サーバー(Ubuntu + NVIDIA)での導入例:
#     pip install torch --index-url https://download.pytorch.org/whl/cu121
#   CPUのみ(Windows/Mac ノート等)での導入例:
#     pip install torch
# ============================================================================

import numpy as np

try:
    import torch
    _HAS_TORCH = True
except Exception:
    torch = None
    _HAS_TORCH = False


def has_torch() -> bool:
    return _HAS_TORCH


def get_device(prefer: str = "auto"):
    """使用デバイスを返す。'auto'=CUDA→MPS→CPU の順に自動選択。"""
    if not _HAS_TORCH:
        return None
    if prefer and prefer != "auto":
        return prefer
    try:
        if torch.cuda.is_available():
            return "cuda"
    except Exception:
        pass
    try:
        if getattr(torch.backends, "mps", None) is not None and torch.backends.mps.is_available():
            return "mps"
    except Exception:
        pass
    return "cpu"


def device_info() -> str:
    """現在のデバイス情報の文字列（ログ表示用）。"""
    if not _HAS_TORCH:
        return "PyTorch未導入（CPUフォールバック）"
    dev = get_device()
    if dev == "cuda":
        try:
            return f"CUDA: {torch.cuda.get_device_name(0)}"
        except Exception:
            return "CUDA"
    return dev or "cpu"


def _closest_dist2_point_triangle(p, a, b, c):
    """点 p と三角形(a,b,c) の最近点までの距離の二乗を返す（Ericson法のベクトル化）。

    p: (K,1,3), a/b/c: (1,M,3) → ブロードキャストで (K,M,3)。戻り値 (K,M)。
    """
    ab = b - a
    ac = c - a
    ap = p - a
    d1 = (ab * ap).sum(-1)
    d2 = (ac * ap).sum(-1)
    bp = p - b
    d3 = (ab * bp).sum(-1)
    d4 = (ac * bp).sum(-1)
    cp = p - c
    d5 = (ab * cp).sum(-1)
    d6 = (ac * cp).sum(-1)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    denom = 1.0 / (va + vb + vc + 1e-20)
    v = (vb * denom).unsqueeze(-1)
    w = (vc * denom).unsqueeze(-1)

    # 既定: 三角形内部（重心座標）
    closest = a + v * ab + w * ac

    def where3(mask, val):
        return torch.where(mask.unsqueeze(-1), val.expand_as(closest), closest)

    # 頂点A領域
    closest = where3((d1 <= 0) & (d2 <= 0), a)
    # 頂点B領域
    closest = where3((d3 >= 0) & (d4 <= d3), b)
    # 頂点C領域
    closest = where3((d6 >= 0) & (d5 <= d6), c)
    # 辺AB領域
    mAB = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
    tAB = (d1 / (d1 - d3 + 1e-20)).clamp(0.0, 1.0).unsqueeze(-1)
    closest = torch.where(mAB.unsqueeze(-1), a + tAB * ab, closest)
    # 辺AC領域
    mAC = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
    tAC = (d2 / (d2 - d6 + 1e-20)).clamp(0.0, 1.0).unsqueeze(-1)
    closest = torch.where(mAC.unsqueeze(-1), a + tAC * ac, closest)
    # 辺BC領域
    mBC = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
    tBC = ((d4 - d3) / ((d4 - d3) + (d5 - d6) + 1e-20)).clamp(0.0, 1.0).unsqueeze(-1)
    closest = torch.where(mBC.unsqueeze(-1), b + tBC * (c - b), closest)

    diff = p - closest
    return (diff * diff).sum(-1)


def point_to_mesh_distance(query_points: np.ndarray,
                           tri_vertices: np.ndarray,
                           tri_faces: np.ndarray,
                           batch: int = 1024,
                           device: str = "auto"):
    """各 query 点から三角形メッシュ表面までの厳密最短距離を一括計算する。

    Args:
        query_points: (N,3)
        tri_vertices: (V,3)
        tri_faces: (M,3) 三角形の頂点インデックス
        batch: query 点のバッチサイズ（メモリ調整用）
        device: 'auto'|'cuda'|'mps'|'cpu'

    Returns:
        (N,) の距離配列（np.float32）。PyTorch未導入なら None。
    """
    if not _HAS_TORCH:
        return None
    if len(tri_faces) == 0:
        return None
    dev = get_device(device)
    with torch.no_grad():
        V = torch.as_tensor(np.asarray(tri_vertices), dtype=torch.float32, device=dev)
        F = torch.as_tensor(np.asarray(tri_faces), dtype=torch.long, device=dev)
        A = V[F[:, 0]].unsqueeze(0)  # (1,M,3)
        B = V[F[:, 1]].unsqueeze(0)
        C = V[F[:, 2]].unsqueeze(0)
        Q = torch.as_tensor(np.asarray(query_points), dtype=torch.float32, device=dev)
        n = Q.shape[0]
        cur_batch = max(int(batch), 1)
        while True:
            try:
                out = torch.empty(n, dtype=torch.float32, device=dev)
                for i in range(0, n, cur_batch):
                    q = Q[i:i + cur_batch].unsqueeze(1)  # (K,1,3)
                    d2 = _closest_dist2_point_triangle(q, A, B, C)  # (K,M)
                    out[i:i + cur_batch] = d2.min(dim=1).values
                return out.clamp_min(0).sqrt().detach().cpu().numpy().astype(np.float32)
            except RuntimeError as e:
                # GPUメモリ不足ならバッチを半分にして再試行
                if "out of memory" in str(e).lower() and cur_batch > 32:
                    if dev == "cuda":
                        try:
                            torch.cuda.empty_cache()
                        except Exception:
                            pass
                    cur_batch = max(32, cur_batch // 2)
                    print(f"[frs_gpu] メモリ不足 → バッチを {cur_batch} に縮小して再試行")
                    continue
                raise
