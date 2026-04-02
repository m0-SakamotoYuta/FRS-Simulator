# ============================================================================
# FEM Contact Solver — 表面有限要素法による関節面接触力学解析
# ============================================================================
# FRS_Simulator Phase 2 拡張モジュール
#
# ■ 概要:
#   三角形シェル（CST膜）要素を使った表面FEM接触解析ソルバー。
#   PyVistaの三角形メッシュをそのまま有限要素として利用し、
#   ペナルティ法による接触力学解析を実行する。
#
# ■ 理論背景:
#   - 要素: CST (Constant Strain Triangle) 膜要素（3D空間内）
#   - 接触: ペナルティ法（侵入量に比例した法線力）
#   - 材料: 線形弾性（平面応力仮定）
#   - 解法: スパース直接法（scipy.sparse.linalg.spsolve）
#
# ■ 依存ライブラリ:
#   numpy, scipy（標準的な科学計算ライブラリのみ）
#   pyvista（入出力インターフェースとして）
#
# ■ 使用例:
#   solver = FEMContactSolver()
#   results = solver.analyze(prox_mesh, dist_mesh)
#   # results.contact_pressure  → 接触圧 [MPa]
#   # results.von_mises_stress  → von Mises応力 [MPa]
#   # results.displacement      → 変位ベクトル [mm]
#
# ============================================================================

import numpy as np
from scipy.spatial import cKDTree
from scipy import sparse
from scipy.sparse.linalg import spsolve
from dataclasses import dataclass, field
from typing import Optional, Tuple
import time


# ============================================================================
# データクラス
# ============================================================================

@dataclass
class MaterialProperties:
    """軟骨材料特性
    
    Attributes:
        E: ヤング率 [MPa]（関節軟骨の典型値: 1〜20 MPa）
        nu: ポアソン比（関節軟骨: 0.45〜0.49, 非圧縮に近い）
        thickness: 軟骨厚さ [mm]（典型値: 1〜4 mm）
    """
    E: float = 10.0        # MPa
    nu: float = 0.45
    thickness: float = 2.0  # mm

    def plane_stress_matrix(self) -> np.ndarray:
        """平面応力の構成マトリクス D (3×3)
        
        Returns:
            D = E/(1-ν²) * [[1,  ν,  0        ],
                            [ν,  1,  0        ],
                            [0,  0,  (1-ν)/2  ]]
        """
        E, nu = self.E, self.nu
        coeff = E / (1.0 - nu * nu)
        D = coeff * np.array([
            [1.0,  nu,  0.0],
            [nu,   1.0, 0.0],
            [0.0,  0.0, (1.0 - nu) / 2.0]
        ])
        return D


@dataclass
class ContactParameters:
    """接触解析パラメータ
    
    Attributes:
        penalty_stiffness: ペナルティ剛性 [MPa/mm]
            関節面接触では 100〜1000 MPa/mm が適切。
            大きすぎると数値不安定、小さすぎると侵入量が大きくなる。
        contact_tolerance: 接触判定閾値 [mm]
            この距離以下の点を接触候補とする。
        friction_coefficient: 摩擦係数（0 = 摩擦なし）
            関節軟骨: 0.001〜0.03（滑液潤滑下では極めて低い）
    """
    penalty_stiffness: float = 500.0   # MPa/mm
    contact_tolerance: float = 2.0     # mm
    friction_coefficient: float = 0.0  # 現バージョンでは摩擦なし


@dataclass
class FEMResults:
    """FEM解析結果
    
    全てのスカラー値はメッシュの頂点数と同じ長さの配列。
    PyVistaメッシュに直接代入して可視化できる。
    """
    # 応力 [MPa]
    contact_pressure: np.ndarray = field(default_factory=lambda: np.array([]))
    von_mises_stress: np.ndarray = field(default_factory=lambda: np.array([]))
    max_principal_stress: np.ndarray = field(default_factory=lambda: np.array([]))
    min_principal_stress: np.ndarray = field(default_factory=lambda: np.array([]))
    
    # ひずみ [-]
    max_principal_strain: np.ndarray = field(default_factory=lambda: np.array([]))
    
    # 変位 [mm]
    displacement: np.ndarray = field(default_factory=lambda: np.array([]))
    displacement_magnitude: np.ndarray = field(default_factory=lambda: np.array([]))
    
    # 接触情報
    penetration_depth: np.ndarray = field(default_factory=lambda: np.array([]))
    contact_area: float = 0.0          # 接触面積 [mm²]
    total_contact_force: float = 0.0   # 全接触力 [N]
    peak_contact_pressure: float = 0.0 # 最大接触圧 [MPa]
    
    # メタ情報
    n_nodes: int = 0
    n_elements: int = 0
    n_contact_nodes: int = 0
    solve_time_sec: float = 0.0
    
    def summary(self) -> str:
        """解析結果のサマリーを返す"""
        lines = [
            "=" * 60,
            "FEM 接触解析結果サマリー",
            "=" * 60,
            f"  節点数:             {self.n_nodes:,}",
            f"  要素数:             {self.n_elements:,}",
            f"  接触節点数:         {self.n_contact_nodes:,}",
            f"  接触面積:           {self.contact_area:.2f} mm²",
            f"  全接触力:           {self.total_contact_force:.2f} N",
            f"  最大接触圧:         {self.peak_contact_pressure:.4f} MPa",
            f"  最大von Mises応力:  {np.max(self.von_mises_stress):.4f} MPa" if len(self.von_mises_stress) > 0 else "",
            f"  最大変位:           {np.max(self.displacement_magnitude):.4f} mm" if len(self.displacement_magnitude) > 0 else "",
            f"  解析時間:           {self.solve_time_sec:.2f} 秒",
            "=" * 60,
        ]
        return "\n".join(line for line in lines if line)


# ============================================================================
# FEMContactSolver 本体
# ============================================================================

class FEMContactSolver:
    """表面FEM接触解析ソルバー
    
    三角形メッシュ（STL等）をそのままCST膜要素として利用し、
    ペナルティ法による接触力学解析を実行する。
    
    ■ 処理フロー:
      1. メッシュ前処理（三角形分割確認、法線計算）
      2. 接触検出（KD-Tree近傍探索 + 符号付き距離）
      3. 要素剛性行列計算（CST膜要素、3D空間内）
      4. 全体剛性行列組み立て（スパース行列）
      5. 接触力ベクトル計算（ペナルティ法）
      6. 境界条件適用（接触領域外の固定）
      7. 連立方程式求解
      8. 応力・ひずみ後処理
    
    ■ 使用方法:
      solver = FEMContactSolver(
          material=MaterialProperties(E=10.0, nu=0.45, thickness=2.0),
          contact=ContactParameters(penalty_stiffness=500.0)
      )
      results = solver.analyze(prox_mesh, dist_mesh)
    """
    
    def __init__(
        self,
        material: Optional[MaterialProperties] = None,
        contact: Optional[ContactParameters] = None,
        verbose: bool = True,
    ):
        self.material = material or MaterialProperties()
        self.contact = contact or ContactParameters()
        self.verbose = verbose
        
        # 内部データ（analyze() で初期化）
        self._nodes: Optional[np.ndarray] = None      # (N, 3) 節点座標
        self._elements: Optional[np.ndarray] = None    # (M, 3) 要素接続（三角形）
        self._normals: Optional[np.ndarray] = None     # (N, 3) 節点法線
        self._n_nodes: int = 0
        self._n_elements: int = 0
        self._n_dof: int = 0
    
    # ----------------------------------------------------------------
    # 公開メソッド
    # ----------------------------------------------------------------
    
    def analyze(self, prox_mesh, dist_mesh,
                boundary_mode: str = "auto",
                max_nodes: int = 50000) -> FEMResults:
        """接触FEM解析を実行
        
        Args:
            prox_mesh: 近位メッシュ (pyvista.PolyData)
                解析対象の面。この面上の応力分布を計算する。
            dist_mesh: 遠位メッシュ (pyvista.PolyData)
                接触相手の面。剛体として扱う。
            boundary_mode: 境界条件モード
                "auto" — 接触領域外の縁を自動固定
                "rim"  — メッシュの縁（境界辺）のみ固定
            max_nodes: 最大節点数（超える場合はデシメーション）
            
        Returns:
            FEMResults: 解析結果
        """
        t_start = time.time()
        
        self._log("=" * 60)
        self._log("FEM 接触解析開始")
        self._log("=" * 60)
        
        # ----------------------------------------------------------
        # Step 1: メッシュ前処理
        # ----------------------------------------------------------
        self._log("\n[Step 1] メッシュ前処理...")
        prox_tri = self._ensure_triangulated(prox_mesh)
        
        # 大規模メッシュのデシメーション
        if prox_tri.n_points > max_nodes:
            ratio = max_nodes / prox_tri.n_points
            self._log(f"  デシメーション: {prox_tri.n_points:,} → ~{max_nodes:,} 節点 (ratio={ratio:.3f})")
            prox_tri = prox_tri.decimate(1.0 - ratio)
            prox_tri = self._ensure_triangulated(prox_tri)
        
        # 内部データ構造に変換
        self._nodes = np.array(prox_tri.points, dtype=np.float64)
        self._elements = self._extract_triangles(prox_tri)
        self._n_nodes = self._nodes.shape[0]
        self._n_elements = self._elements.shape[0]
        self._n_dof = self._n_nodes * 3
        
        self._log(f"  節点数: {self._n_nodes:,}")
        self._log(f"  要素数: {self._n_elements:,}")
        self._log(f"  自由度数: {self._n_dof:,}")
        
        # 節点法線の計算
        self._normals = self._compute_vertex_normals()
        self._log(f"  節点法線計算完了")
        
        # ----------------------------------------------------------
        # Step 2: 接触検出
        # ----------------------------------------------------------
        self._log("\n[Step 2] 接触検出...")
        penetration, contact_normals, contact_mask = self._detect_contact(
            prox_tri, dist_mesh
        )
        n_contact = np.sum(contact_mask)
        self._log(f"  接触節点数: {n_contact:,} / {self._n_nodes:,}")
        
        if n_contact == 0:
            self._log("  接触なし — 解析をスキップ")
            results = FEMResults(
                contact_pressure=np.zeros(self._n_nodes),
                von_mises_stress=np.zeros(self._n_nodes),
                max_principal_stress=np.zeros(self._n_nodes),
                min_principal_stress=np.zeros(self._n_nodes),
                max_principal_strain=np.zeros(self._n_nodes),
                displacement=np.zeros((self._n_nodes, 3)),
                displacement_magnitude=np.zeros(self._n_nodes),
                penetration_depth=penetration,
                n_nodes=self._n_nodes,
                n_elements=self._n_elements,
                n_contact_nodes=0,
                solve_time_sec=time.time() - t_start,
            )
            self._log(results.summary())
            return results
        
        max_pen = np.max(penetration[contact_mask])
        mean_pen = np.mean(penetration[contact_mask])
        self._log(f"  最大侵入量: {max_pen:.4f} mm")
        self._log(f"  平均侵入量: {mean_pen:.4f} mm")
        
        # ----------------------------------------------------------
        # Step 3: 全体剛性行列の組み立て
        # ----------------------------------------------------------
        self._log("\n[Step 3] 全体剛性行列の組み立て...")
        K = self._assemble_global_stiffness()
        self._log(f"  剛性行列サイズ: {K.shape[0]:,} × {K.shape[1]:,}")
        self._log(f"  非ゼロ要素数: {K.nnz:,}")
        
        # ----------------------------------------------------------
        # Step 4: 接触力ベクトルの計算
        # ----------------------------------------------------------
        self._log("\n[Step 4] 接触力ベクトルの計算...")
        F_contact, K_contact = self._compute_contact_forces(
            penetration, contact_normals, contact_mask
        )
        
        # 接触剛性を全体剛性行列に加算
        K_total = K + K_contact
        
        total_force = np.sqrt(np.sum(F_contact.reshape(-1, 3) ** 2, axis=1)).sum()
        self._log(f"  全接触力: {total_force:.2f} N")
        
        # ----------------------------------------------------------
        # Step 5: 境界条件の適用
        # ----------------------------------------------------------
        self._log("\n[Step 5] 境界条件の適用...")
        K_bc, F_bc, fixed_dofs = self._apply_boundary_conditions(
            K_total, F_contact, contact_mask, boundary_mode
        )
        self._log(f"  固定自由度数: {len(fixed_dofs):,}")
        
        # ----------------------------------------------------------
        # Step 6: 連立方程式の求解
        # ----------------------------------------------------------
        self._log("\n[Step 6] 連立方程式の求解...")
        t_solve_start = time.time()
        
        try:
            u = spsolve(K_bc.tocsc(), F_bc)
            if np.any(np.isnan(u)):
                self._log("  警告: NaN検出 — 正則化を適用")
                # 対角正則化
                reg = sparse.eye(K_bc.shape[0]) * (self.material.E * 1e-6)
                u = spsolve((K_bc + reg).tocsc(), F_bc)
        except Exception as e:
            self._log(f"  求解失敗: {e}")
            self._log("  正則化付きで再試行...")
            reg = sparse.eye(K_bc.shape[0]) * (self.material.E * 1e-4)
            u = spsolve((K_bc + reg).tocsc(), F_bc)
        
        t_solve = time.time() - t_solve_start
        self._log(f"  求解時間: {t_solve:.2f} 秒")
        
        # 固定DOFの変位をゼロに
        u_full = u.copy()
        u_full[list(fixed_dofs)] = 0.0
        
        displacement = u_full.reshape(-1, 3)
        disp_mag = np.linalg.norm(displacement, axis=1)
        self._log(f"  最大変位: {np.max(disp_mag):.4f} mm")
        
        # ----------------------------------------------------------
        # Step 7: 応力・ひずみの後処理
        # ----------------------------------------------------------
        self._log("\n[Step 7] 応力・ひずみの後処理...")
        (von_mises, sigma_max, sigma_min,
         eps_max, elem_stress) = self._compute_stress_strain(u_full)
        self._log(f"  最大von Mises応力: {np.max(von_mises):.4f} MPa")
        
        # 接触圧の計算（法線方向の接触力 / ノード面積）
        contact_pressure = self._compute_contact_pressure(
            penetration, contact_mask
        )
        
        # 接触面積の計算
        contact_area = self._compute_contact_area(contact_mask)
        self._log(f"  接触面積: {contact_area:.2f} mm²")
        
        # ----------------------------------------------------------
        # 結果の整理
        # ----------------------------------------------------------
        results = FEMResults(
            contact_pressure=contact_pressure,
            von_mises_stress=von_mises,
            max_principal_stress=sigma_max,
            min_principal_stress=sigma_min,
            max_principal_strain=eps_max,
            displacement=displacement,
            displacement_magnitude=disp_mag,
            penetration_depth=penetration,
            contact_area=contact_area,
            total_contact_force=total_force,
            peak_contact_pressure=np.max(contact_pressure),
            n_nodes=self._n_nodes,
            n_elements=self._n_elements,
            n_contact_nodes=int(n_contact),
            solve_time_sec=time.time() - t_start,
        )
        
        self._log("\n" + results.summary())
        return results
    
    # ----------------------------------------------------------------
    # メッシュ前処理
    # ----------------------------------------------------------------
    
    def _ensure_triangulated(self, mesh) -> 'pv.PolyData':
        """メッシュが三角形のみで構成されていることを確認"""
        if not mesh.is_all_triangles:
            return mesh.triangulate()
        return mesh
    
    def _extract_triangles(self, mesh) -> np.ndarray:
        """PyVistaメッシュから三角形接続配列を抽出
        
        Returns:
            (M, 3) 配列。各行が三角形の3頂点インデックス。
        """
        faces = mesh.faces
        # PyVista faces format: [n, v0, v1, v2, n, v0, v1, v2, ...]
        # 三角形の場合 n=3
        n_faces = mesh.n_cells
        triangles = np.zeros((n_faces, 3), dtype=np.int64)
        
        idx = 0
        for i in range(n_faces):
            n_verts = faces[idx]
            if n_verts != 3:
                raise ValueError(f"要素 {i} が三角形ではありません (頂点数={n_verts})")
            triangles[i] = faces[idx + 1: idx + 4]
            idx += n_verts + 1
        
        return triangles
    
    def _compute_vertex_normals(self) -> np.ndarray:
        """面積加重平均による節点法線の計算
        
        Returns:
            (N, 3) 単位法線ベクトル配列
        """
        normals = np.zeros_like(self._nodes)
        
        for elem in self._elements:
            v0, v1, v2 = self._nodes[elem[0]], self._nodes[elem[1]], self._nodes[elem[2]]
            edge1 = v1 - v0
            edge2 = v2 - v0
            face_normal = np.cross(edge1, edge2)  # 面積×2 の大きさ
            # 面積加重で各頂点に加算
            normals[elem[0]] += face_normal
            normals[elem[1]] += face_normal
            normals[elem[2]] += face_normal
        
        # 正規化
        norms = np.linalg.norm(normals, axis=1, keepdims=True)
        norms[norms < 1e-12] = 1.0  # ゼロ除算防止
        normals = normals / norms
        
        return normals
    
    # ----------------------------------------------------------------
    # 接触検出
    # ----------------------------------------------------------------
    
    def _detect_contact(self, prox_mesh, dist_mesh
                        ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """KD-Treeによる接触検出と符号付き侵入量の計算
        
        Args:
            prox_mesh: 近位メッシュ（解析対象）
            dist_mesh: 遠位メッシュ（接触相手、剛体）
            
        Returns:
            penetration: (N,) 侵入量 [mm]（正=侵入、負=離間）
            contact_normals: (N, 3) 接触法線
            contact_mask: (N,) bool 接触フラグ
        """
        prox_pts = self._nodes
        dist_pts = np.array(dist_mesh.points, dtype=np.float64)
        
        # KD-Treeで最近傍距離を計算
        tree = cKDTree(dist_pts)
        distances, closest_idx = tree.query(prox_pts, k=1)
        
        # 符号判定: 法線と最近傍ベクトルの内積
        # 内積 > 0 → 遠位は近位の外側（非侵入）
        # 内積 < 0 → 遠位は近位の内側（侵入）
        closest_pts = dist_pts[closest_idx]
        to_closest = closest_pts - prox_pts
        
        dot_products = np.sum(to_closest * self._normals, axis=1)
        
        # 符号付き侵入量（正=侵入）
        signed_distance = distances.copy()
        signed_distance[dot_products < 0] *= -1.0  # 内側にある場合は負距離
        
        # 侵入量 = -符号付き距離（侵入が正）
        penetration = np.zeros(self._n_nodes)
        penetration[signed_distance < 0] = np.abs(signed_distance[signed_distance < 0])
        
        # 接触マスク（侵入 or 接触公差以内）
        contact_mask = (penetration > 0) | (distances < self.contact.contact_tolerance)
        
        # 公差内だが非侵入の点にも微小侵入量を設定（スムーズな接触遷移）
        near_contact = (distances < self.contact.contact_tolerance) & (penetration == 0)
        # 公差内の点には線形減衰の接触圧を付与
        if np.any(near_contact):
            t = self.contact.contact_tolerance
            penetration[near_contact] = (t - distances[near_contact]) * 0.1
        
        # 接触法線（遠位面の法線 → 近位法線で代用）
        contact_normals = self._normals.copy()
        
        return penetration, contact_normals, contact_mask
    
    # ----------------------------------------------------------------
    # CST膜要素の剛性行列
    # ----------------------------------------------------------------
    
    def _element_local_system(self, elem_idx: int
                              ) -> Tuple[np.ndarray, np.ndarray, float]:
        """三角形要素のローカル座標系と面積を計算
        
        Args:
            elem_idx: 要素インデックス
            
        Returns:
            local_coords: (3, 2) ローカル座標 [(x1,y1), (x2,y2), (x3,y3)]
            T: (3, 2) ローカル→グローバル変換行列 [e1, e2]
            area: 三角形面積
        """
        n0, n1, n2 = self._elements[elem_idx]
        p0, p1, p2 = self._nodes[n0], self._nodes[n1], self._nodes[n2]
        
        # ローカル座標系
        e1 = p1 - p0
        L01 = np.linalg.norm(e1)
        if L01 < 1e-12:
            return np.zeros((3, 2)), np.zeros((3, 2)), 0.0
        e1 = e1 / L01
        
        v02 = p2 - p0
        e3 = np.cross(e1, v02)
        area2 = np.linalg.norm(e3)
        if area2 < 1e-12:
            return np.zeros((3, 2)), np.zeros((3, 2)), 0.0
        e3 = e3 / area2
        
        e2 = np.cross(e3, e1)
        
        area = area2 / 2.0
        
        # ローカル座標に変換
        local_coords = np.zeros((3, 2))
        # node 0: origin → (0, 0)
        local_coords[1, 0] = L01                        # (L01, 0)
        local_coords[2, 0] = np.dot(v02, e1)            # (dot, ...)
        local_coords[2, 1] = np.dot(v02, e2)            # (..., dot)
        
        # 変換行列 T: ローカル2D → グローバル3D
        T = np.column_stack([e1, e2])  # (3, 2)
        
        return local_coords, T, area
    
    def _element_stiffness_local(self, local_coords: np.ndarray,
                                  area: float) -> np.ndarray:
        """CST膜要素のローカル剛性行列 (6×6)
        
        CST (Constant Strain Triangle) 平面応力要素
        
        Args:
            local_coords: (3, 2) ローカル座標
            area: 三角形面積
            
        Returns:
            Ke_local: (6, 6) ローカル剛性行列
        """
        if area < 1e-12:
            return np.zeros((6, 6))
        
        x1, y1 = local_coords[0]
        x2, y2 = local_coords[1]
        x3, y3 = local_coords[2]
        
        # B行列 (3×6): ε = B * u
        # B = (1/2A) * [y2-y3  0     y3-y1  0     y1-y2  0    ]
        #              [0      x3-x2 0      x1-x3 0      x2-x1]
        #              [x3-x2  y2-y3 x1-x3  y3-y1 x2-x1  y1-y2]
        inv2A = 1.0 / (2.0 * area)
        
        b1 = y2 - y3
        b2 = y3 - y1
        b3 = y1 - y2
        c1 = x3 - x2
        c2 = x1 - x3
        c3 = x2 - x1
        
        B = inv2A * np.array([
            [b1,  0,  b2,  0,  b3,  0 ],
            [0,   c1, 0,   c2, 0,   c3],
            [c1,  b1, c2,  b2, c3,  b3],
        ])
        
        # 構成マトリクス
        D = self.material.plane_stress_matrix()
        t = self.material.thickness
        
        # Ke = t * A * B^T * D * B
        Ke = t * area * (B.T @ D @ B)
        
        return Ke
    
    def _element_stiffness_global(self, elem_idx: int) -> Tuple[np.ndarray, list]:
        """要素剛性行列をグローバル座標系で計算 (9×9)
        
        ローカル2D (6 DOF) → グローバル3D (9 DOF) の変換を行う。
        
        Args:
            elem_idx: 要素インデックス
            
        Returns:
            Ke_global: (9, 9) グローバル剛性行列
            dof_indices: 対応する全体自由度インデックス (9,)
        """
        local_coords, T, area = self._element_local_system(elem_idx)
        
        if area < 1e-12:
            nodes = self._elements[elem_idx]
            dofs = []
            for n in nodes:
                dofs.extend([3 * n, 3 * n + 1, 3 * n + 2])
            return np.zeros((9, 9)), dofs
        
        # ローカル剛性行列 (6×6)
        Ke_local = self._element_stiffness_local(local_coords, area)
        
        # 変換行列: ローカル2D DOF → グローバル3D DOF
        # 各ノードについて u_local(2) = T^T * u_global(3)
        # Te = block_diag(T^T, T^T, T^T)  → (6×9)
        TT = T.T  # (2, 3)
        Te = np.zeros((6, 9))
        Te[0:2, 0:3] = TT
        Te[2:4, 3:6] = TT
        Te[4:6, 6:9] = TT
        
        # Ke_global = Te^T * Ke_local * Te  → (9×9)
        Ke_global = Te.T @ Ke_local @ Te
        
        # 全体自由度インデックス
        nodes = self._elements[elem_idx]
        dofs = []
        for n in nodes:
            dofs.extend([3 * n, 3 * n + 1, 3 * n + 2])
        
        return Ke_global, dofs
    
    # ----------------------------------------------------------------
    # 全体剛性行列の組み立て
    # ----------------------------------------------------------------
    
    def _assemble_global_stiffness(self) -> sparse.csr_matrix:
        """全要素の剛性行列を組み立ててスパース行列として返す
        
        COO形式で要素を蓄積し、最後にCSR形式に変換する。
        """
        # 非ゼロ要素の推定数（各要素 9×9 = 81 エントリ）
        nnz_estimate = self._n_elements * 81
        rows = np.zeros(nnz_estimate, dtype=np.int64)
        cols = np.zeros(nnz_estimate, dtype=np.int64)
        vals = np.zeros(nnz_estimate, dtype=np.float64)
        
        ptr = 0
        for e in range(self._n_elements):
            Ke, dofs = self._element_stiffness_global(e)
            
            n_local = len(dofs)
            for i in range(n_local):
                for j in range(n_local):
                    if abs(Ke[i, j]) > 1e-20:
                        rows[ptr] = dofs[i]
                        cols[ptr] = dofs[j]
                        vals[ptr] = Ke[i, j]
                        ptr += 1
            
            # 進捗表示（1000要素ごと）
            if self.verbose and (e + 1) % 2000 == 0:
                self._log(f"  組み立て進捗: {e + 1:,} / {self._n_elements:,} 要素")
        
        # 使用された分だけトリミング
        rows = rows[:ptr]
        cols = cols[:ptr]
        vals = vals[:ptr]
        
        K = sparse.coo_matrix(
            (vals, (rows, cols)),
            shape=(self._n_dof, self._n_dof)
        ).tocsr()
        
        return K
    
    # ----------------------------------------------------------------
    # 接触力の計算
    # ----------------------------------------------------------------
    
    def _compute_contact_forces(self, penetration: np.ndarray,
                                 contact_normals: np.ndarray,
                                 contact_mask: np.ndarray
                                 ) -> Tuple[np.ndarray, sparse.csr_matrix]:
        """ペナルティ法による接触力ベクトルと接触剛性行列の計算
        
        F_contact = k_p * δ * n  （ペナルティ力）
        K_contact = k_p * n ⊗ n  （接触剛性、対角ブロック）
        
        Args:
            penetration: (N,) 侵入量
            contact_normals: (N, 3) 接触法線
            contact_mask: (N,) bool
            
        Returns:
            F: (3N,) 接触力ベクトル
            K_contact: (3N, 3N) 接触剛性行列（スパース）
        """
        k_p = self.contact.penalty_stiffness
        F = np.zeros(self._n_dof)
        
        rows_list = []
        cols_list = []
        vals_list = []
        
        contact_indices = np.where(contact_mask)[0]
        
        for idx in contact_indices:
            delta = penetration[idx]
            if delta <= 0:
                continue
            
            n = contact_normals[idx]
            n_unit = n / (np.linalg.norm(n) + 1e-12)
            
            # 接触力（法線方向、侵入を押し戻す向き）
            f_contact = k_p * delta * n_unit
            
            dof_start = 3 * idx
            F[dof_start:dof_start + 3] += f_contact
            
            # 接触剛性（n ⊗ n のブロック）
            nn = np.outer(n_unit, n_unit)
            for i in range(3):
                for j in range(3):
                    if abs(nn[i, j]) > 1e-20:
                        rows_list.append(dof_start + i)
                        cols_list.append(dof_start + j)
                        vals_list.append(k_p * nn[i, j])
        
        K_contact = sparse.coo_matrix(
            (vals_list, (rows_list, cols_list)),
            shape=(self._n_dof, self._n_dof)
        ).tocsr()
        
        return F, K_contact
    
    # ----------------------------------------------------------------
    # 境界条件
    # ----------------------------------------------------------------
    
    def _apply_boundary_conditions(self, K: sparse.csr_matrix,
                                    F: np.ndarray,
                                    contact_mask: np.ndarray,
                                    mode: str
                                    ) -> Tuple[sparse.csr_matrix, np.ndarray, set]:
        """境界条件の適用（ペナルティ法 — 大数法）
        
        固定DOFに対して対角に大きな値を加えることで
        実質的にゼロ変位を強制する。
        
        Args:
            K: 全体剛性行列
            F: 荷重ベクトル
            contact_mask: 接触ノードのマスク
            mode: "auto" or "rim"
            
        Returns:
            K_bc: 境界条件適用後の剛性行列
            F_bc: 境界条件適用後の荷重ベクトル
            fixed_dofs: 固定された自由度のセット
        """
        fixed_dofs = set()
        
        if mode == "rim":
            # メッシュ境界辺上の節点を固定
            boundary_nodes = self._find_boundary_nodes()
            for n in boundary_nodes:
                fixed_dofs.update([3 * n, 3 * n + 1, 3 * n + 2])
        else:
            # "auto": 接触領域から離れた節点を固定
            # 接触節点から一定距離以上離れた節点
            contact_nodes = np.where(contact_mask)[0]
            if len(contact_nodes) > 0:
                contact_center = np.mean(self._nodes[contact_nodes], axis=0)
                contact_radius = np.max(np.linalg.norm(
                    self._nodes[contact_nodes] - contact_center, axis=1
                ))
                
                # 接触領域の2倍以上離れた節点を固定
                threshold = max(contact_radius * 2.5, 10.0)  # 最低10mm
                all_distances = np.linalg.norm(
                    self._nodes - contact_center, axis=1
                )
                far_nodes = np.where(all_distances > threshold)[0]
                
                for n in far_nodes:
                    fixed_dofs.update([3 * n, 3 * n + 1, 3 * n + 2])
            
            # 境界ノードも追加固定
            boundary_nodes = self._find_boundary_nodes()
            for n in boundary_nodes:
                fixed_dofs.update([3 * n, 3 * n + 1, 3 * n + 2])
        
        # 剛体移動防止: 固定DOFが少なすぎる場合、最低6 DOF確保
        if len(fixed_dofs) < 6:
            # 最も遠い3点を固定
            center = np.mean(self._nodes, axis=0)
            dists = np.linalg.norm(self._nodes - center, axis=1)
            far3 = np.argsort(dists)[-3:]
            for n in far3:
                fixed_dofs.update([3 * n, 3 * n + 1, 3 * n + 2])
        
        # 大数法で境界条件を適用
        K_bc = K.copy().tolil()
        F_bc = F.copy()
        big_number = self.material.E * 1e8
        
        for dof in fixed_dofs:
            K_bc[dof, dof] += big_number
            F_bc[dof] = 0.0
        
        return K_bc.tocsr(), F_bc, fixed_dofs
    
    def _find_boundary_nodes(self) -> set:
        """メッシュ境界辺（1つの三角形にのみ属する辺）上の節点を検出"""
        edge_count = {}
        
        for elem in self._elements:
            edges = [
                (min(elem[0], elem[1]), max(elem[0], elem[1])),
                (min(elem[1], elem[2]), max(elem[1], elem[2])),
                (min(elem[0], elem[2]), max(elem[0], elem[2])),
            ]
            for e in edges:
                edge_count[e] = edge_count.get(e, 0) + 1
        
        boundary_nodes = set()
        for (n0, n1), count in edge_count.items():
            if count == 1:
                boundary_nodes.add(n0)
                boundary_nodes.add(n1)
        
        return boundary_nodes
    
    # ----------------------------------------------------------------
    # 応力・ひずみの後処理
    # ----------------------------------------------------------------
    
    def _compute_stress_strain(self, u: np.ndarray
                                ) -> Tuple[np.ndarray, np.ndarray,
                                           np.ndarray, np.ndarray, list]:
        """要素ごとの応力・ひずみを計算し、節点値に平均化
        
        Args:
            u: (3N,) 全体変位ベクトル
            
        Returns:
            von_mises: (N,) von Mises応力
            sigma_max: (N,) 最大主応力
            sigma_min: (N,) 最小主応力
            eps_max: (N,) 最大主ひずみ
            elem_stresses: 要素応力のリスト（デバッグ用）
        """
        D = self.material.plane_stress_matrix()
        
        # 節点への寄与を蓄積
        node_von_mises = np.zeros(self._n_nodes)
        node_sigma_max = np.zeros(self._n_nodes)
        node_sigma_min = np.zeros(self._n_nodes)
        node_eps_max = np.zeros(self._n_nodes)
        node_count = np.zeros(self._n_nodes)
        
        elem_stresses = []
        
        for e in range(self._n_elements):
            local_coords, T, area = self._element_local_system(e)
            
            if area < 1e-12:
                elem_stresses.append(np.zeros(3))
                continue
            
            # ローカル変位を取得
            nodes = self._elements[e]
            TT = T.T  # (2, 3)
            
            u_local = np.zeros(6)
            for i, n in enumerate(nodes):
                u_global_node = u[3 * n: 3 * n + 3]
                u_local[2 * i: 2 * i + 2] = TT @ u_global_node
            
            # B行列
            x1, y1 = local_coords[0]
            x2, y2 = local_coords[1]
            x3, y3 = local_coords[2]
            
            inv2A = 1.0 / (2.0 * area)
            B = inv2A * np.array([
                [y2 - y3,  0,       y3 - y1,  0,       y1 - y2,  0      ],
                [0,        x3 - x2, 0,        x1 - x3, 0,        x2 - x1],
                [x3 - x2,  y2 - y3, x1 - x3,  y3 - y1, x2 - x1,  y1 - y2],
            ])
            
            # ひずみ ε = B * u
            strain = B @ u_local  # [εxx, εyy, γxy]
            
            # 応力 σ = D * ε
            stress = D @ strain  # [σxx, σyy, τxy]
            elem_stresses.append(stress)
            
            # von Mises応力（平面応力）
            sxx, syy, txy = stress
            vm = np.sqrt(sxx**2 - sxx * syy + syy**2 + 3 * txy**2)
            
            # 主応力
            s_avg = (sxx + syy) / 2.0
            s_diff = np.sqrt(((sxx - syy) / 2.0)**2 + txy**2)
            s1 = s_avg + s_diff
            s2 = s_avg - s_diff
            
            # 主ひずみ
            exx, eyy, gxy = strain
            e_avg = (exx + eyy) / 2.0
            e_diff = np.sqrt(((exx - eyy) / 2.0)**2 + (gxy / 2.0)**2)
            e1 = e_avg + e_diff
            
            # 節点に分配
            for n in nodes:
                node_von_mises[n] += vm
                node_sigma_max[n] += s1
                node_sigma_min[n] += s2
                node_eps_max[n] += e1
                node_count[n] += 1.0
        
        # 平均化
        mask = node_count > 0
        node_von_mises[mask] /= node_count[mask]
        node_sigma_max[mask] /= node_count[mask]
        node_sigma_min[mask] /= node_count[mask]
        node_eps_max[mask] /= node_count[mask]
        
        return node_von_mises, node_sigma_max, node_sigma_min, node_eps_max, elem_stresses
    
    def _compute_contact_pressure(self, penetration: np.ndarray,
                                   contact_mask: np.ndarray) -> np.ndarray:
        """接触圧の計算
        
        接触圧 = ペナルティ剛性 × 侵入量
        ※ Winklerモデルと等価
        """
        pressure = np.zeros(self._n_nodes)
        pressure[contact_mask] = self.contact.penalty_stiffness * penetration[contact_mask]
        return pressure
    
    def _compute_contact_area(self, contact_mask: np.ndarray) -> float:
        """接触面積の計算（接触節点を含む要素の面積を合計）"""
        total_area = 0.0
        
        for elem in self._elements:
            # 要素の全節点が接触している場合のみカウント
            # （少なくとも1節点でもOKにする場合はanyに変更）
            if np.any(contact_mask[elem]):
                v0 = self._nodes[elem[0]]
                v1 = self._nodes[elem[1]]
                v2 = self._nodes[elem[2]]
                area = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0))
                
                # 接触節点の割合で面積を按分
                ratio = np.sum(contact_mask[elem]) / 3.0
                total_area += area * ratio
        
        return total_area
    
    # ----------------------------------------------------------------
    # ユーティリティ
    # ----------------------------------------------------------------
    
    def _log(self, message: str):
        """ログ出力"""
        if self.verbose:
            print(message)


# ============================================================================
# PyVista 統合ヘルパー
# ============================================================================

def apply_fem_results_to_mesh(mesh, results: FEMResults):
    """FEM解析結果をPyVistaメッシュのスカラーとして適用
    
    Args:
        mesh: pyvista.PolyData（解析に使用した近位メッシュ）
        results: FEMResults
        
    Returns:
        mesh: スカラー追加済みの同一メッシュ（in-place変更）
    """
    if results.n_nodes != mesh.n_points:
        print(f"警告: 節点数不一致 (FEM={results.n_nodes}, mesh={mesh.n_points})")
        print("デシメーションが適用された可能性があります。")
        return mesh
    
    mesh['contact_pressure'] = results.contact_pressure
    mesh['von_mises_stress'] = results.von_mises_stress
    mesh['max_principal_stress'] = results.max_principal_stress
    mesh['min_principal_stress'] = results.min_principal_stress
    mesh['max_principal_strain'] = results.max_principal_strain
    mesh['displacement_magnitude'] = results.displacement_magnitude
    mesh['penetration_depth'] = results.penetration_depth
    
    return mesh


def visualize_fem_results(mesh, results: FEMResults,
                          dist_mesh=None,
                          scalar_name: str = 'contact_pressure',
                          cmap: str = 'jet',
                          show_edges: bool = False,
                          window_size: tuple = (1200, 800)):
    """FEM解析結果のインタラクティブ可視化
    
    Args:
        mesh: 解析済み近位メッシュ
        results: FEMResults
        dist_mesh: 遠位メッシュ（表示用、任意）
        scalar_name: 表示するスカラー名
        cmap: カラーマップ
        show_edges: メッシュエッジの表示
        window_size: ウィンドウサイズ
    """
    import pyvista as pv
    
    # 結果をメッシュに適用
    mesh_vis = mesh.copy()
    apply_fem_results_to_mesh(mesh_vis, results)
    
    # プロッター設定
    plotter = pv.Plotter(window_size=window_size)
    plotter.set_background('white')
    
    # 利用可能なスカラー
    available = {
        'contact_pressure': ('接触圧 [MPa]', 'jet'),
        'von_mises_stress': ('von Mises応力 [MPa]', 'plasma'),
        'max_principal_stress': ('最大主応力 [MPa]', 'coolwarm'),
        'displacement_magnitude': ('変位量 [mm]', 'viridis'),
        'penetration_depth': ('侵入量 [mm]', 'hot'),
    }
    
    if scalar_name in available:
        label, default_cmap = available[scalar_name]
        if cmap == 'jet':
            cmap = default_cmap
    else:
        label = scalar_name
    
    # 近位メッシュ（FEM結果）
    plotter.add_mesh(
        mesh_vis,
        scalars=scalar_name,
        cmap=cmap,
        show_edges=show_edges,
        scalar_bar_args={'title': label, 'color': 'black'},
        opacity=1.0,
    )
    
    # 遠位メッシュ（半透明）
    if dist_mesh is not None:
        plotter.add_mesh(
            dist_mesh,
            color='#FFB6C1',
            opacity=0.3,
            show_edges=False,
        )
    
    plotter.add_axes()
    
    # サマリーテキスト
    text = (
        f"接触面積: {results.contact_area:.1f} mm²\n"
        f"最大接触圧: {results.peak_contact_pressure:.3f} MPa\n"
        f"接触節点: {results.n_contact_nodes:,}\n"
        f"解析時間: {results.solve_time_sec:.1f} 秒"
    )
    plotter.add_text(text, position='upper_right', font_size=10, color='black')
    
    plotter.show()


# ============================================================================
# スタンドアロン実行用
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("FEM Contact Solver — テスト実行")
    print("=" * 60)
    print()
    print("使用方法:")
    print("  from fem_contact_solver import FEMContactSolver, MaterialProperties")
    print("  solver = FEMContactSolver()")
    print("  results = solver.analyze(prox_mesh, dist_mesh)")
    print()
    print("デモ実行には fem_integration_example.py を参照してください。")
