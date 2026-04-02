"""
3D Nonlinear FEM Solver for Hip Joint Biomechanics — v4 Smart Cache Release

Implements:
- 4-node tetrahedral (Tet4) elements with constant strain
- Linear elastic and Neo-Hookean hyperelastic material models
- Newton-Raphson solver with Total Lagrangian formulation
- Penalty-based contact mechanics with KDTree acceleration
- Prescribed displacement boundary conditions (bone pushes cartilage)
- Comprehensive stress/strain analysis and post-processing
- Smart caching: input-hash-based, size-limited, auto-invalidating

Changes from v3:
- Added FEM3DCacheManager with SHA-256 input hashing
- Disk cache size limit (default 2GB) with LRU eviction
- Solver version embedded in hash key → solver update auto-invalidates
- Cache hit/miss logging for full transparency
- UI-callable clear_cache() method
- Cache can be enabled/disabled per-call and globally

Author: FRS Simulator Project
"""

import hashlib
import json
import logging
import os
import pickle
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Any, Dict

import numpy as np
from scipy.sparse import coo_matrix, csr_matrix, lil_matrix
from scipy.spatial import cKDTree
from scipy.sparse.linalg import spsolve

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

# Solver version — changing this auto-invalidates ALL existing caches
_SOLVER_VERSION = "4.0.0"


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class MaterialRegion:
    """Material properties for a region of elements"""
    name: str                    # 'cartilage' or 'bone'
    E: float                     # Young's modulus [MPa]
    nu: float                    # Poisson's ratio
    element_ids: np.ndarray      # element indices
    model: str = 'neo_hookean'   # 'linear' or 'neo_hookean'

    def __post_init__(self):
        self.element_ids = np.asarray(self.element_ids, dtype=np.int32)


@dataclass
class BoundaryCondition:
    """Boundary condition specification"""
    node_ids: np.ndarray
    dof_mask: np.ndarray         # bool[3]: which DOFs are constrained (x, y, z)
    prescribed_disp: Optional[np.ndarray] = None  # None, (3,), or (n_nodes, 3)

    def __post_init__(self):
        self.node_ids = np.asarray(self.node_ids, dtype=np.int32)
        self.dof_mask = np.asarray(self.dof_mask, dtype=bool)


@dataclass
class ContactPair:
    """Contact surface pair definition"""
    master_surface_nodes: np.ndarray
    slave_surface_nodes: np.ndarray
    master_surface_mesh: Optional[Any] = None
    penalty_stiffness: float = 500.0
    contact_tolerance: float = 2.0

    def __post_init__(self):
        self.master_surface_nodes = np.asarray(self.master_surface_nodes, dtype=np.int32)
        self.slave_surface_nodes = np.asarray(self.slave_surface_nodes, dtype=np.int32)


@dataclass
class FEM3DResults:
    """Complete FEM analysis results"""
    n_nodes: int
    n_elements: int
    n_materials: int
    displacement: np.ndarray
    displacement_magnitude: np.ndarray
    stress_tensor: np.ndarray
    von_mises_stress: np.ndarray
    max_principal_stress: np.ndarray
    min_principal_stress: np.ndarray
    strain_tensor: np.ndarray
    deformation_gradient: np.ndarray
    jacobian: np.ndarray
    contact_pressure: np.ndarray
    contact_area: float
    total_contact_force: float
    peak_contact_pressure: float
    n_contact_nodes: int
    contact_node_ids: np.ndarray
    n_iterations: int
    converged: bool
    residual_norm: float
    solve_time_sec: float
    material_ids: np.ndarray
    residual_history: List[float] = field(default_factory=list)
    from_cache: bool = False  # True if result was loaded from cache


# ============================================================================
# Smart Cache Manager
# ============================================================================

class FEM3DCacheManager:
    """Hash-based smart cache for FEM3D results.

    Improvements over v2 FEMCacheManager:
    1. SHA-256 hash of ALL inputs (nodes, elements, materials, BCs, contacts, solver version)
    2. Disk size limit with LRU eviction (oldest-access files removed first)
    3. Solver version in hash → code update auto-invalidates old cache
    4. Cache metadata (creation time, input summary) stored alongside
    5. Memory LRU cache (limited entries)
    6. Full logging of hit/miss for transparency
    """

    def __init__(
        self,
        cache_dir: Optional[str] = None,
        enabled: bool = True,
        max_disk_bytes: int = 2 * 1024**3,     # 2 GB default
        max_memory_entries: int = 10,
    ):
        self.enabled = enabled
        self.max_disk_bytes = max_disk_bytes
        self.max_memory_entries = max_memory_entries

        if cache_dir is None:
            try:
                base_dir = os.path.dirname(os.path.abspath(__file__))
                self.cache_dir = os.path.join(base_dir, '.fem3d_cache')
            except Exception:
                self.cache_dir = os.path.join(os.getcwd(), '.fem3d_cache')
        else:
            self.cache_dir = cache_dir

        # Memory cache: key → (FEM3DResults, access_time)
        self._memory: Dict[str, Tuple[FEM3DResults, float]] = {}

    def compute_hash(
        self,
        nodes: np.ndarray,
        elements: np.ndarray,
        material_regions: List[MaterialRegion],
        boundary_conditions: List[BoundaryCondition],
        contact_pairs: Optional[List[ContactPair]],
        max_iter: int,
        tol: float,
        nonlinear: bool,
    ) -> str:
        """Compute SHA-256 hash of ALL solver inputs + solver version."""
        hasher = hashlib.sha256()

        # Solver version (auto-invalidates on update)
        hasher.update(f"solver_v{_SOLVER_VERSION}".encode())

        # Geometry (rounded to 3 decimals = 1μm precision, robust to noise)
        # +0.0 normalizes -0.0 → 0.0 to ensure identical byte representation
        nodes_rounded = np.round(nodes, decimals=3) + 0.0
        hasher.update(nodes_rounded.tobytes())
        hasher.update(elements.tobytes())

        # Material regions
        for mat in material_regions:
            mat_str = f"{mat.name}|{mat.E}|{mat.nu}|{mat.model}|{mat.element_ids.tobytes().hex()}"
            hasher.update(mat_str.encode())

        # Boundary conditions
        for bc in boundary_conditions:
            hasher.update(bc.node_ids.tobytes())
            hasher.update(bc.dof_mask.tobytes())
            if bc.prescribed_disp is not None:
                hasher.update(np.round(bc.prescribed_disp, decimals=6).tobytes())
            else:
                hasher.update(b"no_prescribed")

        # Contact pairs
        if contact_pairs:
            for cp in contact_pairs:
                hasher.update(cp.master_surface_nodes.tobytes())
                hasher.update(cp.slave_surface_nodes.tobytes())
                cp_str = f"{cp.penalty_stiffness}|{cp.contact_tolerance}"
                hasher.update(cp_str.encode())

        # Solver parameters
        hasher.update(f"iter{max_iter}|tol{tol}|nl{nonlinear}".encode())

        return hasher.hexdigest()

    def get(self, cache_key: str) -> Optional[FEM3DResults]:
        """Try to retrieve cached results. Returns None on miss."""
        if not self.enabled:
            return None

        # 1. Memory cache
        if cache_key in self._memory:
            results, _ = self._memory[cache_key]
            self._memory[cache_key] = (results, time.time())
            logger.info(f"  [キャッシュ] メモリヒット (key={cache_key[:12]}...)")
            return results

        # 2. Disk cache
        pkl_path = os.path.join(self.cache_dir, f"{cache_key}.pkl")
        if os.path.exists(pkl_path):
            try:
                with open(pkl_path, 'rb') as f:
                    results = pickle.load(f)
                if isinstance(results, FEM3DResults):
                    # Update access time on disk
                    os.utime(pkl_path, None)
                    # Put in memory cache
                    self._memory_put(cache_key, results)
                    logger.info(f"  [キャッシュ] ディスクヒット (key={cache_key[:12]}...)")
                    return results
                else:
                    logger.info(f"  [キャッシュ] ディスクファイルの型が不正 — 無視")
                    os.remove(pkl_path)
            except Exception as e:
                logger.info(f"  [キャッシュ] ディスク読み込みエラー（無視）: {e}")
                try:
                    os.remove(pkl_path)
                except OSError:
                    pass

        logger.info(f"  [キャッシュ] ミス (key={cache_key[:12]}...)")
        return None

    def put(self, cache_key: str, results: FEM3DResults,
            nodes: np.ndarray, elements: np.ndarray) -> None:
        """Store results in memory + disk cache."""
        if not self.enabled:
            return

        # Memory
        self._memory_put(cache_key, results)

        # Disk
        try:
            os.makedirs(self.cache_dir, exist_ok=True)

            # Save result
            pkl_path = os.path.join(self.cache_dir, f"{cache_key}.pkl")
            with open(pkl_path, 'wb') as f:
                pickle.dump(results, f, protocol=pickle.HIGHEST_PROTOCOL)

            # Save metadata (human-readable)
            meta_path = os.path.join(self.cache_dir, f"{cache_key}.meta.json")
            meta = {
                "solver_version": _SOLVER_VERSION,
                "created": time.strftime("%Y-%m-%d %H:%M:%S"),
                "n_nodes": int(results.n_nodes),
                "n_elements": int(results.n_elements),
                "converged": bool(results.converged),
                "n_iterations": int(results.n_iterations),
                "max_displacement_mm": float(np.max(results.displacement_magnitude)),
                "max_von_mises_MPa": float(np.max(results.von_mises_stress)),
                "solve_time_sec": float(results.solve_time_sec),
            }
            with open(meta_path, 'w') as f:
                json.dump(meta, f, indent=2, ensure_ascii=False)

            logger.info(f"  [キャッシュ] 保存完了 (key={cache_key[:12]}...)")

            # Enforce disk size limit
            self._enforce_disk_limit()

        except Exception as e:
            logger.info(f"  [キャッシュ] ディスク保存エラー（無視）: {e}")

    def _memory_put(self, key: str, results: FEM3DResults):
        """Add to memory cache with LRU eviction."""
        self._memory[key] = (results, time.time())
        if len(self._memory) > self.max_memory_entries:
            oldest_key = min(self._memory, key=lambda k: self._memory[k][1])
            del self._memory[oldest_key]

    def _enforce_disk_limit(self):
        """Remove oldest cache files if total disk usage exceeds limit."""
        try:
            if not os.path.exists(self.cache_dir):
                return

            pkl_files = []
            total_size = 0
            for fname in os.listdir(self.cache_dir):
                if fname.endswith('.pkl'):
                    fpath = os.path.join(self.cache_dir, fname)
                    stat = os.stat(fpath)
                    pkl_files.append((fpath, stat.st_atime, stat.st_size))
                    total_size += stat.st_size

            if total_size <= self.max_disk_bytes:
                return

            # Sort by access time (oldest first)
            pkl_files.sort(key=lambda x: x[1])

            removed = 0
            for fpath, _, fsize in pkl_files:
                if total_size <= self.max_disk_bytes:
                    break
                try:
                    os.remove(fpath)
                    # Also remove metadata
                    meta_path = fpath.replace('.pkl', '.meta.json')
                    if os.path.exists(meta_path):
                        os.remove(meta_path)
                    total_size -= fsize
                    removed += 1
                except OSError:
                    pass

            if removed > 0:
                logger.info(f"  [キャッシュ] ディスク制限超過 → {removed}件の古いキャッシュを削除")

        except Exception as e:
            logger.info(f"  [キャッシュ] ディスク制限チェックエラー: {e}")

    def clear(self):
        """Clear all caches (memory + disk)."""
        self._memory.clear()
        try:
            if os.path.exists(self.cache_dir):
                import shutil
                shutil.rmtree(self.cache_dir, ignore_errors=True)
                logger.info("[キャッシュ] 全キャッシュをクリアしました")
        except Exception as e:
            logger.info(f"[キャッシュ] クリアエラー: {e}")

    def get_stats(self) -> dict:
        """Return cache statistics."""
        stats = {
            "enabled": self.enabled,
            "memory_entries": len(self._memory),
            "max_memory_entries": self.max_memory_entries,
            "disk_files": 0,
            "disk_size_bytes": 0,
            "disk_size_mb": 0.0,
            "max_disk_mb": self.max_disk_bytes / (1024**2),
        }
        try:
            if os.path.exists(self.cache_dir):
                for fname in os.listdir(self.cache_dir):
                    if fname.endswith('.pkl'):
                        fpath = os.path.join(self.cache_dir, fname)
                        stats["disk_files"] += 1
                        stats["disk_size_bytes"] += os.path.getsize(fpath)
                stats["disk_size_mb"] = stats["disk_size_bytes"] / (1024**2)
        except Exception:
            pass
        return stats


# ============================================================================
# FEM3D Solver
# ============================================================================

class FEM3DSolver:
    """3D Nonlinear FEM Solver for hip joint biomechanics.

    The solver implements Newton-Raphson iteration with:
    - Prescribed displacement BCs via penalty method (primary load driver)
    - Penalty-based contact between cartilage surfaces
    - Neo-Hookean hyperelastic material model for large deformation
    - Total Lagrangian formulation
    - Smart input-hash-based caching (optional, enabled by default)

    Load path: Distal bone pushes distal cartilage (via prescribed displacement
    on interface nodes) toward proximal cartilage (fixed at bone interface).
    Contact penalty forces resist interpenetration of the two cartilage surfaces.
    """

    def __init__(
        self,
        verbose: bool = True,
        nonlinear: bool = True,
        cache_enabled: bool = True,
        cache_dir: Optional[str] = None,
        cache_max_disk_gb: float = 2.0,
    ):
        self.verbose = verbose
        self.nonlinear = nonlinear
        self.penalty_bc = 1e15  # Penalty for prescribed displacement BCs

        # Smart cache
        self.cache = FEM3DCacheManager(
            cache_dir=cache_dir,
            enabled=cache_enabled,
            max_disk_bytes=int(cache_max_disk_gb * 1024**3),
        )

    def _log(self, msg: str):
        if self.verbose:
            logger.info(msg)

    # ========================================================================
    # Main Entry Point
    # ========================================================================

    def analyze(
        self,
        nodes: np.ndarray,
        elements: np.ndarray,
        material_regions: List[MaterialRegion],
        boundary_conditions: List[BoundaryCondition],
        contact_pairs: Optional[List[ContactPair]] = None,
        max_nodes: int = 50000,
        max_iter: int = 20,
        tol: float = 1e-4,
        use_cache: bool = True,
    ) -> FEM3DResults:
        """Run FEM analysis with optional smart caching.

        Parameters:
            use_cache: If True (default) and cache is globally enabled,
                       check cache before computing. Set to False to
                       force a fresh computation.
        """
        nodes = np.asarray(nodes, dtype=np.float64)
        elements = np.asarray(elements, dtype=np.int32)

        if nodes.shape[0] > max_nodes:
            raise ValueError(f"Mesh too large: {nodes.shape[0]} > {max_nodes}")

        self._log("=" * 60)
        self._log(f"3D FEM解析開始 (v4 — スマートキャッシュ{'ON' if (use_cache and self.cache.enabled) else 'OFF'})")
        self._log("=" * 60)
        self._log(f"  節点数: {nodes.shape[0]:,}")
        self._log(f"  要素数: {elements.shape[0]:,}")
        self._log(f"  材料領域: {len(material_regions)}")
        self._log(f"  境界条件: {len(boundary_conditions)}")
        self._log(f"  接触ペア: {len(contact_pairs) if contact_pairs else 0}")
        self._log(f"  解析モード: {'非線形 (Neo-Hookean)' if self.nonlinear else '線形弾性'}")

        # --- Cache Lookup ---
        cache_key = None
        if use_cache and self.cache.enabled:
            cache_key = self.cache.compute_hash(
                nodes, elements, material_regions, boundary_conditions,
                contact_pairs, max_iter, tol, self.nonlinear,
            )
            cached = self.cache.get(cache_key)
            if cached is not None:
                # Return cached result with from_cache flag
                cached.from_cache = True
                self._log(f"  → キャッシュから結果を読み込みました（計算スキップ）")
                self._log(f"    元の解析時間: {cached.solve_time_sec:.2f}秒")
                self._log(f"    最大変位: {np.max(cached.displacement_magnitude):.4f} mm")
                self._log(f"    最大von Mises: {np.max(cached.von_mises_stress):.4f} MPa")
                self._log("=" * 60)
                return cached

        # --- Count prescribed displacement DOFs ---
        n_prescribed = 0
        n_nonzero_prescribed = 0
        for bc in boundary_conditions:
            for i_bc, nid in enumerate(bc.node_ids):
                for d in range(3):
                    if bc.dof_mask[d]:
                        n_prescribed += 1
                        p = self._get_prescribed(bc, i_bc, d)
                        if abs(p) > 1e-12:
                            n_nonzero_prescribed += 1

        self._log(f"  拘束DOF数: {n_prescribed}")
        self._log(f"  非ゼロ強制変位DOF: {n_nonzero_prescribed}")

        if n_nonzero_prescribed == 0 and (contact_pairs is None or len(contact_pairs) == 0):
            self._log("  警告: 外力・強制変位・接触なし → 自明解(変位=0)になる可能性")

        # --- Solve ---
        if self.nonlinear:
            results = self._solve_nonlinear(
                nodes, elements, material_regions, boundary_conditions,
                contact_pairs or [], max_iter, tol
            )
        else:
            results = self._solve_linear(
                nodes, elements, material_regions, boundary_conditions
            )

        # --- Store in Cache ---
        if cache_key is not None:
            self.cache.put(cache_key, results, nodes, elements)

        return results

    @staticmethod
    def _get_prescribed(bc: BoundaryCondition, i_bc: int, d: int) -> float:
        """Get prescribed displacement value from BC."""
        if bc.prescribed_disp is None:
            return 0.0
        elif bc.prescribed_disp.ndim == 1:
            return float(bc.prescribed_disp[d])
        else:
            return float(bc.prescribed_disp[i_bc, d])

    # ========================================================================
    # Nonlinear Solver (Newton-Raphson)
    # ========================================================================

    def _solve_nonlinear(self, nodes, elements, material_regions,
                         boundary_conditions, contact_pairs, max_iter, tol):
        t0 = time.time()
        n_nodes = nodes.shape[0]
        n_elements = elements.shape[0]
        n_dof = n_nodes * 3

        self._log("\n[Step 1] 要素データ前処理中...")
        material_ids = np.zeros(n_elements, dtype=np.int32)
        for mat_id, region in enumerate(material_regions):
            material_ids[region.element_ids] = mat_id

        ref_B, ref_vol = self._precompute_element_data(nodes, elements)
        self._log(f"  前処理完了: {n_elements}要素")

        # Initialize
        displacement = np.zeros(n_dof)
        nodes_ref = nodes.copy()
        residual_history = []
        converged = False
        residual_norm = 0.0

        self._log(f"\n[Step 2] Newton-Raphson反復開始 (最大{max_iter}回)...")

        for iteration in range(max_iter):
            t_iter = time.time()
            nodes_current = nodes_ref + displacement.reshape((n_nodes, 3))

            # Assemble internal forces and tangent stiffness
            f_int, K_T = self._assemble_system(
                nodes_ref, nodes_current, elements, material_regions,
                material_ids, ref_B, ref_vol
            )

            # Compute contact forces
            f_contact = np.zeros(n_dof)
            K_contact_rows, K_contact_cols, K_contact_data = [], [], []
            if contact_pairs:
                f_contact, K_contact_rows, K_contact_cols, K_contact_data = \
                    self._compute_contact_forces(nodes_current, contact_pairs)

            # Add contact stiffness to K_T
            if K_contact_data:
                K_c = coo_matrix(
                    (K_contact_data, (K_contact_rows, K_contact_cols)),
                    shape=(n_dof, n_dof)
                ).tocsr()
                K_T = K_T + K_c

            # Residual = contact_forces - internal_forces
            # BCs add penalty*(prescribed - u) in _apply_bc
            residual = f_contact - f_int

            # Norms for logging
            f_int_norm = np.linalg.norm(f_int)
            f_contact_norm = np.linalg.norm(f_contact)
            residual_norm_pre_bc = np.linalg.norm(residual)

            # Apply boundary conditions (penalty method)
            K_bc, residual_bc = self._apply_boundary_conditions(
                K_T, residual, boundary_conditions, displacement
            )

            # Residual norm AFTER BC application
            residual_norm = np.linalg.norm(residual_bc)
            residual_history.append(residual_norm)

            # Reference norm for convergence (include BC forces)
            ref_norm = max(residual_history[0] if iteration == 0 else residual_history[0], 1.0)
            residual_ratio = residual_norm / ref_norm

            max_disp = np.max(np.abs(displacement))

            self._log(
                f"  反復{iteration:2d}: |R|={residual_norm:.4e}, "
                f"|R|/|R0|={residual_ratio:.4e}, "
                f"|f_int|={f_int_norm:.4e}, |f_contact|={f_contact_norm:.4e}, "
                f"max|u|={max_disp:.4f}mm, "
                f"({time.time() - t_iter:.2f}s)"
            )

            # Check convergence (require at least 2 iterations)
            if iteration >= 1 and residual_ratio < tol:
                converged = True
                self._log(f"  → 収束しました！ ({iteration + 1}回反復)")
                break

            # Solve for displacement increment
            try:
                du = spsolve(K_bc.tocsr(), residual_bc)
            except Exception as e:
                self._log(f"  線形ソルバー警告: {e}")
                du = np.zeros(n_dof)

            # Check for NaN
            if np.any(np.isnan(du)):
                self._log("  警告: NaN検出 — 反復を中止")
                break

            # Update displacement
            displacement += du

        if not converged:
            self._log(f"  警告: {max_iter}回で未収束 (残差比={residual_ratio:.4e})")

        # Post-processing
        self._log("\n[Step 3] 後処理（応力・ひずみ計算）...")
        t_post = time.time()
        nodes_final = nodes_ref + displacement.reshape((n_nodes, 3))

        stress_tensor, strain_tensor, F_tensor, J_det = self._compute_stress_strain(
            nodes_ref, nodes_final, elements, material_regions, material_ids,
            ref_B, ref_vol
        )

        von_mises = self._compute_von_mises(stress_tensor)
        max_principal, min_principal = self._compute_principal_stresses(stress_tensor)

        # Contact analysis
        contact_pressure, contact_area, total_cf, peak_cp, n_contact, contact_ids = \
            self._analyze_contact(nodes_final, contact_pairs)

        disp_2d = displacement.reshape((n_nodes, 3))
        disp_mag = np.linalg.norm(disp_2d, axis=1)

        t_solve = time.time() - t0
        self._log(f"  後処理完了: {time.time() - t_post:.2f}秒")
        self._log(f"\n解析完了: 合計 {t_solve:.2f}秒")
        self._log(f"  最大変位: {np.max(disp_mag):.4f} mm")
        self._log(f"  最大von Mises: {np.max(von_mises):.4f} MPa")
        self._log(f"  接触面積: {contact_area:.2f} mm², 最大接触圧: {peak_cp:.4f} MPa")
        self._log("=" * 60)

        return FEM3DResults(
            n_nodes=n_nodes, n_elements=n_elements,
            n_materials=len(material_regions),
            displacement=disp_2d, displacement_magnitude=disp_mag,
            stress_tensor=stress_tensor, von_mises_stress=von_mises,
            max_principal_stress=max_principal, min_principal_stress=min_principal,
            strain_tensor=strain_tensor, deformation_gradient=F_tensor,
            jacobian=J_det, contact_pressure=contact_pressure,
            contact_area=contact_area, total_contact_force=total_cf,
            peak_contact_pressure=peak_cp, n_contact_nodes=n_contact,
            contact_node_ids=contact_ids,
            n_iterations=iteration + 1, converged=converged,
            residual_norm=residual_norm, solve_time_sec=t_solve,
            material_ids=material_ids, residual_history=residual_history,
            from_cache=False,
        )

    # ========================================================================
    # Linear Solver
    # ========================================================================

    def _solve_linear(self, nodes, elements, material_regions, boundary_conditions):
        t0 = time.time()
        n_nodes, n_dof = nodes.shape[0], nodes.shape[0] * 3
        n_elements = elements.shape[0]

        self._log("\n[Step 1] 線形解析: 剛性行列組立中...")
        material_ids = np.zeros(n_elements, dtype=np.int32)
        for mat_id, region in enumerate(material_regions):
            material_ids[region.element_ids] = mat_id

        ref_B, ref_vol = self._precompute_element_data(nodes, elements)

        # Assemble K using reference=current (no deformation)
        _, K = self._assemble_system(
            nodes, nodes, elements, material_regions, material_ids, ref_B, ref_vol
        )

        F = np.zeros(n_dof)
        K_bc, F_bc = self._apply_boundary_conditions(K, F, boundary_conditions, None)

        self._log("  連立方程式を解いています...")
        displacement = spsolve(K_bc.tocsr(), F_bc)

        # Post-process
        stress_tensor, strain_tensor, F_def, J = self._compute_stress_strain(
            nodes, nodes, elements, material_regions, material_ids, ref_B, ref_vol
        )
        von_mises = self._compute_von_mises(stress_tensor)
        max_p, min_p = self._compute_principal_stresses(stress_tensor)

        disp_2d = displacement.reshape((n_nodes, 3))
        disp_mag = np.linalg.norm(disp_2d, axis=1)
        t_solve = time.time() - t0
        self._log(f"  線形解析完了: {t_solve:.2f}秒")

        return FEM3DResults(
            n_nodes=n_nodes, n_elements=n_elements,
            n_materials=len(material_regions),
            displacement=disp_2d, displacement_magnitude=disp_mag,
            stress_tensor=stress_tensor, von_mises_stress=von_mises,
            max_principal_stress=max_p, min_principal_stress=min_p,
            strain_tensor=strain_tensor, deformation_gradient=F_def,
            jacobian=J,
            contact_pressure=np.zeros(n_nodes), contact_area=0.0,
            total_contact_force=0.0, peak_contact_pressure=0.0,
            n_contact_nodes=0, contact_node_ids=np.array([], dtype=np.int32),
            n_iterations=1, converged=True, residual_norm=0.0,
            solve_time_sec=t_solve, material_ids=material_ids,
            from_cache=False,
        )

    # ========================================================================
    # Element Precomputation
    # ========================================================================

    def _precompute_element_data(self, nodes, elements):
        """Pre-compute B matrices and volumes for reference configuration."""
        n_elements = elements.shape[0]
        B_list = []
        vol_list = np.zeros(n_elements)

        dN_xi = np.array([[-1.0, 1.0, 0.0, 0.0],
                          [-1.0, 0.0, 1.0, 0.0],
                          [-1.0, 0.0, 0.0, 1.0]])

        for e_id in range(n_elements):
            coords = nodes[elements[e_id]]
            J_mat = dN_xi @ coords  # (3, 3)
            det_J = np.linalg.det(J_mat)
            vol = abs(det_J) / 6.0
            vol_list[e_id] = vol

            if abs(det_J) < 1e-20:
                B_list.append(np.zeros((6, 12)))
                continue

            J_inv = np.linalg.inv(J_mat)
            dN_dx = (J_inv.T @ dN_xi).T  # (4, 3)

            B = np.zeros((6, 12))
            for i in range(4):
                B[0, 3*i]   = dN_dx[i, 0]
                B[1, 3*i+1] = dN_dx[i, 1]
                B[2, 3*i+2] = dN_dx[i, 2]
                B[3, 3*i]   = dN_dx[i, 1]; B[3, 3*i+1] = dN_dx[i, 0]
                B[4, 3*i]   = dN_dx[i, 2]; B[4, 3*i+2] = dN_dx[i, 0]
                B[5, 3*i+1] = dN_dx[i, 2]; B[5, 3*i+2] = dN_dx[i, 1]

            B_list.append(B)

        return B_list, vol_list

    # ========================================================================
    # System Assembly (Internal Forces + Tangent Stiffness)
    # ========================================================================

    def _assemble_system(self, nodes_ref, nodes_cur, elements,
                         material_regions, material_ids, ref_B, ref_vol):
        n_nodes = nodes_ref.shape[0]
        n_elements = elements.shape[0]
        n_dof = n_nodes * 3
        f_int = np.zeros(n_dof)
        rows, cols, data = [], [], []

        dN_xi = np.array([[-1.0, 1.0, 0.0, 0.0],
                          [-1.0, 0.0, 1.0, 0.0],
                          [-1.0, 0.0, 0.0, 1.0]])

        for e_id, elem in enumerate(elements):
            region = material_regions[material_ids[e_id]]
            vol = ref_vol[e_id]
            if vol < 1e-20:
                continue

            B = ref_B[e_id]
            ref_coords = nodes_ref[elem]
            cur_coords = nodes_cur[elem]
            disp_e = (cur_coords - ref_coords).flatten()  # (12,)

            if region.model == 'neo_hookean':
                # Deformation gradient
                F, J_det = self._compute_deformation_gradient(ref_coords, cur_coords, dN_xi)
                sigma, C_mat = self._compute_neo_hookean_stress(F, region.E, region.nu)

                # Internal force via B^T @ sigma_voigt
                sigma_voigt = self._tensor_to_voigt(sigma)
                f_e = vol * (B.T @ sigma_voigt)  # (12,)
            else:
                # Linear elastic
                D = self._compute_D_matrix(region.E, region.nu)
                C_mat = D
                strain_voigt = B @ disp_e
                stress_voigt = D @ strain_voigt
                f_e = vol * (B.T @ stress_voigt)

            # Scatter internal forces
            for i in range(4):
                gid = elem[i]
                f_int[gid*3:gid*3+3] += f_e[i*3:i*3+3]

            # Element stiffness K_e = V * B^T @ C @ B
            K_e = vol * (B.T @ C_mat @ B)

            # Assemble to COO
            for i in range(4):
                for j in range(4):
                    for di in range(3):
                        for dj in range(3):
                            rows.append(elem[i] * 3 + di)
                            cols.append(elem[j] * 3 + dj)
                            data.append(K_e[i*3+di, j*3+dj])

        K = coo_matrix((data, (rows, cols)), shape=(n_dof, n_dof)).tocsr()
        return f_int, K

    # ========================================================================
    # Deformation Gradient and Material Models
    # ========================================================================

    def _compute_deformation_gradient(self, ref_coords, cur_coords, dN_xi):
        """F = dx/dX for Tet4 element using actual reference coords."""
        J_ref = dN_xi @ ref_coords
        J_ref_inv = np.linalg.inv(J_ref)
        dN_dX = (J_ref_inv.T @ dN_xi).T  # (4, 3)
        F = cur_coords.T @ dN_dX  # (3, 3)
        J = np.linalg.det(F)
        return F, J

    def _compute_neo_hookean_stress(self, F, E, nu):
        """Cauchy stress + material tangent for Neo-Hookean."""
        mu = E / (2.0 * (1.0 + nu))
        lam = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))

        J = np.linalg.det(F)
        if J < 0.01:
            J = 0.01  # Prevent collapse
        B_CG = F @ F.T  # Left Cauchy-Green tensor
        I = np.eye(3)
        sigma = (mu / J) * (B_CG - I) + (lam / J) * np.log(J) * I

        # Material tangent (simplified isotropic)
        K_bulk = lam + 2.0 * mu / 3.0
        G = mu
        c1 = K_bulk - 2.0 * G / 3.0
        c2 = 2.0 * G

        C = np.zeros((6, 6))
        C[0,0] = C[1,1] = C[2,2] = c1 + c2
        C[0,1] = C[1,0] = C[0,2] = C[2,0] = C[1,2] = C[2,1] = c1
        C[3,3] = C[4,4] = C[5,5] = G

        return sigma, C

    def _compute_D_matrix(self, E, nu):
        """Linear elastic constitutive matrix (6x6)."""
        c = E / ((1 + nu) * (1 - 2*nu))
        D = np.zeros((6, 6))
        D[0,0] = D[1,1] = D[2,2] = c * (1 - nu)
        D[0,1] = D[1,0] = D[0,2] = D[2,0] = D[1,2] = D[2,1] = c * nu
        D[3,3] = D[4,4] = D[5,5] = c * (1 - 2*nu) / 2
        return D

    # ========================================================================
    # Contact Forces
    # ========================================================================

    def _compute_contact_forces(self, nodes_current, contact_pairs):
        """Compute repulsive contact forces using penalty method."""
        n_dof = nodes_current.shape[0] * 3
        f_contact = np.zeros(n_dof)
        rows, cols, data = [], [], []
        n_detected = 0

        for pair in contact_pairs:
            slave_ids = pair.slave_surface_nodes
            master_ids = pair.master_surface_nodes
            penalty = pair.penalty_stiffness

            if len(master_ids) == 0 or len(slave_ids) == 0:
                continue

            master_pts = nodes_current[master_ids]
            tree = cKDTree(master_pts)

            for slave_id in slave_ids:
                x_s = nodes_current[slave_id]
                dist, idx = tree.query(x_s)

                if dist < pair.contact_tolerance:
                    n_detected += 1
                    master_id = master_ids[idx]
                    x_m = master_pts[idx]
                    penetration = pair.contact_tolerance - dist

                    # Normal: slave away from master (repulsive)
                    if dist > 1e-12:
                        normal = (x_s - x_m) / dist
                    else:
                        normal = np.array([0.0, 0.0, 1.0])

                    # Repulsive force
                    f_c = penalty * penetration * normal
                    f_contact[slave_id*3:slave_id*3+3] += f_c
                    f_contact[master_id*3:master_id*3+3] -= f_c

                    # Contact stiffness (diagonal)
                    k_c = penalty
                    for d in range(3):
                        rows.append(slave_id*3+d)
                        cols.append(slave_id*3+d)
                        data.append(k_c)
                        rows.append(master_id*3+d)
                        cols.append(master_id*3+d)
                        data.append(k_c)

        if self.verbose and n_detected > 0:
            logger.info(f"    接触検出: {n_detected}ノード")

        return f_contact, rows, cols, data

    # ========================================================================
    # Boundary Conditions
    # ========================================================================

    def _apply_boundary_conditions(self, K, F, boundary_conditions, displacement):
        """Apply Dirichlet BCs via penalty method.

        For NR: enforces u[dof] = prescribed by adding
        penalty * (prescribed - u_current[dof]) to the residual.
        """
        K_bc = K.tolil()
        F_bc = F.copy()

        for bc in boundary_conditions:
            for i_bc, node_id in enumerate(bc.node_ids):
                for d in range(3):
                    if bc.dof_mask[d]:
                        dof = node_id * 3 + d
                        prescribed = self._get_prescribed(bc, i_bc, d)
                        current_u = displacement[dof] if displacement is not None else 0.0

                        K_bc[dof, dof] += self.penalty_bc
                        F_bc[dof] += self.penalty_bc * (prescribed - current_u)

        return K_bc.tocsr(), F_bc

    # ========================================================================
    # Contact Post-Processing
    # ========================================================================

    def _analyze_contact(self, nodes_final, contact_pairs):
        """Analyze final contact state for results."""
        n_nodes = nodes_final.shape[0]
        contact_pressure = np.zeros(n_nodes)
        contact_area = 0.0
        total_force = 0.0
        peak_pressure = 0.0
        contact_ids = []

        for pair in contact_pairs:
            slave_ids = pair.slave_surface_nodes
            master_ids = pair.master_surface_nodes
            penalty = pair.penalty_stiffness

            if len(master_ids) == 0:
                continue

            master_pts = nodes_final[master_ids]
            tree = cKDTree(master_pts)

            for slave_id in slave_ids:
                x_s = nodes_final[slave_id]
                dist, _ = tree.query(x_s)

                if dist < pair.contact_tolerance:
                    pressure = penalty * (pair.contact_tolerance - dist)
                    contact_pressure[slave_id] = pressure
                    total_force += pressure
                    peak_pressure = max(peak_pressure, pressure)
                    contact_ids.append(slave_id)

        n_contact = len(contact_ids)
        contact_area = n_contact * 1.0  # Simplified: 1 mm² per node
        return (contact_pressure, contact_area, total_force, peak_pressure,
                n_contact, np.array(contact_ids, dtype=np.int32))

    # ========================================================================
    # Stress / Strain Post-Processing
    # ========================================================================

    def _compute_stress_strain(self, nodes_ref, nodes_cur, elements,
                               material_regions, material_ids, ref_B, ref_vol):
        n_elements = elements.shape[0]
        stress_tensor = np.zeros((n_elements, 6))
        strain_tensor = np.zeros((n_elements, 6))
        F_tensor = np.zeros((n_elements, 3, 3))
        J_det = np.zeros(n_elements)

        dN_xi = np.array([[-1.0, 1.0, 0.0, 0.0],
                          [-1.0, 0.0, 1.0, 0.0],
                          [-1.0, 0.0, 0.0, 1.0]])

        for e_id, elem in enumerate(elements):
            region = material_regions[material_ids[e_id]]
            ref_coords = nodes_ref[elem]
            cur_coords = nodes_cur[elem]
            B = ref_B[e_id]
            disp_e = (cur_coords - ref_coords).flatten()

            strain_voigt = B @ disp_e
            strain_tensor[e_id] = strain_voigt

            F, J = self._compute_deformation_gradient(ref_coords, cur_coords, dN_xi)
            F_tensor[e_id] = F
            J_det[e_id] = J

            if region.model == 'neo_hookean':
                sigma, _ = self._compute_neo_hookean_stress(F, region.E, region.nu)
                stress_tensor[e_id] = self._tensor_to_voigt(sigma)
            else:
                D = self._compute_D_matrix(region.E, region.nu)
                stress_tensor[e_id] = D @ strain_voigt

        return stress_tensor, strain_tensor, F_tensor, J_det

    def _compute_von_mises(self, stress_tensor):
        s = stress_tensor
        return np.sqrt(0.5 * (
            (s[:,0]-s[:,1])**2 + (s[:,1]-s[:,2])**2 + (s[:,2]-s[:,0])**2 +
            6.0 * (s[:,3]**2 + s[:,4]**2 + s[:,5]**2)
        ))

    def _compute_principal_stresses(self, stress_tensor):
        n = stress_tensor.shape[0]
        max_p = np.zeros(n)
        min_p = np.zeros(n)
        for i in range(n):
            s = stress_tensor[i]
            mat = np.array([
                [s[0], s[3], s[4]],
                [s[3], s[1], s[5]],
                [s[4], s[5], s[2]]
            ])
            evals = np.linalg.eigvalsh(mat)
            max_p[i] = evals[2]
            min_p[i] = evals[0]
        return max_p, min_p

    # ========================================================================
    # Utility
    # ========================================================================

    @staticmethod
    def _tensor_to_voigt(tensor):
        return np.array([
            tensor[0,0], tensor[1,1], tensor[2,2],
            tensor[0,1], tensor[0,2], tensor[1,2]
        ])

    @staticmethod
    def _voigt_to_tensor(voigt):
        return np.array([
            [voigt[0], voigt[3], voigt[4]],
            [voigt[3], voigt[1], voigt[5]],
            [voigt[4], voigt[5], voigt[2]]
        ])


# ============================================================================
# Visualization Helper
# ============================================================================

def apply_fem3d_results_to_mesh(mesh, results: FEM3DResults, element_to_point: bool = True):
    """Apply FEM3D results to a PyVista mesh for visualization."""
    n_points = mesh.n_points
    n_cells = mesh.n_cells

    # Point data
    if results.displacement_magnitude.shape[0] == n_points:
        mesh['displacement_magnitude'] = results.displacement_magnitude
    if results.contact_pressure.shape[0] == n_points:
        mesh['contact_pressure'] = results.contact_pressure

    # Cell data → point data conversion
    if results.von_mises_stress.shape[0] == n_cells:
        mesh.cell_data['von_mises_stress'] = results.von_mises_stress
        mesh.cell_data['max_principal_stress'] = results.max_principal_stress
        mesh.cell_data['min_principal_stress'] = results.min_principal_stress
        if element_to_point:
            try:
                converted = mesh.cell_data_to_point_data()
                for key in ['von_mises_stress', 'max_principal_stress', 'min_principal_stress']:
                    if key in converted.array_names:
                        mesh[key] = converted[key]
            except Exception:
                pass
    elif results.von_mises_stress.shape[0] == n_points:
        mesh['von_mises_stress'] = results.von_mises_stress
        mesh['max_principal_stress'] = results.max_principal_stress
        mesh['min_principal_stress'] = results.min_principal_stress
