# ============================================================================
# FEM Integration Example — FRS_Simulator への FEM 統合ガイド
# ============================================================================
#
# このファイルは2つのパートで構成されています:
#   Part 1: 合成メッシュを使ったスタンドアロンデモ（PyVistaなしでも動作確認可能）
#   Part 2: FRS_Simulator の _compute_distance_heatmap に FEM を統合するコード
#
# ============================================================================

import numpy as np
import sys
import os

# fem_contact_solver.py と同じディレクトリにあることを想定
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fem_contact_solver import (
    FEMContactSolver,
    MaterialProperties,
    ContactParameters,
    FEMResults,
    apply_fem_results_to_mesh,
    visualize_fem_results,
)


# ============================================================================
# Part 1: スタンドアロンデモ（合成球体メッシュ）
# ============================================================================

def create_demo_meshes():
    """デモ用の近位・遠位球体メッシュを生成
    
    股関節を模した半球メッシュのペア:
    - 近位（寛骨臼）: 半径26mmの凹面（内側を向いた半球）
    - 遠位（大腿骨頭）: 半径25mmの凸面（一部が寛骨臼に侵入）
    """
    import pyvista as pv
    
    # 近位: 寛骨臼モデル（凹面）
    prox_sphere = pv.Sphere(
        radius=26.0,
        center=(0, 0, 0),
        theta_resolution=40,
        phi_resolution=40,
    )
    # 上半球のみ抽出（z > -5mm）
    prox_mesh = prox_sphere.clip(
        normal='z', origin=(0, 0, -5), invert=False
    )
    prox_mesh = prox_mesh.extract_surface().triangulate()
    
    # 遠位: 大腿骨頭モデル（凸面、少し侵入させる）
    # 中心を少しずらして部分的に侵入させる
    dist_sphere = pv.Sphere(
        radius=25.0,
        center=(0, 0, 1.5),  # 1.5mm 上にオフセット → 部分侵入
        theta_resolution=40,
        phi_resolution=40,
    )
    dist_mesh = dist_sphere.clip(
        normal='z', origin=(0, 0, -3), invert=True
    )
    dist_mesh = dist_mesh.extract_surface().triangulate()
    
    print(f"近位メッシュ: {prox_mesh.n_points:,} 節点, {prox_mesh.n_faces:,} 面")
    print(f"遠位メッシュ: {dist_mesh.n_points:,} 節点, {dist_mesh.n_faces:,} 面")
    
    return prox_mesh, dist_mesh


def run_demo():
    """デモ実行: 合成メッシュでFEM接触解析"""
    print("=" * 60)
    print("FEM Contact Solver — デモ実行")
    print("=" * 60)
    print()
    
    # メッシュ生成
    print("[1] デモ用メッシュを生成中...")
    prox_mesh, dist_mesh = create_demo_meshes()
    
    # ソルバー設定
    print("\n[2] ソルバーを設定中...")
    material = MaterialProperties(
        E=10.0,          # 関節軟骨ヤング率 [MPa]
        nu=0.45,         # ポアソン比
        thickness=2.0,   # 軟骨厚 [mm]
    )
    contact = ContactParameters(
        penalty_stiffness=500.0,  # ペナルティ剛性 [MPa/mm]
        contact_tolerance=1.5,    # 接触判定閾値 [mm]
    )
    
    solver = FEMContactSolver(
        material=material,
        contact=contact,
        verbose=True,
    )
    
    # 解析実行
    print("\n[3] FEM接触解析を実行中...")
    results = solver.analyze(prox_mesh, dist_mesh)
    
    # 結果の可視化
    print("\n[4] 結果を可視化中...")
    try:
        import pyvista as pv
        
        # 接触圧分布を表示
        print("\n--- 接触圧分布 ---")
        visualize_fem_results(
            prox_mesh, results,
            dist_mesh=dist_mesh,
            scalar_name='contact_pressure',
        )
        
        # von Mises応力分布を表示
        print("\n--- von Mises 応力分布 ---")
        visualize_fem_results(
            prox_mesh, results,
            dist_mesh=dist_mesh,
            scalar_name='von_mises_stress',
        )
        
    except ImportError:
        print("PyVistaが利用できないため、可視化をスキップします。")
        print("数値結果は上記サマリーを参照してください。")
    
    return results


# ============================================================================
# Part 2: FRS_Simulator への統合コード
# ============================================================================
# 
# 以下のコードを FRS_Simulator_1_5.py の MainMenuGUI クラスに追加することで、
# 既存の距離ヒートマップに加えてFEM接触解析機能が使えるようになります。
#
# ■ 統合手順:
#   1. fem_contact_solver.py を FRS_Simulator_1_5.py と同じディレクトリに配置
#   2. インポート部に追加:
#      from fem_contact_solver import (FEMContactSolver, MaterialProperties, 
#                                       ContactParameters, apply_fem_results_to_mesh)
#   3. 以下の2メソッドを MainMenuGUI クラスに追加
#   4. UIにFEM実行ボタンを追加（_create_simulator_tab 内）
#
# ============================================================================

# --- ここから MainMenuGUI クラスに追加するメソッド ---

def _compute_fem_contact_analysis(self, prox_mesh, dist_mesh,
                                   prox_joint_region=None,
                                   dist_joint_region=None):
    """FEM接触解析を実行して結果を可視化
    
    既存の _compute_distance_heatmap と同じ入力を受け取り、
    幾何距離に加えてFEMベースの物理量を計算する。
    
    ■ _compute_distance_heatmap との関係:
      - distance_heatmap: 幾何学的な符号付き距離（高速、定性的）
      - fem_contact:      FEMベースの接触圧・応力（物理的、定量的）
      両方を併用することを想定。
    
    Args:
        prox_mesh: 近位メッシュ全体 (pv.PolyData)
        dist_mesh: 遠位メッシュ全体 (pv.PolyData)
        prox_joint_region: 近位関節面領域（球体抽出済み、任意）
        dist_joint_region: 遠位関節面領域（球体抽出済み、任意）
    
    Returns:
        FEMResults or None
    """
    import pyvista as pv
    from fem_contact_solver import (
        FEMContactSolver, MaterialProperties,
        ContactParameters, apply_fem_results_to_mesh,
    )
    
    # 関節面領域が指定されていればそちらを使用
    analysis_prox = prox_joint_region if prox_joint_region is not None else prox_mesh
    analysis_dist = dist_joint_region if dist_joint_region is not None else dist_mesh
    
    # 材料パラメータ（将来的にGUIから設定可能にする）
    material = MaterialProperties(
        E=10.0,          # 関節軟骨 [MPa]
        nu=0.45,
        thickness=2.0,   # [mm]
    )
    contact = ContactParameters(
        penalty_stiffness=500.0,
        contact_tolerance=2.0,
    )
    
    solver = FEMContactSolver(
        material=material,
        contact=contact,
        verbose=True,
    )
    
    try:
        results = solver.analyze(
            analysis_prox, analysis_dist,
            boundary_mode="auto",
            max_nodes=30000,
        )
        return results
    except Exception as e:
        print(f"FEM解析エラー: {e}")
        import traceback
        traceback.print_exc()
        return None


def _visualize_fem_results_in_plotter(self, plotter, prox_mesh,
                                        results, scalar_name='contact_pressure'):
    """既存のPyVistaプロッターにFEM結果を追加
    
    on_animate() や on_visualize_all() 内で、既存の可視化に
    FEM結果レイヤーを重ねて表示する場合に使用。
    
    Args:
        plotter: pv.Plotter（既存のプロッターインスタンス）
        prox_mesh: 近位メッシュ
        results: FEMResults
        scalar_name: 表示するスカラー
    """
    if results is None:
        return
    
    mesh_vis = prox_mesh.copy()
    
    if results.n_nodes == mesh_vis.n_points:
        apply_fem_results_to_mesh(mesh_vis, results)
        
        plotter.add_mesh(
            mesh_vis,
            scalars=scalar_name,
            cmap='jet',
            show_edges=False,
            scalar_bar_args={
                'title': f'FEM: {scalar_name}',
                'color': 'black',
            },
            opacity=0.9,
            name='fem_result',
        )
    else:
        print(f"警告: FEM結果の節点数({results.n_nodes})がメッシュ({mesh_vis.n_points})と不一致")


# ============================================================================
# Part 3: on_animate() への統合例（差分コード）
# ============================================================================
#
# 既存の on_animate() メソッド内、距離ヒートマップを計算している箇所
# （_compute_distance_heatmap 呼び出し付近）に以下を追加:
#
# === 追加コード（on_animate 内、ヒートマップ計算の直後に挿入）===
#
#   # FEM接触解析（オプション — チェックボックスで切替）
#   if hasattr(self, 'show_fem_analysis') and self.show_fem_analysis.get():
#       fem_results = self._compute_fem_contact_analysis(
#           prox_mesh, dist_mesh,
#           prox_joint_region=prox_joint_region,
#           dist_joint_region=dist_joint_region,
#       )
#       if fem_results is not None:
#           self._visualize_fem_results_in_plotter(
#               plotter, prox_joint_region or prox_mesh,
#               fem_results,
#               scalar_name='contact_pressure',
#           )
#
# === UIへのチェックボックス追加（__init__ 内）===
#
#   self.show_fem_analysis = tk.BooleanVar(value=False)
#
# === Simulatorタブへのウィジェット追加 ===
#
#   ttk.Checkbutton(
#       options_frame,
#       text="FEM接触解析を実行",
#       variable=self.show_fem_analysis,
#   ).grid(row=N, column=0, sticky="w")
#
# ============================================================================


# ============================================================================
# メインエントリーポイント
# ============================================================================

if __name__ == "__main__":
    results = run_demo()
