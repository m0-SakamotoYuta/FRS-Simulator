"""
Bone-cartilage mesh separation and 3D tetrahedral volume mesh generation.

This module handles the separation of bone and cartilage surfaces and generates
volumetric tetrahedral meshes suitable for FEM hip joint analysis. It supports
automatic generation of meshes for both proximal (acetabular) and distal (femoral)
sides with complete setup of material regions, boundary conditions, and contact pairs.

Core functionality:
1. Surface extrusion to create volumetric tet meshes from surface triangles
2. Identification of bone-cartilage interface nodes (for fixed BCs)
3. Identification of outer contact surface nodes
4. Automatic processing of both proximal and distal cartilage layers
5. Generation of ContactPair objects for penalty-based contact
6. Complete mesh quality assessment
"""

import logging
import warnings
from typing import Dict, List, Tuple, Optional, Any

import numpy as np
import pyvista as pv
from scipy.spatial import cKDTree
from scipy.spatial.distance import cdist

try:
    import tetgen
    _HAS_TETGEN = True
except ImportError:
    _HAS_TETGEN = False
    warnings.warn('tetgen not available, will use extrusion-based volume meshing')

# Import types from FEM solver module (if available)
try:
    from fem_3d_solver import MaterialRegion, BoundaryCondition, ContactPair
    _HAS_FEM_TYPES = True
except ImportError:
    _HAS_FEM_TYPES = False


class MeshSeparator:
    """
    Handles bone-cartilage mesh separation and volumetric mesh generation.

    This class manages the complete pipeline for:
    - Separating bone and cartilage surfaces (proximal and distal)
    - Generating tetrahedral volume meshes with consistent quality
    - Identifying bone-cartilage interface and contact regions
    - Creating material regions and boundary conditions
    - Setting up contact pairs for mechanical coupling
    """

    def __init__(self, verbose: bool = True):
        """
        Initialize the MeshSeparator.

        Args:
            verbose: If True, enable detailed logging output
        """
        self.logger = logging.getLogger('MeshSeparator')
        if verbose:
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.INFO)

        # Add console handler if not already present
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s [%(name)s] %(levelname)s: %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

    def separate_and_mesh(
        self,
        prox_bone: pv.PolyData,
        prox_cartilage: pv.PolyData,
        dist_bone: pv.PolyData,
        dist_cartilage: pv.PolyData,
        cartilage_thickness: float = 2.0,
        n_layers: int = 2,
        bone_E: float = 17000.0,
        bone_nu: float = 0.3,
        cart_E: float = 10.0,
        cart_nu: float = 0.45,
        penalty_stiffness: float = 500.0,
        contact_tolerance: float = 2.0,
        dist_prescribed_disp: Optional[np.ndarray] = None,
    ) -> Dict[str, Any]:
        """
        Full pipeline: separate bone/cartilage for BOTH sides, create volume meshes,
        set up material regions, BCs, and contact pairs.

        This is the main entry point that processes proximal and distal cartilage
        simultaneously and creates a complete FEM model.

        Args:
            prox_bone: Proximal (acetabular) bone surface mesh
            prox_cartilage: Proximal cartilage surface mesh
            dist_bone: Distal (femoral) bone surface mesh
            dist_cartilage: Distal cartilage surface mesh
            cartilage_thickness: Nominal cartilage thickness in mm
            n_layers: Number of extrusion layers through thickness
            bone_E: Bone Young's modulus (MPa)
            bone_nu: Bone Poisson's ratio
            cart_E: Cartilage Young's modulus (MPa)
            cart_nu: Cartilage Poisson's ratio
            penalty_stiffness: Contact penalty stiffness
            contact_tolerance: Contact detection tolerance in mm
            dist_prescribed_disp: Optional (3,) displacement to apply to distal
                interface nodes. If None, a default push toward proximal centroid
                is computed automatically from the gap between surfaces.

        Returns:
            Dict with keys:
                'nodes': np.ndarray (N, 3) - combined nodes for both cartilages
                'elements': np.ndarray (M, 4) - combined tet connectivity
                'material_regions': List[MaterialRegion]
                'boundary_conditions': List[BoundaryCondition]
                'contact_pairs': List[ContactPair]
                'prox_nodes': np.ndarray - proximal cartilage nodes
                'prox_elements': np.ndarray - proximal cartilage elements
                'prox_interface_nodes': np.ndarray - proximal interface node IDs
                'prox_outer_nodes': np.ndarray - proximal outer surface node IDs
                'dist_nodes': np.ndarray - distal cartilage nodes
                'dist_elements': np.ndarray - distal cartilage elements
                'dist_interface_nodes': np.ndarray - distal interface node IDs
                'dist_outer_nodes': np.ndarray - distal outer surface node IDs
                'prox_volume_mesh': pv.UnstructuredGrid - proximal visualization mesh
                'dist_volume_mesh': pv.UnstructuredGrid - distal visualization mesh
                'mesh_quality': dict - quality metrics
        """
        self.logger.info('[メッシュ分離] 近位・遠位軟骨メッシュの処理を開始します')

        # Process proximal side
        self.logger.info('[メッシュ分離] 近位側の処理を開始します')
        prox_result = self.process_single_side(
            prox_bone, prox_cartilage, cartilage_thickness, n_layers
        )

        # Process distal side
        self.logger.info('[メッシュ分離] 遠位側の処理を開始します')
        dist_result = self.process_single_side(
            dist_bone, dist_cartilage, cartilage_thickness, n_layers
        )

        # Combine meshes: offset distal indices by proximal node count
        prox_n_nodes = len(prox_result['nodes'])
        dist_elements_offset = dist_result['elements'] + prox_n_nodes

        # Combine nodes and elements
        combined_nodes = np.vstack([prox_result['nodes'], dist_result['nodes']])
        combined_elements = np.vstack([
            prox_result['elements'],
            dist_elements_offset
        ])

        # Create material regions (proximal and distal)
        material_regions = self._create_material_regions_dual(
            prox_result['elements'],
            dist_elements_offset,
            bone_E, bone_nu, cart_E, cart_nu
        )

        # Create boundary conditions
        prox_iface = prox_result['interface_nodes']
        dist_iface = dist_result['interface_nodes'] + prox_n_nodes

        # Auto-compute push displacement if not provided
        # Measure the gap between proximal and distal outer surfaces and push
        # the distal cartilage toward the proximal to ensure contact
        if dist_prescribed_disp is None:
            prox_outer_pts = combined_nodes[prox_result['outer_nodes']]
            dist_outer_pts = combined_nodes[dist_result['outer_nodes'] + prox_n_nodes]
            prox_centroid = np.mean(prox_outer_pts, axis=0)
            dist_centroid = np.mean(dist_outer_pts, axis=0)
            gap_vec = prox_centroid - dist_centroid
            gap_dist = np.linalg.norm(gap_vec)
            if gap_dist > 1e-6:
                # Push distal toward proximal by gap distance + small extra for contact
                push_extra = min(contact_tolerance * 0.5, 1.0)
                dist_prescribed_disp = gap_vec + (gap_vec / gap_dist) * push_extra
                self.logger.info(
                    f'[メッシュ分離] 自動押込み変位: gap={gap_dist:.2f}mm, '
                    f'push={np.linalg.norm(dist_prescribed_disp):.2f}mm, '
                    f'direction={gap_vec / gap_dist}'
                )
            else:
                # Already overlapping
                dist_prescribed_disp = np.zeros(3)

        boundary_conditions = self._create_boundary_conditions_dual(
            prox_iface, dist_iface, combined_nodes, dist_prescribed_disp
        )

        # Create contact pair between outer surfaces
        prox_outer = prox_result['outer_nodes']
        dist_outer = dist_result['outer_nodes'] + prox_n_nodes

        contact_pairs = self._create_contact_pairs(
            prox_outer, dist_outer, penalty_stiffness, contact_tolerance
        )

        # Assess combined mesh quality
        mesh_quality = self.check_mesh_quality(combined_nodes, combined_elements)

        self.logger.info(
            f'[メッシュ分離] 統合メッシュ: {mesh_quality["n_nodes"]} 節点, '
            f'{mesh_quality["n_elements"]} 要素'
        )

        # Create visualization meshes
        prox_vol_mesh = create_visualization_mesh(prox_result['nodes'], prox_result['elements'])
        dist_vol_mesh = create_visualization_mesh(dist_result['nodes'], dist_result['elements'])

        self.logger.info('[メッシュ分離] メッシュ生成処理が完了しました')

        return {
            'nodes': combined_nodes,
            'elements': combined_elements,
            'material_regions': material_regions,
            'boundary_conditions': boundary_conditions,
            'contact_pairs': contact_pairs,
            'prox_nodes': prox_result['nodes'],
            'prox_elements': prox_result['elements'],
            'prox_interface_nodes': prox_result['interface_nodes'],
            'prox_outer_nodes': prox_result['outer_nodes'],
            'dist_nodes': dist_result['nodes'],
            'dist_elements': dist_result['elements'],
            'dist_interface_nodes': dist_result['interface_nodes'],
            'dist_outer_nodes': dist_result['outer_nodes'],
            'prox_volume_mesh': prox_vol_mesh,
            'dist_volume_mesh': dist_vol_mesh,
            'mesh_quality': mesh_quality,
        }

    def process_single_side(
        self,
        bone_mesh: pv.PolyData,
        cartilage_mesh: pv.PolyData,
        thickness: float,
        n_layers: int,
    ) -> Dict[str, Any]:
        """
        Process one side (proximal or distal) completely.

        Args:
            bone_mesh: Bone surface mesh
            cartilage_mesh: Cartilage surface mesh
            thickness: Cartilage thickness in mm
            n_layers: Number of extrusion layers

        Returns:
            Dict with 'nodes', 'elements', 'interface_nodes', 'outer_nodes'
        """
        # Validate and triangulate
        self._validate_input_meshes(bone_mesh, cartilage_mesh)

        if not bone_mesh.is_all_triangles():
            bone_mesh = bone_mesh.triangulate()
        if not cartilage_mesh.is_all_triangles():
            cartilage_mesh = cartilage_mesh.triangulate()

        # Compute normals
        cartilage_mesh = cartilage_mesh.compute_normals()

        # Generate volume mesh
        if _HAS_TETGEN:
            self.logger.debug('TetGenを使用したメッシュ生成')
            nodes, elements, inner_ids, outer_ids = self._generate_volume_mesh_tetgen(
                cartilage_mesh
            )
        else:
            self.logger.debug('押出法を使用したメッシュ生成')
            nodes, elements, inner_ids, outer_ids = self.extrude_surface_to_volume(
                cartilage_mesh, thickness, n_layers, direction='inward'
            )

        # Remove degenerate elements
        nodes, elements = self._remove_degenerate_elements(nodes, elements)

        # Identify interface nodes
        interface_nodes = self.identify_interface_nodes(
            nodes, inner_ids, bone_mesh, tolerance=1.0
        )

        # Identify outer surface nodes
        outer_nodes = self.identify_contact_surface(nodes, outer_ids)

        return {
            'nodes': nodes,
            'elements': elements,
            'interface_nodes': interface_nodes,
            'outer_nodes': outer_nodes,
        }

    def extrude_surface_to_volume(
        self,
        surface: pv.PolyData,
        thickness: float,
        n_layers: int = 2,
        direction: str = 'inward'
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Extrude triangulated surface to create volumetric tet mesh.

        Algorithm:
        1. Compute surface normals (per vertex)
        2. For n_layers, create offset surfaces at thickness/n_layers intervals
        3. Between consecutive surfaces, create prisms from triangle pairs
        4. Split each prism into 3 tets consistently

        For each layer between surface[i] and surface[i+1]:
          For each triangle (a, b, c) on surface[i] -> (a', b', c') on surface[i+1]:
            Prism vertices: bottom=(a,b,c), top=(a',b',c')
            Tet1: (a, b, c, a')
            Tet2: (b, c, a', b')
            Tet3: (c, a', b', c')

        Args:
            surface: Triangulated PyVista PolyData with normals
            thickness: Total extrusion thickness in mm
            n_layers: Number of layers through thickness
            direction: 'inward' (along -normal) or 'outward' (along +normal)

        Returns:
            Tuple of:
                nodes: (N, 3) node coordinates
                elements: (M, 4) tet connectivity
                inner_surface_node_ids: nodes at innermost surface (bone interface)
                outer_surface_node_ids: nodes at outermost surface (contact)
        """
        if not surface.is_all_triangles():
            surface = surface.triangulate()

        if 'Normals' not in surface.array_names:
            surface = surface.compute_normals(point_normals=True, cell_normals=False)

        normals = surface['Normals']
        outer_points = np.array(surface.points)
        n_outer = len(outer_points)

        self.logger.debug(
            f'[メッシュ分離] 押出メッシュ: {n_outer} 外側ポイント, 厚さ {thickness}mm, '
            f'{n_layers} レイヤー, 方向={direction}'
        )

        # Determine extrusion direction
        if direction == 'inward':
            direction_sign = -1.0
        elif direction == 'outward':
            direction_sign = 1.0
        else:
            raise ValueError('direction must be "inward" or "outward"')

        layer_thickness = thickness / n_layers

        # Build all node layers
        all_nodes = [outer_points]
        for layer in range(1, n_layers + 1):
            offset_dist = layer_thickness * layer * direction_sign
            offset_points = outer_points + normals * offset_dist
            all_nodes.append(offset_points)

        # Stack all layers
        nodes = np.vstack(all_nodes)

        # Get surface connectivity
        faces = surface.faces.reshape(-1, 4)[:, 1:4]  # Remove face size marker
        n_triangles = len(faces)

        all_tets = []

        # For each layer, create tets between this and next layer
        for layer in range(n_layers):
            layer_offset = layer * n_outer
            next_layer_offset = (layer + 1) * n_outer

            for i0, i1, i2 in faces:
                # Bottom triangle (current layer)
                b0, b1, b2 = (i0 + layer_offset,
                             i1 + layer_offset,
                             i2 + layer_offset)

                # Top triangle (next layer)
                t0, t1, t2 = (i0 + next_layer_offset,
                             i1 + next_layer_offset,
                             i2 + next_layer_offset)

                # Split prism into 3 consistent tetrahedra
                tet1 = [b0, b1, b2, t0]
                tet2 = [b1, b2, t0, t1]
                tet3 = [b2, t0, t1, t2]

                all_tets.extend([tet1, tet2, tet3])

        elements = np.array(all_tets, dtype=np.int32)

        # Outer surface: first layer nodes (indices 0 to n_outer-1)
        outer_surface_ids = np.arange(n_outer, dtype=np.int32)

        # Inner surface: last layer nodes (indices (n_layers)*n_outer to (n_layers+1)*n_outer-1)
        inner_layer_offset = n_layers * n_outer
        inner_surface_ids = np.arange(inner_layer_offset, inner_layer_offset + n_outer, dtype=np.int32)

        self.logger.info(
            f'[メッシュ分離] 押出メッシュ: {nodes.shape[0]} 節点, '
            f'{elements.shape[0]} 四面体要素'
        )

        return nodes, elements, inner_surface_ids, outer_surface_ids

    def _generate_volume_mesh_tetgen(
        self,
        surface_mesh: pv.PolyData
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate volumetric mesh using TetGen if available.

        Args:
            surface_mesh: Triangulated surface mesh

        Returns:
            Tuple of (nodes, elements, inner_ids, outer_ids)
        """
        try:
            tet = tetgen.TetGen(surface_mesh)
            tet.tetrahedralize(order=1, mindihedral=20, minratio=1.5)
            grid = tet.grid

            nodes = grid.points
            cells = grid.cells.reshape(-1, 5)[:, 1:5]

            # Approximate inner/outer surfaces (all nodes in outer)
            outer_ids = np.arange(len(surface_mesh.points), dtype=np.int32)
            inner_ids = np.arange(len(surface_mesh.points), dtype=np.int32)

            self.logger.info(f'[メッシュ分離] TetGen: {nodes.shape[0]} 節点, {cells.shape[0]} 要素')

            return nodes, cells, inner_ids, outer_ids

        except Exception as e:
            self.logger.error(f'[メッシュ分離] TetGen エラー: {str(e)}')
            raise RuntimeError('TetGen meshing failed')

    def identify_interface_nodes(
        self,
        tet_nodes: np.ndarray,
        inner_surface_ids: np.ndarray,
        bone_mesh: pv.PolyData,
        tolerance: float = 1.0
    ) -> np.ndarray:
        """
        Find which inner surface nodes are close to bone surface.

        Uses KDTree for nearest neighbor search to identify nodes
        at the bone-cartilage interface.

        Args:
            tet_nodes: (N, 3) tet mesh node coordinates
            inner_surface_ids: Node IDs at inner surface
            bone_mesh: Bone surface mesh
            tolerance: Distance tolerance in mm

        Returns:
            (K,) array of interface node indices
        """
        # Build KDTree of bone surface
        bone_points = bone_mesh.points
        bone_tree = cKDTree(bone_points)

        # Get inner surface nodes
        inner_nodes = tet_nodes[inner_surface_ids]

        # Find distances to bone surface
        distances, _ = bone_tree.query(inner_nodes, k=1)

        # Nodes within tolerance are interface nodes
        interface_mask = distances <= tolerance
        interface_node_ids = inner_surface_ids[interface_mask]

        self.logger.debug(
            f'[メッシュ分離] 骨-軟骨界面: {len(interface_node_ids)} 節点を検出'
        )

        return interface_node_ids

    def identify_contact_surface(
        self,
        tet_nodes: np.ndarray,
        outer_surface_ids: np.ndarray,
    ) -> np.ndarray:
        """
        Return outer surface node IDs for contact detection.

        Args:
            tet_nodes: (N, 3) tet mesh node coordinates
            outer_surface_ids: Node IDs at outer surface

        Returns:
            (K,) array of outer surface node indices
        """
        self.logger.debug(
            f'[メッシュ分離] 接触面: {len(outer_surface_ids)} 節点'
        )

        return outer_surface_ids

    def check_mesh_quality(
        self,
        nodes: np.ndarray,
        elements: np.ndarray
    ) -> Dict[str, Any]:
        """
        Check tetrahedral mesh quality metrics.

        Computes volume statistics, degenerate element detection,
        and aspect ratio information.

        Args:
            nodes: (N, 3) node coordinates
            elements: (M, 4) tet connectivity

        Returns:
            Dict with: n_nodes, n_elements, min_volume, max_volume, mean_volume,
                      n_degenerate, aspect_ratio stats
        """
        n_nodes = len(nodes)
        n_elements = len(elements)

        volumes = []
        aspect_ratios = []
        degenerate_count = 0

        for i0, i1, i2, i3 in elements:
            v0 = nodes[i0]
            v1 = nodes[i1]
            v2 = nodes[i2]
            v3 = nodes[i3]

            # Compute signed volume (6x actual volume)
            vol = np.dot(
                v1 - v0,
                np.cross(v2 - v0, v3 - v0)
            ) / 6.0

            if vol <= 0:
                degenerate_count += 1
                vol = abs(vol)

            volumes.append(vol)

            # Aspect ratio: max edge / min edge
            edges = [
                np.linalg.norm(v1 - v0),
                np.linalg.norm(v2 - v0),
                np.linalg.norm(v3 - v0),
                np.linalg.norm(v2 - v1),
                np.linalg.norm(v3 - v1),
                np.linalg.norm(v3 - v2),
            ]
            aspect = max(edges) / (min(edges) + 1e-12)
            aspect_ratios.append(aspect)

        volumes = np.array(volumes)
        aspect_ratios = np.array(aspect_ratios)

        quality = {
            'n_nodes': n_nodes,
            'n_elements': n_elements,
            'n_degenerate': degenerate_count,
            'min_volume': float(np.min(volumes)) if len(volumes) > 0 else 0.0,
            'max_volume': float(np.max(volumes)) if len(volumes) > 0 else 0.0,
            'mean_volume': float(np.mean(volumes)) if len(volumes) > 0 else 0.0,
            'std_volume': float(np.std(volumes)) if len(volumes) > 0 else 0.0,
            'min_aspect_ratio': float(np.min(aspect_ratios)) if len(aspect_ratios) > 0 else 0.0,
            'mean_aspect_ratio': float(np.mean(aspect_ratios)) if len(aspect_ratios) > 0 else 0.0,
            'max_aspect_ratio': float(np.max(aspect_ratios)) if len(aspect_ratios) > 0 else 0.0,
        }

        if degenerate_count > 0:
            self.logger.warning(
                f'[メッシュ品質] 最小体積: {quality["min_volume"]:.6f}, '
                f'退化要素: {degenerate_count} 個'
            )

        return quality

    def _remove_degenerate_elements(
        self,
        nodes: np.ndarray,
        elements: np.ndarray,
        volume_threshold: float = 1e-8
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Remove tetrahedral elements with zero or negative volume.

        Args:
            nodes: (N, 3) node coordinates
            elements: (M, 4) tet connectivity
            volume_threshold: Minimum volume to keep element

        Returns:
            Tuple of cleaned (nodes, elements)
        """
        valid_mask = []

        for i0, i1, i2, i3 in elements:
            v0 = nodes[i0]
            v1 = nodes[i1]
            v2 = nodes[i2]
            v3 = nodes[i3]

            vol = abs(np.dot(
                v1 - v0,
                np.cross(v2 - v0, v3 - v0)
            )) / 6.0

            valid_mask.append(vol > volume_threshold)

        valid_elements = elements[np.array(valid_mask)]

        # Rebuild node list: keep only referenced nodes
        used_node_indices = np.unique(valid_elements.flatten())
        node_mapping = {old_idx: new_idx for new_idx, old_idx
                       in enumerate(used_node_indices)}

        new_nodes = nodes[used_node_indices]
        new_elements = np.array(
            [[node_mapping[idx] for idx in elem] for elem in valid_elements],
            dtype=np.int32
        )

        n_removed = len(elements) - len(valid_elements)
        if n_removed > 0:
            self.logger.info(f'[メッシュ分離] {n_removed} 個の退化要素を削除しました')

        return new_nodes, new_elements

    def _validate_input_meshes(
        self,
        bone_mesh: pv.PolyData,
        cartilage_mesh: pv.PolyData
    ) -> None:
        """
        Validate input meshes for correctness.

        Args:
            bone_mesh: Bone surface mesh
            cartilage_mesh: Cartilage surface mesh

        Raises:
            ValueError: If meshes are invalid
        """
        if not isinstance(bone_mesh, pv.PolyData):
            raise ValueError('bone_mesh must be PyVista PolyData')

        if not isinstance(cartilage_mesh, pv.PolyData):
            raise ValueError('cartilage_mesh must be PyVista PolyData')

        if len(bone_mesh.points) < 3:
            raise ValueError('bone_mesh must have at least 3 points')

        if len(cartilage_mesh.points) < 3:
            raise ValueError('cartilage_mesh must have at least 3 points')

        if not bone_mesh.is_valid():
            self.logger.warning('[メッシュ分離] bone_mesh は無効な要素を含んでいます')

        if not cartilage_mesh.is_valid():
            self.logger.warning('[メッシュ分離] cartilage_mesh は無効な要素を含んでいます')

    def _create_material_regions_dual(
        self,
        prox_elements: np.ndarray,
        dist_elements: np.ndarray,
        bone_E: float,
        bone_nu: float,
        cart_E: float,
        cart_nu: float
    ) -> List[Any]:
        """
        Create material regions for proximal and distal cartilage.

        Args:
            prox_elements: Proximal element array
            dist_elements: Distal element array (with offset indices)
            bone_E: Bone Young's modulus
            bone_nu: Bone Poisson's ratio
            cart_E: Cartilage Young's modulus
            cart_nu: Cartilage Poisson's ratio

        Returns:
            List of MaterialRegion objects (or dicts)
        """
        prox_elem_ids = np.arange(len(prox_elements), dtype=np.int32)
        dist_elem_ids = np.arange(len(dist_elements), dtype=np.int32)

        if _HAS_FEM_TYPES:
            regions = [
                MaterialRegion(
                    name='Proximal_Cartilage',
                    E=cart_E,
                    nu=cart_nu,
                    element_ids=prox_elem_ids,
                    model='neo_hookean'
                ),
                MaterialRegion(
                    name='Distal_Cartilage',
                    E=cart_E,
                    nu=cart_nu,
                    element_ids=dist_elem_ids + len(prox_elements),
                    model='neo_hookean'
                ),
            ]
        else:
            regions = [
                {
                    'name': 'Proximal_Cartilage',
                    'E': cart_E,
                    'nu': cart_nu,
                    'element_ids': prox_elem_ids,
                    'model': 'neo_hookean'
                },
                {
                    'name': 'Distal_Cartilage',
                    'E': cart_E,
                    'nu': cart_nu,
                    'element_ids': dist_elem_ids + len(prox_elements),
                    'model': 'neo_hookean'
                },
            ]

        return regions

    def _create_boundary_conditions_dual(
        self,
        prox_interface_nodes: np.ndarray,
        dist_interface_nodes: np.ndarray,
        combined_nodes: np.ndarray,
        dist_prescribed_disp: Optional[np.ndarray] = None,
    ) -> List[Any]:
        """
        Create boundary conditions for both sides.

        - Proximal interface nodes: FIXED (zero displacement) — pelvis doesn't move
        - Distal interface nodes: PRESCRIBED DISPLACEMENT — bone pushes cartilage

        Args:
            prox_interface_nodes: Proximal interface node indices (fixed)
            dist_interface_nodes: Distal interface node indices (pushed)
            combined_nodes: All combined node coordinates
            dist_prescribed_disp: (3,) displacement to apply to all distal
                interface nodes. If None, distal is also fixed at zero.

        Returns:
            List of BoundaryCondition objects (or dicts)
        """
        bcs = []

        # Build per-node prescribed displacement arrays
        prox_disp = np.zeros((len(prox_interface_nodes), 3))

        if dist_prescribed_disp is not None:
            dist_disp_vec = np.asarray(dist_prescribed_disp, dtype=np.float64)
            if dist_disp_vec.ndim == 1 and dist_disp_vec.shape[0] == 3:
                # Uniform displacement for all distal interface nodes
                dist_disp = np.tile(dist_disp_vec, (len(dist_interface_nodes), 1))
            elif dist_disp_vec.ndim == 2 and dist_disp_vec.shape[0] == len(dist_interface_nodes):
                dist_disp = dist_disp_vec
            else:
                dist_disp = np.zeros((len(dist_interface_nodes), 3))
        else:
            dist_disp = np.zeros((len(dist_interface_nodes), 3))

        if _HAS_FEM_TYPES:
            # Proximal interface: fixed at zero
            bc_prox = BoundaryCondition(
                node_ids=prox_interface_nodes,
                dof_mask=np.array([True, True, True], dtype=bool),
                prescribed_disp=prox_disp,
            )
            bcs.append(bc_prox)

            # Distal interface: prescribed displacement (bone pushes cartilage)
            bc_dist = BoundaryCondition(
                node_ids=dist_interface_nodes,
                dof_mask=np.array([True, True, True], dtype=bool),
                prescribed_disp=dist_disp,
            )
            bcs.append(bc_dist)
        else:
            bcs.append({
                'node_ids': prox_interface_nodes,
                'dof_mask': np.array([True, True, True], dtype=bool),
                'prescribed_disp': prox_disp,
            })
            bcs.append({
                'node_ids': dist_interface_nodes,
                'dof_mask': np.array([True, True, True], dtype=bool),
                'prescribed_disp': dist_disp,
            })

        return bcs

    def _create_contact_pairs(
        self,
        prox_outer: np.ndarray,
        dist_outer: np.ndarray,
        penalty_stiffness: float,
        contact_tolerance: float
    ) -> List[Any]:
        """
        Create contact pair between proximal and distal outer surfaces.

        Args:
            prox_outer: Proximal outer surface node indices
            dist_outer: Distal outer surface node indices
            penalty_stiffness: Contact penalty stiffness
            contact_tolerance: Contact tolerance

        Returns:
            List of ContactPair objects (or dicts)
        """
        if _HAS_FEM_TYPES:
            contact = ContactPair(
                master_surface_nodes=prox_outer,
                slave_surface_nodes=dist_outer,
                penalty_stiffness=penalty_stiffness,
                contact_tolerance=contact_tolerance
            )
            return [contact]
        else:
            return [{
                'master_surface_nodes': prox_outer,
                'slave_surface_nodes': dist_outer,
                'penalty_stiffness': penalty_stiffness,
                'contact_tolerance': contact_tolerance
            }]


def create_visualization_mesh(
    nodes: np.ndarray,
    elements: np.ndarray,
    material_ids: Optional[np.ndarray] = None,
) -> pv.UnstructuredGrid:
    """
    Create a PyVista UnstructuredGrid from tetrahedral mesh data.

    Args:
        nodes: (N, 3) node coordinates
        elements: (M, 4) tet connectivity
        material_ids: Optional (M,) material region IDs

    Returns:
        pv.UnstructuredGrid for visualization
    """
    # VTK cell format: [4, n0, n1, n2, n3, 4, n0, n1, n2, n3, ...]
    cells = []
    for elem in elements:
        cells.extend([4, elem[0], elem[1], elem[2], elem[3]])

    # VTK_TETRA = 10
    celltypes = np.full(len(elements), 10, dtype=np.uint8)
    grid = pv.UnstructuredGrid(cells, celltypes, nodes)

    if material_ids is not None:
        grid.cell_data['material_id'] = material_ids

    return grid


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)

    print('MeshSeparator モジュールが正常にロードされました')
    print(f'TetGen 利用可能: {_HAS_TETGEN}')
    print(f'FEM タイプ利用可能: {_HAS_FEM_TYPES}')
