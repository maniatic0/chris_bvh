extern crate glam;
use glam::Vec3A;

use strum::IntoEnumIterator;

use crate::{
    Axis, CompiledBinnedSAHNodeStrategy, FastRayIntersect, Grow, InPlaceRayIntersect,
    LongestExtentNodeStrategy, Ray, RayIntersect, SAHNodeStrategy, SplitPlane, SplitPlaneStrategy,
    Triangle, AABB, BVH, RAY_INTERSECT_EPSILON,
};

use parking_lot::RwLock;
use std::cmp::min;
use std::sync::Arc;

#[derive(Debug, Clone, Copy, Default)]
pub struct SimpleBVHNode {
    pub aabb: AABB,
    left_first: u32,
    pub prim_count: u32,
}

impl SimpleBVHNode {
    /// Compute the SAH cost of this node
    #[inline]
    pub fn compute_sah(&self) -> f32 {
        self.prim_count as f32 * self.aabb.area()
    }

    /// Evaluate the SAH heuristic code at a position
    fn evaluate_sah(
        &self,
        triangles: &[Triangle],
        triangles_id: &[u32],
        axis: Axis,
        pos: f32,
    ) -> f32 {
        // determine triangle counts and bounds for this split candidate
        let mut left_box: AABB = Default::default();
        let mut right_box: AABB = Default::default();
        let mut left_count = 0;
        let mut right_count = 0;

        let first_prim = self.left_first as usize;
        let prim_count = self.prim_count as usize;

        for i in 0..prim_count {
            let triangle = &triangles[triangles_id[first_prim + i] as usize];
            if triangle.centroid[axis] < pos {
                left_count += 1;
                left_box.grow(triangle);
            } else {
                right_count += 1;
                right_box.grow(triangle);
            }
        }

        let cost = left_count as f32 * left_box.area() + right_count as f32 * right_box.area();

        if cost > 0.0 {
            cost
        } else {
            f32::INFINITY
        }
    }

    #[inline]
    pub fn is_leaf(&self) -> bool {
        self.prim_count > 0
    }

    #[inline]
    pub fn left_child(&self) -> u32 {
        assert!(!self.is_leaf());
        self.left_first
    }

    #[inline]
    pub fn left_child_mut(&mut self) -> &mut u32 {
        assert!(!self.is_leaf());
        &mut self.left_first
    }

    #[inline]
    pub fn first_prim(&self) -> u32 {
        assert!(self.is_leaf());
        self.left_first
    }

    #[inline]
    pub fn first_prim_mut(&mut self) -> &mut u32 {
        assert!(self.is_leaf());
        &mut self.left_first
    }

    #[inline]
    pub fn right_child(&self) -> u32 {
        self.left_child() + 1
    }

    #[inline]
    pub fn setup_prims(&mut self, first_prim: u32, prim_count: u32) {
        self.prim_count = prim_count;
        self.left_first = first_prim;
    }

    #[inline]
    pub fn setup_left_child(&mut self, left_child: u32) {
        self.prim_count = 0;
        self.left_first = left_child;
    }
}

impl LongestExtentNodeStrategy for SimpleBVHNode {
    /// Get split plane from a node using the longest extent of the aabb.
    fn get_longest_extent_split_plane(&self) -> SplitPlane {
        if self.prim_count <= 2 {
            return SplitPlane::default();
        }

        // Get Longest Extent Axis
        let extent = self.aabb.extent();
        let mut axis = Axis::X;
        if extent.y > extent.x {
            axis = Axis::Y;
        }
        if extent.z > extent[axis] {
            axis = Axis::Z;
        }

        let axis = axis;

        let split_pos = self.aabb.min[axis] + extent[axis] * 0.5;
        SplitPlane {
            axis,
            split_position: split_pos,
            should_split: true,
        }
    }
}

impl SAHNodeStrategy for SimpleBVHNode {
    /// Get split plane from a node using the SAH strategy.
    fn get_sah_split_plane(&self, triangles: &[Triangle], triangles_id: &[u32]) -> SplitPlane {
        let mut best_axis = Axis::X;
        let mut best_pos: f32 = 0.0;
        let mut best_cost = f32::INFINITY;

        let first_prim = self.left_first as usize;
        let prim_count = self.prim_count as usize;

        for axis in Axis::iter() {
            for i in 0..prim_count {
                let triangle = &triangles[triangles_id[first_prim + i] as usize];
                let candidate_pos = triangle.centroid[axis];
                let cost = self.evaluate_sah(triangles, triangles_id, axis, candidate_pos);
                if cost < best_cost {
                    best_pos = candidate_pos;
                    best_axis = axis;
                    best_cost = cost;
                }
            }
        }

        if best_cost >= self.compute_sah() {
            return SplitPlane::default();
        }

        SplitPlane {
            axis: best_axis,
            split_position: best_pos,
            should_split: true,
        }
    }
}

impl CompiledBinnedSAHNodeStrategy for SimpleBVHNode {
    /// Get split plane from a node using the binned SAH strategy using. Number of intervals set at compile time .
    fn get_compile_binned_sah_split_plane<const INTERVAL_NUM: usize>(
        &self,
        triangles: &[Triangle],
        triangles_id: &[u32],
    ) -> SplitPlane
    where
        [f32; INTERVAL_NUM - 1]: Sized,
    {
        assert!(INTERVAL_NUM >= 2, "At least two intervals are needed");
        let mut best_axis = Axis::X;
        let mut best_pos: f32 = 0.0;
        let mut best_cost = f32::INFINITY;

        let first_prim = self.left_first as usize;
        let prim_count = self.prim_count as usize;

        let mut bounds_min = glam::Vec3A::splat(f32::INFINITY);
        let mut bounds_max = glam::Vec3A::splat(-f32::INFINITY);

        for i in 0..prim_count {
            let triangle = &triangles[triangles_id[first_prim + i] as usize];
            bounds_min = bounds_min.min(triangle.centroid);
            bounds_max = bounds_max.max(triangle.centroid);
        }

        let bounds_min = bounds_min;
        let bounds_max = bounds_max;

        #[derive(Debug, Clone, Copy, Default)]
        struct Bin {
            pub bounds: AABB,
            pub tri_count: u32,
        }

        for axis in Axis::iter() {
            let bounds_min = bounds_min[axis];
            let bounds_max = bounds_max[axis];

            if approx::abs_diff_eq!(bounds_min, bounds_max, epsilon = RAY_INTERSECT_EPSILON) {
                continue;
            }

            let mut bins: [Bin; INTERVAL_NUM] = [Default::default(); INTERVAL_NUM];

            let scale = INTERVAL_NUM as f32 / (bounds_max - bounds_min);

            for i in 0..prim_count {
                let triangle = &triangles[triangles_id[first_prim + i] as usize];

                let bin_id = min(
                    INTERVAL_NUM - 1,
                    ((triangle.centroid[axis] - bounds_min) * scale) as usize,
                );

                let bin = &mut bins[bin_id];

                bin.tri_count += 1;
                bin.bounds.grow(triangle);
            }

            let bins = bins;

            let mut left_area: [f32; INTERVAL_NUM - 1] = [0.0; INTERVAL_NUM - 1];
            let mut right_area: [f32; INTERVAL_NUM - 1] = [0.0; INTERVAL_NUM - 1];
            let mut left_count: [u32; INTERVAL_NUM - 1] = [0; INTERVAL_NUM - 1];
            let mut right_count: [u32; INTERVAL_NUM - 1] = [0; INTERVAL_NUM - 1];

            let mut left_box: AABB = Default::default();
            let mut right_box: AABB = Default::default();

            let mut left_sum: u32 = 0;
            let mut right_sum: u32 = 0;

            for i in 0..(INTERVAL_NUM - 1) {
                left_sum += bins[i].tri_count;
                left_count[i] = left_sum;
                left_box.grow(&bins[i].bounds);
                left_area[i] = left_box.area();

                right_sum += bins[INTERVAL_NUM - 1 - i].tri_count;
                right_count[INTERVAL_NUM - 2 - i] = right_sum;
                right_box.grow(&bins[INTERVAL_NUM - 1 - i].bounds);
                right_area[INTERVAL_NUM - 2 - i] = right_box.area();
            }

            let left_area = left_area;
            let right_area = right_area;
            let left_count = left_count;
            let right_count = right_count;

            let scale = (bounds_max - bounds_min) / INTERVAL_NUM as f32;
            for i in 0..(INTERVAL_NUM - 1) {
                let plane_cost =
                    left_count[i] as f32 * left_area[i] + right_count[i] as f32 * right_area[i];
                if plane_cost < best_cost {
                    best_pos = bounds_min + scale * (i + 1) as f32;
                    best_axis = axis;
                    best_cost = plane_cost;
                }
            }
        }

        if best_cost >= self.compute_sah() {
            return SplitPlane::default();
        }

        SplitPlane {
            axis: best_axis,
            split_position: best_pos,
            should_split: true,
        }
    }
}

#[derive(Default)]
pub struct SimpleBVH {
    triangles: Arc<RwLock<Vec<Triangle>>>,
    triangles_id: Vec<u32>,
    nodes: Vec<SimpleBVHNode>,
    root_node_id: u32,
    nodes_used: u32,
}

impl SimpleBVH {
    /// Max stack size for build and traverse operations (can address 4 GB)
    const MAX_STACK_SIZE: usize = 64;

    #[inline]
    pub fn init(&mut self, triangles: Arc<RwLock<Vec<Triangle>>>) {
        self.triangles = triangles;
    }

    pub fn build<Strat>(&mut self)
    where
        Strat: SplitPlaneStrategy<SimpleBVHNode>,
    {
        let triangles_arc = self.triangles.clone();
        let triangle_ref = triangles_arc.read();
        let triangles: &[Triangle] = &triangle_ref;
        let tri_count = triangles.len();
        self.triangles_id.resize(tri_count, Default::default());
        for i in 0..tri_count {
            self.triangles_id[i] = i as u32;
        }

        self.root_node_id = 1;
        self.nodes_used = 2; // skip one for cache alignment

        self.nodes.resize(
            {
                if tri_count > 0 {
                    2 * tri_count // we use 2 * N as node[0] is empty to make the caches fit
                } else {
                    2 // we use 2 as node[0] is empty to make the caches fit (in this case to avoid special code)
                }
            },
            Default::default(),
        );

        let root = &mut self.nodes[self.root_node_id as usize];
        root.setup_prims(0, tri_count as u32);
        root.aabb = Default::default();

        if tri_count > 0 {
            self.build_node_bounds(triangles, self.root_node_id);
            self.subdivide::<Strat>(triangles, self.root_node_id);
        }
    }

    fn build_node_bounds(&mut self, triangles: &[Triangle], node_id: u32) {
        let node = &mut self.nodes[node_id as usize];
        assert!(node.is_leaf(), "Not valid for internal nodes");
        node.aabb = Default::default();

        let first: usize = node.first_prim() as usize;
        for i in 0..node.prim_count {
            let tri = &triangles[self.triangles_id[first + i as usize] as usize];
            node.aabb.grow(tri);
        }
    }

    fn subdivide<Strat>(&mut self, triangles: &[Triangle], node_id: u32)
    where
        Strat: SplitPlaneStrategy<SimpleBVHNode>,
    {
        let node = &mut self.nodes[node_id as usize];
        assert!(node.is_leaf(), "Not valid for internal nodes");

        let SplitPlane {
            axis,
            split_position: split_pos,
            should_split,
        } = Strat::get_split_plane(node, triangles, &self.triangles_id);

        if !should_split {
            return;
        }

        // Quick partition
        // j might go below 0 in the case i == 0
        let axis = axis;

        let mut i = node.first_prim() as isize;
        let mut j = i + node.prim_count as isize - 1;
        while i <= j {
            if triangles[self.triangles_id[i as usize] as usize].centroid[axis] < split_pos {
                i += 1;
            } else {
                self.triangles_id.swap(i as usize, j as usize);
                j -= 1;
            }
        }

        let i = i as usize;

        // One side is empty
        let left_count = i - node.first_prim() as usize;
        if left_count == 0 || left_count == node.prim_count as usize {
            return;
        }

        // create child nodes
        let left_child_idx = self.nodes_used as usize;
        self.nodes_used += 1;
        let right_child_idx = self.nodes_used as usize;
        self.nodes_used += 1;

        let node_first_prim = node.first_prim(); // it is lost after prim_count = 0
        let node_prim_count = node.prim_count; // it is lost after prim_count = 0

        node.setup_left_child(left_child_idx as u32);

        assert!(left_count < node_prim_count as usize, "No empty childs");

        self.nodes[left_child_idx].setup_prims(node_first_prim, left_count as u32);
        self.nodes[right_child_idx].setup_prims(i as u32, node_prim_count - left_count as u32);

        let left_child_idx = left_child_idx as u32;
        let right_child_idx = right_child_idx as u32;
        self.build_node_bounds(triangles, left_child_idx);
        self.build_node_bounds(triangles, right_child_idx);
        self.subdivide::<Strat>(triangles, left_child_idx);
        self.subdivide::<Strat>(triangles, right_child_idx);
    }

    /// Split a node to get the node and both children as mutable
    ///
    /// # Return
    ///
    /// (Node, Left Child, Right Child)
    ///
    /// # Panic
    /// This can panic if the node is a leaf or if the children are wrongly setup
    #[allow(dead_code)]
    #[inline]
    fn split_mut_node_at_mut(
        &mut self,
        node_id: u32,
    ) -> (&mut SimpleBVHNode, &mut SimpleBVHNode, &mut SimpleBVHNode) {
        let node_id = node_id as usize;
        let node = &self.nodes[node_id];
        assert!(
            !node.is_leaf(),
            "This is only valid if the node is not a leaf"
        );

        let left_child_id = node.left_child() as usize;
        let right_child_id = node.right_child() as usize;

        let len = self.nodes.len();

        assert_ne!(node_id, left_child_id);
        assert_ne!(node_id, right_child_id);
        assert_ne!(left_child_id, right_child_id);
        assert!(left_child_id < len);
        assert!(right_child_id < len);

        let ptr = self.nodes.as_mut_ptr();

        unsafe {
            (
                &mut *ptr.add(node_id),
                &mut *ptr.add(left_child_id),
                &mut *ptr.add(right_child_id),
            )
        }
    }

    /// Split a node to get the node as mutable and both children as immutable
    ///
    /// # Return
    ///
    /// (Node, Left Child, Right Child)
    ///
    /// # Panic
    /// This can panic if the node is a leaf or if the children are wrongly setup
    #[allow(dead_code)]
    #[inline]
    fn split_mut_node_at(
        &mut self,
        node_id: u32,
    ) -> (&mut SimpleBVHNode, &SimpleBVHNode, &SimpleBVHNode) {
        let node_id = node_id as usize;
        let node = &self.nodes[node_id];
        assert!(
            !node.is_leaf(),
            "This is only valid if the node is not a leaf"
        );

        let left_child_id = node.left_child() as usize;
        let right_child_id = node.right_child() as usize;

        let len = self.nodes.len();

        assert_ne!(node_id, left_child_id);
        assert_ne!(node_id, right_child_id);
        assert_ne!(left_child_id, right_child_id);
        assert!(left_child_id < len);
        assert!(right_child_id < len);

        let ptr = self.nodes.as_mut_ptr();

        unsafe {
            (
                &mut *ptr.add(node_id),
                &*ptr.add(left_child_id),
                &*ptr.add(right_child_id),
            )
        }
    }

    pub fn refit(&mut self) {
        let triangles_arc = self.triangles.clone();
        let triangle_ref = triangles_arc.read();
        let triangles: &[Triangle] = &triangle_ref;

        if triangles.is_empty() {
            return;
        }

        for i in (self.root_node_id..self.nodes_used).rev() {
            let node = &self.nodes[i as usize];
            if node.is_leaf() {
                self.build_node_bounds(triangles, i);
                continue;
            }

            let (node, left_child, right_child) = self.split_mut_node_at(i);

            node.aabb.grow(&left_child.aabb);
            node.aabb.grow(&right_child.aabb);
        }
    }

    fn inplace_intersect_ray(&self, triangles: &[Triangle], node_id: u32, ray: &mut Ray) {
        let mut node = Option::Some(&self.nodes[node_id as usize]);

        let node_ref = node.unwrap();
        if !node_ref.aabb.fast_ray_intersect(ray) {
            return;
        }

        let mut stack: [Option<&SimpleBVHNode>; SimpleBVH::MAX_STACK_SIZE] =
            [Default::default(); SimpleBVH::MAX_STACK_SIZE];
        let mut stack_ptr = 0_usize;

        loop {
            let node_ref = node.unwrap();

            if node_ref.is_leaf() {
                let first_prim = node_ref.first_prim();

                for i in 0..node_ref.prim_count {
                    triangles[self.triangles_id[(first_prim + i) as usize] as usize]
                        .inplace_ray_intersect(ray);
                }

                if stack_ptr == 0 {
                    break;
                } else {
                    stack_ptr -= 1;
                    node = stack[stack_ptr];
                    continue;
                }
            } else {
                let child1 = &self.nodes[node_ref.left_child() as usize];
                let child2 = &self.nodes[node_ref.right_child() as usize];

                let mut dist1 = child1.aabb.ray_intersect(ray);
                let mut dist2 = child2.aabb.ray_intersect(ray);

                let mut child1 = Option::Some(child1);
                let mut child2 = Option::Some(child2);

                if dist1 > dist2 {
                    (dist1, dist2) = (dist2, dist1);
                    (child1, child2) = (child2, child1);
                }

                let dist1 = dist1;
                let dist2 = dist2;

                let child1 = child1;
                let child2 = child2;

                if dist1.is_infinite() {
                    if stack_ptr == 0 {
                        break;
                    } else {
                        stack_ptr -= 1;
                        node = stack[stack_ptr];
                        continue;
                    }
                } else {
                    node = child1;
                    if dist2.is_finite() {
                        stack[stack_ptr] = child2;
                        stack_ptr += 1;
                    }
                }
            }
        }
    }

    pub fn triangles(&self) -> &Arc<RwLock<Vec<Triangle>>> {
        &self.triangles
    }

    /// Intersect the BVH and use the triangles from the vector for the leaves
    ///
    /// # Safety
    /// The triangles must be the same as the one stored in the BVH (you can get the Arc-Mutex from the BVH)
    #[inline]
    pub unsafe fn unsafe_inplace_ray_intersect(&self, triangles: &[Triangle], ray: &mut Ray) {
        if !triangles.is_empty() {
            self.inplace_intersect_ray(triangles, self.root_node_id, ray);
        }
    }
}

impl InPlaceRayIntersect for SimpleBVH {
    #[inline]
    fn inplace_ray_intersect(&self, ray: &mut Ray) {
        let triangle_ref = self.triangles.read();
        let triangles: &[Triangle] = &triangle_ref;
        if !triangles.is_empty() {
            self.inplace_intersect_ray(triangles, self.root_node_id, ray);
        }
    }
}

impl BVH for SimpleBVH {
    #[inline]
    fn bounds(&self) -> AABB {
        self.nodes[self.root_node_id as usize].aabb
    }

    #[inline]
    fn centroid(&self) -> Vec3A {
        self.bounds().center()
    }
}

#[cfg(test)]
mod tests {

    use std::{iter, sync::Arc};

    use parking_lot::RwLock;

    use rand::{prelude::SliceRandom, thread_rng, Rng};

    use glam::Vec3A;

    use approx::*;

    use crate::*;

    static TRIANGLES_NUM: usize = 64;

    #[test]
    fn empty_no_intersect() {
        let mut ray = Ray::infinite_ray(Vec3A::ZERO, Vec3A::X);

        let mut bvh = SimpleBVH::default();

        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(vec![]));
        bvh.init(triangles);
        bvh.build::<CompiledBinnedSAHStrategy>();

        bvh.inplace_ray_intersect(&mut ray);

        assert!(ray.distance.is_infinite());
    }

    #[test]
    fn ray_triangle_intersect() {
        let mut rng = thread_rng();
        let tri: Triangle = {
            let v0 = rng.gen::<Vec3A>() * 9.0 - Vec3A::splat(5.0);
            let v1 = rng.gen();
            let v2 = rng.gen();
            Triangle::new(v0, v1, v2)
        };

        let mut ray = Ray::infinite_ray(Vec3A::ZERO, tri.centroid.normalize_or_zero());

        let mut bvh = SimpleBVH::default();

        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(vec![tri]));

        bvh.init(triangles);
        bvh.build::<CompiledBinnedSAHStrategy>();

        bvh.inplace_ray_intersect(&mut ray);

        assert_abs_diff_eq!(
            ray.distance,
            tri.centroid.distance(Vec3A::ZERO),
            epsilon = RAY_INTERSECT_EPSILON
        );
    }

    #[test]
    fn ray_triangle_no_intersect() {
        let mut rng = thread_rng();
        let tri: Triangle = {
            let v0 = rng.gen::<Vec3A>() * 9.0 - Vec3A::splat(5.0);
            let v1 = rng.gen();
            let v2 = rng.gen();
            Triangle::new(v0, v1, v2)
        };

        let mut ray = Ray::infinite_ray(Vec3A::ZERO, -tri.centroid.normalize_or_zero());

        let mut bvh = SimpleBVH::default();

        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(vec![tri]));

        bvh.init(triangles);
        bvh.build::<CompiledBinnedSAHStrategy>();

        bvh.inplace_ray_intersect(&mut ray);

        assert!(ray.distance.is_infinite());
    }

    #[test]
    fn ray_triangles_intersect() {
        let mut rng = thread_rng();
        let triangles: Vec<Triangle> = iter::repeat(0)
            .take(TRIANGLES_NUM)
            .map(|_| {
                let v0 = rng.gen::<Vec3A>() * 9.0 - Vec3A::splat(5.0);
                let v1 = rng.gen();
                let v2 = rng.gen();
                Triangle::new(v0, v1, v2)
            })
            .collect();

        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(triangles));

        let triangles_ref = triangles.read();
        let tri = triangles_ref.choose(&mut rng).unwrap();

        let mut ray = Ray::infinite_ray(Vec3A::ZERO, tri.centroid.normalize_or_zero());

        let mut bvh = SimpleBVH::default();
        bvh.init(triangles.clone());
        bvh.build::<CompiledBinnedSAHStrategy>();

        bvh.inplace_ray_intersect(&mut ray);

        assert!(
            ray.distance - tri.centroid.distance(Vec3A::ZERO) <= RAY_INTERSECT_EPSILON,
            "Failed: {} <= {} with error {} and epsilon {}",
            ray.distance,
            tri.centroid.distance(Vec3A::ZERO),
            tri.centroid.distance(Vec3A::ZERO) - ray.distance,
            RAY_INTERSECT_EPSILON
        );

        let mut ray2 = Ray::infinite_ray(Vec3A::ZERO, tri.centroid.normalize_or_zero());

        triangles_ref.iter().for_each(|tri| {
            tri.inplace_ray_intersect(&mut ray2);
        });

        assert_abs_diff_eq!(ray.distance, ray2.distance, epsilon = RAY_INTERSECT_EPSILON);
    }

    #[test]
    fn ray_triangles_refit_intersect() {
        let mut rng = thread_rng();
        let triangles: Vec<Triangle> = iter::repeat(0)
            .take(TRIANGLES_NUM)
            .map(|_| {
                let v0 = rng.gen::<Vec3A>() * 9.0 - Vec3A::splat(5.0);
                let v1 = rng.gen();
                let v2 = rng.gen();
                Triangle::new(v0, v1, v2)
            })
            .collect();

        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(triangles));

        let mut bvh = SimpleBVH::default();
        bvh.init(triangles.clone());
        bvh.build::<CompiledBinnedSAHStrategy>();

        {
            let triangles_ref = triangles.read();
            let tri = triangles_ref.choose(&mut rng).unwrap();

            let mut ray = Ray::infinite_ray(Vec3A::ZERO, tri.centroid.normalize_or_zero());

            bvh.inplace_ray_intersect(&mut ray);

            assert!(
                ray.distance - tri.centroid.distance(Vec3A::ZERO) <= RAY_INTERSECT_EPSILON,
                "Failed: {} <= {} with error {} and epsilon {}",
                ray.distance,
                tri.centroid.distance(Vec3A::ZERO),
                tri.centroid.distance(Vec3A::ZERO) - ray.distance,
                RAY_INTERSECT_EPSILON
            );

            let mut ray2 = Ray::infinite_ray(Vec3A::ZERO, tri.centroid.normalize_or_zero());

            triangles_ref.iter().for_each(|tri| {
                tri.inplace_ray_intersect(&mut ray2);
            });

            assert_abs_diff_eq!(ray.distance, ray2.distance, epsilon = RAY_INTERSECT_EPSILON);
        }

        {
            let mut triangles_write_ref = triangles.write();
            triangles_write_ref.iter_mut().for_each(|tri| {
                let v0 = rng.gen::<Vec3A>() * 9.0 - Vec3A::splat(5.0);
                let v1 = rng.gen();
                let v2 = rng.gen();
                *tri = Triangle::new(v0, v1, v2)
            });
        }

        bvh.refit();

        {
            let triangles_ref = triangles.read();
            let tri = triangles_ref.choose(&mut rng).unwrap();

            let mut ray = Ray::infinite_ray(Vec3A::ZERO, tri.centroid.normalize_or_zero());

            bvh.inplace_ray_intersect(&mut ray);

            assert!(
                ray.distance - tri.centroid.distance(Vec3A::ZERO) <= RAY_INTERSECT_EPSILON,
                "Failed: {} <= {} with error {} and epsilon {}",
                ray.distance,
                tri.centroid.distance(Vec3A::ZERO),
                tri.centroid.distance(Vec3A::ZERO) - ray.distance,
                RAY_INTERSECT_EPSILON
            );

            let mut ray2 = Ray::infinite_ray(Vec3A::ZERO, tri.centroid.normalize_or_zero());

            triangles_ref.iter().for_each(|tri| {
                tri.inplace_ray_intersect(&mut ray2);
            });

            assert_abs_diff_eq!(ray.distance, ray2.distance, epsilon = RAY_INTERSECT_EPSILON);
        }
    }
}
