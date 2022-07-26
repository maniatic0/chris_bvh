extern crate glam;

use strum::EnumIter;
use strum::IntoEnumIterator;

use crate::{FastRayIntersect, InPlaceRayIntersect, Ray, Triangle, AABB};

pub trait BVH: InPlaceRayIntersect {}

#[derive(Debug, Clone, Copy)]
pub struct SimpleBVHNode {
    pub aabb: AABB,
    left_first: u32,
    pub prim_count: u32,
}

impl Default for SimpleBVHNode {
    fn default() -> Self {
        Self {
            aabb: Default::default(),
            left_first: 0,
            prim_count: 0,
        }
    }
}

/// 3D Axis
#[derive(Debug, Clone, Copy, EnumIter)]
pub enum Axis {
    X = 0,
    Y = 1,
    Z = 2,
}

impl SimpleBVHNode {
    /// Get split plane from a node using the longest extent of the aabb. Returns the axis, the split position and if the node should be split
    pub fn get_longest_extent_split_plane(
        &self,
        _triangles: &Vec<Triangle>,
        _triangles_id: &Vec<u32>,
    ) -> (Axis, f32, bool) {
        if self.prim_count <= 2 {
            return (Axis::X, f32::INFINITY, false);
        }

        let extent = self.aabb.max - self.aabb.min;
        let mut axis = Axis::X;
        if extent.y > extent.x {
            axis = Axis::Y;
        }
        if extent.z > extent[axis as usize] {
            axis = Axis::Z;
        }
        let split_pos = self.aabb.min[axis as usize] + extent[axis as usize] * 0.5;
        (axis, split_pos, true)
    }

    /// Get split plane from a node using the SAH strategy. Returns the axis, the split position and if the node should be split
    pub fn get_sah_split_plane(
        &self,
        triangles: &Vec<Triangle>,
        triangles_id: &Vec<u32>,
    ) -> (Axis, f32, bool) {
        let mut best_axis = Axis::X;
        let mut best_pos: f32 = 0.0;
        let mut best_cost = f32::INFINITY;

        let first_prim = self.left_first as usize;
        let prim_count = self.prim_count as usize;

        for axis in Axis::iter() {
            for i in 0..prim_count {
                let triangle = &triangles[triangles_id[first_prim + i] as usize];
                let candidate_pos = triangle.centroid[axis as usize];
                let cost = self.evaluate_sah(triangles, triangles_id, axis, candidate_pos);
                if cost < best_cost {
                    best_pos = candidate_pos;
                    best_axis = axis;
                    best_cost = cost;
                }
            }
        }

        if best_cost >= self.compute_sah() {
            return (Axis::X, f32::INFINITY, false);
        }

        return (best_axis, best_pos, true);
    }

    /// Compute the SAH cost of this node
    #[inline]
    pub fn compute_sah(&self) -> f32 {
        self.prim_count as f32 * self.aabb.area()
    }

    /// Evaluate the SAH heuristic code at a position
    fn evaluate_sah(
        &self,
        triangles: &Vec<Triangle>,
        triangles_id: &Vec<u32>,
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
            if triangle.centroid[axis as usize] < pos {
                left_count += 1;
                left_box.grow(triangle.vertex0);
                left_box.grow(triangle.vertex1);
                left_box.grow(triangle.vertex2);
            } else {
                right_count += 1;
                right_box.grow(triangle.vertex0);
                right_box.grow(triangle.vertex1);
                right_box.grow(triangle.vertex2);
            }
        }
        let cost = left_count as f32 * left_box.area() + right_count as f32 * right_box.area();
        return if cost > 0.0 { cost } else { f32::INFINITY };
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

pub struct SimpleBVH {
    pub triangles: Vec<Triangle>,
    triangles_id: Vec<u32>,
    nodes: Vec<SimpleBVHNode>,
    root_node_id: u32,
    nodes_used: u32,
}

impl Default for SimpleBVH {
    fn default() -> Self {
        Self {
            triangles: Default::default(),
            triangles_id: Default::default(),
            nodes: Default::default(),
            root_node_id: 0,
            nodes_used: 0,
        }
    }
}

impl SimpleBVH {
    pub fn init(&mut self, triangles: Vec<Triangle>) {
        self.triangles = triangles;
    }

    pub fn build(&mut self) {
        let tri_count = self.triangles.len();
        self.triangles_id.resize(tri_count, Default::default());
        for i in 0..tri_count {
            self.triangles_id[i] = i as u32;
        }

        self.root_node_id = 0;
        self.nodes_used = 2; // skip one for cache alignment

        self.nodes.resize(
            {
                if tri_count > 0 {
                    2 * tri_count // we use 2 * N as node[1] is empty to make the caches fit
                } else {
                    1
                }
            },
            Default::default(),
        );

        let root = &mut self.nodes[self.root_node_id as usize];
        root.setup_prims(0, tri_count as u32);
        root.aabb = Default::default();

        if tri_count > 0 {
            self.build_node_bounds(self.root_node_id);
            self.subdivide(self.root_node_id);
        }
    }

    fn build_node_bounds(&mut self, node_id: u32) {
        let node = &mut self.nodes[node_id as usize];
        assert!(node.is_leaf(), "Not valid for internal nodes");
        node.aabb = Default::default();

        let first: usize = node.first_prim() as usize;
        for i in 0..node.prim_count {
            let tri = self.triangles[self.triangles_id[first + i as usize] as usize];
            node.aabb.grow(tri.vertex0);
            node.aabb.grow(tri.vertex1);
            node.aabb.grow(tri.vertex2);
        }
    }

    fn subdivide(&mut self, node_id: u32) {
        let node = &mut self.nodes[node_id as usize];
        assert!(node.is_leaf(), "Not valid for internal nodes");

        let (axis, split_pos, should_split) =
            node.get_sah_split_plane(&self.triangles, &self.triangles_id);

        if !should_split {
            return;
        }

        // Quick partition
        // j might go below 0 in the case i == 0
        let axis = axis as usize;

        let mut i = node.first_prim() as isize;
        let mut j = i + node.prim_count as isize - 1;
        while i <= j {
            if self.triangles[self.triangles_id[i as usize] as usize].centroid[axis] < split_pos {
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
        self.build_node_bounds(left_child_idx);
        self.build_node_bounds(right_child_idx);
        self.subdivide(left_child_idx);
        self.subdivide(right_child_idx);
    }

    fn intersect_ray(&self, node_id: u32, ray: &mut Ray) {
        let node = &self.nodes[node_id as usize];
        if !node.aabb.fast_ray_intersect(ray) {
            return;
        }

        if node.is_leaf() {
            for i in 0..node.prim_count {
                self.triangles[self.triangles_id[(node.first_prim() + i) as usize] as usize]
                    .inplace_ray_intersect(ray);
            }
        } else {
            self.intersect_ray(node.left_first, ray);
            self.intersect_ray(node.right_child(), ray);
        }
    }
}

impl InPlaceRayIntersect for SimpleBVH {
    #[inline]
    fn inplace_ray_intersect(&self, ray: &mut Ray) {
        if self.triangles.len() > 0 {
            self.intersect_ray(self.root_node_id, ray);
        }
    }
}

impl BVH for SimpleBVH {}

#[cfg(test)]
mod tests {

    use std::iter;

    use rand::{prelude::SliceRandom, thread_rng, Rng};

    use glam::Vec3A;

    use approx::*;

    use crate::*;

    static TRIANGLES_NUM: usize = 64;

    #[test]
    fn empty_no_intersect() {
        let mut ray = Ray::infinite_ray(Vec3A::ZERO, Vec3A::X);

        let mut bvh = SimpleBVH::default();
        bvh.init(vec![]);
        bvh.build();

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
        bvh.init(vec![tri]);
        bvh.build();

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
        bvh.init(vec![tri]);
        bvh.build();

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

        let tri = triangles.choose(&mut rng).unwrap();

        let mut ray = Ray::infinite_ray(Vec3A::ZERO, tri.centroid.normalize_or_zero());

        let mut bvh = SimpleBVH::default();
        bvh.init(triangles.clone());
        bvh.build();

        bvh.inplace_ray_intersect(&mut ray);

        assert!(ray.distance < tri.centroid.distance(Vec3A::ZERO));

        let mut ray2 = Ray::infinite_ray(Vec3A::ZERO, tri.centroid.normalize_or_zero());

        triangles.iter().for_each(|tri| {
            tri.inplace_ray_intersect(&mut ray2);
        });

        assert_abs_diff_eq!(ray.distance, ray2.distance, epsilon = RAY_INTERSECT_EPSILON);
    }
}
