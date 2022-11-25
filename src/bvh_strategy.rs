use crate::{Axis, Triangle};

#[derive(Debug, Clone, Copy)]
pub struct SplitPlane {
    pub axis: Axis,
    pub split_position: f32,
    pub should_split: bool,
}

impl Default for SplitPlane {
    fn default() -> Self {
        Self {
            axis: Axis::X,
            split_position: f32::INFINITY,
            should_split: false,
        }
    }
}

pub trait SplitPlaneStrategy<Node> {
    /// Get split plane from a node using an strategy.
    fn get_split_plane(node: &Node, triangles: &[Triangle], triangles_id: &[u32]) -> SplitPlane;
}

pub trait LongestExtentNodeStrategy {
    /// Get split plane from a node using the longest extent of the aabb.
    fn get_longest_extent_split_plane(&self) -> SplitPlane;
}

pub struct LongestExtentStrategy {}
impl<Node> SplitPlaneStrategy<Node> for LongestExtentStrategy
where
    Node: LongestExtentNodeStrategy,
{
    /// Get split plane from a node using the longest extent of the aabb.
    #[inline(always)]
    fn get_split_plane(node: &Node, _triangles: &[Triangle], _triangles_id: &[u32]) -> SplitPlane {
        node.get_longest_extent_split_plane()
    }
}

pub trait SAHNodeStrategy {
    /// Get split plane from a node using the SAH strategy.
    fn get_sah_split_plane(&self, triangles: &[Triangle], triangles_id: &[u32]) -> SplitPlane;
}

pub struct SAHStrategy {}
impl<Node> SplitPlaneStrategy<Node> for SAHStrategy
where
    Node: SAHNodeStrategy,
{
    /// Get split plane from a node using the SAH strategy.
    #[inline(always)]
    fn get_split_plane(node: &Node, triangles: &[Triangle], triangles_id: &[u32]) -> SplitPlane {
        node.get_sah_split_plane(triangles, triangles_id)
    }
}

pub trait CompiledBinnedSAHNodeStrategy {
    /// Get split plane from a node using the binned SAH strategy using. Number of intervals set at compile time .
    fn get_compile_binned_sah_split_plane<const INTERVAL_NUM: usize>(
        &self,
        triangles: &[Triangle],
        triangles_id: &[u32],
    ) -> SplitPlane
    where
        [f32; INTERVAL_NUM - 1]: Sized;
}

pub struct CompiledBinnedSAHStrategy<const INTERVAL_NUM: usize = 8>
where
    [f32; INTERVAL_NUM - 1]: Sized, {}

impl<Node, const INTERVAL_NUM: usize> SplitPlaneStrategy<Node>
    for CompiledBinnedSAHStrategy<INTERVAL_NUM>
where
    Node: CompiledBinnedSAHNodeStrategy,
    [f32; INTERVAL_NUM - 1]: Sized,
{
    /// Get split plane from a node using the binned SAH strategy using. Number of intervals set at compile time .
    #[inline(always)]
    fn get_split_plane(node: &Node, triangles: &[Triangle], triangles_id: &[u32]) -> SplitPlane {
        node.get_compile_binned_sah_split_plane::<INTERVAL_NUM>(triangles, triangles_id)
    }
}
