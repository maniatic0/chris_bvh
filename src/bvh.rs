extern crate glam;
use glam::Vec3A;

use crate::{GrowAABB, InPlaceRayIntersect, AABB};

pub trait BVH: InPlaceRayIntersect + GrowAABB {
    fn bounds(&self) -> AABB;
    fn centroid(&self) -> Vec3A;
}
