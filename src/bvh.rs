extern crate glam;
use glam::Vec3A;

use crate::{InPlaceRayIntersect, AABB};

pub trait BVH: InPlaceRayIntersect {
    fn bounds(&self) -> AABB;
    fn centroid(&self) -> Vec3A;
}
