use crate::{Grow, Triangle, AABB, BVH};

impl BVH for Triangle {
    #[inline]
    fn bounds(&self) -> AABB {
        let mut aabb = AABB::default();

        aabb.grow(self);

        aabb
    }

    #[inline]
    fn centroid(&self) -> glam::Vec3A {
        self.centroid
    }
}
