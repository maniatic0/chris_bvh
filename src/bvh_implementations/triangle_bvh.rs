use crate::{Grow, GrowAABB, Triangle, AABB, BVH};

impl BVH for Triangle {
    #[inline]
    fn bounds(&self) -> AABB {
        let mut aabb = AABB::default();

        aabb.grow(self);

        aabb
    }

    #[inline(always)]
    fn centroid(&self) -> glam::Vec3A {
        self.centroid
    }
}

impl GrowAABB for Triangle {
    #[inline]
    fn grow_aabb(&self, aabb: &mut AABB) {
        aabb.grow(self.vertex0);
        aabb.grow(self.vertex1);
        aabb.grow(self.vertex2);
    }
}
