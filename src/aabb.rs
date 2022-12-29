use crate::Axis;

pub trait Grow<T> {
    fn grow(&mut self, object: T);
}

pub trait GrowAABB {
    fn grow_aabb(&self, aabb: &mut AABB);
}

#[derive(Debug, Clone, Copy)]
pub struct AABB {
    pub min: glam::Vec3A,
    pub max: glam::Vec3A,
}

impl Default for AABB {
    fn default() -> Self {
        Self {
            min: glam::Vec3A::splat(f32::INFINITY),
            max: glam::Vec3A::splat(-f32::INFINITY),
        }
    }
}

impl AABB {
    /// If the AABB is valid (min <= max)
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.min.le(&self.max)
    }

    #[inline]
    pub fn extent(&self) -> glam::Vec3A {
        self.max - self.min
    }

    #[inline]
    pub fn center(&self) -> glam::Vec3A {
        (self.max + self.min) * 0.5
    }

    #[inline]
    pub fn area(&self) -> f32 {
        self.extent().length_squared()
    }

    #[inline]
    pub fn longest_extent_axis(&self) -> Axis {
        let extent = self.extent();
        let mut axis = Axis::X;
        if extent.y > extent.x {
            axis = Axis::Y;
        }
        if extent.z > extent[axis] {
            axis = Axis::Z;
        }
        axis
    }
}

impl Grow<glam::Vec3A> for AABB {
    /// Grow the box to contain a point
    #[inline]
    fn grow(&mut self, point: glam::Vec3A) {
        self.max = self.max.max(point);
        self.min = self.min.min(point);
    }
}

impl Grow<&AABB> for AABB {
    /// Grow the box to contain another box
    #[inline(always)]
    fn grow(&mut self, aabb: &AABB) {
        self.grow(aabb.min);
        self.grow(aabb.max);
    }
}

impl<T> Grow<&T> for AABB
where
    T: GrowAABB,
{
    /// Grow the box to contain the object
    #[inline(always)]
    fn grow(&mut self, object: &T) {
        object.grow_aabb(self);
    }
}
