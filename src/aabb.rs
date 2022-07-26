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
    /// Grow the box to contain a point
    #[inline]
    pub fn grow(&mut self, point: glam::Vec3A) {
        self.max = self.max.max(point);
        self.min = self.min.min(point);
    }

    /// Grow the box to contain another box
    #[inline]
    pub fn grow_aabb(&mut self, aabb: &AABB) {
        self.grow(aabb.min);
        self.grow(aabb.max);
    }

    /// If the AABB is valid (min <= max)
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.min.le(&self.max)
    }

    pub fn extent(&self) -> glam::Vec3A {
        self.max - self.min
    }

    pub fn area(&self) -> f32 {
        self.extent().length_squared()
    }
}
