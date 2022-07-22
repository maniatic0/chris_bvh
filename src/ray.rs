/// Ray object. Might be a proper ray (distance = infinity) or a line segment (distance is finite)
#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub origin: glam::Vec3A,
    pub direction: glam::Vec3A,
    pub distance: f32,
}

impl Default for Ray {
    fn default() -> Self {
        Self {
            origin: Default::default(),
            direction: glam::Vec3A::new(1.0, 0.0, 0.0),
            distance: 1.0,
        }
    }
}

impl Ray {
    #[inline]
    pub fn new(origin: glam::Vec3A, direction: glam::Vec3A, distance: f32) -> Self {
        Self {
            origin,
            direction,
            distance,
        }
    }

    /// Create a ray with infinite length (a proper ray)
    #[inline]
    pub fn infinite_ray(origin: glam::Vec3A, direction: glam::Vec3A) -> Self {
        Self::new(origin, direction, f32::INFINITY)
    }
}
