/// Ray object. Might be a proper ray (distance = infinity) or a line segment (distance is finite)
#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub origin: glam::Vec3A,
    direction: glam::Vec3A,
    direction_recip: glam::Vec3A,
    pub distance: f32,
}

impl Default for Ray {
    fn default() -> Self {
        Self {
            origin: Default::default(),
            direction: glam::Vec3A::X,
            direction_recip: glam::Vec3A::X.recip(),
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
            direction_recip: direction.recip(),
            distance,
        }
    }

    /// Create a ray with infinite length (a proper ray)
    #[inline]
    pub fn infinite_ray(origin: glam::Vec3A, direction: glam::Vec3A) -> Self {
        Self::new(origin, direction, f32::INFINITY)
    }

    #[inline]
    pub fn direction(&self) -> glam::Vec3A {
        self.direction
    }

    #[inline]
    pub fn direction_recip(&self) -> glam::Vec3A {
        self.direction_recip
    }

    #[inline]
    pub fn set_direction(&mut self, direction: glam::Vec3A) {
        self.direction = direction;
        self.direction_recip = direction.recip();
    }
}
