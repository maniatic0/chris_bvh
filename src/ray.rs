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
