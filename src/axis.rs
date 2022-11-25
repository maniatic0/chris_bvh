extern crate glam;

use std::ops::{Index, IndexMut};

use strum::EnumIter;

/// 3D Axis
#[derive(Debug, Clone, Copy, EnumIter)]
#[repr(u8)]
pub enum Axis {
    X = 0,
    Y = 1,
    Z = 2,
}

impl Index<Axis> for glam::Vec3A {
    type Output = f32;

    fn index(&self, axis: Axis) -> &Self::Output {
        match axis {
            Axis::X => &self.x,
            Axis::Y => &self.y,
            Axis::Z => &self.z,
        }
    }
}

impl IndexMut<Axis> for glam::Vec3A {
    fn index_mut(&mut self, axis: Axis) -> &mut Self::Output {
        match axis {
            Axis::X => &mut self.x,
            Axis::Y => &mut self.y,
            Axis::Z => &mut self.z,
        }
    }
}
