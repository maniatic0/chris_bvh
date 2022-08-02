extern crate glam;

use std::ops::{Index, IndexMut};

use rand::{
    distributions::{Distribution, Standard},
    Rng,
};

#[derive(Debug, Clone, Copy)]
pub struct Triangle {
    pub vertex0: glam::Vec3A,
    pub vertex1: glam::Vec3A,
    pub vertex2: glam::Vec3A,
    pub centroid: glam::Vec3A,
}

impl Triangle {
    /// Zeroed Triangle
    pub const ZERO: Self = Triangle {
        vertex0: glam::Vec3A::ZERO,
        vertex1: glam::Vec3A::ZERO,
        vertex2: glam::Vec3A::ZERO,
        centroid: glam::Vec3A::ZERO,
    };

    #[inline]
    pub fn new(vertex0: glam::Vec3A, vertex1: glam::Vec3A, vertex2: glam::Vec3A) -> Triangle {
        let mut tri = Triangle {
            vertex0,
            vertex1,
            vertex2,
            centroid: glam::Vec3A::ZERO,
        };
        tri.compute_centroid();
        tri
    }

    #[inline]
    pub fn compute_centroid(&mut self) {
        self.centroid = (self.vertex0 + self.vertex1 + self.vertex2) / 3.0;
    }
}

impl Default for Triangle {
    #[inline]
    fn default() -> Self {
        Self::ZERO
    }
}

impl Distribution<Triangle> for Standard {
    #[inline]
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Triangle {
        Triangle::new(rng.gen(), rng.gen(), rng.gen())
    }
}

impl Index<usize> for Triangle {
    type Output = glam::Vec3A;

    fn index(&self, index: usize) -> &Self::Output {
        assert!(index < 3, "Out of bounds indexing");
        match index {
            0 => &self.vertex0,
            1 => &self.vertex1,
            2 => &self.vertex2,
            _ => panic!("Out of bounds indexing"),
        }
    }
}

impl Index<u8> for Triangle {
    type Output = glam::Vec3A;

    fn index(&self, index: u8) -> &Self::Output {
        assert!(index < 3, "Out of bounds indexing");
        match index {
            0 => &self.vertex0,
            1 => &self.vertex1,
            2 => &self.vertex2,
            _ => panic!("Out of bounds indexing"),
        }
    }
}

impl IndexMut<usize> for Triangle {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        assert!(index < 3, "Out of bounds indexing");
        match index {
            0 => &mut self.vertex0,
            1 => &mut self.vertex1,
            2 => &mut self.vertex2,
            _ => panic!("Out of bounds indexing"),
        }
    }
}

impl IndexMut<u8> for Triangle {
    fn index_mut(&mut self, index: u8) -> &mut Self::Output {
        assert!(index < 3, "Out of bounds indexing");
        match index {
            0 => &mut self.vertex0,
            1 => &mut self.vertex1,
            2 => &mut self.vertex2,
            _ => panic!("Out of bounds indexing"),
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::Triangle;

    use rand::{thread_rng, Rng};

    use approx::*;

    #[test]
    fn compute_centroid() {
        let mut rng = thread_rng();
        let tri: Triangle = rng.gen();
        assert_relative_eq!(
            tri.centroid,
            (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0
        );
    }
}
