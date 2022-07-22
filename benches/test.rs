// bench.rs
#![feature(test)]
extern crate test;

#[cfg(test)]
mod benchmarks {
    use test::Bencher;

    use std::iter;

    use rand::{thread_rng, Rng};

    use glam::Vec3A;

    use chris_bvh::*;

    static TRIANGLES_NUM: usize = 64;

    #[bench]
    fn ray_triangle_intersect(b: &mut Bencher) {
        let mut rng = thread_rng();
        let triangles: Vec<Triangle> = iter::repeat(0)
            .take(TRIANGLES_NUM)
            .map(|_| {
                let v0 = rng.gen::<Vec3A>() * 9.0 - Vec3A::splat(5.0);
                let v1 = rng.gen();
                let v2 = rng.gen();
                Triangle::new(v0, v1, v2)
            })
            .collect();

        let mut ray = Ray::default();

        triangles.iter().for_each(|tri| {
            b.iter(|| tri.ray_intersect(&mut ray));
        });
    }
}
