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

    static CAM_POS: Vec3A = Vec3A::new(0.0, 0.0, -18.0);
    static P0: Vec3A = Vec3A::new(-1.0, 1.0, -15.0);
    static P1: Vec3A = Vec3A::new(1.0, 1.0, -15.0);
    static P2: Vec3A = Vec3A::new(-1.0, -1.0, -15.0);

    static RESOLUTION_X: i32 = 640;
    static RESOLUTION_Y: i32 = 640;

    #[bench]
    fn no_bvh(b: &mut Bencher) {
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

        b.iter(|| {
            for y in 0..RESOLUTION_Y {
                for x in 0..RESOLUTION_X {
                    let pixel_pos: Vec3A = P0
                        + (P1 - P0) * (x as f32 / RESOLUTION_X as f32)
                        + (P2 - P0) * (y as f32 / RESOLUTION_Y as f32);
                    let mut ray =
                        Ray::infinite_ray(CAM_POS, (pixel_pos - CAM_POS).normalize_or_zero());

                    triangles.iter().for_each(|tri| {
                        tri.inplace_ray_intersect(&mut ray);
                    });
                }
            }
        });
    }

    #[bench]
    fn simple_bvh(b: &mut Bencher) {
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

        let mut bvh = SimpleBVH::default();
        bvh.init(triangles.clone());
        bvh.build();

        b.iter(|| {
            for y in 0..RESOLUTION_Y {
                for x in 0..RESOLUTION_X {
                    let pixel_pos: Vec3A = P0
                        + (P1 - P0) * (x as f32 / RESOLUTION_X as f32)
                        + (P2 - P0) * (y as f32 / RESOLUTION_Y as f32);
                    let mut ray =
                        Ray::infinite_ray(CAM_POS, (pixel_pos - CAM_POS).normalize_or_zero());

                    bvh.inplace_ray_intersect(&mut ray);
                }
            }
        });
    }
}
