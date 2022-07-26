#![feature(test)]
extern crate test;

#[cfg(test)]
mod benchmarks {
    use test::Bencher;

    use std::{
        fs::File,
        io::{BufRead, BufReader},
        iter,
        path::PathBuf,
    };

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
    static RESOLUTION_STEP_X: i32 = 4;
    static RESOLUTION_STEP_Y: i32 = 4;

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
            for y in (0..RESOLUTION_Y).step_by(RESOLUTION_STEP_X as usize) {
                for x in (0..RESOLUTION_X).step_by(RESOLUTION_STEP_Y as usize) {
                    for v in 0..RESOLUTION_STEP_X {
                        for u in 0..RESOLUTION_STEP_Y {
                            let pixel_pos: Vec3A = P0
                                + (P1 - P0) * ((x + v) as f32 / RESOLUTION_X as f32)
                                + (P2 - P0) * ((y + u) as f32 / RESOLUTION_Y as f32);
                            let mut ray = Ray::infinite_ray(
                                CAM_POS,
                                (pixel_pos - CAM_POS).normalize_or_zero(),
                            );

                            triangles.iter().for_each(|tri| {
                                tri.inplace_ray_intersect(&mut ray);
                            });
                        }
                    }
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

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles.clone());
        bvh.build();

        b.iter(|| {
            for y in (0..RESOLUTION_Y).step_by(RESOLUTION_STEP_X as usize) {
                for x in (0..RESOLUTION_X).step_by(RESOLUTION_STEP_Y as usize) {
                    for v in 0..RESOLUTION_STEP_X {
                        for u in 0..RESOLUTION_STEP_Y {
                            let pixel_pos: Vec3A = P0
                                + (P1 - P0) * ((x + v) as f32 / RESOLUTION_X as f32)
                                + (P2 - P0) * ((y + u) as f32 / RESOLUTION_Y as f32);
                            let mut ray = Ray::infinite_ray(
                                CAM_POS,
                                (pixel_pos - CAM_POS).normalize_or_zero(),
                            );

                            bvh.inplace_ray_intersect(&mut ray);
                        }
                    }
                }
            }
        });
    }

    fn load_unity_model() -> Vec<Triangle> {
        let mut unity_file = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        unity_file.push("assets/models/unity.tri");

        let mut f = BufReader::new(File::open(unity_file).unwrap());

        // read the first line and extract the number from it
        let mut num_line = String::new();
        f.read_line(&mut num_line).unwrap();
        let n: usize = num_line.trim().parse().unwrap();

        // preallocate the array and read the data into it
        let mut model: Vec<Triangle> = vec![Default::default(); n];
        for (i, line) in f.lines().enumerate().take(n) {
            let numbers: Vec<f32> = line
                .unwrap()
                .split(char::is_whitespace)
                .map(|num_str| num_str.trim().parse::<f32>().unwrap())
                .collect();
            model[i].vertex0 = Vec3A::new(numbers[0], numbers[1], numbers[2]);
            model[i].vertex1 = Vec3A::new(numbers[3], numbers[4], numbers[5]);
            model[i].vertex2 = Vec3A::new(numbers[6], numbers[7], numbers[8]);
            model[i].compute_centroid();
        }

        model
    }

    #[bench]
    fn simple_bvh_unity_build(b: &mut Bencher) {
        let triangles: Vec<Triangle> = load_unity_model();

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles.clone());

        b.iter(|| {
            bvh.build();
        });
    }

    #[bench]
    fn simple_bvh_unity_intersect(b: &mut Bencher) {
        let triangles: Vec<Triangle> = load_unity_model();

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles.clone());
        bvh.build();

        b.iter(|| {
            for y in (0..RESOLUTION_Y).step_by(RESOLUTION_STEP_X as usize) {
                for x in (0..RESOLUTION_X).step_by(RESOLUTION_STEP_Y as usize) {
                    for v in 0..RESOLUTION_STEP_X {
                        for u in 0..RESOLUTION_STEP_Y {
                            let pixel_pos: Vec3A = P0
                                + (P1 - P0) * ((x + v) as f32 / RESOLUTION_X as f32)
                                + (P2 - P0) * ((y + u) as f32 / RESOLUTION_Y as f32);
                            let mut ray = Ray::infinite_ray(
                                CAM_POS,
                                (pixel_pos - CAM_POS).normalize_or_zero(),
                            );

                            bvh.inplace_ray_intersect(&mut ray);
                        }
                    }
                }
            }
        });
    }
}
