#![feature(test)]
extern crate test;

#[cfg(test)]
mod benchmarks {
    use test::Bencher;

    use std::{
        f32::consts::PI,
        fs::File,
        io::{BufRead, BufReader},
        iter,
        path::{Path, PathBuf},
        sync::Arc,
    };

    use parking_lot::RwLock;

    use rand::{thread_rng, Rng};

    use glam::Vec3A;

    use rayon::{prelude::*, ThreadPoolBuilder};

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

        b.bytes = (RESOLUTION_Y * RESOLUTION_X) as u64;
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

        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(triangles));

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles);
        bvh.build();

        b.bytes = (RESOLUTION_Y * RESOLUTION_X) as u64;
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

    fn load_tri_model<P>(path: P) -> Vec<Triangle>
    where
        P: AsRef<Path>,
    {
        let mut f = BufReader::new(File::open(path).unwrap());

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

    fn load_unity_model() -> Vec<Triangle> {
        let mut unity_file = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        unity_file.push("assets/models/unity.tri");

        load_tri_model(unity_file)
    }

    #[bench]
    fn simple_bvh_unity_build(b: &mut Bencher) {
        let triangles: Vec<Triangle> = load_unity_model();
        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(triangles));

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles);

        b.iter(|| {
            bvh.build();
        });
    }

    #[bench]
    fn simple_bvh_unity_intersect(b: &mut Bencher) {
        let triangles: Vec<Triangle> = load_unity_model();
        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(triangles));

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles);
        bvh.build();

        b.bytes = (RESOLUTION_Y * RESOLUTION_X) as u64;
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

    fn load_bigben_model() -> Vec<Triangle> {
        let mut bigben_file = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        bigben_file.push("assets/models/bigben.tri");

        load_tri_model(bigben_file)
    }

    fn animate_bigben(time: f32, original: &Vec<Triangle>, animation: &mut [Triangle]) -> f32 {
        assert_eq!(
            original.len(),
            animation.len(),
            "Different number of triangles"
        );
        let time = time + 0.05;
        let time = if time > PI * 2.0 {
            time - 2.0 * PI
        } else {
            time
        };

        let a = time.sin() * 0.5;

        for i in 0..original.len() {
            for j in 0_u8..3 {
                let o = original[i][j];
                let s = a * (o.y - 0.2) * 0.2;
                let x = o.x * s.cos() - o.y * s.sin();
                let y = o.x * s.sin() - o.y * s.cos();
                animation[i][j] = Vec3A::new(x, y, o.z);
            }
        }

        time
    }

    #[bench]
    fn simple_bvh_bigben_build(b: &mut Bencher) {
        let triangles: Vec<Triangle> = load_bigben_model();
        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(triangles));

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles);

        b.iter(|| {
            bvh.build();
        });
    }

    #[bench]
    fn simple_bvh_bigben_refit(b: &mut Bencher) {
        let triangles: Vec<Triangle> = load_bigben_model();
        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(triangles));

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles);
        bvh.build();

        b.iter(|| {
            bvh.refit();
        });
    }

    #[bench]
    fn simple_bvh_bigben_intersect_singlethread(b: &mut Bencher) {
        let triangles: Vec<Triangle> = load_bigben_model();
        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(triangles));

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles);
        bvh.build();

        let tile_num = (RESOLUTION_Y / RESOLUTION_STEP_Y) * (RESOLUTION_X / RESOLUTION_STEP_X);

        let tile_width = RESOLUTION_X / RESOLUTION_STEP_X;

        let p0 = Vec3A::new(-1.0, 1.0, 2.0);
        let p1 = Vec3A::new(1.0, 1.0, 2.0);
        let p2 = Vec3A::new(-1.0, -1.0, 2.0);

        b.bytes = (RESOLUTION_Y * RESOLUTION_X) as u64;
        b.iter(|| {
            (0..tile_num).into_iter().for_each(|tile| {
                let x = tile % tile_width;
                let y = tile / tile_width;

                for v in 0..RESOLUTION_STEP_X {
                    for u in 0..RESOLUTION_STEP_Y {
                        let pixel_pos: Vec3A = P0
                            + (p1 - p0) * ((x + v) as f32 / RESOLUTION_X as f32)
                            + (p2 - p0) * ((y + u) as f32 / RESOLUTION_Y as f32);
                        let mut ray =
                            Ray::infinite_ray(CAM_POS, (pixel_pos - CAM_POS).normalize_or_zero());

                        bvh.inplace_ray_intersect(&mut ray);
                    }
                }
            });
        });
    }

    #[bench]
    fn simple_bvh_bigben_intersect_multithread(b: &mut Bencher) {
        let triangles: Vec<Triangle> = load_bigben_model();
        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(triangles));

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles);
        bvh.build();

        let tile_num = (RESOLUTION_Y / RESOLUTION_STEP_Y) * (RESOLUTION_X / RESOLUTION_STEP_X);

        let tile_width = RESOLUTION_X / RESOLUTION_STEP_X;

        let p0 = Vec3A::new(-1.0, 1.0, 2.0);
        let p1 = Vec3A::new(1.0, 1.0, 2.0);
        let p2 = Vec3A::new(-1.0, -1.0, 2.0);

        let pool = ThreadPoolBuilder::new().num_threads(2).build().unwrap();

        b.bytes = (RESOLUTION_Y * RESOLUTION_X) as u64;
        b.iter(|| {
            pool.install(|| {
                (0..tile_num).into_par_iter().for_each(|tile| {
                    let x = tile % tile_width;
                    let y = tile / tile_width;

                    for v in 0..RESOLUTION_STEP_X {
                        for u in 0..RESOLUTION_STEP_Y {
                            let pixel_pos: Vec3A = P0
                                + (p1 - p0) * ((x + v) as f32 / RESOLUTION_X as f32)
                                + (p2 - p0) * ((y + u) as f32 / RESOLUTION_Y as f32);
                            let mut ray = Ray::infinite_ray(
                                CAM_POS,
                                (pixel_pos - CAM_POS).normalize_or_zero(),
                            );

                            bvh.inplace_ray_intersect(&mut ray);
                        }
                    }
                });
            });
        });
    }

    #[bench]
    fn simple_bvh_bigben_intersect_multithread_unsafe(b: &mut Bencher) {
        let triangles: Vec<Triangle> = load_bigben_model();
        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(triangles));

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles);
        bvh.build();

        let tile_num = (RESOLUTION_Y / RESOLUTION_STEP_Y) * (RESOLUTION_X / RESOLUTION_STEP_X);

        let tile_width = RESOLUTION_X / RESOLUTION_STEP_X;

        let p0 = Vec3A::new(-1.0, 1.0, 2.0);
        let p1 = Vec3A::new(1.0, 1.0, 2.0);
        let p2 = Vec3A::new(-1.0, -1.0, 2.0);

        //let pool = ThreadPoolBuilder::new().num_threads(2).build().unwrap();
        let pool = ThreadPoolBuilder::new().build().unwrap();

        let triangles_arc = bvh.triangles().clone();
        let triangles_lock = triangles_arc.read();
        let triangles: &Vec<Triangle> = &triangles_lock;

        b.bytes = (RESOLUTION_Y * RESOLUTION_X) as u64;
        b.iter(|| {
            pool.install(|| {
                (0..tile_num).into_par_iter().for_each(|tile| {
                    let x = tile % tile_width;
                    let y = tile / tile_width;

                    for v in 0..RESOLUTION_STEP_X {
                        for u in 0..RESOLUTION_STEP_Y {
                            let pixel_pos: Vec3A = P0
                                + (p1 - p0) * ((x + v) as f32 / RESOLUTION_X as f32)
                                + (p2 - p0) * ((y + u) as f32 / RESOLUTION_Y as f32);
                            let mut ray = Ray::infinite_ray(
                                CAM_POS,
                                (pixel_pos - CAM_POS).normalize_or_zero(),
                            );

                            unsafe {
                                bvh.unsafe_inplace_ray_intersect(triangles, &mut ray);
                            }
                        }
                    }
                });
            });
        });
    }

    #[bench]
    fn simple_bvh_bigben_intersect_animation(b: &mut Bencher) {
        let triangles: Vec<Triangle> = load_bigben_model();
        let triangles: Arc<RwLock<Vec<Triangle>>> = Arc::new(RwLock::new(triangles));

        let mut bvh = SimpleBVH::<CompiledBinnedSAHStrategy>::default();
        bvh.init(triangles);
        bvh.build();

        let tile_num = (RESOLUTION_Y / RESOLUTION_STEP_Y) * (RESOLUTION_X / RESOLUTION_STEP_X);

        let tile_width = RESOLUTION_X / RESOLUTION_STEP_X;

        let p0 = Vec3A::new(-1.0, 1.0, 2.0);
        let p1 = Vec3A::new(1.0, 1.0, 2.0);
        let p2 = Vec3A::new(-1.0, -1.0, 2.0);

        //let pool = ThreadPoolBuilder::new().num_threads(2).build().unwrap();
        let pool = ThreadPoolBuilder::new().build().unwrap();

        let triangles_arc = bvh.triangles().clone();
        let original_tris = triangles_arc.read().clone();

        let mut time = 0.0;

        b.bytes = (RESOLUTION_Y * RESOLUTION_X) as u64;
        b.iter(|| {
            {
                let mut triangles_lock = triangles_arc.write();
                let triangles: &mut Vec<Triangle> = triangles_lock.as_mut();
                time = animate_bigben(time, &original_tris, triangles);
            }

            let triangles_lock = triangles_arc.read();
            let triangles: &Vec<Triangle> = &triangles_lock;

            pool.install(|| {
                (0..tile_num).into_par_iter().for_each(|tile| {
                    let x = tile % tile_width;
                    let y = tile / tile_width;

                    for v in 0..RESOLUTION_STEP_X {
                        for u in 0..RESOLUTION_STEP_Y {
                            let pixel_pos: Vec3A = P0
                                + (p1 - p0) * ((x + v) as f32 / RESOLUTION_X as f32)
                                + (p2 - p0) * ((y + u) as f32 / RESOLUTION_Y as f32);
                            let mut ray = Ray::infinite_ray(
                                CAM_POS,
                                (pixel_pos - CAM_POS).normalize_or_zero(),
                            );

                            unsafe {
                                bvh.unsafe_inplace_ray_intersect(triangles, &mut ray);
                            }
                        }
                    }
                });
            });
        });
    }
}
