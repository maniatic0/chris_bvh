use glam::Vec3A;

use crate::{Ray, Triangle, AABB};

/// Objects capable of being intersected by a ray that store the distance in the ray
pub trait InPlaceRayIntersect {
    fn inplace_ray_intersect(&self, ray: &mut Ray);
}

/// Objects capable of being intersected by a ray that return the distance from the function
pub trait RayIntersect {
    fn ray_intersect(&self, ray: &Ray) -> f32;
}

/// Objects capable of being intersected by a ray that only return if there was an intersection
pub trait FastRayIntersect {
    fn fast_ray_intersect(&self, ray: &Ray) -> bool;
}

/// Epsilon used for ray intersections
pub const RAY_INTERSECT_EPSILON: f32 = 0.0001;

/// Intersect a triangle with a ray, then store the intersection result in the ray
pub fn inplace_ray_triangle_intersect(tri: &Triangle, ray: &mut Ray) {
    let edge1 = tri.vertex1 - tri.vertex0;
    let edge2 = tri.vertex2 - tri.vertex0;
    let h = ray.direction().cross(edge2);
    let a = edge1.dot(h);
    if a > -RAY_INTERSECT_EPSILON && a < RAY_INTERSECT_EPSILON {
        // ray parallel to triangle
        return;
    }
    let f = 1.0 / a;
    let s = ray.origin - tri.vertex0;
    let u = f * s.dot(h);
    if u < 0.0 && 1.0 < u {
        return;
    }
    let q = s.cross(edge1);
    let v = f * ray.direction().dot(q);
    if v < 0.0 || u + v > 1.0 {
        return;
    }
    let t = f * edge2.dot(q);
    if t > RAY_INTERSECT_EPSILON {
        ray.distance = ray.distance.min(t);
    }
}

impl InPlaceRayIntersect for Triangle {
    #[inline]
    fn inplace_ray_intersect(&self, ray: &mut Ray) {
        inplace_ray_triangle_intersect(self, ray);
    }
}

#[cfg(test)]
mod triangle_tests {

    use rand::{thread_rng, Rng};

    use glam::Vec3A;

    use approx::*;

    use crate::*;

    #[test]
    fn ray_triangle_intersect() {
        let mut rng = thread_rng();
        let tri: Triangle = {
            let v0 = rng.gen::<Vec3A>() * 9.0 - Vec3A::splat(5.0);
            let v1 = rng.gen();
            let v2 = rng.gen();
            Triangle::new(v0, v1, v2)
        };

        let mut ray = Ray::infinite_ray(Vec3A::ZERO, tri.centroid.normalize_or_zero());

        tri.inplace_ray_intersect(&mut ray);

        assert_abs_diff_eq!(
            ray.distance,
            tri.centroid.distance(Vec3A::ZERO),
            epsilon = RAY_INTERSECT_EPSILON
        );
    }

    #[test]
    fn ray_triangle_no_intersect() {
        let mut rng = thread_rng();
        let tri: Triangle = {
            let v0 = rng.gen::<Vec3A>() * 9.0 - Vec3A::splat(5.0);
            let v1 = rng.gen();
            let v2 = rng.gen();
            Triangle::new(v0, v1, v2)
        };

        let mut ray = Ray::infinite_ray(Vec3A::ZERO, -tri.centroid.normalize_or_zero());

        tri.inplace_ray_intersect(&mut ray);

        assert!(ray.distance.is_infinite());
    }
}

pub fn aabb_slab_test(aabb: &AABB, ray: &Ray) -> bool {
    assert!(aabb.is_valid(), "This test doesn't work with invalid boxes");

    let t1 = (aabb.min - ray.origin) * ray.direction_recip();
    let t2 = (aabb.max - ray.origin) * ray.direction_recip();

    let tmin = t1.min(t2);
    let tmax = t1.max(t2);

    let ttmin = tmin.max_element();
    let ttmax = tmax.min_element();

    let tmax_gt_zero = tmax.cmpgt(Vec3A::ZERO).any();

    // ttmax > 0.0 && ttmax >= ttmin && ttmin < ray.distance
    tmax_gt_zero && ttmax >= ttmin && ttmin < ray.distance
}

impl FastRayIntersect for AABB {
    #[inline(always)]
    fn fast_ray_intersect(&self, ray: &Ray) -> bool {
        aabb_slab_test(self, ray)
    }
}

#[cfg(test)]
mod aabb_slab_tests {

    use glam::Vec3A;

    use crate::*;

    #[test]
    fn ray_aabb_intersect_inside() {
        let mut aabb = AABB::default();

        // Box from [-2,-2,-2] to [2,2,2]
        aabb.max = Vec3A::ONE * 2.0;
        aabb.min = -aabb.max;

        let ray = Ray::infinite_ray(Vec3A::ONE, aabb.center().normalize_or_zero());

        assert!(aabb.fast_ray_intersect(&ray), "Should intersect");
    }

    #[test]
    fn ray_aabb_intersect_outside() {
        let mut aabb = AABB::default();

        // Box from [-2,-2,-2] to [2,2,2]
        aabb.max = Vec3A::ONE * 2.0;
        aabb.min = -aabb.max;

        let ori = aabb.max * 2.0;
        let dir = (aabb.center() - ori).normalize_or_zero();

        let ray = Ray::infinite_ray(ori, dir);

        assert!(aabb.fast_ray_intersect(&ray), "Should intersect");
    }

    #[test]
    fn ray_aabb_no_intersect() {
        let mut aabb = AABB::default();

        // Box from [-2,-2,-2] to [2,2,2]
        aabb.max = Vec3A::ONE * 2.0;
        aabb.min = -aabb.max;

        let ori = aabb.max * 2.0;
        let dir = (aabb.center() - ori).normalize_or_zero();

        let ray = Ray::infinite_ray(ori, -dir);

        assert!(!aabb.fast_ray_intersect(&ray), "Should not intersect");
    }
}

pub fn aabb_slab_distance(aabb: &AABB, ray: &Ray) -> f32 {
    assert!(aabb.is_valid(), "This test doesn't work with invalid boxes");
    let t1 = (aabb.min - ray.origin) * ray.direction_recip();
    let t2 = (aabb.max - ray.origin) * ray.direction_recip();

    let tmin = t1.min(t2);
    let tmax = t1.max(t2);

    let ttmin = tmin.max_element();
    let ttmax = tmax.min_element();

    if ttmax >= ttmin && ttmin < ray.distance && ttmax > 0.0 {
        ttmin
    } else {
        f32::INFINITY
    }
}

impl RayIntersect for AABB {
    fn ray_intersect(&self, ray: &Ray) -> f32 {
        aabb_slab_distance(self, ray)
    }
}
