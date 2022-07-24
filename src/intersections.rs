use crate::{Ray, Triangle};

/// Objects capable of being intersected by a ray in place
pub trait InPlaceRayIntersect {
    fn inplace_ray_intersect(&self, ray: &mut Ray);
}

/// Epsilon used for ray intersections
pub const RAY_INTERSECT_EPSILON: f32 = 0.0001;

/// Intersect a triangle with a ray, then store the intersection result in the ray
pub fn inplace_ray_triangle_intersect(tri: &Triangle, ray: &mut Ray) {
    let edge1 = tri.vertex1 - tri.vertex0;
    let edge2 = tri.vertex2 - tri.vertex0;
    let h = ray.direction.cross(edge2);
    let a = edge1.dot(h);
    if a > -RAY_INTERSECT_EPSILON && a < RAY_INTERSECT_EPSILON {
        // ray parallel to triangle
        return;
    }
    let f = 1.0 / a;
    let s = ray.origin - tri.vertex0;
    let u = f * s.dot(h);
    if u < 0.0 || u > 1.0 {
        return;
    }
    let q = s.cross(edge1);
    let v = f * ray.direction.dot(q);
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
mod tests {

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

        assert_abs_diff_eq!(ray.distance, tri.centroid.distance(Vec3A::ZERO), epsilon = RAY_INTERSECT_EPSILON);
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
