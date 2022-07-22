use crate::{Ray, Triangle};

/// Objects capable of being intersected by a ray
pub trait RayIntersect {
    fn ray_intersect(&self, ray: &mut Ray);
}

// Epsilon used for ray intersections
pub const RAY_INTERSECT_EPSILON: f32 = 0.0001;

pub fn ray_triangle_intersect(tri: &Triangle, ray: &mut Ray) {
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

impl RayIntersect for Triangle {
    fn ray_intersect(&self, ray: &mut Ray) {
        ray_triangle_intersect(self, ray);
    }
}
