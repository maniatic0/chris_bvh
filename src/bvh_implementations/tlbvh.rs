use crate::{
    object_pool::{Handle, HandlePool},
    FastRayIntersect, Grow, Ray, RayIntersect, AABB, BVH,
};

#[derive(Debug, Clone, Copy)]
struct BVHInstance {
    pub inv_transform: glam::Affine3A,
    pub bounds: AABB,
    pub bvh_handle: Handle,
}

impl Default for BVHInstance {
    fn default() -> Self {
        Self {
            inv_transform: glam::Affine3A::IDENTITY,
            bounds: Default::default(),
            bvh_handle: Default::default(),
        }
    }
}

impl BVHInstance {
    #[inline]
    pub fn set_bvh<BVHType>(
        &mut self,
        bvh_handle: &Handle,
        bvh: &BVHType,
        transform: &glam::Affine3A,
    ) where
        BVHType: BVH,
    {
        self.bvh_handle = *bvh_handle;
        self.inv_transform = transform.inverse();

        let bvh_bounds = bvh.bounds();

        self.bounds = AABB::default();

        for i in 0..8 {
            self.bounds
                .grow(transform.transform_point3a(glam::Vec3A::new(
                    if i & 1 != 0 {
                        bvh_bounds.max.x
                    } else {
                        bvh_bounds.min.x
                    },
                    if i & 1 != 0 {
                        bvh_bounds.max.y
                    } else {
                        bvh_bounds.min.y
                    },
                    if i & 1 != 0 {
                        bvh_bounds.max.z
                    } else {
                        bvh_bounds.min.z
                    },
                )));
        }
    }

    pub fn inplace_ray_intersect<BVHType>(&self, bvh: &BVHType, ray: &mut Ray)
    where
        BVHType: BVH,
    {
        // Fast test
        if !self.bounds.fast_ray_intersect(ray) {
            return;
        }

        let mut relative_ray = Ray::new(
            self.inv_transform.transform_point3a(ray.origin),
            self.inv_transform.transform_vector3a(ray.direction()),
            ray.distance,
        );

        bvh.inplace_ray_intersect(&mut relative_ray);

        ray.distance = relative_ray.distance;
    }
}

pub struct TLBVH<BVHType>
where
    BVHType: BVH + Default,
{
    bvhs: HandlePool<BVHType>,
    bvh_instances: HandlePool<BVHInstance>,
}

impl<BVHType> TLBVH<BVHType>
where
    BVHType: BVH + Default,
{
    pub fn add_bvh(&mut self) -> (&mut BVHType, Handle) {
        self.bvhs.add()
    }

    pub fn add_bvh_instance(&mut self, bvh_handle: &Handle, transform: &glam::Affine3A) -> bool {
        let maybe_bvh = self.bvhs.get(bvh_handle);

        match maybe_bvh {
            None => false,
            Some(bvh) => {
                let (instance, _) = self.bvh_instances.add();
                // TODO add the handle to the tree
                instance.set_bvh(bvh_handle, bvh, transform);
                true
            }
        }
    }
}
