#![allow(dead_code)]

use std::f64;
use ::ray::Ray;
use vec3::Vec3;

#[derive(Clone, Copy)]
pub struct BBox {
    pub min: Vec3,
    pub max: Vec3
}

pub trait BoundingBox {
    fn bounding_box(&self) -> BBox;
}

/// Given a bounding box and a point, compute and return a new BBox that
/// encompasses the point and the space the original box encompassed.
pub fn union_point(b: &BBox, p: &Vec3) -> BBox {
    BBox {
        min: Vec3 {
            x: b.min.x.min(p.x),
            y: b.min.y.min(p.y),
            z: b.min.z.min(p.z)
        },
        max: Vec3 {
            x: b.max.x.max(p.x),
            y: b.max.y.max(p.y),
            z: b.max.z.max(p.z)
        }
    }
}

/// Given two points, compute and return a new BBox that encompasses both points
pub fn union_points(p1: &Vec3, p2: &Vec3) -> BBox {
    BBox {
        min: Vec3 {
            x: p1.x.min(p2.x),
            y: p1.y.min(p2.y),
            z: p1.z.min(p2.z)
        },
        max: Vec3 {
            x: p1.x.max(p2.x),
            y: p1.y.max(p2.y),
            z: p1.z.max(p2.z)
        }
    }
}

/// Given two bounding boxes, compute and return a new BBox that encompasses
/// both spaces the original two boxes encompassed.
pub fn union_bbox(b1: &BBox, b2: &BBox) -> BBox {
    BBox {
        min: Vec3 {
            x: b1.min.x.min(b2.min.x),
            y: b1.min.y.min(b2.min.y),
            z: b1.min.z.min(b2.min.z)
        },
        max: Vec3 {
            x: b1.max.x.max(b2.max.x),
            y: b1.max.y.max(b2.max.y),
            z: b1.max.z.max(b2.max.z)
        }
    }
}

/// Given a vector of prims, compute and return a new BBox that encompasses
/// all finite prims (ie. not including planes) in that vector.
pub fn get_bounds_from_objects(prims: &Vec<Box<BoundingBox+Send+Sync>>) -> BBox {
    let mut max = Vec3 { x: f64::MIN, y: f64::MIN, z: f64::MIN };
    let mut min = Vec3 { x: f64::MAX, y: f64::MAX, z: f64::MAX };

    for prim in prims.iter() {
        let bounding = prim.bounding_box();
        min.x = min.x.min(bounding.min.x);
        min.y = min.y.min(bounding.min.y);
        min.z = min.z.min(bounding.min.z);

        max.x = max.x.max(bounding.max.x);
        max.y = max.y.max(bounding.max.y);
        max.z = max.z.max(bounding.max.z);
    }

    BBox {
        min: min,
        max: max
    }
}

impl BBox {
    pub fn intersects(&self, ray: &Ray) -> bool {
        // Using ray.inverse_dir is an optimisation. Normally, for simplicity we would do
        //
        //     let d = -ray.direction;
        //     tx1 = (self.min.x - o.x) / d.x;
        //     ty1 = (self.min.y - o.y) / d.y;
        //     ...
        //
        // but:
        //
        //    1. div is usually more expensive than mul
        //    2. we are recomputing the inverse of d each time we do an intersection check
        //
        // By caching 1.0 / -ray.direction inside the ray itself we do not need
        // to waste CPU cycles recomputing that every intersection check.
        //
        // See: https://truesculpt.googlecode.com/hg-history/Release%25200.8/Doc/ray_box_intersect.pdf

        let o = ray.origin;

        let (min_bound, max_bound) = if ray.signs[0] {
            (self.min, self.max)
        } else {
            (self.max, self.min)
        };
        let mut t_min = (min_bound.x - o.x) * ray.inverse_dir.x;
        let mut t_max = (max_bound.x - o.x) * ray.inverse_dir.x;

        let (min_y_bound, max_y_bound) = if ray.signs[1] {
            (self.min, self.max)
        } else {
            (self.max, self.min)
        };
        let ty_min = (min_y_bound.y - o.y) * ray.inverse_dir.y;
        let ty_max = (max_y_bound.y - o.y) * ray.inverse_dir.y;

        if t_min > ty_max || ty_min > t_max {
            return false
        }
        if ty_min > t_min {
            t_min = ty_min;
        }
        if ty_max < t_max {
            t_max = ty_max;
        }

        let (min_z_bound, max_z_bound) = if ray.signs[2] {
            (self.min, self.max)
        } else {
            (self.max, self.min)
        };
        let tz_min = (min_z_bound.z - o.z) * ray.inverse_dir.z;
        let tz_max = (max_z_bound.z - o.z) * ray.inverse_dir.z;

        if t_min > tz_max || tz_min > t_max {
            return false
        }
        if tz_min > t_min {
            t_min = tz_min;
        }
        if tz_max < t_max {
            t_max = tz_max;
        }

        // tmin < t1 && tmax > t0
        t_min < ::std::f64::INFINITY && t_max > 0.0
    }

    pub fn overlaps(&self, other: &BBox) -> bool {
        let x = self.max.x >= other.min.x && self.min.x <= other.max.x;
        let y = self.max.y >= other.min.y && self.min.y <= other.max.y;
        let z = self.max.z >= other.min.z && self.min.z <= other.max.z;

        x && y && z
    }

    pub fn inside(&self, p: &Vec3) -> bool {
        p.x >= self.min.x && p.x <= self.max.x &&
        p.y >= self.min.y && p.y <= self.max.y &&
        p.z >= self.min.z && p.z <= self.max.z
    }

    pub fn contains(&self, other: &BBox) -> bool {
        other.min.x >= self.min.x &&
        other.min.y >= self.min.y &&
        other.min.z >= self.min.z &&
        other.max.x <= self.max.x &&
        other.max.y <= self.max.y &&
        other.max.z <= self.max.z
    }

    /// Pad bounding box by a constant factor.
    pub fn expand(&self, delta: f64) -> BBox {
        let delta_vec3 = Vec3 { x: delta, y: delta, z: delta };

        BBox {
            min: self.min - delta_vec3,
            max: self.max + delta_vec3
        }
    }

    /// Returns which axis is the widest. 0: x, 1: y, 2: z
    pub fn max_extent(&self) -> u8 {
        let diag = self.max - self.min;
        if diag.x > diag.y && diag.x > diag.z {
            0
        } else if diag.y > diag.z {
            1
        } else {
            2
        }
    }

    /// Interpolate between corners of the box.
    pub fn lerp(&self, t_x: f64, t_y: f64, t_z: f64) -> Vec3 {
        let diag = self.max - self.min;
        Vec3 {
            x: self.min.x + diag.x * t_x,
            y: self.min.y + diag.y * t_y,
            z: self.min.z + diag.z * t_z
        }
    }

    /// Offset from minimum corner point
    pub fn offset(&self, offset: &Vec3) -> Vec3 {
        let diag = self.max - self.min;
        Vec3 {
            x: (offset.x - self.min.x) / diag.x,
            y: (offset.y - self.min.y) / diag.y,
            z: (offset.z - self.min.z) / diag.z
        }
    }

    pub fn union(&self, other: &BBox) -> BBox {
        union_bbox(self, other)
    }

    pub fn x_len(&self) -> f64 {
        self.max.x - self.min.x
    }

    pub fn y_len(&self) -> f64 {
        self.max.y - self.min.y
    }

    pub fn z_len(&self) -> f64 {
        self.max.z - self.min.z
    }

    pub fn len(&self) -> Vec3 {
        self.max - self.min
    }

    pub fn volume(&self) -> f64 {
        (self.max.x - self.min.x) * 
        (self.max.y - self.min.y) * 
        (self.max.z - self.min.z)
    }
}