use std::cmp;
use std::fmt;
use std::ops::{Add, Mul, Div, Neg, Sub};

#[derive(Clone, Copy)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64
}

impl Vec3 {
    pub fn xyz(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x: x, y: y, z: z }
    }
    
    pub fn zero() -> Vec3 {
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        }
    }

    pub fn one() -> Vec3 {
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0
        }
    }

    pub fn len(&self) -> f64 {
        (self.x * self.x +
         self.y * self.y +
         self.z * self.z).sqrt()
    }

    pub fn dot(&self, other: &Vec3) -> f64 {
        self.x * other.x +
        self.y * other.y +
        self.z * other.z
    }

    pub fn cross(&self, other: &Vec3) -> Vec3 {
        Vec3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x
        }
    }

    pub fn unit(&self) -> Vec3 {
        let len = self.len();

        Vec3 {
            x: self.x / len,
            y: self.y / len,
            z: self.z / len
        }
    }

    pub fn scale(&self, scalar: f64) -> Vec3 {
        Vec3 {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar
        }
    }

    /// V, N should be unit vectors
    ///
    ///  ^  ^
    /// V \ | N
    ///    \|
    /// =========
    pub fn reflect(v: &Vec3, n: &Vec3) -> Vec3 {
        n.scale(2.0 * (n.dot(v))) - *v
    }

    /// V, N should be unit vectors
    /// ior: Refractive index
    /// inside: Is the ray inside an object (ie. going out of an object)?
    pub fn refract(v: &Vec3, n: &Vec3, ior: f64, inside: bool) -> Option<Vec3> {
        let (n1, n2, n_dot_v, nn): (f64, f64, _, _) = if !inside {
            (1.0, ior, n.dot(v), *n)
        } else {
            (ior, 1.0, -n.dot(v), -*n)
        };

        let ratio = n1 / n2;
        let disc = 1.0 - ((ratio * ratio) * (1.0 - n_dot_v * n_dot_v));

        if disc < 0.0 {
            None // Total internal reflection
        } else {
            Some(v.scale(-ratio) + nn.scale(ratio * n_dot_v - disc.sqrt()))
        }
    }

    pub fn lerp(v1: &Vec3, v2: &Vec3, alpha: f64) -> Vec3 {
        Vec3 {
            x: v1.x + (v2.x - v1.x) * alpha,
            y: v1.y + (v2.y - v1.y) * alpha,
            z: v1.z + (v2.z - v1.z) * alpha
        }
    }

    pub fn clamp(&self, min: f64, max: f64) -> Vec3 {
        Vec3 {
            x: self.x.max(min).min(max),
            y: self.y.max(min).min(max),
            z: self.z.max(min).min(max)
        }
    }
}

impl Add for Vec3 {
    type Output = Vec3;

    fn add(self, other: Vec3) -> Vec3 {
        Vec3 {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z
        }
    }
}

impl Add<f64> for Vec3 {
    type Output = Vec3;

    fn add(self, other: f64) -> Vec3 {
        Vec3 {
            x: self.x + other,
            y: self.y + other,
            z: self.z + other
        }
    }
}

impl Sub for Vec3 {
    type Output = Vec3;

    fn sub(self, other: Vec3) -> Vec3 {
        Vec3 {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z
        }
    }
}

impl Sub<f64> for Vec3 {
    type Output = Vec3;

    fn sub(self, other: f64) -> Vec3 {
        Vec3 {
            x: self.x - other,
            y: self.y - other,
            z: self.z - other
        }
    }
}

impl Mul for Vec3 {
    type Output = Vec3;

    fn mul(self, other: Vec3) -> Vec3 {
        Vec3 {
            x: self.x * other.x,
            y: self.y * other.y,
            z: self.z * other.z
        }
    }
}

impl Mul<f64> for Vec3 {
    type Output = Vec3;

    fn mul(self, other: f64) -> Vec3 {
        Vec3 {
            x: self.x * other,
            y: self.y * other,
            z: self.z * other
        }
    }
}

impl Div for Vec3 {
    type Output = Vec3;

    fn div(self, other: Vec3) -> Vec3 {
        Vec3 {
            x: self.x / other.x,
            y: self.y / other.y,
            z: self.z / other.z
        }
    }
}

impl Div<f64> for Vec3 {
    type Output = Vec3;

    fn div(self, other: f64) -> Vec3 {
        Vec3 {
            x: self.x / other,
            y: self.y / other,
            z: self.z / other
        }
    }
}

impl Neg for Vec3 {
    type Output = Vec3;

    fn neg(self) -> Vec3 {
        Vec3 {
            x: -self.x,
            y: -self.y,
            z: -self.z
        }
    }
}

impl cmp::PartialEq for Vec3 {
    fn eq(&self, other: &Vec3) -> bool {
        self.x == other.x && self.y == other.y && self.z == other.z
    }

    fn ne(&self, other: &Vec3) -> bool {
        !(self.eq(other))
    }
}

impl fmt::Debug for Vec3 {
    fn fmt(&self, f: &mut  fmt::Formatter) -> fmt::Result {
        write!(f, "({}, {}, {})", self.x, self.y, self.z)
    }
}

macro_rules! vec3 {
    ($x:expr, $y:expr, $z:expr) => {
        Vec3 { x: $x, y: $y, z: $z }
    };

    ($s:expr) => {
        Vec3 { x: $s, y: $s, z: $s }
    }
}
