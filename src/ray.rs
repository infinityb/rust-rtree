use ::vec3::Vec3;

pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,

    pub inverse_dir: Vec3, // This is used to optimise ray-bbox intersection checks
    pub signs: [bool; 3] // Handle degenerate case in bbox intersection
}


impl Ray {
	pub fn new(origin: Vec3, direction: Vec3) -> Ray {
        let inv_x = 1.0 / direction.x;
        let inv_y = 1.0 / direction.y;
        let inv_z = 1.0 / direction.z;

        Ray {
            origin: origin,
            direction: direction,
            inverse_dir: Vec3 {
                x: inv_x,
                y: inv_y,
                z: inv_z
            },
            signs: [
                inv_x > 0.0,
                inv_y > 0.0,
                inv_z > 0.0
            ]
        }
    }
}