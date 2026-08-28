//! Minimal 3-vector — no external math deps, everything the dipole solver needs.

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }
    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    pub fn norm2(self) -> f64 {
        self.dot(self)
    }
    pub fn norm(self) -> f64 {
        self.norm2().sqrt()
    }
    pub fn normalized(self) -> Vec3 {
        let n = self.norm();
        if n > 0.0 { self * (1.0 / n) } else { Vec3::ZERO }
    }
    /// Rotate about +z by angle [rad].
    pub fn rot_z(self, a: f64) -> Vec3 {
        let (s, c) = a.sin_cos();
        Vec3::new(c * self.x - s * self.y, s * self.x + c * self.y, self.z)
    }
}

impl std::ops::Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl std::ops::AddAssign for Vec3 {
    fn add_assign(&mut self, o: Vec3) {
        *self = *self + o;
    }
}
impl std::ops::Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl std::ops::Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, k: f64) -> Vec3 {
        Vec3::new(self.x * k, self.y * k, self.z * k)
    }
}
impl std::ops::Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        self * -1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cross_right_handed() {
        let e = Vec3::new(1.0, 0.0, 0.0).cross(Vec3::new(0.0, 1.0, 0.0));
        assert!((e.z - 1.0).abs() < 1e-15);
    }
    #[test]
    fn rot_z_quarter() {
        let v = Vec3::new(1.0, 0.0, 0.0).rot_z(std::f64::consts::FRAC_PI_2);
        assert!(v.x.abs() < 1e-15 && (v.y - 1.0).abs() < 1e-15);
    }
}
