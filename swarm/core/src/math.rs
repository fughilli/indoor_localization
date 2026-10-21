//! Minimal fixed-size math for the estimator: Vec3, Mat3, unit quaternion,
//! and the NxN covariance helpers. No allocation, f32 throughout.

pub const STATE_DIM: usize = 16;

/// Dense covariance / system matrix over the 16-dim error state.
pub type Cov = [[f32; STATE_DIM]; STATE_DIM];

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub fn new(x: f32, y: f32, z: f32) -> Vec3 {
        Vec3 { x, y, z }
    }

    pub fn from_array(a: &[f32; 3]) -> Vec3 {
        Vec3::new(a[0], a[1], a[2])
    }

    pub fn to_array(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    pub fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }

    pub fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }

    pub fn scale(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }

    pub fn dot(self, o: Vec3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    pub fn norm(self) -> f32 {
        libm::sqrtf(self.dot(self))
    }

    pub fn normalized(self) -> Vec3 {
        let n = self.norm();
        if n > 1e-12 {
            self.scale(1.0 / n)
        } else {
            Vec3::ZERO
        }
    }
}

/// Row-major 3x3 matrix.
pub type Mat3 = [[f32; 3]; 3];

pub const MAT3_IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Skew-symmetric cross-product matrix: skew(v) * w == v x w.
pub fn skew(v: Vec3) -> Mat3 {
    [
        [0.0, -v.z, v.y],
        [v.z, 0.0, -v.x],
        [-v.y, v.x, 0.0],
    ]
}

pub fn mat3_mul_vec(m: &Mat3, v: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
    )
}

pub fn mat3_transpose(m: &Mat3) -> Mat3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

pub fn mat3_scale(m: &Mat3, s: f32) -> Mat3 {
    let mut out = *m;
    for row in out.iter_mut() {
        for e in row.iter_mut() {
            *e *= s;
        }
    }
    out
}

pub fn mat3_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0f32; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, e) in row.iter_mut().enumerate() {
            *e = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    out
}

/// Unit quaternion, scalar-first. Represents the body-to-world rotation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    pub const IDENTITY: Quat = Quat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    pub fn from_array(a: &[f32; 4]) -> Quat {
        Quat { w: a[0], x: a[1], y: a[2], z: a[3] }
    }

    pub fn to_array(self) -> [f32; 4] {
        [self.w, self.x, self.y, self.z]
    }

    /// Small-angle-safe exponential map: rotation vector (axis * angle) -> quat.
    pub fn from_rotvec(v: Vec3) -> Quat {
        let angle = v.norm();
        let (s, w) = if angle < 1e-6 {
            // sin(a/2)/a ~ 1/2 - a^2/48
            (0.5 - angle * angle / 48.0, 1.0 - angle * angle / 8.0)
        } else {
            (libm::sinf(0.5 * angle) / angle, libm::cosf(0.5 * angle))
        };
        Quat { w, x: v.x * s, y: v.y * s, z: v.z * s }
    }

    pub fn mul(self, o: Quat) -> Quat {
        Quat {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
    }

    pub fn normalized(self) -> Quat {
        let n = libm::sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if n < 1e-12 {
            return Quat::IDENTITY;
        }
        Quat { w: self.w / n, x: self.x / n, y: self.y / n, z: self.z / n }
    }

    /// Rotate a body-frame vector into the world frame.
    pub fn rotate(self, v: Vec3) -> Vec3 {
        // v' = v + 2 * q_vec x (q_vec x v + w * v)
        let qv = Vec3::new(self.x, self.y, self.z);
        let t = qv.cross(v).scale(2.0);
        v.add(t.scale(self.w)).add(qv.cross(t))
    }

    /// Rotate a world-frame vector into the body frame.
    pub fn rotate_inv(self, v: Vec3) -> Vec3 {
        Quat { w: self.w, x: -self.x, y: -self.y, z: -self.z }.rotate(v)
    }

    /// Body-to-world rotation matrix.
    pub fn to_mat3(self) -> Mat3 {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ]
    }

    /// Heading (rotation about world +z of the body +x axis), radians.
    pub fn yaw(self) -> f32 {
        let fwd = self.rotate(Vec3::new(1.0, 0.0, 0.0));
        libm::atan2f(fwd.y, fwd.x)
    }
}

pub fn cov_zero() -> Cov {
    [[0.0; STATE_DIM]; STATE_DIM]
}

/// out = a * b for full state-size matrices.
pub fn cov_mul(a: &Cov, b: &Cov) -> Cov {
    let mut out = cov_zero();
    for i in 0..STATE_DIM {
        for k in 0..STATE_DIM {
            let aik = a[i][k];
            if aik == 0.0 {
                continue;
            }
            for j in 0..STATE_DIM {
                out[i][j] += aik * b[k][j];
            }
        }
    }
    out
}

/// P <- F * P * F^T, in one call.
pub fn cov_propagate(p: &Cov, f: &Cov) -> Cov {
    let fp = cov_mul(f, p);
    let mut out = cov_zero();
    for i in 0..STATE_DIM {
        for j in 0..STATE_DIM {
            let mut s = 0.0;
            for k in 0..STATE_DIM {
                s += fp[i][k] * f[j][k];
            }
            out[i][j] = s;
        }
    }
    out
}

/// Force exact symmetry (P <- (P + P^T) / 2) to fight round-off drift.
pub fn cov_symmetrize(p: &mut Cov) {
    for i in 0..STATE_DIM {
        for j in (i + 1)..STATE_DIM {
            let m = 0.5 * (p[i][j] + p[j][i]);
            p[i][j] = m;
            p[j][i] = m;
        }
    }
}

/// Write a 3x3 block of a state-size matrix at (row, col).
pub fn cov_set_block(p: &mut Cov, row: usize, col: usize, block: &Mat3) {
    for (i, brow) in block.iter().enumerate() {
        p[row + i][col..col + 3].copy_from_slice(brow);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_near(a: f32, b: f32, tol: f32) {
        assert!((a - b).abs() < tol, "{} !~ {}", a, b);
    }

    #[test]
    fn quat_rotvec_round_trip() {
        let v = Vec3::new(0.1, -0.2, 0.3);
        let q = Quat::from_rotvec(v);
        let r = q.to_mat3();
        let w = Vec3::new(1.0, 2.0, -0.5);
        let via_q = q.rotate(w);
        let via_m = mat3_mul_vec(&r, w);
        assert_near(via_q.x, via_m.x, 1e-5);
        assert_near(via_q.y, via_m.y, 1e-5);
        assert_near(via_q.z, via_m.z, 1e-5);
        // rotate then rotate_inv is identity
        let back = q.rotate_inv(via_q);
        assert_near(back.x, w.x, 1e-5);
        assert_near(back.y, w.y, 1e-5);
        assert_near(back.z, w.z, 1e-5);
    }

    #[test]
    fn quat_small_angle() {
        let q = Quat::from_rotvec(Vec3::new(1e-8, 0.0, 0.0));
        assert_near(q.w, 1.0, 1e-9);
        let n = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
        assert_near(n, 1.0, 1e-6);
    }

    #[test]
    fn yaw_of_z_rotation() {
        let q = Quat::from_rotvec(Vec3::new(0.0, 0.0, 0.7));
        assert_near(q.yaw(), 0.7, 1e-5);
    }

    #[test]
    fn skew_matches_cross() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(-0.5, 0.4, 2.0);
        let via_m = mat3_mul_vec(&skew(a), b);
        let via_c = a.cross(b);
        assert_near(via_m.x, via_c.x, 1e-6);
        assert_near(via_m.y, via_c.y, 1e-6);
        assert_near(via_m.z, via_c.z, 1e-6);
    }

    #[test]
    fn cov_propagate_identity() {
        let mut p = cov_zero();
        let mut f = cov_zero();
        for i in 0..STATE_DIM {
            p[i][i] = (i + 1) as f32;
            f[i][i] = 1.0;
        }
        let out = cov_propagate(&p, &f);
        assert_eq!(out, p);
    }
}
