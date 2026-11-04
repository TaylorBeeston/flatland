//! Curved (cylinder-slice) panels, WayVR-style. Opt-in via env, set per window by the panel shell:
//!   FLATLAND_CURVE_RADIUS  cylinder radius in m (enables curved mode)
//!   FLATLAND_CURVE_YAW_DEG panel centre angle on the cylinder, + = right (WayVR convention)
//!   FLATLAND_CURVE_ANCHOR  "x,y,z" cylinder axis point in stage space (default 0.008,1.60,0.044)
//!                          or "hmd" to use the head pose (snapped upright) at launch.
//! Visual only: the toplevel is drawn on a curved mesh; input planes stay flat (phase 2).
use glam::{Quat, Vec3, vec3};
use std::{env, path::PathBuf, sync::OnceLock};

pub const DEFAULT_ANCHOR: Vec3 = vec3(0.008, 1.60, 0.044);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Curve {
	pub radius: f32,
	/// radians, + = right
	pub yaw: f32,
	/// None = HMD at launch
	pub anchor: Option<Vec3>,
}

fn parse3(s: &str) -> Option<Vec3> {
	let v: Vec<f32> = s.split(',').map(|p| p.trim().parse().ok()).collect::<Option<_>>()?;
	(v.len() == 3).then(|| vec3(v[0], v[1], v[2]))
}

pub fn curve() -> Option<Curve> {
	static CURVE: OnceLock<Option<Curve>> = OnceLock::new();
	*CURVE.get_or_init(|| {
		let radius: f32 = env::var("FLATLAND_CURVE_RADIUS").ok()?.trim().parse().ok()?;
		if !(0.05..=100.0).contains(&radius) {
			return None;
		}
		let yaw = env::var("FLATLAND_CURVE_YAW_DEG")
			.ok()
			.and_then(|v| v.trim().parse::<f32>().ok())
			.unwrap_or(0.0)
			.to_radians();
		let anchor = match env::var("FLATLAND_CURVE_ANCHOR").ok().as_deref().map(str::trim) {
			Some("hmd") => None,
			Some(s) if !s.is_empty() => Some(parse3(s).unwrap_or(DEFAULT_ANCHOR)),
			_ => Some(DEFAULT_ANCHOR),
		};
		let c = Curve { radius, yaw, anchor };
		tracing::info!(?c, "curved panel mode");
		Some(c)
	})
}

/// Mesh bend factor. Always 1: the panel mesh is baked at exactly the cylinder radius, so every panel
/// lies on the same cylinder the wireframe rings draw (axis at the head anchor). A tighter bend (>1) made
/// the panels diverge from the rings; FLATLAND_CURVE_BEND is now ignored on purpose.
pub fn bend() -> f32 {
	1.0
}
/// radius the mesh is baked at for a `w` m panel on cylinder `r` (arc capped at 175 deg)
pub fn mesh_radius(w: f32, r: f32) -> f32 {
	quantize_radius((r / bend()).max(w / 175f32.to_radians()))
}
/// azimuth from the cylinder centre of a panel edge, relative to the panel centre (panel centre at r)
pub fn edge_half_az(w: f32, r: f32) -> f32 {
	let m = mesh_radius(w, r);
	let th = w / (2.0 * m);
	(m * th.sin()).atan2(r - m * (1.0 - th.cos()))
}
static LIVE_RADIUS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
/// quantize to 2 cm so a radius drag doesn't rebuild the mesh every frame
pub fn quantize_radius(r: f32) -> f32 {
	((r * 50.0).round() / 50.0).max(0.02)
}
/// set by placement whenever the group radius (adjust radius_m) changes
pub fn set_live_radius(r: f32) {
	LIVE_RADIUS.store(r.to_bits(), std::sync::atomic::Ordering::Relaxed);
}

impl Curve {
	/// current cylinder radius (live adjust if placed, else env radius)
	pub fn live_radius(&self) -> f32 {
		let v = f32::from_bits(LIVE_RADIUS.load(std::sync::atomic::Ordering::Relaxed));
		if v > 0.0 { v } else { self.radius }
	}
	/// same layout on radius `r` for a panel `w` m wide: the panel's inner edge (the one facing the gap)
	/// keeps its arc position (yaw * radius), so the gaps stay constant in metres, and the centre is set
	/// from that edge by the bent mesh's real half-angle so tighter bends never overlap.
	pub fn on_radius(&self, r: f32, w: f32) -> Curve {
		let s0 = self.yaw * self.radius;
		let sign = if s0 < 0.0 { 1.0 } else { -1.0 }; // inner edge is toward the group centre
		let inner = (s0 + sign * w / 2.0) / r;
		Curve { radius: r, yaw: inner - sign * edge_half_az(w, r), anchor: self.anchor }
	}
	/// Panel centre pose in the anchor frame (-Z forward, +Y up).
	pub fn local_pose(&self) -> (Vec3, Quat) {
		let (s, c) = self.yaw.sin_cos();
		(
			vec3(s * self.radius, 0.0, -c * self.radius),
			Quat::from_rotation_y(-self.yaw),
		)
	}
}

/// Write (once) a GLB holding a "Panel" node: a cylinder slice `width` m of arc, `height` m tall,
/// centred at the origin, edges bending toward -Z around an axis at (0, 0, -radius), i.e. concave to the user.
/// UVs follow Flatland's panel.glb (+X -> u=0, +Y -> v=0). Double-sided.
pub fn curved_panel_glb(width: f32, height: f32, radius: f32) -> Option<PathBuf> {
	// quantize to 1 cm so live resizes don't rebuild the model every frame
	let q = |v: f32| (v * 100.0).round().max(1.0) / 100.0;
	let (width, height) = (q(width), q(height));
	let mm = |v: f32| (v * 1000.0).round() as i64;
	let dir = cache_dir();
	let path = dir.join(format!("curve3-{}x{}-r{}.glb", mm(width), mm(height), mm(radius)));
	if path.exists() {
		return Some(path);
	}
	std::fs::create_dir_all(&dir).ok()?;
	let arc = width / radius;
	let segs = ((arc.to_degrees() / 1.5).ceil() as usize).clamp(8, 600);
	let (mut pos, mut uv, mut nrm, mut idx) = (vec![], vec![], vec![], vec![]);
	for i in 0..=segs {
		let t = i as f32 / segs as f32;
		let th = (t - 0.5) * arc;
		let (s, c) = th.sin_cos();
		// edges wrap toward -Z: concave as seen by the user (measured: +Z read as convex in headset)
		let (x, z) = (radius * s, -radius * (1.0 - c));
		for (y, v) in [(height / 2.0, 0.0f32), (-height / 2.0, 1.0)] {
			pos.extend([x, y, z]);
			uv.extend([1.0 - t, v]);
			nrm.extend([s, 0.0, c]); // centre normal stays +Z like panel.glb
		}
	}
	for i in 0..segs as u16 {
		let (a, b, c, d) = (2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 3);
		idx.extend([a, b, c, c, b, d]); // one winding
		idx.extend([a, c, b, c, d, b]); // and the other
	}
	let mut bin: Vec<u8> = vec![];
	let f = |v: &Vec<f32>, bin: &mut Vec<u8>| v.iter().for_each(|x| bin.extend(x.to_le_bytes()));
	f(&pos, &mut bin);
	let uv_off = bin.len();
	f(&uv, &mut bin);
	let n_off = bin.len();
	f(&nrm, &mut bin);
	let i_off = bin.len();
	idx.iter().for_each(|x| bin.extend(x.to_le_bytes()));
	while bin.len() % 4 != 0 {
		bin.push(0);
	}
	let nv = pos.len() / 3;
	let zmax = radius * (1.0 - (arc / 2.0).cos());
	let xmax = if arc / 2.0 >= std::f32::consts::FRAC_PI_2 { radius } else { radius * (arc / 2.0).sin() };
	let json = format!(
		r#"{{"asset":{{"version":"2.0","generator":"flatland curve.rs"}},"scene":0,"scenes":[{{"nodes":[0]}}],"nodes":[{{"mesh":0,"name":"Panel"}}],"meshes":[{{"name":"Panel","primitives":[{{"attributes":{{"POSITION":0,"TEXCOORD_0":1,"NORMAL":2}},"indices":3}}]}}],"accessors":[{{"bufferView":0,"componentType":5126,"count":{nv},"type":"VEC3","min":[{nx},{ny},{nz}],"max":[{xmax},{hy},0]}},{{"bufferView":1,"componentType":5126,"count":{nv},"type":"VEC2"}},{{"bufferView":2,"componentType":5126,"count":{nv},"type":"VEC3"}},{{"bufferView":3,"componentType":5123,"count":{ni},"type":"SCALAR"}}],"bufferViews":[{{"buffer":0,"byteOffset":0,"byteLength":{uv_off},"target":34962}},{{"buffer":0,"byteOffset":{uv_off},"byteLength":{uvl},"target":34962}},{{"buffer":0,"byteOffset":{n_off},"byteLength":{nl},"target":34962}},{{"buffer":0,"byteOffset":{i_off},"byteLength":{il},"target":34963}}],"buffers":[{{"byteLength":{bl}}}]}}"#,
		nx = -xmax,
		nz = -zmax,
		ny = -height / 2.0,
		hy = height / 2.0,
		ni = idx.len(),
		uvl = n_off - uv_off,
		nl = i_off - n_off,
		il = idx.len() * 2,
		bl = bin.len(),
	);
	let mut json = json.into_bytes();
	while json.len() % 4 != 0 {
		json.push(b' ');
	}
	let total = 12 + 8 + json.len() + 8 + bin.len();
	let mut out = Vec::with_capacity(total);
	out.extend(b"glTF");
	out.extend(2u32.to_le_bytes());
	out.extend((total as u32).to_le_bytes());
	out.extend((json.len() as u32).to_le_bytes());
	out.extend(b"JSON");
	out.extend(&json);
	out.extend((bin.len() as u32).to_le_bytes());
	out.extend(b"BIN\0");
	out.extend(&bin);
	let tmp = path.with_extension(format!("tmp{}", std::process::id()));
	std::fs::write(&tmp, out).ok()?;
	std::fs::rename(&tmp, &path).ok()?;
	tracing::info!(?path, segs, "wrote curved panel mesh");
	Some(path)
}

/// $XDG_RUNTIME_DIR/flatland-curve: meshes, shared head anchor, recenter request.
pub fn cache_dir() -> PathBuf {
	PathBuf::from(env::var("XDG_RUNTIME_DIR").unwrap_or_else(|_| "/tmp".into())).join("flatland-curve")
}
