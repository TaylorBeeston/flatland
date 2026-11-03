use glam::{Quat, Vec3, vec3};
use stardust_xr_asteroids::{Context, CreateInnerInfo, CustomElement, ValidState};
use stardust_xr_fusion::{
	Error, Result,
	client::{Client, ClientHandler},
	spatial::{Spatial, SpatialRef, SpatialRefOpError, Transform},
	tracked::{Tracked, TrackedExt},
	types::CreateError,
};
use std::{
	env,
	f32::consts::PI,
	path::{Path, PathBuf},
	sync::Arc,
	time::{Duration, Instant, SystemTime},
};
use tracing::info;

fn look_direction(direction: Vec3) -> Quat {
	let pitch = direction.y.asin();
	let yaw = direction.z.atan2(direction.x);
	Quat::from_rotation_y(-yaw - PI / 2.0) * Quat::from_rotation_x(pitch)
}

async fn initial_placement(
	client: &Client<impl ClientHandler>,
	spatial_root: Spatial,
) -> Result<()> {
	let hmd = Tracked::hmd_spatial().await?;
	let root = client.root();

	let Transform {
		translation: item_translation,
		..
	} = spatial_root.get_relative_transform(root.clone()).await??;
	let Transform {
		translation: hmd_translation,
		..
	} = client
		.spatial_interface()
		.get_relative_transform(root.clone(), hmd.clone())
		.await?
		.map_err(|err| match err {
			SpatialRefOpError::RelativeToInvalid => CreateError::InvalidRef,
			SpatialRefOpError::SpatialRefInvalid => CreateError::InvalidRef,
		})?;

	// if the distance between the panel item and the client origin is basically nothing, it must be unpositioned
	if env::var_os("STARDUST_STARTUP_TOKEN").is_none_or(|v| v.is_empty()) {
		info!("launched without a sense of space");
		// so we want to position it in front of the user
		let _ = spatial_root.set_relative_transform(
			hmd,
			Transform::from_translation_rotation(vec3(0.0, 0.0, -0.25), Quat::IDENTITY),
		);
		return Ok(());
	}

	// otherwise make the panel look at the user
	let look_rotation =
		look_direction((Vec3::from(item_translation) - Vec3::from(hmd_translation)).normalize());
	let _ =
		spatial_root.set_relative_transform(root.clone(), Transform::from_rotation(look_rotation));

	Ok(())
}

/// Head pose (position + upright yaw) in some reference frame.
#[derive(Debug, Clone, Copy)]
struct HeadPose {
	pos: Vec3,
	yaw: f32,
}

fn run_dir() -> PathBuf {
	crate::curve::cache_dir()
}
fn mtime(p: &Path) -> Option<SystemTime> {
	std::fs::metadata(p).and_then(|m| m.modified()).ok()
}

/// Shared head anchor so every curved panel uses the same head pose.
/// `recenter.req` is touched by `stardust-dual-mirror-recenter` (and at mirror launch);
/// the first panel to see a request newer than `anchor` samples the HMD and writes `anchor`,
/// the others reuse it. File: "x y z yaw" in the stage (or client root) frame.
fn read_anchor(not_before: Option<SystemTime>) -> Option<HeadPose> {
	let p = run_dir().join("anchor");
	let t = mtime(&p)?;
	if not_before.is_some_and(|nb| t < nb) {
		return None;
	}
	let v: Vec<f32> = std::fs::read_to_string(p)
		.ok()?
		.split_whitespace()
		.filter_map(|x| x.parse().ok())
		.collect();
	(v.len() == 4).then(|| HeadPose { pos: vec3(v[0], v[1], v[2]), yaw: v[3] })
}
fn write_anchor(h: HeadPose) {
	let dir = run_dir();
	let tmp = dir.join(format!("anchor.{}", std::process::id()));
	let body = format!("{} {} {} {}\n", h.pos.x, h.pos.y, h.pos.z, h.yaw);
	if std::fs::write(&tmp, body).is_ok() {
		let _ = std::fs::rename(&tmp, dir.join("anchor"));
	}
}

fn yaw_of(rot: Quat) -> f32 {
	let f = rot * Vec3::NEG_Z;
	(-f.x).atan2(-f.z)
}
fn wrap(a: f32) -> f32 {
	(a + PI).rem_euclid(2.0 * PI) - PI
}

async fn hmd_in(
	client: &Client<impl ClientHandler>,
	frame: &SpatialRef,
	hmd: &SpatialRef,
) -> Result<HeadPose> {
	let Transform { translation, rotation, .. } = client
		.spatial_interface()
		.get_relative_transform(frame.clone(), hmd.clone())
		.await?
		.map_err(|_| CreateError::InvalidRef)?;
	Ok(HeadPose { pos: Vec3::from(translation), yaw: yaw_of(Quat::from(rotation)) })
}

/// Sample the HMD at ~20 Hz until it is still (<2° yaw and <2 cm over 250 ms) or 2 s pass,
/// then return the mean of the last window. Softens "turned my head while it launched".
async fn stable_head(
	client: &Client<impl ClientHandler>,
	frame: &SpatialRef,
) -> Result<HeadPose> {
	const WIN: usize = 6;
	let hmd = Tracked::hmd_spatial().await?;
	let mut buf: Vec<HeadPose> = Vec::new();
	let deadline = Instant::now() + Duration::from_millis(
		env::var("FLATLAND_RECENTER_SETTLE_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(2000),
	);
	loop {
		buf.push(hmd_in(client, frame, &hmd).await?);
		if buf.len() > WIN {
			buf.remove(0);
		}
		if buf.len() == WIN {
			let y0 = buf[0].yaw;
			let dy = buf.iter().map(|h| wrap(h.yaw - y0).abs()).fold(0.0, f32::max);
			let dp = buf.iter().map(|h| h.pos.distance(buf[0].pos)).fold(0.0, f32::max);
			let still = dy < 2f32.to_radians() && dp < 0.02;
			if still || Instant::now() >= deadline {
				let n = buf.len() as f32;
				let pos = buf.iter().map(|h| h.pos).sum::<Vec3>() / n;
				let yaw = y0 + buf.iter().map(|h| wrap(h.yaw - y0)).sum::<f32>() / n;
				info!(still, dy_deg = dy.to_degrees(), dp, "head sample");
				return Ok(HeadPose { pos, yaw: wrap(yaw) });
			}
		}
		tokio::time::sleep(Duration::from_millis(50)).await;
	}
}

/// Shared anchor for `req` (the recenter request time): reuse a fresh one, or take the lock and sample.
async fn shared_head(
	client: &Client<impl ClientHandler>,
	frame: &SpatialRef,
	req: Option<SystemTime>,
) -> Result<HeadPose> {
	if let Some(h) = read_anchor(req) {
		return Ok(h);
	}
	let lock = run_dir().join("anchor.lock");
	if mtime(&lock).is_some_and(|t| t.elapsed().unwrap_or_default() > Duration::from_secs(6)) {
		let _ = std::fs::remove_file(&lock);
	}
	let got = std::fs::OpenOptions::new().write(true).create_new(true).open(&lock).is_ok();
	if !got {
		// another panel is sampling; wait for its anchor
		for _ in 0..80 {
			tokio::time::sleep(Duration::from_millis(50)).await;
			if let Some(h) = read_anchor(req) {
				return Ok(h);
			}
		}
	}
	let h = stable_head(client, frame).await;
	if let Ok(h) = h {
		write_anchor(h);
	}
	if got {
		let _ = std::fs::remove_file(&lock);
	}
	h
}

async fn reference_frame(client: &Client<impl ClientHandler>) -> SpatialRef {
	// stage is shared across clients; fall back to this client's root
	match Tracked::stage_spatial().await {
		Ok(s) => s,
		Err(_) => client.root().clone(),
	}
}

/// Place the panel on the WayVR-style cylinder: fixed stage anchor, or the (shared, settled) head pose.
async fn curved_placement(
	client: &Client<impl ClientHandler>,
	spatial_root: &Spatial,
	curve: crate::curve::Curve,
	req: Option<SystemTime>,
) -> Result<()> {
	let (local_pos, local_rot) = curve.local_pose();
	match curve.anchor {
		Some(anchor) => {
			let stage = Tracked::stage_spatial().await?;
			spatial_root.set_relative_transform(
				stage,
				Transform::from_translation_rotation(anchor + local_pos, local_rot),
			)?;
		}
		None => {
			let frame = reference_frame(client).await;
			let head = shared_head(client, &frame, req).await?;
			let q = Quat::from_rotation_y(head.yaw);
			spatial_root.set_relative_transform(
				frame.clone(),
				Transform::from_translation_rotation(head.pos + q * local_pos, q * local_rot),
			)?;
			info!(?curve, ?head, "placed curved panel");
		}
	}
	Ok(())
}

/// Re-place whenever `recenter.req` is touched (stardust-dual-mirror-recenter).
fn spawn_recenter_watch<H: ClientHandler + Send + Sync + 'static>(
	client: Arc<Client<H>>,
	spatial_root: Spatial,
	curve: crate::curve::Curve,
	mut last: Option<SystemTime>,
) {
	tokio::spawn(async move {
		let req_path = run_dir().join("recenter.req");
		loop {
			tokio::time::sleep(Duration::from_millis(200)).await;
			let t = mtime(&req_path);
			if t.is_some() && t != last {
				last = t;
				info!("recenter requested");
				if let Err(e) = curved_placement(&client, &spatial_root, curve, t).await {
					tracing::warn!("recenter failed: {e:?}");
				}
			}
		}
	});
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct InitialPanelPlacement {
	pub curve: Option<crate::curve::Curve>,
}
impl<State: ValidState> CustomElement<State> for InitialPanelPlacement {
	type Inner = ();
	type Error = Error;

	async fn create_inner(&self, ctx: &Context, info: CreateInnerInfo) -> Result<Self::Inner> {
		if let Some(curve) = self.curve {
			let _ = std::fs::create_dir_all(run_dir());
			let req = mtime(&run_dir().join("recenter.req"));
			match curved_placement(&ctx.stardust_client, &info.child_space, curve, req).await {
				Ok(()) => {
					spawn_recenter_watch(ctx.stardust_client.clone(), info.child_space.clone(), curve, req);
					return Ok(());
				}
				Err(e) => tracing::warn!("curved placement failed ({e:?}); falling back"),
			}
		}
		initial_placement(&ctx.stardust_client, info.child_space).await
	}

	fn diff(&self, _old_self: &Self, _ctx: &Context, _inner: &mut Self::Inner) {}
}
