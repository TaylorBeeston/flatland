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

/// Group offset for all curved panels, from `$XDG_RUNTIME_DIR/flatland-curve/adjust`
/// (written by `stardust-dual-mirror-recenter --yaw/--height/--radius`). Lines `key value`:
/// `yaw_deg` (+ = rotate the set right), `height_m` (+ = up), `radius_m` (absolute; 0/absent = env radius).
#[derive(Debug, Default, Clone, Copy)]
struct Adjust {
	yaw_deg: f32,
	height_m: f32,
	radius_m: f32,
}
fn read_adjust() -> Adjust {
	let mut a = Adjust::default();
	let Ok(body) = std::fs::read_to_string(run_dir().join("adjust")) else { return a };
	for line in body.lines() {
		let mut it = line.split_whitespace();
		let (Some(k), Some(v)) = (it.next(), it.next().and_then(|v| v.parse::<f32>().ok())) else { continue };
		if !v.is_finite() {
			continue;
		}
		match k {
			"yaw_deg" => a.yaw_deg = v.clamp(-180.0, 180.0),
			"height_m" => a.height_m = v.clamp(-1.5, 1.5),
			"radius_m" => a.radius_m = if v == 0.0 { 0.0 } else { v.clamp(0.3, 5.0) },
			_ => {}
		}
	}
	a
}

/// Place the panel on the WayVR-style cylinder: fixed stage anchor, or the (shared, settled) head pose.
async fn curved_placement(
	client: &Client<impl ClientHandler>,
	spatial_root: &Spatial,
	curve: crate::curve::Curve,
	req: Option<SystemTime>,
) -> Result<()> {
	let adj = read_adjust();
	let (mut local_pos, local_rot) = curve.local_pose();
	if adj.radius_m > 0.0 {
		// mesh keeps its baked curvature; only the distance changes
		local_pos *= adj.radius_m / curve.radius;
	}
	let yq = Quat::from_rotation_y(-adj.yaw_deg.to_radians());
	let (local_pos, local_rot) = (yq * local_pos + Vec3::Y * adj.height_m, yq * local_rot);
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
			info!(?curve, ?head, ?adj, "placed curved panel");
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
		let adj_path = run_dir().join("adjust");
		let mut last_adj = mtime(&adj_path);
		loop {
			tokio::time::sleep(Duration::from_millis(200)).await;
			let t = mtime(&req_path);
			let ta = mtime(&adj_path);
			if (t.is_some() && t != last) || ta != last_adj {
				// adjust-only change reuses the current anchor (same `last` => no new head sample)
				let req = if t != last { t } else { last };
				last = t;
				last_adj = ta;
				info!("recenter/adjust requested");
				if let Err(e) = curved_placement(&client, &spatial_root, curve, req).await {
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
					if curve.anchor.is_none() && env::var("FLATLAND_GROUP_HANDLE").is_ok_and(|v| v == "1") {
						spawn_group_handle(ctx.stardust_client.clone(), curve);
					}
					return Ok(());
				}
				Err(e) => tracing::warn!("curved placement failed ({e:?}); falling back"),
			}
		}
		initial_placement(&ctx.stardust_client, info.child_space).await
	}

	fn diff(&self, _old_self: &Self, _ctx: &Context, _inner: &mut Self::Inner) {}
}

fn write_adjust(a: Adjust) {
	let dir = run_dir();
	let tmp = dir.join(format!("adjust.{}", std::process::id()));
	let body = format!(
		"yaw_deg {:.3}\nheight_m {:.3}\nradius_m {:.3}\n",
		a.yaw_deg, a.height_m, a.radius_m
	);
	if std::fs::write(&tmp, body).is_ok() {
		let _ = std::fs::rename(&tmp, dir.join("adjust"));
	}
}

/// Rest pose of the group grab handle in `frame`: centre of the set, below eye level, a bit closer than the panels.
fn handle_rest(head: HeadPose, adj: Adjust, base_r: f32) -> Vec3 {
	let r = if adj.radius_m > 0.0 { adj.radius_m } else { base_r };
	let drop = env::var("FLATLAND_GROUP_HANDLE_DROP").ok().and_then(|v| v.parse().ok()).unwrap_or(0.42f32);
	let local = Quat::from_rotation_y(-adj.yaw_deg.to_radians()) * vec3(0.0, 0.0, -r * 0.8)
		+ Vec3::Y * (adj.height_m - drop);
	head.pos + Quat::from_rotation_y(head.yaw) * local
}
/// horizontal azimuth (+ = right, matching yaw_deg) and distance of `p` around the head anchor
fn polar(head: Vec3, p: Vec3) -> (f32, f32) {
	let d = p - head;
	(d.x.atan2(-d.z), (d.x * d.x + d.z * d.z).sqrt())
}

/// One grab handle (orange sphere) for the whole curved set. Grab with controller grip or hand pinch and drag:
/// left/right around you = yaw, up/down = height, toward/away = radius. Writes `adjust`, which every curved
/// Flatland (this one included) already polls, so both panels move together.
fn spawn_group_handle<H: ClientHandler + Send + Sync + 'static>(
	client: Arc<Client<H>>,
	curve: crate::curve::Curve,
) {
	use stardust_xr_fusion::{
		drawable::{MaterialParameter, Model, ModelExt as _},
		fields::{Field, FieldExt as _, Shape},
		spatial::SpatialExt as _,
		suis::InputDataType,
		types::{Resource, rgba_linear},
	};
	use stardust_xr_molecules::input_action::{InputQueue, SingleAction};
	const R: f32 = 0.03;
	tokio::spawn(async move {
		let res: Result<()> = async {
			let frame = reference_frame(&client).await;
			let (spatial, _) = Spatial::new(&client, &frame, Transform::IDENTITY).await?;
			let model = Model::new(
				&client,
				&spatial,
				Resource::Namespaced { namespace: <crate::Flatland as stardust_xr_asteroids::ClientState>::APP_ID.into(), path: "resize_handle".into() },
			)
			.await?;
			let sphere = model.get_part("sphere").await?;
			let set_color = |c| {
				if let Some(s) = sphere.clone() {
					tokio::spawn(async move {
						let _ = s.set_material_parameter("color", MaterialParameter::Color { value: c }).await;
					});
				}
			};
			let idle = rgba_linear!(1.0, 0.45, 0.05, 1.0);
			set_color(idle);
			let (field, _) = Field::new(&client, &spatial, Shape::Sphere { radius: R }).await?;
			let input = InputQueue::new(&client, spatial.clone(), field.clone(), frame.clone()).await?;
			let mut action = SingleAction::default();
			let scale = env::var("FLATLAND_GROUP_HANDLE_SCALE").ok().and_then(|v| v.parse().ok()).unwrap_or(6.0f32);
			let place = |p: Vec3| {
				let _ = spatial.set_relative_transform(
					frame.clone(),
					Transform::from_translation_scale(p, [scale; 3]),
				);
			};
			// grab start: (grab point, adjust)
			let mut start: Option<(Vec3, Adjust)> = None;
			let mut last_write = Instant::now();
			let mut pending: Option<Adjust> = None;
			let mut rest = Vec3::ZERO;
			let mut hovered = false;
			info!("group grab handle ready");
			loop {
				tokio::time::sleep(Duration::from_millis(16)).await;
				let Some(head) = read_anchor(None) else { continue };
				let _ = input.handle_events();
				action.update(
					true,
					&input,
					|i| match i.input() {
						InputDataType::Pointer { .. } => false,
						_ => i.distance() < R + 0.05,
					},
					|i| match i.input() {
						InputDataType::Hand { .. } => i.datamap_f32("pinch_strength") > 0.90,
						_ => i.datamap_f32("grab") > 0.90,
					},
				);
				let h = !action.hovering().current().is_empty() || action.actor_acting();
				if h != hovered {
					hovered = h;
					set_color(if h { rgba_linear!(1.0, 0.85, 0.3, 1.0) } else { idle });
				}
				let point = action.actor().and_then(|a| match a.input() {
					InputDataType::Hand { data } => Some(
						Vec3::from(data.thumb.tip.pose.position)
							.lerp(Vec3::from(data.index.tip.pose.position), 0.5),
					),
					InputDataType::Tip { data } => Some(Vec3::from(data.pose.position)),
					InputDataType::Pointer { .. } => None,
				});
				match (point, start) {
					(Some(p), None) => {
						start = Some((p, read_adjust()));
						info!("group handle grabbed");
					}
					(Some(p), Some((p0, a0))) => {
						let (az0, d0) = polar(head.pos, p0);
						let (az, d) = polar(head.pos, p);
						let base = if a0.radius_m > 0.0 { a0.radius_m } else { curve.radius };
						let a = Adjust {
							yaw_deg: (a0.yaw_deg + wrap(az - az0).to_degrees()).clamp(-180.0, 180.0),
							height_m: (a0.height_m + (p.y - p0.y)).clamp(-1.5, 1.5),
							radius_m: (base + (d - d0) * 1.5).clamp(0.3, 5.0),
						};
						pending = Some(a);
						place(p);
					}
					(None, Some(_)) => {
						start = None;
						if let Some(a) = pending.take() {
							write_adjust(a);
						}
						info!(adjust = ?read_adjust(), "group handle released");
					}
					(None, None) => {}
				}
				if let Some(a) = pending {
					if last_write.elapsed() >= Duration::from_millis(100) {
						write_adjust(a);
						pending = None;
						last_write = Instant::now();
					}
				}
				if start.is_none() {
					let r = handle_rest(head, read_adjust(), curve.radius);
					if r.distance(rest) > 0.002 {
						rest = r;
						place(r);
					}
				}
			}
		}
		.await;
		if let Err(e) = res {
			tracing::warn!("group grab handle failed: {e:?}");
		}
	});
}
