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
/// below this the eDP|gap|Meta set (2.83 m of arc) would wrap past 360 deg
const MIN_R: f32 = 0.5;
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
			"radius_m" => a.radius_m = if v == 0.0 { 0.0 } else { v.clamp(MIN_R, 5.0) },
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
	let r = crate::curve::quantize_radius(if adj.radius_m > 0.0 { adj.radius_m } else { curve.radius });
	crate::curve::set_live_radius(r);
	let pw = panel_size_m().map(|p| p.0).unwrap_or(1.0);
	let live = curve.on_radius(r, pw);
	let mesh_r = crate::curve::mesh_radius(pw, r);
	let (local_pos, local_rot) = live.local_pose();
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
			info!(?live, mesh_r, ?head, ?adj, "placed curved panel");
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
					if curve.anchor.is_none() && !env::var("FLATLAND_EDGE_GRAB").is_ok_and(|v| v == "0") {
						spawn_edge_grab(ctx.stardust_client.clone(), curve);
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

/// horizontal azimuth (+ = right, matching yaw_deg) and distance of `p` around the head anchor
fn polar(head: Vec3, p: Vec3) -> (f32, f32) {
	let d = p - head;
	(d.x.atan2(-d.z), (d.x * d.x + d.z * d.z).sqrt())
}
/// azimuth (in `frame`, same convention as `polar`) of local yaw `y` after the head yaw `hy`
fn world_az(hy: f32, local_yaw: f32) -> f32 {
	let f = Quat::from_rotation_y(hy) * Quat::from_rotation_y(-local_yaw) * Vec3::NEG_Z;
	f.x.atan2(-f.z)
}

/// panel size in m from FLATLAND_FIXED_RES / FLATLAND_DENSITY (what snap_fixed_size uses)
fn panel_size_m() -> Option<(f32, f32)> {
	let res = env::var("FLATLAND_FIXED_RES").ok()?;
	let (w, h) = res.split_once('x')?;
	let d: f32 = env::var("FLATLAND_DENSITY").ok()?.parse().ok()?;
	Some((w.parse::<f32>().ok()? / d, h.parse::<f32>().ok()? / d))
}

/// Edge grab: grip/pinch within EDGE m of this panel's rim (angular left/right edges, top/bottom) and drag to
/// move the whole cylinder: around you = yaw, up/down = height, toward/away = radius. Writes the shared
/// `adjust` file that every curved Flatland polls, so all panels move rigidly and never overlap.
/// With FLATLAND_CYLINDER_WIRE=1 this Flatland also draws a faint cylinder wireframe (one panel should).
fn spawn_edge_grab<H: ClientHandler + Send + Sync + 'static>(
	client: Arc<Client<H>>,
	curve: crate::curve::Curve,
) {
	use stardust_xr_fusion::{
		drawable::{Line, LinePoint, Lines, LinesExt as _},
		fields::{Field, FieldExt as _, Shape},
		spatial::SpatialExt as _,
		suis::InputDataType,
		types::{Color, rgba_linear},
	};
	use stardust_xr_molecules::input_action::{InputQueue, SingleAction};
	let Some((pw, ph)) = panel_size_m() else {
		tracing::warn!("edge grab: no FLATLAND_FIXED_RES/DENSITY, disabled");
		return;
	};
	let edge: f32 = env::var("FLATLAND_EDGE_GRAB_M").ok().and_then(|v| v.parse().ok()).unwrap_or(0.07);
	let wire = env::var("FLATLAND_CYLINDER_WIRE").is_ok_and(|v| v == "1");
	tokio::spawn(async move {
		let res: Result<()> = async {
			let frame = reference_frame(&client).await;
			let (spatial, _) = Spatial::new(&client, &frame, Transform::IDENTITY).await?;
			// big sphere around the head: every hand/controller near the cylinder reaches the queue,
			// the edge test below decides what counts
			let (field, _) = Field::new(&client, &spatial, Shape::Sphere { radius: 6.0 }).await?;
			let input = InputQueue::new(&client, spatial.clone(), field.clone(), frame.clone()).await?;
			let lines = if wire { Some(Lines::new(&client, &spatial, Vec::new()).await?) } else { None };
			let mut action = SingleAction::default();
			let mut start: Option<(Vec3, Adjust)> = None;
			let mut pending: Option<Adjust> = None;
			let mut last_write = Instant::now();
			let mut drawn: Option<(Vec3, f32, f32, f32, bool)> = None;
			info!(pw, ph, edge, wire, "edge grab ready");
			loop {
				tokio::time::sleep(Duration::from_millis(16)).await;
				let Some(head) = read_anchor(None) else { continue };
				let adj = read_adjust();
				let r = if adj.radius_m > 0.0 { adj.radius_m } else { curve.radius };
				let r = crate::curve::quantize_radius(r);
				let centre_az = world_az(head.yaw, curve.on_radius(r, pw).yaw + adj.yaw_deg.to_radians());
				let half_az = crate::curve::edge_half_az(pw, r);
				let yc = head.pos.y + adj.height_m;
				let on_edge = |p: Vec3| {
					let (az, d) = polar(head.pos, p);
					let daz = wrap(az - centre_az).abs();
					let dy = (p.y - yc).abs();
					let near = (d - r).abs() < 0.3 && daz < half_az + edge / r && dy < ph / 2.0 + edge;
					near && (daz > half_az - edge / r || dy > ph / 2.0 - edge)
				};
				let point = |i: &stardust_xr_molecules::input_action::InputSnapshot| match i.input() {
					InputDataType::Hand { data } => Some(
						Vec3::from(data.thumb.tip.pose.position).lerp(Vec3::from(data.index.tip.pose.position), 0.5),
					),
					InputDataType::Tip { data } => Some(Vec3::from(data.pose.position)),
					InputDataType::Pointer { .. } => None,
				};
				let _ = input.handle_events();
				action.update(
					true,
					&input,
					|i| point(i).is_some_and(|p| on_edge(p)),
					|i| match i.input() {
						InputDataType::Hand { .. } => i.datamap_f32("pinch_strength") > 0.90,
						_ => i.datamap_f32("grab") > 0.90,
					},
				);
				let p = action.actor().and_then(|a| point(a));
				match (p, start) {
					(Some(p), None) => {
						start = Some((p, adj));
						info!("cylinder edge grabbed");
					}
					(Some(p), Some((p0, a0))) => {
						let (az0, d0) = polar(head.pos, p0);
						let (az, d) = polar(head.pos, p);
						let base = if a0.radius_m > 0.0 { a0.radius_m } else { curve.radius };
						pending = Some(Adjust {
							yaw_deg: wrap((a0.yaw_deg.to_radians()) + wrap(az - az0)).to_degrees(),
							height_m: (a0.height_m + (p.y - p0.y)).clamp(-1.5, 1.5),
							radius_m: (base + (d - d0)).clamp(MIN_R, 5.0),
						});
					}
					(None, Some(_)) => {
						start = None;
						if let Some(a) = pending.take() {
							write_adjust(a);
						}
						info!(adjust = ?read_adjust(), "cylinder edge released");
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
				if let Some(lines) = &lines {
					let hot = start.is_some();
					let key = (head.pos, head.yaw, r, yc, hot);
					if drawn != Some(key) {
						drawn = Some(key);
						let col: Color = if hot { rgba_linear!(0.5, 0.7, 1.0, 0.6) } else { rgba_linear!(0.35, 0.45, 0.6, 0.18) };
						let th = 0.0015;
						let lp = |v: Vec3| LinePoint { point: v.into(), thickness: th, color: col };
						let ring = |y: f32| Line {
							points: (0..96)
								.map(|k| {
									let a = k as f32 / 96.0 * 2.0 * PI;
									lp(vec3(head.pos.x + r * a.sin(), y, head.pos.z - r * a.cos()))
								})
								.collect(),
							cyclic: true,
						};
						// rings only, clear of the screens (never across content)
						let gap = 0.06;
						let ls = vec![ring(yc + ph / 2.0 + gap), ring(yc - ph / 2.0 - gap)];
						let _ = lines.set_lines(ls);
					}
				}
			}
		}
		.await;
		if let Err(e) = res {
			tracing::warn!("edge grab failed: {e:?}");
		}
	});
}
