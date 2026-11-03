use glam::{Quat, Vec3, vec3};
use stardust_xr_asteroids::{Context, CreateInnerInfo, CustomElement, ValidState};
use stardust_xr_fusion::{
	Error, Result,
	client::{Client, ClientHandler},
	spatial::{Spatial, SpatialRefOpError, Transform},
	tracked::{Tracked, TrackedExt},
	types::CreateError,
};
use std::{env, f32::consts::PI};
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

/// Place the panel on the WayVR-style cylinder: anchor in stage space, or the HMD (upright) at launch.
async fn curved_placement(
	client: &Client<impl ClientHandler>,
	spatial_root: Spatial,
	curve: crate::curve::Curve,
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
			let hmd = Tracked::hmd_spatial().await?;
			let root = client.root();
			let Transform {
				translation,
				rotation,
				..
			} = client
				.spatial_interface()
				.get_relative_transform(root.clone(), hmd)
				.await?
				.map_err(|_| CreateError::InvalidRef)?;
			let hmd_pos = Vec3::from(translation);
			let hmd_rot = Quat::from(rotation);
			let f = hmd_rot * Vec3::NEG_Z;
			let q = Quat::from_rotation_y((-f.x).atan2(-f.z));
			spatial_root.set_relative_transform(
				root.clone(),
				Transform::from_translation_rotation(hmd_pos + q * local_pos, q * local_rot),
			)?;
		}
	}
	info!(?curve, "placed curved panel");
	Ok(())
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
			match curved_placement(&ctx.stardust_client, info.child_space.clone(), curve).await {
				Ok(()) => return Ok(()),
				Err(e) => tracing::warn!("curved placement failed ({e:?}); falling back"),
			}
		}
		initial_placement(&ctx.stardust_client, info.child_space).await
	}

	fn diff(&self, _old_self: &Self, _ctx: &Context, _inner: &mut Self::Inner) {}
}
