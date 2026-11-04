//! Side channel for host-mirror: Stardust keyboard events for this panel go to
//! `$XDG_RUNTIME_DIR/host-mirror-keys/<SDXR_WL_APP_ID>.sock` (unix datagram) when that
//! socket exists, instead of wl_keyboard. Wire format (24 bytes, LE u32):
//! keycode (evdev), pressed, depressed, latched, locked, layout_group.
//! Disable with FLATLAND_KEY_SINK=0.
use std::os::unix::net::UnixDatagram;
use std::path::PathBuf;
use std::sync::OnceLock;

struct Sink {
	sock: UnixDatagram,
	path: PathBuf,
}

fn sink() -> Option<&'static Sink> {
	static SINK: OnceLock<Option<Sink>> = OnceLock::new();
	SINK.get_or_init(|| {
		if std::env::var("FLATLAND_KEY_SINK").is_ok_and(|v| v == "0") {
			return None;
		}
		let app_id = std::env::var("SDXR_WL_APP_ID").ok()?;
		if !app_id.starts_with("host-mirror") {
			return None;
		}
		let dir = std::env::var_os("XDG_RUNTIME_DIR")?;
		let path = PathBuf::from(dir).join("host-mirror-keys").join(format!("{app_id}.sock"));
		let sock = UnixDatagram::unbound().ok()?;
		sock.set_nonblocking(true).ok()?;
		tracing::info!(?path, "host-mirror key sink enabled");
		Some(Sink { sock, path })
	})
	.as_ref()
}

/// True when the mirror took the key (socket present and send succeeded).
pub fn send(keycode: u32, pressed: bool, mods: [u32; 4]) -> bool {
	let Some(s) = sink() else { return false };
	let [d, la, lo, g] = mods;
	let mut buf = [0u8; 24];
	for (i, v) in [keycode, pressed as u32, d, la, lo, g].into_iter().enumerate() {
		buf[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
	}
	match s.sock.send_to(&buf, &s.path) {
		Ok(_) => true,
		Err(e) => {
			tracing::debug!(%e, "key sink send failed; using panel item");
			false
		}
	}
}
