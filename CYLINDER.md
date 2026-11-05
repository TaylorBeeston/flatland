# Flatland cylinder (curved panels)

Opt-in WayVR-style cylinder WM for host-mirror panels. Visual mesh is curved; **pointer hit-test stays flat** (phase 2).

Branch for strangers: `roadie/curve-bake-on-release` on https://github.com/TaylorBeeston/flatland (not merged to StardustXR/flatland).

## Env

| Var | Role |
|-----|------|
| `FLATLAND_CURVE_RADIUS` | Cylinder radius in m; **required** to enable curved mode (0.05–100) |
| `FLATLAND_CURVE_YAW_DEG` | Panel centre angle on cylinder, + = right (default 0) |
| `FLATLAND_CURVE_ANCHOR` | Axis point `"x,y,z"` in stage space (default `0.008,1.60,0.044`), or `hmd` for shared head at launch |
| `FLATLAND_CURVE_BEND` | **Ignored** — mesh radius always equals cylinder radius |
| `FLATLAND_DENSITY` | Panel px/m (default 3000; also `--density` / `STARDUST_PANEL_DENSITY`) |
| `FLATLAND_FIXED_RES` | `WxH` pinned buffer; with density → panel size in m (needed for grab/wire) |
| `FLATLAND_CYLINDER_WIRE` | `1` → faint cylinder wireframe rings (one panel should own this) |
| `FLATLAND_EDGE_GRAB` | `1` → grab any panel rim to drag the whole set; default is **bead-only** handle |
| `FLATLAND_EDGE_GRAB_M` | Rim/edge grab thickness in m (default 0.07) |
| `FLATLAND_HANDLE_GRAB_M` | Bead grab radius in m (default 0.045) |
| `FLATLAND_HANDLE_ARC_M` | Bead arc offset from gap in m (default 0) |
| `FLATLAND_REANCHOR_ON_RELEASE` | Default on: on handle release, move axis xz to current head. Set `0` to disable |
| `FLATLAND_KEY_SINK` | Default on for `host-mirror*` app ids; set `0` to force wl_keyboard |
| `FLATLAND_RECENTER_SETTLE_MS` | HMD settle before writing shared anchor (default 2000) |

Related host-mirror UI flags: `FLATLAND_SCALE_MODE`, `FLATLAND_HANDLES`, `FLATLAND_CLOSE_BUTTON`.

## Shared runtime files (`$XDG_RUNTIME_DIR/flatland-curve`)

All curved Flatland processes poll the same dir so panels move as one set.

| File | Purpose |
|------|---------|
| `anchor` | Shared head pose (first panel after a recenter samples HMD) |
| `adjust` | Group offset: lines `yaw_deg`, `height_m`, `radius_m`, `dragging` (1 while handle held) |
| `recenter.req` | Touch/create to request recenter (e.g. `stardust-dual-mirror-recenter`); panels re-place |
| `*.glb` | Baked curve meshes (stale ones pruned after ~10 min) |

`adjust` is also written by CLI recenter (`--yaw` / `--height` / `--radius`) and by the in-VR handle.

## Bead handle

Default: pinch/grip **>0.95** on a small bead under the gap (lower ring) drags the whole cylinder — yaw / height / radius via `adjust`. One owner at a time. While `dragging=1`, placement follows radius but **mesh bake is deferred**; on release the mesh swaps once to the final radius (old GLB dropped). With `FLATLAND_REANCHOR_ON_RELEASE` (default), axis xz snaps to head on release.

## Key sink

For `SDXR_WL_APP_ID` starting with `host-mirror`, keys go to `$XDG_RUNTIME_DIR/host-mirror-keys/<app_id>.sock` (24-byte LE datagram: evdev keycode, pressed, depressed/latched/locked/layout) when the socket exists; else panel-item fallback. Disable with `FLATLAND_KEY_SINK=0`.

## Known gap

Flat pointer / input planes on a curved mesh — hit-test does not follow the curve yet (**phase 2**). Controllers/hands still work via existing flat fields.
