# RasterPropMonitor Changelog

Discord support server: https://discord.gg/vBDzZAq3AF.

Please always post your [KSP.log file](https://gist.github.com/JonnyOThan/04c2074b56f78e56d115621effee30f9) when reporting issues.

## Unreleased

### New Features

- Props can now reference methods on internalmodules that are placed directly in the Internal if one isn't found in the prop [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/240ee4d1e8ca1114c8948221be3245f79fb11e8e)
- JSINumericInput can now take a variable as its increment term (thanks MirageDev) [PR](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/11a01f13b910394fcb05463403dae97a6e0d57de)

### Bug Fixes

- Add RPM computer to CRY-5000Freezer from DeepFreeze (fixes crashes) [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/a369c889c6fd9e6c3ccfda35507532f573bb5693)
- Add overlay depth masks to remade stock internals [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/7771d9481dcf348cfafe3d63de8783177cdbbf02)
- Fix looping shutter animation in ALCOR pod caused by multiple animation clips on the same object [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/958720c1d9ad2145cacf5f5098d8984682f5e00e)
- Fix black outline around text when using deferred rendering (thanks LGhassen) [PR](https://github.com/FirstPersonKSP/RasterPropMonitor/pull/136)

## 0.31.13.4 - "Yet even more bug fixes" - 2023-11-10

### Changes

- Add compatibility patches (fixes crashes) when using hypersonic cockpit from B9Aerospace
- Fix navball rendering for non-square screens
- Fix another case of garbled text
- Improve science container menu page

## 0.31.13.3 - "Even more bug fixes" - 2023-09-21

### Changes

- Add compatibility patches (fixes crashes) when using DeepFreeze or Heisenberg
- The undock action now also decouples the attachnode (thanks MirageDev)
- Fixed corrupted text on batched labels

### New Contributors

- MirageDev

## 0.31.13.2 - "Bug fixes" - 2023-05-22

### Changes

- Fix indexing issue in MFD graph pages
- Fix material sharing for emissive changes (improves performance for batched label system)
- Add compatibility patches for Endurance mod, which is missing RPM computer modules
- Fix a fatal bug when certain incorrectly configured props are installed (e.g. Warbirds cockpits)

## 0.31.13.1 - "Bug fixes" - 2023-05-15

### Changes

- Fixed lots of bugs introduced with batching systems
- Fixed persistence issues introduced in 0.31.12.0
- Added patches to add Rashttps://github.com/FirstPersonKSP/RasterPropMonitor/releases/tag/v0.31.13.4terPropMonitorComputer modules to SSPX parts that are missing them

## 0.31.13.0 - "Performance improvements" - 2023-05-12

### Changes

- Prop models can be batched into larger meshes that can be drawn at once for improved performance

## 0.31.12.0 - "More bug fixes" - 2023-05-05

### Changes

- Fixed blurry text when playing on lower texture quality settings
- Fixed a bug where certain props wouldn't turn back on when losing and regaining power
- Fixed a bug where RPM would prevent ResourceConverters from switching recipes with B9PartSwitch
- Fixed navball rendering in the JSI MFD
- Disable DE_IVA mk1-3 patch if it already has warp props
- RPM will no longer dynamically create RasterPropMonitorComputer modules
- Remove genericSpace3 patch as it interferes with MAS and ModularLaunchPads
- Rebuilt variable persistence system so it makes more sense (note Reviva still interferes with this)

## 0.31.11.1 - 2023-02-26

### WARNING

Some props that were previously part of RPM have been moved to ASET Consolidated Props Pack. If you are using IVAs that use those props, you'll need to make sure to switch to that version. If you're using CKAN, simply update RPM and ASET Props and you're good to go.

### Changes

- Fixed NREs from navball in Near Future pods
- Force all clickable buttons to layer 20 (fixes certain props when FreeIva is installed)
- Fix navball markers being visible when they shouldn't
- Add support for switchable labels to preserve the selected index

## 0.31.11.0 - "Bugfixes, performance, ASET Consolidated" - 2023-02-15

### WARNING

Some props that were previously part of RPM have been moved to ASET Consolidated Props Pack. If you are using IVAs that use those props, you'll need to make sure to switch to that version. If you're using CKAN, simply update RPM and ASET Props and you're good to go.

### Changes

- Moved ASET-derived props and patches to ASET Consolidated Props
- Moved thrust reverser patches from ASET
- Fixed "double-vision" bug that could occur when going EVA while an external camera was active
- Fixed pan/tilt/zoom buttons on external cameras
- Fix some harmless error spew
- Fixed "warp to next" button for maneuver nodes
- Fixed node burn time sometimes reading as NaN
- Fixed variables flickering when the game was running faster than physics
- Fixed terminal velocity readout flickering to NaN on the launchpad
- Optimized navball page rendering

## 0.31.10.3 - 2023-02-06

### Changes

- Fixed some variables being stuck on NaN
- Fixed time-based variables (rate of change, etc) sometimes flickering to NaN
- Fixed missing [ characters on some props
- Fixed backlight button not working the first time it's clicked
- Use fast path when changing color of a label that doesn't have color tags
- Don't install plugin modules for mods that aren't installed
- Keep persistent variables out of the update lists
- Remove obsolete patches for stock parts that would cause harmless errors in the log
- Prep for moving some of the custom ASET-derived parts into ASET itself

## 0.31.10.2 - 2023-02-01

### Changes

- Fixed a crash when using external cameras in VR
- Code support to allow interacting with props when moving to other parts with FreeIva

## 0.31.10.1 - 2023-01-19

### Changes

- Fixed missing camera part when using Restock
- Massive performance improvements

## 0.31.10 - 2023-01-18 [Pre-release]

### Changes

- Fixed missing camera part when using Restock
- Massive performance improvements

## 0.31.9 - 2022-10-17

This update has no new features, but should improve performance.

### Changes

- Moved MFD rendering to LateUpdate
- Reducing some temporary allocations that contribute to GC pressure

## 0.31.8 - 2022-10-08

### Changes

- MM Patch fixes to remove unnecessary log spam by @StoneBlue in #36
- Fixes a memory leak in RPMVesselComputer.cs by @linuxgurugamer in #48
- Fix JSI resource page formatting by @k-dueb in #45
- Fix ACTIVEENGINECOUNT variable
- External camera is no longer a reskinned linear RCS part. Now it's a reskinned Ant engine so you can tell which way it's oriented.
- Warp props are now placeable in Unity
- The external camera FOV is now taken from the part instead of the MFD config
- External camera FOV previews in the VAB are now accurate
- rpm-config is now loaded properly and can be targeted with MM patches
- Add "remove node" to target menu on MFDs
- Add WHEELSTEER and WHEELTHROTTLE builtin variables
- Support relative paths in most transform fields
- Add quicksave/quickload/revert buttons to JSIMainCompUnit prop
- Add deployable antenna and radiator functionality and props
- Fix translation indicators on ASET 60x30 docking screen
- Add snacks support to life support monitor
- Re-exported shader package in hopes of fixing linux text issues
- Fix time to impact calculation
- Fix time to an/dn calculations

### New Contributors

- StoneBlue
- linuxgurugamer
- k-dueb

## 0.31.7 - 2022-06-14

### Changes

- Fix a race condition during loading that could result in purple textures and a non-functioning IVA
- JSIActionGroupSwitch can now control multiple internal lights, separated by a | character

## 0.31.6 - 2022-01-15

### Changes

- Reworked parachute actions:
  - Clicking the "Deploy" button now sets the minPressure to minimum, safety mode to "unsafe," and arms the chute. In other words, it tries to immediately deploy the chute.
  - Clicking "Arm" simply arms the chute
  - Clicking "Disarm" disarms the chute and sets the safety mode to "when safe."
- Added timewarp button support  
  "Warp to Next" button will warp to the next maneuver node, SOI change, PE, or AP depending on context
- Added an example patch for DE_IVAExtensions to put timewarp buttons in the IVA

## 0.31.5 - 2021-09-06

### Changes

- Update compatibility to ksp 1.12
- Merge PRs related to camera drag cube & ascending node calculation
- Fix ATMOSPHEREDEPTH variable
- Fix SAS mode strings in ASET MFDs
- Fix radial in/out swapping on SAS mode buttons
- Fix Mechjeb integration with latest MJ version

## 0.31.4 - 2020-07-12

Remove dependencies on flight UI elements, because they no longer update when not visible in KSP 1.10. Specifically this fixes navball orientation and atmosphere depth gauge when in IVA, and any variables derived from those inputs.

This same release is likely compatible with KSP 1.8.x and 1.9.x, but I have not tested it (and there's no reason you should need this one if you're not on KSP 1.10).

Many thanks to Manul and Zorkinian on the KSP forums for pinpointing the root cause!

## 0.31.3 - 2020-02-23

### Changes

- Updated external camera code for the new camera system in 1.9. This is on a runtime switch so this release is compatible with 1.8 and opengl in 1.9.

## 0.31.2 - 2020-01-22

### Changes

- CKAN is now officially supported. There are 2 CKAN packages, see the readme for details.
- Fix KOSPropMonitor not refreshing the screen properly
- Fix orbit display icon rendering
- Fix vessel recovery prop soft-locking the game in some circumstances
- Add missing vessel types to target menu
- Fix some debug-only errors
- Fix null reference exception on revert to editor
- Add compatibility patches for incorrectly configured mods (NMB and OPT specifically)
- Fix null reference exception when using internal light switch on incorrectly configured cockpits
- External camera pages default to skip missing cameras, so the ALCOR MFD landing page will start on ExtCam1 if it exists
- Move variable handler for plugins before builtins like it says in the docs
- Fix line drawing on NAV pages due to broken shader reference in scansat

## 0.31.0 - 2019-12-29

### Changes

- Updated for KSP 1.8.X