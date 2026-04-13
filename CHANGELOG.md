# RasterPropMonitor Changelog

Discord support server: https://discord.gg/vBDzZAq3AF.

Please always post your [KSP.log file](https://gist.github.com/JonnyOThan/04c2074b56f78e56d115621effee30f9) when reporting issues.

## 1.1.0.0 - 2026-04-13

Note that due to changes in MechJeb, this version of RPM is not compatible with MJ 2.14 and earlier.  If you wish to use earlier versions of MJ, please use RPM 1.0.3.

### New Features

- Rebuilt MechJeb2 integration for MJ 2.15.  Nearly all features of MJ are now available on the MFD. (thanks @bryguy4 / @adventure-gpt)


## 1.0.3.2 - 2026-04-11

Note that due to changes in MechJeb, this version (and all previous versions of RPM) are not compatible with MJ 2.15 and later. If you use MJ, please downgrade it to 2.14.3 before trying to install RPM 1.0.3 or else you will likely see conflicts in CKAN.

### New Features

- Add support for scrolling through orbit patches on the MFD.  This includes many new RPM variables that could be used in other props. (thanks @andymac-2) [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/698b1c6cb37d43a1baa74de0a8d87148558f10c0)
- Allow targeting flags and landed vessels (thanks @andymac-2) [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/dfc9858f5cc7cca698699f556901e691995053c2)
- ISJIModule can now be implemented in 3rd party mods to integrate with RPM [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/a476486706e4d64e36467d8d1eefe37795e05f18)
- Add waypoints to navball and MFD menus (thanks @andymac-2) [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/b93fd4e4f818bd7676086e251837818b2c6c1e5f)

### Bug Fixes

- Fix fallback evaluators for lift, drag, and terminal velocity when FAR is not installed (thanks @KlyithSA) [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/6a40b2076de68c24475ec5c6f62c44408dbdc6e0)
- Add RPMComputer modules to a few B9 Aerospace parts that were missing them [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/2ed262fb2404e7841b3f8e339388aaa17cc7624d)
- Disabled some propbatching when MAS is installed because it interferes with their patches [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/40e6f2bba75a595e7a5636ab2f5f53b27a7a5b97)
- Fix NRE in ExternalCameraSelector when the camera transform is missing [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/c6e7a43b70212ddce1e8324945584203315789fc)
- Fix possible NRE when changing vessels [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/82b3f5f0c1caf5e456f450eac1d8c3b73ff5001e)


## 1.0.2 - 2024-09-24

### Bug Fixes

- Fixed AGMEMO system (did I even test this?)


## 1.0.1 - 2024-09-13

### New Features

- Props can now reference methods on internalmodules that are placed directly in the Internal if one isn't found in the prop [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/240ee4d1e8ca1114c8948221be3245f79fb11e8e)
- JSINumericInput can now take a variable as its increment term (thanks MirageDev) [PR](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/11a01f13b910394fcb05463403dae97a6e0d57de)
- Added `PROPELLANTR_` variables to access the resources that are in use by engines on the vessel [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/f1261464100e22819835382624b3afc9b7b1384b)
- `AGMEMO` system is less sensitive to whitespace [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/e343c57adb4dadcefbc753c8623419c29b08ca88)
- Increased pitch bounds for all stock cockpits and pods [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/f0d592d0df828d7278d07038b8bfe8e3535d7de1)
- Improved performance by removing modules that become useless due to prop batching [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/91071da584632b60e7128702396e2ff20a375eff)
- Improved performance by doing more cfg parsing in OnLoad instead of Start [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/91071da584632b60e7128702396e2ff20a375eff)
- Improved performance by removing props entirely if they have no InternalModules [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/7c714fa59750e600c183e6e03ef61b8aae8a56a0)

### Bug Fixes

- Add RPM computer to several mods that were missing them (fixes crashes) [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/a369c889c6fd9e6c3ccfda35507532f573bb5693)
  - CRY-5000Freezer from DeepFreeze [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/a369c889c6fd9e6c3ccfda35507532f573bb5693)
  - DSEV [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/ab4423ac769c899c0db1e6494d5cddee54eb3608)
  - Aviation Cockpits [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/9cbde4cf7436a1fed10ed6c0c27cbd3e72c758e6)
- Add overlay depth masks to remade stock internals [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/7771d9481dcf348cfafe3d63de8783177cdbbf02)
- Fix looping shutter animation in ALCOR pod caused by multiple animation clips on the same object [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/958720c1d9ad2145cacf5f5098d8984682f5e00e)
- Fix black outline around text when using deferred rendering (thanks LGhassen) [PR](https://github.com/FirstPersonKSP/RasterPropMonitor/pull/136)
- Fixed benign yet annoying error spew caused by prop batching system [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/91071da584632b60e7128702396e2ff20a375eff)
- Fixed RPM_COLOROVERRIDE not working [commit](https://github.com/FirstPersonKSP/RasterPropMonitor/commit/3cd8e5caac36398ce5a5d1e43e5f428f9a36af3b)


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

### Warning

Some props that were previously part of RPM have been moved to ASET Consolidated Props Pack. If you are using IVAs that use those props, you'll need to make sure to switch to that version. If you're using CKAN, simply update RPM and ASET Props and you're good to go.

### Changes

- Fixed NREs from navball in Near Future pods
- Force all clickable buttons to layer 20 (fixes certain props when FreeIva is installed)
- Fix navball markers being visible when they shouldn't
- Add support for switchable labels to preserve the selected index


## 0.31.11.0 - "Bugfixes, performance, ASET Consolidated" - 2023-02-15

### Warning

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


## 0.31.10 - 2023-01-18 [PRE-RELEASE]

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
