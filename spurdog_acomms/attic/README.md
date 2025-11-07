Attic README — spurdog_acomms/attic

Purpose
-------
This folder contains legacy experiments, prototype scripts, message definitions, and older launch configs that were kept for historical reference and debugging. Files in this directory are not part of the active code path and may be out-of-date or untested.

Guidelines
---------
- Do NOT delete files in this directory without verifying they are truly unused in the current workflow.
- If you want to resurrect or reuse a file from here, copy it into the relevant `src/` or `launch/` area and update imports and style as needed.
- Add a short note next to resurrected files about what changed and why.

Common contents
---------------
- Old variants of `comms_cycle_mgr` and test harnesses (v0, v2, v3, etc.)
- Early message prototypes (.msg) that may have inspired the current message structures
- Dockside and experiment-specific launch files used for early testing

Next steps
---------
- Consider a future sweep to:
  - Move any still-useful scripts into `src/` and add tests or documentation.
  - Archive very old files into a separate `archive/` directory if they are no longer needed, keeping a short index here.

Contact
-------
If you're unsure about a file's purpose, ask the original author or open an issue describing the file and why you think it might be removable.
