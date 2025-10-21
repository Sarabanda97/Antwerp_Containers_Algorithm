# Antwerp Containers Algorithm — README

Summary
-------
This repository contains a small container-terminal simulator (Rust) plus instance files, example solutions and a checker program to validate solution files against instances. The goal is to provide a reproducible pipeline:

1. Parse an instance (text format).
2. Produce a carrier action plan (solution file).
3. Validate the plan with the external `checker_windows.exe`.

This README explains repository layout, file formats, how to run everything (including the checker), common issues and developer notes.

Repository layout
----------------
- container-terminal-sim/  
  - Cargo.toml, src/ (main.rs, lib.rs, parser.rs, planner.rs, writer.rs, model/)  
  - builds and runs the simulator that reads an instance and writes a solution file.
- instances/  
  - toy_instance/, small_instances/ — example instance text files.
- solutions/  
  - (generated) solution files. Example: solutions/toy/solution_toy.txt
- checker/  
  - Expected location for the external validator binary `checker_windows.exe`.
- scripts/  
  - run_checker.ps1 — PowerShell wrapper that calls `checker_windows.exe` using full paths (recommended).
- checker_run.log — example of a common checker invocation error (illustrative).

Instance file format (human-readable)
-------------------------------------
Files under `instances/` are plain text. Comments after `%` are ignored. The parser is tolerant of headers and spacing. Basic structure:

- First numeric line: map width height
- `crane section` header then number of cranes, then one line per crane:
  - crane line: id x1 y1 x2 y2 nd d1x d1y [d2x d2y ...]
- `storage section` header then number of storage areas, then each line:
  - storage line: id bl_x bl_y
- `carrier section` header then number of carriers, then each line:
  - carrier line: id crane_assignment bl_x bl_y
- `container section` header then number of initially placed containers, then each line:
  - container line: container_id storage_id
- `demand section` header and then demand blocks:
  - demands use `demand crane <crane_id>` then `ship <id>` blocks with lines:
    - `unload crane_id container_id storage_id`
    - `load crane_id container_id`

Parser behavior
---------------
- Implemented in `container-terminal-sim/src/parser.rs`.
- Comments start with `%` and are removed.
- Skips header lines and finds numeric sections automatically.
- Demand operations are case-insensitive (`load` / `unload`).
- Produces an `Instance` struct as defined in `src/model/mod.rs`.

Solution / plan file format
---------------------------
Solution files are plain text where each line is an action in this format:

<time> <carrier_id> <action> [params...]

Examples:
- `0 0 face right`
- `1 0 move 0`
- `2 0 load`
- `3 0 unload`

Time semantics (current implementation)
- `face` consumes 1 time unit.
- `move k` consumes |k| time units.
- `load` and `unload` each consume 1 time unit.

The planner in `src/planner.rs` creates sequences following these rules (simple, sequential logic).

How to run the simulator (build & run)
--------------------------------------
Open a PowerShell terminal at the repository root (Windows):

1. Build & run from the `container-terminal-sim` package:
   - cd container-terminal-sim
   - cargo run --release
   This will:
   - read the instance at `..\instances\toy_instance\toy.txt` (default in main.rs),
   - produce `..\solutions\toy\solution_toy.txt`.

2. To run a different instance or change outputs, edit `main.rs` or modify the code to accept command-line args.

How to run the unit / integration tests
---------------------------------------
- From the `container-terminal-sim` folder:
  - cargo test

Using the external checker
--------------------------
The checker binary must be placed in `checker/checker_windows.exe`.

Manual invocation (recommended to use absolute paths):
- Open PowerShell and change to the `checker` folder or give absolute paths:
  - cd c:\Users\limak\Documents\kuleuven\PSA\VSCODE_PROJWT\Antwerp_Containers_Algorithm\checker
  - .\checker_windows.exe --instance "C:\...\instances\toy_instance\toy.txt" --solution "C:\...\solutions\toy\solution_toy.txt"

PowerShell wrapper (provided)
- Use the included script `scripts\run_checker.ps1` which:
  - resolves absolute paths,
  - unblocks the checker file (tries `Unblock-File`),
  - changes directory to the checker folder and runs the checker with the required flags.

Example (from repo root):
- PowerShell:
  - Set execution policy for the session (if needed):
    - Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
  - Then run:
    - .\scripts\run_checker.ps1 -InstancePath ".\instances\toy_instance\toy.txt" -SolutionPath ".\solutions\toy\solution_toy.txt"

Common checker error and cause
------------------------------
If you see:
- "error: unexpected argument '..\instances\toy_instance\toy.txt' found"
This means the checker was called with positional arguments instead of flag-style arguments. The checker requires the flags:
- --instance <FILE>
- --solution <FILE>

The provided PowerShell wrapper calls the checker correctly using flags and absolute paths. Use it or call the checker manually with `--instance` and `--solution`.

Troubleshooting
---------------
- Permission: Windows may block downloaded executables. The script attempts `Unblock-File`. If execution still fails, right-click the exe → Properties → unblock, or run `Unblock-File .\checker_windows.exe`.
- Paths: Always prefer absolute paths. `run_checker.ps1` resolves absolute paths for you.
- Working directory: The checker expects file paths passed with flags; don't pass positional args.
- If `cargo run` writes output into `..\solutions\...` ensure `solutions/` dir exists or the program will create it (main.rs creates parent dirs).

Developer notes & extension points
----------------------------------
- parser.rs: robust but simple; can be extended to validate semantic constraints (unique ids, coordinates inside map, etc).
- planner.rs: current `plan_sequential` uses a dummy sequential strategy. Replace with an optimized planner that considers multiple carriers, travel time, and concurrency.
- writer.rs: helpers to write outputs and solutions; can be reused by new planners.
- model/mod.rs: central data types (Crane, Storage, Carrier, Demand, Instance). Add fields (e.g., orientation, capacity) as needed.

Example files
-------------
- Instance example: `instances/toy_instance/toy.txt`
- Example solution created by current main: `solutions/toy/solution_toy.txt` (already included in repo)
- Checker wrapper: `scripts/run_checker.ps1`

License & contacts
------------------
- No explicit license included. Add a LICENSE file if needed.
- For questions: open an issue in your local workflow or modify the code and run tests.

Quick checklist to run everything (1,2,3)
-----------------------------------------
1. Build and run simulator:
   - cd container-terminal-sim
   - cargo run --release
2. Ensure `checker_windows.exe` is in the `checker` folder and unblocked.
3. Validate solution:
   - From repo root PowerShell:
     - Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
     - .\scripts\run_checker.ps1 -InstancePath ".\instances\toy_instance\toy.txt" -SolutionPath ".\solutions\toy\solution_toy.txt"

If you want, I can:
- add CLI args to `main.rs` to pass instance/solution paths,
- improve `parser.rs` validation,
- implement a better planner, or
- produce a small CONTRIBUTING.md.