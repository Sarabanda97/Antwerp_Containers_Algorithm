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
The checker expects blocks per carrier. This project writes solution files like:

- Header line per carrier: `carrier <id>`
- Then action lines with absolute time and action (no carrier id on action lines):
  - `0 face right`
  - `1 move 3`
  - `4 load`
  - `5 unload`

Time semantics (current implementation)
- `face` consumes 1 time unit
- `move k` consumes |k| time units (k may be negative to move backward)
- `load` and `unload` each consume 1 time unit

The toy solution is currently produced by the hardcoded planner in `src/planner_simple.rs` and matches the included reference.

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

Open the GUI quickly (VS Code Task)
-----------------------------------
- Use the task "Open Checker GUI (Toy)" via Command Palette → Run Task.
- Or run the helper script directly from the repo root:
  - `pwsh -NoProfile -ExecutionPolicy Bypass -File .\scripts\open_checker_gui.ps1 -InstancePath ".\instances\toy_instance\toy.txt" -SolutionPath ".\solutions\toy\solution_toy.txt"`

Common checker error and cause
------------------------------
If you see:
- "error: unexpected argument '..\instances\toy_instance\toy.txt' found"
This means the checker was called with positional arguments instead of flag-style arguments. The checker requires the flags:
- --instance <FILE>
- --solution <FILE>

The provided PowerShell wrapper calls the checker correctly using flags and absolute paths. Use it or call the checker manually with `--instance` and `--solution`.

Developer notes & extension points
----------------------------------
- `parser.rs`: robust but simple; can be extended to validate semantic constraints (unique ids, coordinates inside map, etc.)
- `planner_simple.rs`: contains a hardcoded toy solution that passes the checker.
- `planner.rs`: contains exploratory sequential logic; not used for the validated toy plan.
- `writer.rs`: helpers to write outputs/solutions in the required format.
- `model/mod.rs`: core data types; extend as needed (e.g., orientation, capacity).

Example files
-------------
- Instance example: `instances/toy_instance/toy.txt`
- Produced solution: `solutions/toy/solution_toy.txt`
- Reference solution: `solutions/toy/solution_toy_reference.txt`
- Checker wrappers: `scripts/run_checker.ps1`, `scripts/open_checker_gui.ps1`

License & contacts
