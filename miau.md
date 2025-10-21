Commands to run (exact)

Build & run (generate solution):

cd container-terminal-sim
cargo run --release
Result: writes solution_toy.txt
Run only first demand:

cd container-terminal-sim
cargo run --release -- first
Result: writes ..\solutions\toy\solution_toy_first.txt
Validate with checker (PowerShell, from repo root):

Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
run_checker.ps1 -InstancePath ".\instances\toy_instance\toy.txt" -SolutionPath ".\solutions\toy\solution_toy.txt"