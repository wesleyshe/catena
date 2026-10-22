# AGENTS.md: CATENA

Instructions for any coding agent (Claude Code, Codex, Cursor, Copilot, ...) working in this repo.
Read this file first, then the `AGENTS.md` in the folder you're working in.

## What this is
- **CATENA** is a chain of 12 spherical modules joined by hinges that folds itself into shapes: the MIT Universal
  Self-Folding Chain Robot (Mutis et al., ACM SCF 2025). Each module has up to two Dynamixel AX-12A servos, yaw and pitch.
- **Goal:** design microgravity (parabolic flight) experiments with it for MIT 16.88J / Columbia AERO E4450,
  Fall 2026. The proposal is `docs/CATENA Proposal.docx`, and `experiments/README.md` summarises the experiments.
- **Course dates:**
  - PDR (Preliminary Design Review): 27/29 Oct 2026.
  - CDR (Critical Design Review): 1/3 Dec.
  - PIP (Payload Integration Package) due: 10 Dec.
  - Flights are picked in IAP / spring 2027. The schedule is in `docs/16.88J_E4450 Syllabus Fall 2026_20260914.pdf`.

## Map
| Folder | What's in it | Read first |
|---|---|---|
| `hardware/` | CAD (`cad/*.3dm`), measurements, kinematics, BOM decisions | `hardware/AGENTS.md` |
| `software/` | Code for the real chain (servo control, logging, telemetry). Empty so far. | `software/AGENTS.md` |
| `simulation/` | MuJoCo model of the chain, control window, regression checks | `simulation/AGENTS.md` |
| `sequences/` | Fold sequences: scripted joint moves (CSV) that the sim plays and the real chain will play | `sequences/README.md` |
| `experiments/` | The flight experiments, what the sim predicts for each, measured findings | `experiments/README.md` |
| `docs/` | Proposal, course syllabus, images | |

Each area has its own setup, its own checks, its own `AGENTS.md` and its own `LOG.md`.

## Status (2026-10-22; keep this current)
- **hardware:** The 12-module chain exists and has been published. Per the proposal, flight still needs the soft
  damping membrane, an IMU and onboard data logging.
- **software:** Nothing in this repo yet.
- **simulation:** Working. `verify.py` passes. Used to answer the professor's physics questions
  (`experiments/findings.md`).

## Facts every area shares
These are confirmed by the team. Change one only together with the other areas, and log it in the root `LOG.md`.
- **Modules:** `m1` … `m12`.
  - Each has a base `m{i}` (shell, bracket, servos) and an arm `m{i}_arm` (yoke and hub).
  - Module 12 has no arm.
- **Joints:**
  - `m{i}_pitch` sits between base i and arm i.
  - `m{i+1}_yaw` sits between arm i and base i+1.
  - Module 1 has no yaw servo and module 12 has no pitch servo, so there are **22 actuated joints**.
  - Both end modules carry the U2D2 and power hub.
- **Sequence columns:** a fold sequence has 24 columns in the order `m1_yaw, m1_pitch, …, m12_yaw, m12_pitch`.
  `m1_yaw` and `m12_pitch` are always 0.
- **Ranges:** pitch ±124° (where the hub meets the end of the shell slot) and yaw ±150° (servo range).
- **Size:** 140 mm between module centres; shell Ø139 mm.
- **Units:** degrees in sequence CSVs; SI (m, kg, s, N·m) in code; mm in CAD.

## Rules
- Never modify the `.3dm` files in `hardware/cad/`. Work on exports. A new design goes in as a new file.
- Ask before guessing anything that changes joint locations or which parts move together.
- Keep code short and readable. People edit parameters directly in the code.
- Report measured numbers with units, and flag any that disagree with stated specs.
- Verify before saying something is done. Each area's `AGENTS.md` says how.
- Answer physics questions with a simulation run and measured numbers, not just theory.
- Stay inside your area. If a change affects another area (joint names, the sequence format, a shared fact above),
  log it in the root `LOG.md` and tell the person you're working with.
- Look at anything before deleting or overwriting it.

## When a change crosses areas
The sim and the robot software both depend on the hardware, and nothing updates them automatically yet. So when you
change something on the left, do the rest of its row in the same piece of work, or log what's left undone.

| If you change… | Also do this | Areas affected |
|---|---|---|
| The CAD (a new Rhino design) | Follow "Incorporating a new design" in `hardware/AGENTS.md`: archive the old file, re-export the meshes, update the numbers, run `verify.py`. | hardware, simulation; software if joints or ranges moved |
| A measured number (mass, size, range, servo) | Update `hardware/AGENTS.md`, `simulation/make_chain.py`, and the software's config once it exists. Run `verify.py`. | hardware, simulation, software |
| A shared fact above (joint names, order, ranges, sequence format) | Update this file and `sequences/README.md`, plus both the sim and the software. | everyone |
| The sim's physics (servo model, presets, integrator, timestep) | Run `verify.py`, and re-check any `experiments/findings.md` numbers that depend on it. | simulation, experiments |
| A sequence that `verify.py` or an experiment uses | Re-run `verify.py`, or regenerate that experiment's prediction. | simulation, experiments |

When a change affects another area, add a line to the root `LOG.md` saying what changed and what the other area needs to
do, and tell the person you're working with.

## Logging (every task)
Every folder with an `AGENTS.md` also has a `LOG.md`. **`AGENTS.md` says how things are now; `LOG.md` says what
happened.**

When you finish a task:
1. Add an entry at the **top** of the `LOG.md` of each area you changed. Changes to `sequences/`, `experiments/`,
   `docs/` or root files go in the root `LOG.md`.
   ```
   ## 2026-10-22 · Wesley (Claude Code)
   - What changed and why, in a line or two.
   - Checked: what you ran and the result, e.g. "verify.py: ALL CHECKS PASSED". If you didn't check, say so.
   ```
2. If the task changed a command, file, folder, rule, interface or status, update that area's `AGENTS.md` in the same
   commit. Edit or delete lines that are no longer true. Don't add "update:" notes or history; history goes in `LOG.md`.
3. Keep each `AGENTS.md` under about 250 lines. If one grows past that, move detail into a README in that folder.

- Use real dates (2026-10-22), not "today".
- Name the person you're working with and which tool you are.
- Commit the `LOG.md` and `AGENTS.md` updates together with the change they describe.
- Don't commit what `.gitignore` excludes: environments, run logs, screenshots, caches.
