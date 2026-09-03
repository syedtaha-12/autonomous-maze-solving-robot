# Autonomous Maze-Solving Robot

This is my embedded C firmware for a Freescale/NXP HCS12 (MC9S12C32) robot that autonomously navigates and solves a physical line-following maze — exploring unknown branches, backtracking out of dead ends, and retracing the shortest learned path home once it reaches the destination.

## What it does

- **Line following** — reads five analog guider sensors (line, bow, port, mid, starboard) each control loop and steers to stay centered on the track.
- **Junction detection & exploration** — on reaching a branch, records which direction it chose at that intersection.
- **Dead-end recovery** — a front-bumper hit backs the robot up, marks the last choice as failed, and retries the other option at that junction. If both options at a junction fail, the robot stops (per my spec, this shouldn't happen in a valid maze).
- **Path retracing** — a rear-bumper signal marks the destination; the robot then turns around and drives back through every recorded junction in reverse, mirroring each turn, to retrace the shortest known path to the start.
- **Live status on the LCD** — battery voltage and current state name, updated every control loop.

## Hardware target

- MCU: Freescale/NXP MC9S12C32 (HCS12 family)
- Platform: line-following/maze robot with LCD, motor drivers, and five analog guider sensors
- Toolchain: CodeWarrior for HC(S)12 (project files included: `.mcp`, `.prm`, simulator configs)

## Repo layout

```
Sources/            C source (main.c is the firmware entry point + state machine)
prm/                Linker parameter file (memory map)
cmd/                Simulator command scripts
*.ini / *.hwl        CodeWarrior/simulator target configuration
autonomous_maze_solving_robot.mcp   CodeWarrior project file
```

I've gitignored `bin/` (build output) and the CodeWarrior `_Data` cache folder — they get regenerated on build.

## Building

Open `autonomous_maze_solving_robot.mcp` in CodeWarrior for HC(S)12, select a build target (simulator or full-chip/hardware), and build. See `Sources/main.c` for my detailed header comment on the firmware's design and the hardware caveats I still need to retune on the physical robot.

## About

Written by Taha Hasan.
