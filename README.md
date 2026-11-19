# AutonomousLiteWing

Python tooling to measure and fly a [LiteWing](https://circuitdigest.com/)
drone (ESP32, brushed motors, Crazyflie-class firmware) over Wi-Fi, from a PC.

The drone is driven over UDP through a fork of `cflib`. This repository holds
the measurement side: a thrust ramp with on-board logging, pre-flight checks,
and an analysis step that turns a run into a summary and a plot.

## Safety

Propellers spin. Hold the drone, weigh it down, or tether it, keep the area
clear, and stay away from your face. `ramp` can reach flight thrust.

Every run can be stopped three ways: any key decelerates and records the thrust
reached, `x` sheds thrust then cuts, and `Ctrl+C` cuts immediately. Limits live
in one place, `scripts/safety.py`, and a request outside them is refused before
anything spins.

## Requirements

Python **3.11** only — the official `litewing` library pins
`requires-python >=3.11,<3.12`.

```sh
uv venv --python 3.11
# exact pinned commands are in requirements.txt
```

`requirements.txt` pins `cflib` and `litewing` by commit, and documents two
workarounds: `--system-certs`, needed where TLS is intercepted, and `--no-deps`
on `litewing`, whose metadata would otherwise pull an unpinned branch archive
of the `cflib` fork.

The PC joins the drone's own Wi-Fi network, so it has no internet on that
interface. A static address on the wireless adapter is required; a second
adapter or wired connection keeps internet available.

## Use
This shouldn't lift the drone : 
```sh
python scripts/lw.py check            # pre-flight, no motors, PASS/FAIL, exit 0/1
python scripts/lw.py ramp --debut 20000 --fin 24000 --montee 3 --descente 3
python scripts/lw.py voir --png       # summary and plot of the last run
```

`dump` records the firmware's parameters and log variables, which is how a
firmware flash is diffed:

```sh
python scripts/lw.py dump --sortie data/apres-flash
diff data/baseline/params.txt data/apres-flash/params.txt
```

Only one process may talk to the drone: the UDP driver binds a fixed local
port. `voir` reads files and never connects, so it can run at any time.

## Layout

```
scripts/safety.py    limits and keyboard abort, the single source
scripts/lw.py        command line: dump, check, ramp, voir
scripts/analyse.py   summary.json, printed summary, PNG plot
data/baseline/       firmware state captured before flashing
data/runs/<stamp>/   one folder per run: CSVs, run.txt, summary.json
```

## Reading a run

Thrust is on the firmware's 0–65535 scale, and figures are only comparable
between runs **at equal battery voltage**: a brushed motor's thrust falls with
voltage, so the same command gives less lift on a tired pack, and the spread
between motors grows even though the balance has not changed. `analyse.py`
prints the voltage next to the spread and withholds the balance verdict below
3.7 V for that reason.
