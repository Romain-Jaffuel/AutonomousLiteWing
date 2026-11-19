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
`requires-python >=3.11,<3.12`. One command sets everything up:

```sh
uv sync
```

`pyproject.toml` and `uv.lock` pin `cflib` and `litewing` to exact commits.
Two settings in there are deliberate: `native-tls`, needed where TLS is
intercepted and the bundled roots fail with `UnknownIssuer`, and an override
forcing the pinned `cflib`, because `litewing`'s own metadata depends on an
unpinned branch archive of the same fork and would silently move it. Git needs
the same TLS fix: `git config --global http.sslBackend schannel`.

The PC joins the drone's own Wi-Fi network, so it has no internet on that
interface. A static address on the wireless adapter is required; a second
adapter or wired connection keeps internet available.

## Use
This shouldn't lift the drone : 
```sh
uv run scripts/lw.py check            # pre-flight, no motors, PASS/FAIL, exit 0/1
uv run scripts/lw.py ramp --debut 20000 --fin 24000 --montee 3 --descente 3
uv run scripts/lw.py vol --hauteur 0.5 --duree 3     # needs the position module
uv run scripts/lw.py voir --png       # summary and plot of the last run
```

`dump` records the firmware's parameters and log variables, which is how a
firmware flash is diffed:

```sh
uv run scripts/lw.py dump --sortie data/apres-flash
uv run scripts/lw.py diff             # flags critical changes with a leading !
```

Only one process may talk to the drone: the UDP driver binds a fixed local
port. `voir` reads files and never connects, so it can run at any time.

## Layout

```
scripts/safety.py       limits and keyboard abort, the single source
scripts/lw.py           command line: dump, diff, check, ramp, vol, cam, voir
scripts/analyse.py      summary.json, printed summary, PNG plot
scripts/camera.py       MJPEG stream and the drone's ping under it
scripts/bench_vision.py detector cost on this machine, separate environment
data/baseline/          firmware state captured before flashing
data/runs/<stamp>/      one folder per run: CSVs, run.txt, summary.json
data/bench/             detector measurements, one JSON per run
```

## Measuring what a detector costs here

Vision lives in its own environment, kept apart from the pinned one above so
the drone tooling stays small:

```sh
uv venv .venv-vision --python 3.11
uv pip install --python .venv-vision --index-url https://download.pytorch.org/whl/cpu torch torchvision
uv pip install --python .venv-vision ultralytics openvino truststore
.venv-vision/Scripts/python.exe scripts/bench_vision.py
```

The window is one flight, 180 s, reported per 30 s slice: a ten-minute soak
test says little when a flight lasts two or three minutes, and an average
hides a chip that starts fast and slows down. Devices the machine does not
have are reported absent rather than timed, because OpenVINO otherwise falls
back to the CPU and passes the result off as an NPU figure.

`truststore` is not optional where TLS is intercepted: without it Python's
bundled roots reject the model download.

## Reading a run

Thrust is on the firmware's 0–65535 scale, and figures are only comparable
between runs **at equal battery voltage**: a brushed motor's thrust falls with
voltage, so the same command gives less lift on a tired pack, and the spread
between motors grows even though the balance has not changed. `analyse.py`
prints the voltage next to the spread and withholds the balance verdict below
3.7 V for that reason.
