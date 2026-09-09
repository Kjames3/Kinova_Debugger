# Kinova Gen3 Debugger

Read-only diagnostics for a Kinova Gen3 7-DoF arm. The tools do not command motion or change arm configurations. 

## Setup

`kortex_api` is not published on PyPI, so `pip install -r requirements.txt` alone
is not enough. Obtain the wheel from the Kinova download portal or the
`ros2_kortex` release assets, then install it by path before the rest:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install /path/to/kortex_api-2.6.0.post3-py3-none-any.whl
pip install -r requirements.txt
```

The second command pins `protobuf==3.20.3`, overriding the `protobuf==3.5.1` that
the Kortex wheel requires. That old pin fails to import on Python 3.10 and newer
(`AttributeError: module 'collections' has no attribute 'MutableMapping'`). pip
prints a dependency-conflict warning about the override; that is expected, and
the SDK works correctly against 3.20.3.

Verify the install:

```bash
python diagnose_software.py
```

## Run diagnostics

Run the complete suite:

```bash
python diagnose_arm.py --host 192.168.1.10
```

Run individual checks:

```bash
python diagnose_software.py
python diagnose_network.py 192.168.1.10
python diagnose_hardware.py 192.168.1.10
```

`diagnose_hardware.py` uses the Kortex SDK to create a read-only session and query product configuration and device inventory. The default Kortex API port is `10000`; change it with `--port` when the arm uses a different configuration.

## Stored actions and protection zones

The arm stores a handful of fixed joint targets (`Home`, `Retract`, `Packaging`,
`Zero`) and a set of protection zones. A controller button that "goes home" runs
one of these stored actions. If the action's target already lies inside an
enabled zone, the move cannot complete: the controller either raises a
protection-zone fault or stops short of the target without faulting.

Audit every stored action against every enabled zone:

```bash
python diagnose_actions.py --host 192.168.1.10
```

Each action is converted to a tool pose using the arm's own forward kinematics,
so the result reflects the configured tool transform. A `FAIL` reports how many
millimetres inside the zone the target sits.

## Joint calibration (model vs. physical reality)

The wrist interconnect module has an IMU. Stationary, it reads gravity — a
direct physical measurement of the wrist's orientation that does not depend on
the joint encoders. Two correctly-calibrated arms at the same joint angles must
read the same gravity vector.

Capture a reference from a known-good arm, then compare:

```bash
# on the good arm
python diagnose_calibration.py --host 192.168.1.10 --save-reference good.json
# on the suspect arm, driven to the same stored action
python diagnose_calibration.py --host 192.168.1.10 --reference good.json
```

A large difference means the joint zeros are wrong: the controller believes the
arm is somewhere it physically is not, so forward kinematics, protection zones
and collision checks are all computed against a pose the arm is not in.

## Watching a fault happen

`monitor_arm.py` is a read-only live monitor. Start it, then press the
controller button under investigation. It prints fault-bank changes, arm-state
transitions, and — once motion settles — the resting joint angles, which stored
action they were closest to, and the per-joint error if the arm stopped short.

```bash
python monitor_arm.py --host 192.168.1.10 --csv run.csv
```

`--csv` appends every sample (joint angles, tool pose, fault banks, zone margin)
for later inspection. Neither script ever commands motion.

Use `--json` with `diagnose_arm.py` when another script needs machine-readable results:

```bash
python diagnose_arm.py --host 192.168.1.10 --json
```

Exit code `0` means no check failed. Exit code `1` means at least one check failed.
