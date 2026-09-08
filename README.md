# Kinova Gen3 Debugger

Read-only diagnostics for a Kinova Gen3 7-DoF arm. The tools do not command motion or change arm configuration.

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

Use `--json` with `diagnose_arm.py` when another script needs machine-readable results:

```bash
python diagnose_arm.py --host 192.168.1.10 --json
```

Exit code `0` means no check failed. Exit code `1` means at least one check failed.
