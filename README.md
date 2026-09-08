# Kinova Gen3 Debugger

Read-only diagnostics for a Kinova Gen3 7-DoF arm. The tools do not command motion or change arm configuration.

## Setup

```powershell
python -m pip install -r requirements.txt
```

## Run diagnostics

Run the complete suite:

```powershell
python diagnose_arm.py --host 192.168.1.10
```

Run individual checks:

```powershell
python diagnose_software.py
python diagnose_network.py 192.168.1.10
python diagnose_hardware.py 192.168.1.10
```

`diagnose_hardware.py` uses the Kortex SDK to create a read-only session and query product configuration and device inventory. The default Kortex API port is `10000`; change it with `--port` when the arm uses a different configuration.

Use `--json` with `diagnose_arm.py` when another script needs machine-readable results:

```powershell
python diagnose_arm.py --host 192.168.1.10 --json
```

Exit code `0` means no check failed. Exit code `1` means at least one check failed.
