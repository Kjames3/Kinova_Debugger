"""Audit the arm's stored actions against its enabled protection zones.

Every stored action (Home, Retract, Packaging, ...) is a fixed joint target. The
arm's own forward kinematics turns each into a tool pose, which is then tested
against every enabled protection zone. An action whose target already sits
inside a zone cannot complete: the controller either faults on the zone or stops
short of the target.
"""

from .models import CheckResult, Status
from .session import kortex_session
from .zones import penetration


def _joint_angles(values):
    from kortex_api.autogen.messages import Base_pb2

    message = Base_pb2.JointAngles()
    for index, value in enumerate(values):
        angle = message.joint_angles.add()
        angle.joint_identifier = index
        angle.value = float(value)
    return message


def _button_bindings(base, Base_pb2):
    """Yield (map_name, input_id, behavior, action_name, action_type) for digital inputs."""
    for config in base.GetAllControllerConfigurations().controller_configurations:
        mapping = base.ReadMapping(config.active_mapping_handle)
        for map_handle in mapping.map_handles:
            mapped = base.ReadMap(map_handle)
            for element in mapped.elements:
                event = element.event.controller_event
                if Base_pb2.ControllerInputType.Name(event.input_type) != "DIGITAL":
                    continue
                yield (mapped.name, event.input_identifier,
                       Base_pb2.ControllerBehavior.Name(event.behavior),
                       element.action.name,
                       Base_pb2.ActionType.Name(element.action.handle.action_type))


def run_mapping_checks(host: str, port: int = 10000) -> list[CheckResult]:
    """Report which controller buttons run stored actions, and whether they latch.

    A button bound to EXECUTE_ACTION on BUTTON_DOWN and STOP_ACTION on BUTTON_UP is
    momentary: it must be held until the arm settles. Releasing early stops the
    move partway with no fault, which looks like the arm ignoring the target.
    """
    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
    from kortex_api.autogen.messages import Base_pb2

    results: list[CheckResult] = []
    with kortex_session(host, port) as router:
        base = BaseClient(router)
        bindings = list(_button_bindings(base, Base_pb2))

        executes = {}
        stops = set()
        for map_name, input_id, behavior, action_name, action_type in bindings:
            if action_type == "EXECUTE_ACTION":
                executes.setdefault(input_id, set()).add(action_name)
            elif action_type == "STOP_ACTION":
                stops.add(input_id)

        if not executes:
            results.append(CheckResult("button-actions", Status.PASS,
                                       "No controller button runs a stored action."))
        for input_id, names in sorted(executes.items()):
            label = "/".join(sorted(names))
            name = f"button-{input_id}"
            if input_id in stops:
                results.append(CheckResult(
                    name, Status.WARN,
                    f"Input #{input_id} runs '{label}' while held and stops it on release.",
                    "Hold the button until the arm settles. A tap stops the move partway "
                    "with no fault, which looks like the arm missing its target.",
                    details={"action": label, "momentary": True},
                ))
            else:
                results.append(CheckResult(name, Status.PASS,
                                           f"Input #{input_id} runs '{label}' and latches.",
                                           details={"action": label, "momentary": False}))
    return results


def run_action_checks(host: str, port: int = 10000) -> list[CheckResult]:
    try:
        from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
        from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
        from kortex_api.autogen.messages import Base_pb2
    except ImportError as error:
        return [CheckResult("kortex-import", Status.FAIL, f"Kortex SDK imports are unavailable: {error}",
                            "Activate the venv: source .venv/bin/activate")]

    results: list[CheckResult] = []
    try:
        with kortex_session(host, port) as router:
            base = BaseClient(router)
            control_config = ControlConfigClient(router)

            tool = control_config.GetToolConfiguration()
            reach = tool.tool_transform.z
            results.append(CheckResult(
                "tool-configuration", Status.PASS,
                f"Tool transform z={reach:.3f} m, mass={tool.tool_mass:.3f} kg, "
                f"mass centre z={tool.tool_mass_center.z:.3f} m.",
                details={"tool_transform_z": reach, "tool_mass": tool.tool_mass},
            ))

            zones = [z for z in base.ReadAllProtectionZones().protection_zones if z.is_enabled]
            if not zones:
                results.append(CheckResult("protection-zones", Status.PASS, "No protection zones are enabled."))
            else:
                names = ", ".join(z.name for z in zones)
                results.append(CheckResult("protection-zones", Status.PASS,
                                           f"{len(zones)} enabled zone(s): {names}."))

            for action in base.ReadAllActions(Base_pb2.RequestedActionType()).action_list:
                if not action.HasField("reach_joint_angles"):
                    continue
                targets = [a.value for a in action.reach_joint_angles.joint_angles.joint_angles]
                pose = base.ComputeForwardKinematics(_joint_angles(targets))
                point = (pose.x, pose.y, pose.z)

                hits = [penetration(point, zone) for zone in zones]
                worst = max(hits, key=lambda h: h.depth, default=None)
                name = f"action-{action.name.lower()}"
                where = f"tool at ({pose.x:+.3f}, {pose.y:+.3f}, {pose.z:+.3f}) m"

                if worst is not None and worst.violates:
                    results.append(CheckResult(
                        name, Status.FAIL,
                        f"'{action.name}' target is {worst.depth * 1000:.0f} mm inside "
                        f"'{worst.zone_name}' — {where}.",
                        "Shorten the tool transform if it overstates the tool, retarget the action, "
                        "or resize the zone. See the Stored actions section of README.md.",
                        details={"joint_targets": targets, "penetration_m": worst.depth,
                                 "zone": worst.zone_name, "geometry": worst.detail},
                    ))
                else:
                    margin = f", clears '{worst.zone_name}' by {-worst.depth * 1000:.0f} mm" if worst else ""
                    results.append(CheckResult(name, Status.PASS, f"'{action.name}' target is reachable — {where}{margin}.",
                                               details={"joint_targets": targets}))
        results.extend(run_mapping_checks(host, port))
    except Exception as error:
        results.append(CheckResult("action-audit", Status.FAIL, f"Could not audit stored actions: {error}",
                                   "Check the connection and that no other client holds the arm."))
    return results
