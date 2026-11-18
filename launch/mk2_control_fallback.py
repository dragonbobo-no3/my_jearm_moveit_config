"""One-way local control-loss transition; NOT a native pendant handover ACK.

Wait for the write process and execution servers to exit before constructing a
new plan-only server. Preserve visualization clients, which cannot enable the
new server. Never restart a driver, clear a fault, or reuse execution requests.
"""

from launch.actions import EmitEvent, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.events.process import ShutdownProcess


class ControlFallback:
    def __init__(self, execution_frontend, readonly_factory, persistent_frontend=()):
        self.frontend = tuple(execution_frontend)
        self.persistent_frontend = tuple(persistent_frontend)
        self.readonly_factory = readonly_factory
        self.scheduled = False
        self.control_lost = False
        self.exited = set()
        self.started_readonly = False

    @staticmethod
    def stop(process):
        # Target this launch action only, never a process name or external stack.
        return EmitEvent(event=ShutdownProcess(process_matcher=lambda action: action is process))

    def handlers(self):
        actions = []
        for process in self.frontend:
            actions.extend([
                RegisterEventHandler(OnProcessStart(
                    target_action=process,
                    on_start=lambda event, context, p=process: self.process_started(p, context))),
                RegisterEventHandler(OnProcessExit(
                    target_action=process,
                    on_exit=lambda event, context, p=process: self.process_exited(p, context))),
            ])
        return actions

    def start_frontend(self, context):
        if context.is_shutdown or self.control_lost or self.scheduled:
            return []
        self.scheduled = True
        return [*self.frontend, *self.persistent_frontend]

    def process_started(self, process, context):
        # Covers a successful spawner callback queued just before control loss.
        if not context.is_shutdown and self.control_lost and process in self.frontend:
            return [self.stop(process)]
        return []

    def process_exited(self, process, context):
        self.exited.add(process)
        return self._finish(context)

    def controller_exited(self, event, context):
        if context.is_shutdown or self.control_lost:
            return []
        self.control_lost = True
        actions = [LogInfo(msg=(
            f'MK2 CONTROL_UNAVAILABLE: controller exited ({event.returncode}). '
            'Native fault/ownership remains UNKNOWN or FAULT, NOT a pendant handover ACK. '
            'Keeping independent feedback and RViz; retiring execution server. '
            'No automatic clear/recover, driver restart or trajectory replay.'))]
        if self.scheduled:
            actions.extend(self.stop(p) for p in self.frontend if p not in self.exited)
        actions.extend(self._finish(context))
        if not self.started_readonly:
            from launch.actions import OpaqueFunction
            actions.append(TimerAction(period=25.0, actions=[OpaqueFunction(function=self.deadline)]))
        return actions

    def _finish(self, context):
        if (context.is_shutdown or not self.control_lost or self.started_readonly or
                (self.scheduled and not all(p in self.exited for p in self.frontend))):
            return []
        self.started_readonly = True
        return [LogInfo(msg=(
            'MK2 FEEDBACK_ONLY_LATCHED: local execution processes exited; '
            'starting a fresh plan-only server, retaining RViz if scheduled. '
            'Execution remains disabled even if RViz still shows an Execute button. '
            'Physical stop/pendant ownership '
            'is NOT inferred from local process exit. New control requires operator '
            'inspection and a new launch with native preflight.')),
            *self.readonly_factory(),
            *(self.persistent_frontend if not self.scheduled else ())]

    def deadline(self, context):
        if context.is_shutdown or self.started_readonly:
            return []
        return [LogInfo(msg='MK2 execution front end exit unconfirmed; refusing read-only replacement.'),
                EmitEvent(event=Shutdown(reason='MK2 front end retirement timeout'))]