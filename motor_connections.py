"""Idle USB connections for the local workbench header.

The experiment script owns its own connections while a workflow runs. This
manager releases both idle connections before that subprocess starts.
"""
import asyncio
import math
import threading
import time
from collections import deque
from datetime import datetime, timezone


class MotorConnections:
    def __init__(self, driver=None):
        self.driver = driver
        self.lock = threading.RLock()
        self.loop = None
        self.thread = None
        self.operation = None
        self.boards = {}
        self.io_lock = asyncio.Lock()
        self.monitor_task = None
        self.rows = deque(maxlen=240)
        self.sampled_at = None
        self.states = {role: {'state': 'disconnected', 'detail': '', 'open': False} for role in ('test', 'load')}

    def status(self):
        with self.lock:
            return {role: dict(value) for role, value in self.states.items()}

    def live(self):
        with self.lock:
            return {'source': 'Connected USB monitoring' if self.boards else None,
                    'rows': list(self.rows), 'sampled_at': self.sampled_at,
                    'connections': self.status()}

    def _set(self, role, state, detail='', open=False):
        with self.lock:
            self.states[role] = {'state': state, 'detail': detail, 'open': open}

    def _ensure_loop(self):
        with self.lock:
            if self.loop is None:
                self.loop = asyncio.new_event_loop()
                self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
                self.thread.start()

    def _submit(self, coroutine):
        self._ensure_loop()
        return asyncio.run_coroutine_threadsafe(coroutine, self.loop)

    def connect_both(self):
        with self.lock:
            if self.operation is not None and not self.operation.done():
                raise ValueError('A motor connection action is already in progress.')
            for role in self.states:
                if role not in self.boards:
                    self.states[role] = {'state': 'connecting', 'detail': 'Opening USB connection…', 'open': False}
            self.operation = self._submit(self._connect_both())
        return self.status()

    async def _connect_both(self):
        for role in ('test', 'load'):
            with self.lock:
                already_open = role in self.boards
            if already_open:
                continue
            serial = None
            board = None
            try:
                if self.driver is None:
                    import AllTests as driver
                else:
                    driver = self.driver
                serial = driver.SERIALS[role]
                board = await self._connect_driver(driver, serial, role)
                state = await driver.read(board, 'axis0.current_state')
                armed = await driver.read(board, 'axis0.is_armed')
                with self.lock:
                    self.boards[role] = board
                self._set(role, 'connected', f'USB connected · axis state {state} · armed {bool(armed)}', open=True)
            except Exception as exc:
                if board is not None:
                    try:
                        self._release_board(board)
                    except Exception as release_exc:
                        with self.lock:
                            self.boards[role] = board
                        self._set(role, 'error', f'{serial}: {exc}; USB release failed: {release_exc}', open=True)
                        continue
                with self.lock:
                    still_open = role in self.boards
                self._set(role, 'error', f'{serial or role}: {exc}', open=still_open)
        with self.lock:
            has_boards = bool(self.boards)
        if has_boards and (self.monitor_task is None or self.monitor_task.done()):
            try:
                await self._sample_once()
            except Exception as exc:
                self._monitor_error(exc)
            self.monitor_task = asyncio.create_task(self._monitor_loop())

    async def _monitor_loop(self):
        while True:
            await asyncio.sleep(0.75)
            try:
                await self._sample_once()
            except Exception as exc:
                self._monitor_error(exc)

    def _monitor_error(self, exc):
        with self.lock:
            roles = tuple(self.boards)
        for role in roles:
            self._set(role, 'error', f'Live USB monitoring failed: {exc}', open=True)

    async def _sample_once(self):
        with self.lock:
            boards = dict(self.boards)
        if not boards:
            return
        if self.driver is None:
            import AllTests as driver
        else:
            driver = self.driver

        async def one(role, board):
            if self.driver is not None and hasattr(driver, 'sample'):
                sample = await driver.sample(board)
            else:
                signals = driver.SIGNALS
                values = await asyncio.wait_for(board.read_multiple(list(signals.values()), transient=True), 2)
                sample = dict(zip(signals, values))
                sample['rpm'] *= 60
            cleaned = {}
            for key, value in sample.items():
                if hasattr(value, 'item'):
                    value = value.item()
                cleaned[key] = None if isinstance(value, float) and not math.isfinite(value) else value
            return role, cleaned

        async with self.io_lock:
            outcomes = await asyncio.gather(*(one(role, board) for role, board in boards.items()),
                                            return_exceptions=True)
        samples = {}
        for (role, _), outcome in zip(boards.items(), outcomes):
            if isinstance(outcome, BaseException):
                self._set(role, 'error', f'Live USB read failed: {outcome}', open=True)
                continue
            _, sample = outcome
            samples[role] = sample
            self._set(role, 'connected',
                      f'USB connected · state {sample.get("state")} · armed {bool(sample.get("armed"))} · '
                      f'active errors {int(sample.get("errors") or 0):#x} · '
                      f'last disarm {int(sample.get("disarm") or 0):#x}', open=True)
        if samples:
            with self.lock:
                self.rows.append({'host_monotonic_s': time.monotonic(), 'boards': samples})
                self.sampled_at = datetime.now(timezone.utc).isoformat()

    async def _connect_driver(self, driver, serial, role):
        if self.driver is not None:
            return await driver.connect(serial)
        # Reuse AllTests.connect's serial and firmware checks, while retaining
        # the device handle so a failed check can release USB in this process.
        import odrive
        from odrive.runtime_device import RuntimeDevice
        found = None

        async def finder(**kwargs):
            nonlocal found
            found = await odrive.find_async(return_type=RuntimeDevice, **kwargs)
            return found

        try:
            return await driver.connect(serial, finder=finder)
        except Exception as exc:
            if found is not None:
                try:
                    self._release_board(found)
                except Exception as release_exc:
                    with self.lock:
                        self.boards[role] = found
                    raise ValueError(f'{exc}; USB release failed: {release_exc}') from exc
            raise

    def _release_board(self, board):
        if self.driver is None:
            from odrive.device_manager import get_device_manager
            get_device_manager().release_connection(board)
        else:
            self.driver.release_connection(board)

    async def _release_all(self):
        if self.monitor_task is not None:
            self.monitor_task.cancel()
            try:
                await self.monitor_task
            except asyncio.CancelledError:
                pass
            except Exception:
                pass  # A failed read must not prevent USB handoff.
            self.monitor_task = None
        with self.lock:
            boards = list(self.boards.items())
            self.rows.clear()
            self.sampled_at = None
        if not boards:
            for role in self.states:
                self._set(role, 'disconnected')
            return
        errors = []
        for role, board in boards:
            try:
                self._release_board(board)
                with self.lock:
                    del self.boards[role]
                self._set(role, 'disconnected')
            except Exception as exc:
                self._set(role, 'error', f'USB release failed: {exc}', open=True)
                errors.append(f'{role}: {exc}')
        if errors:
            raise ValueError('Could not release USB connections: ' + '; '.join(errors))
        for role in self.states:
            self._set(role, 'disconnected')

    def release_all(self):
        with self.lock:
            current = self.operation
        try:
            if current is not None and not current.done():
                current.result(timeout=60)
            future = self._submit(self._release_all())
            future.result(timeout=15)
        except Exception as exc:
            raise ValueError(f'USB connection handoff failed: {exc}') from exc
        with self.lock:
            self.operation = None
        return self.status()

    def stop_connected(self):
        with self.lock:
            boards = dict(self.boards)
        if not boards:
            return self.status()
        if self.driver is None:
            import AllTests as driver
        else:
            driver = self.driver
        try:
            async def stop():
                async with self.io_lock:
                    return await driver.emergency_idle(boards)
            future = self._submit(stop())
            errors = future.result(timeout=15)
        except Exception as exc:
            raise ValueError(f'Could not stop connected motors: {exc}') from exc
        if errors:
            raise ValueError('Could not verify both motors stopped: ' + '; '.join(errors))
        return self.status()

    def clear_errors(self):
        with self.lock:
            if set(self.boards) != {'test', 'load'}:
                raise ValueError('Connect both motors before clearing errors.')
            current = self.operation
        if current is not None and not current.done():
            raise ValueError('Wait for both motor connections to finish.')
        try:
            return self._submit(self._clear_errors()).result(timeout=15)
        except Exception as exc:
            raise ValueError(f'Could not clear errors on both motors: {exc}') from exc

    async def _clear_errors(self):
        if self.driver is None:
            import AllTests as driver
        else:
            driver = self.driver
        with self.lock:
            boards = dict(self.boards)
        async with self.io_lock:
            for role, board in boards.items():
                state = await driver.read(board, 'axis0.current_state')
                armed = await driver.read(board, 'axis0.is_armed')
                if state != 1 or armed:
                    raise ValueError(f'{role} must be IDLE and disarmed before clearing errors.')
            results = {}
            for role, board in boards.items():
                await driver.call(board, 'clear_errors')
                results[role] = {
                    'active': int(await driver.read(board, 'axis0.active_errors')),
                    'disarm': int(await driver.read(board, 'axis0.disarm_reason')),
                }
            return results
