"""Idle USB connections for the local workbench header.

The experiment script owns its own connections while a workflow runs. This
manager releases both idle connections before that subprocess starts.
"""
import asyncio
import threading


class MotorConnections:
    def __init__(self, driver=None):
        self.driver = driver
        self.lock = threading.RLock()
        self.loop = None
        self.thread = None
        self.operation = None
        self.boards = {}
        self.states = {role: {'state': 'disconnected', 'detail': '', 'open': False} for role in ('test', 'load')}

    def status(self):
        with self.lock:
            return {role: dict(value) for role, value in self.states.items()}

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
        with self.lock:
            boards = list(self.boards.items())
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
            future = self._submit(driver.emergency_idle(boards))
            errors = future.result(timeout=15)
        except Exception as exc:
            raise ValueError(f'Could not stop connected motors: {exc}') from exc
        if errors:
            raise ValueError('Could not verify both motors stopped: ' + '; '.join(errors))
        return self.status()
