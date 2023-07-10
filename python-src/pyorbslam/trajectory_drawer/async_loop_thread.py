# Built-in
import logging
import threading
import asyncio
import traceback
from concurrent.futures import Future
from typing import Coroutine, Callable, Tuple, List, Optional, Any

logger = logging.getLogger("pyorbslam")

# Reference
# https://stackoverflow.com/a/66055205/13231446


def waitable_callback(
    callback: Callable, args: List[Any]
) -> Tuple[threading.Event, Callable]:

    finished = threading.Event()
    finished.clear()

    # Create wrapper that signals when the callback finished
    def _wrapper(func: Callable, *args) -> Any:
        try:
            output = func(*args)
        except Exception:
            logger.error(traceback.format_exc())
            output = None

        finished.set()
        return output

    wrapper = _wrapper(callback, *args)

    return finished, wrapper


# first, we need a loop running in a parallel Thread
class AsyncLoopThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self._loop = asyncio.new_event_loop()

    def callback(self, coro: Callable[[], Coroutine]):
        async def _wrapper():
            try:
                await coro()
            except Exception:
                logger.error(traceback.format_exc())

        self._loop.create_task(_wrapper())

    def exec(self, coro: Coroutine) -> Future:
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    def exec_noncoro(
        self, callback: Callable, args: List[Any], waitable: bool = False
    ) -> Optional[threading.Event]:

        if waitable:
            finished, wrapper = waitable_callback(callback, args)
            self._loop.call_soon_threadsafe(wrapper, *args)
            return finished

        else:
            self._loop.call_soon_threadsafe(callback, *args)

    def run(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def stop(self):

        # Cancel all tasks
        for task in asyncio.all_tasks(self._loop):
            task.cancel()

        # Then stop the loop
        self._loop.stop()

    def __del__(self):
        self.stop()
