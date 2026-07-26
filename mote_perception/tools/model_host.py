"""Load an inference model on demand and release it again when idle.

The inference machine is not necessarily a dedicated server — it may be someone's
gaming PC. Holding ~1 GB of VRAM resident so a robot that is parked and idle can
have depth "ready" is rude on a machine whose owner wants to play a game, so the
model is loaded on the first request and freed after `idle_timeout` seconds with
no traffic. Serving a frame after an idle period pays the model load (a few
seconds); serving continuously never unloads.

This keeps "on demand" a property of the *server*, not of the deployment: it
behaves the same whether the container runs on a gaming PC, a Linux box, or a
cloud GPU instance, with no orchestration, socket activation, or wake proxy.

Lives in tools/ (server side) rather than the mote_perception package, which is
installed on the robot and stays torch-free.
"""

import time


class ModelHost:
    """A lazily-loaded model with an idle release timer.

    `load` is a zero-arg callable returning whatever the server needs (typically
    a (processor, model) tuple). `idle_timeout` of 0 disables releasing, keeping
    the old always-resident behaviour for a truly dedicated box.
    """

    def __init__(self, load, idle_timeout=300.0, log=print):
        self._load = load
        self.idle_timeout = float(idle_timeout)
        self.log = log
        self._obj = None
        self._last_used = None

    @property
    def loaded(self):
        return self._obj is not None

    def get(self):
        """The model, loading it if this is the first use since a release."""
        if self._obj is None:
            t0 = time.perf_counter()
            self._obj = self._load()
            self.log(f"model loaded in {time.perf_counter() - t0:.1f} s")
        self._last_used = time.monotonic()
        return self._obj

    def release_if_idle(self):
        """Free the model (and its VRAM) if nothing has been served recently.

        Called from the server's select() timeout, so it runs whether the robot
        has disconnected or is connected but quiet.
        """
        if self._obj is None or not self.idle_timeout or self._last_used is None:
            return False
        if time.monotonic() - self._last_used < self.idle_timeout:
            return False
        self._obj = None
        self._last_used = None
        try:
            import gc

            import torch

            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception:  # releasing is best-effort; never take the server down
            pass
        self.log(f"idle for {self.idle_timeout:.0f} s; released model")
        return True
