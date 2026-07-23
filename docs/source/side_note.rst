Note to the next developer
==========================

- The Kalman Filter ``Q`` and ``R`` matrices are not properly tuned.
- The bug in the introduction video may be caused by the position-data wrapper algorithm that is
  flashed inside the STM32. This wrapper does not change the zero position when it receives a
  set-home command.
- On test day, we recommend NOT using a student's laptop, because the teaching team's laptop is
  more controlled and avoids issues that can be caused by the LSL protocol (LSL needs all devices
  on the same local network to find each other, so running the whole system locally eliminates
  some uncertainties).
- Multi-machine operation isn't tested yet. However, with the namespace, the localhost web UI, and
  the WebSocket, it should have no problem.
