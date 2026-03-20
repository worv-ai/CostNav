# Canvas Neural Planner: Single-Thread Migration

## Background

The Canvas neural planner (`costnav_isaacsim/canvas/src/canvas/agent/neural_planner/neural_planner.py`) was migrated from a multi-threaded ROS2 executor to a single-threaded executor to eliminate lock contention and improve inference stability.

## Changes

### Executor

- `MultiThreadedExecutor` → `SingleThreadedExecutor`
- Removed `ReentrantCallbackGroup` (not needed with single-threaded executor)

### Threading Model

- Removed `self.lock` — no concurrent callbacks in single-threaded mode
- Renamed `self.processor_lock` → `self._processor_lock` (only guards processor access across main + inference threads)
- Added `ThreadPoolExecutor(max_workers=1)` — inference runs in a background thread so the timer loop is not blocked

### Eval Reset

- Replaced blocking `wait_for_eval_reset()` (which called `time.sleep()` in a loop) with non-blocking polling in `loop()` via `eval_reset_wait_start` timestamp

### Callbacks

- Removed `with self.lock:` from `start_pause_callback`, `stop_model_callback`, `eval_resetting_callback`
- Added `_is_processor_ready()` helper to check processor state before appending messages
- Added `_warn_if_stale()` to detect stale sensor messages
- Added `stale_msg_threshold` config option (default 0.5s)

### Cleanup

- Removed all profiling code (line_profiler, callback tracking, `CANVAS_PROFILE` env var)
- Removed `_debug_log_inference`, `_init_line_profiler`, `_dump_profilers`, `_track_callback`, `_log_and_reset_callback_counts`
- Simplified `inference_model()` (was split into `inference_model` + `_inference_model_inner`, now single method)
- Simplified `shutdown_callback` (just shuts down executor and publishes stop)

## Profiling Results (before cleanup)

Profiling was performed before removing profiling code, using `CANVAS_PROFILE=1` with line_profiler + callback tracking.

### Inference Timing

| Metric           | Multi-Thread | Single-Thread | Change      |
| ---------------- | ------------ | ------------- | ----------- |
| Total time       | 150.6s       | 100.8s        | **-33%**    |
| Per-call avg     | 202.1ms      | 179.6ms       | **-11%**    |
| State extraction | 11.7ms/call  | 6.82ms/call   | **-42%**    |
| Interval std dev | 71.3ms       | 51.1ms        | more stable |

### Per-Callback Elapsed Time (during inference)

| Callback    | Multi-Thread        | Single-Thread      | Reduction |
| ----------- | ------------------- | ------------------ | --------- |
| RGB_FRONT   | 13.31ms (max 103ms) | 3.90ms (max 9ms)   | **71%**   |
| GLOBAL_ODOM | 5.09ms (max 101ms)  | 0.20ms (max 4ms)   | **96%**   |
| CMD_VEL     | 2.95ms (max 72ms)   | 0.08ms (max 1.6ms) | **97%**   |

### Root Cause

Lock contention on `_processor_lock` was the primary issue. In multi-threaded mode, callbacks from multiple threads competed for the lock during inference, causing:

- `_processor_lock` overhead: 8.72s → 3.83s (55% reduction)
- Extreme per-callback spikes (100ms+) that never occur in single-threaded mode
- Higher jitter in inference intervals (std dev 71ms vs 51ms)
