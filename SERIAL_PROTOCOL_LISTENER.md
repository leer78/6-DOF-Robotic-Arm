# Listener-Driven ACK Flow

This document captures the listener-driven ACK-and-wait flow you described. It is an adjunct to the existing SERIAL_PROTOCOL_DESIGN.md and preserves the original content there.

## Purpose

Provide a concrete description of the runtime behavior and the exact functions/globals expected in `serial_protocol.py` to support the GUI flow:

button press → build_packet() → send_packet() → wait_for_ack() (blocks) → continue

A permanent background listener thread parses incoming serial packets (both DATA and ACK) and publishes the latest ACK into a guarded global for the waiter to inspect.

---

## Key runtime elements

- `current_ack` (global variable) — holds the latest ACK packet string received by the listener or `None` when no ACK is available. Access must be protected by a lock (`current_ack_lock`).
- `start_listener(serial_conn)` — starts the background listener thread.
- `stop_listener()` — stops the listener thread.
- `listener_thread_packet()` (internal) — reads bytes, splits on `\n`, calls `parse_packet()`, and for ACK stores into `current_ack`, for DATA routes telemetry to log/UI.
- `send_packet(serial_conn, packet_str)` — writes the packet to the serial connection (ensures newline and flush). Does not itself verify ACK.
- `wait_for_ack(sent_packet, timeout)` — blocks until `current_ack` contains an ACK that verifies against `sent_packet` using `verify_ack()`.

---

## Listener behaviour (detailed)

1. `start_listener(serial_conn)` stores `serial_conn` and spawns a daemon thread running `listener_thread_packet()`.
2. `listener_thread_packet()` loop:
   - Read available bytes from `serial_conn` (use small sleeps to avoid busy-waiting).
   - Accumulate into a buffer; when `\n` appears, split complete lines and `strip()` each line.
   - For each line, call `parse_packet(line)`; if `TYPE == 'ACK'`:
       - Acquire `current_ack_lock`; set `current_ack = line` and release lock.
     If `TYPE == 'DATA'`:
       - Forward to telemetry handler / log (non-blocking for the command lane).
   - Log malformed packets and continue.
3. The listener is resilient: on serial read error it logs, sleeps briefly, and retries until stopped.

Concurrency note: use a `threading.Lock()` for `current_ack`; if you later support concurrent senders, migrate to a `pending_acks` map.

---

## wait_for_ack algorithm

- Called after `send_packet()` has transmitted `sent_packet` to Teensy.
- Repeatedly checks `current_ack` (under `current_ack_lock`):
  - If `current_ack` is `None`: sleep briefly and retry until timeout.
  - If non-empty: call `verify_ack(sent_packet, current_ack + "\n")`.
    - If verification succeeds: under lock set `current_ack = None` and return success.
    - If verification fails: keep waiting (or optionally clear the unmatched ACK depending on policy).
- If timeout expires, raise `ProtocolError` (caller may retry).

Pseudo-code:

```py
start = time.time()
while time.time() - start < timeout:
    with current_ack_lock:
        ack = current_ack
    if ack:
        try:
            verify_ack(sent_packet, ack + "\n")
            with current_ack_lock:
                current_ack = None
            return True
        except ProtocolError:
            # not a matching ACK - keep waiting
            pass
    time.sleep(0.01)
raise ProtocolError("Timeout waiting for ACK")
```

Important: clear `current_ack` after consumption to avoid re-using stale ACKs for identical repeated packets.

---

## Integration points in `gui.py`

- On app startup:
  - `conn = serial_protocol.connect_serial()`
  - `serial_protocol.start_listener(conn)`
- In `_on_send()` / `_do_estop()`:
  - Build packet with `serial_protocol.build_packet(...)`.
  - Call `serial_protocol.send_packet(conn, packet)` to write bytes.
  - Immediately call `serial_protocol.wait_for_ack(packet, timeout=5.0)` to block until ACK.
  - On success: update UI state and log.
  - On failure: show error popup and optionally retry.
- On exit:
  - `serial_protocol.stop_listener()`; then `conn.close()`

Note: This flow blocks the GUI thread by design. If you prefer responsiveness, run send+wait in a worker thread and update UI via thread-safe callbacks.

---

## Use of existing helpers

- Reuse `parse_packet()` inside the listener thread.
- Reuse `verify_ack()` inside `wait_for_ack()`.
- Reuse `build_packet()` in GUI send handlers.

---

## Next implementation steps I can take (pick one):
1. Add `current_ack` + `wait_for_ack()` functions into `serial_protocol.py` and wire them to the existing `start_listener()` implementation.
2. Update `gui.py` to call `start_listener()` on startup and replace the placeholder send logic with `send_packet()` + `wait_for_ack()`.

Tell me which step to run next and I'll implement it.
