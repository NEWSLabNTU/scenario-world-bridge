# 053: The relay dies on an RTC status whose distance is not finite (Autoware 2.1.0)

**Severity**: medium. The relay respawns, but while it is down the scenario loses the ego's
AD API, and a scenario that happens to need it then fails. **Status**: fixed on branch
`053-relay-null`. **Found**: 2026-11-18, swb 241, while reviewing the stack of the 2.1.0
Tutorial 1 run (stack `2026-11-18T21-24-24`). That run passed anyway.

## What happens

About 6 minutes into the run, after the stack was ready, the relay died twice with exit 1:

```
File ".../scenario_agent_relay/relay_node.py", line 379, in _rtc_status
    start_distance=float(s.get("start_distance", 0.0)),
TypeError: float() argument must be a string or a real number, not 'NoneType'
```

## Cause

1. On 2.1.0, the planning modules' rtc_interface publishes cooperate statuses whose
   `start_distance` or `finish_distance` is not finite (an unknown distance).
2. The agent protocol has no NaN or infinity, so it sends any non-finite float as null
   (`scensim_agent.protocol._finite_or_none`).
3. The relay read every field with `s.get(key, default)`. A key that is present with null
   returns None, not the default, and `float(None)` raises.

## Fix

- **A null distance means unknown,** and the relay publishes it as `inf`. Not `0.0`: zero would
  say the module is at the vehicle.
- **Every optional protocol field the relay reads** now goes through `mapping.given(message,
  key, default)`, which takes the default when the key is absent or null:
  - an RTC status's other fields (module, uuid, safe, requested, auto_mode, state);
  - a state's phase;
  - a reply's message;
  - a registration's agent, capabilities and spec_hash;
  - a commander command's args.

  Reads that already used `or` (fault, the timeout), or that test for None, are unchanged.

## Test

`scenario_agent_relay/test/test_null_fields.py`:
- a 2.1.0-shaped status (an `inf` start distance, a `nan` finish distance) goes through the
  protocol's encode and decode, and the relay publishes both distances as `inf`;
- a status whose fields are all null;
- a null phase.

`test_server.py`: a registration whose optional fields are null.
