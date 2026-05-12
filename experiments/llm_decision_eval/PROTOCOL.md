# 5.4 LLM Decision Evaluation Protocol

## Scope

This experiment supports thesis Section 5.4 and compares three locally hosted open-source LLMs for UAV task decomposition:

- `qwen2.5:7b-instruct` -> thesis name `Qwen2.5-7B-Instruct`
- `llama3.2:3b` -> thesis name `Llama 3.2-3B`
- `phi3.5:latest` -> thesis name `Phi-3.5-mini-Instruct`

The experiment uses one cached raw model response per sample and evaluates it under two parsing conditions:

- `baseline`: strict `json.loads(raw.strip())` plus strict schema validation
- `proposed`: multi-strategy robust JSON extraction derived from `hw_insight/llm_client.py`

Only the parser and validator change between the two groups. Prompt, host, model, task text, and sampling parameters remain identical.

## Frozen runtime settings

- Ollama host: `http://39.108.60.130:6300`
- API route: `/api/chat`
- Sampling temperature: `0.1`
- Stream mode: `false`
- Ollama `format`: disabled for the ablation rerun
- Context state injected into the prompt:
  - `arming_state = ARMED`
  - `flight_phase = HOVERING`
  - position `(x=0.0, y=0.0, z=-1.0)` in NED
  - velocity `(vx=0.0, vy=0.0, vz=0.0)`
  - heading `0 deg`
  - current command `HOVER`
- Max altitude: `120.0 m`
- Max speed: `15.0 m/s`

This airborne hover state avoids automatic TAKEOFF insertion and aligns with the thesis description that evaluation starts after the UAV enters autonomous control.

## Parser and validator policy

- Both groups consume the same cached `raw_output` for each `(model, task)` pair.
- `baseline` is intentionally strict:
  - no fence stripping
  - no balanced-brace search
  - no regex rescue
  - no action alias normalization
  - no string-to-number coercion
  - unknown params or missing required params are immediate failures
- `proposed` keeps:
  - fence stripping
  - balanced-brace candidate extraction
  - regex fallback
  - action alias normalization
  - controlled numeric coercion

Required params for executable commands:

- `TAKEOFF.altitude`
- `MOVE_VELOCITY.duration`
- `MOVE_REL.duration`
- `GOTO_NED.x`
- `GOTO_NED.y`
- `GOTO_NED.altitude`
- `ORBIT.cx`
- `ORBIT.cy`
- `ORBIT.radius`
- `ORBIT.speed`
- `ORBIT.duration`
- `YAW_TO.angle`
- `SET_SPEED.speed`

## Sample sizes

- Offline decision set: `50` natural-language tasks
- Online closed-loop set: `20` mission episodes

## Offline metrics

- `JSR-baseline`: baseline parser success ratio on the 50-task set
- `JSR-proposed`: robust parser success ratio on the 50-task set
- `TPA`: among robust-parser successes, ratio of samples whose action type and key parameters match the gold label

`TPA` is reported once per model, matching the current thesis table structure.

## Online metrics

- `MCR-baseline`: mission completion rate when the command/plan is obtained via the baseline parser
- `MCR-proposed`: mission completion rate when the command/plan is obtained via the robust parser
- `ASC`: average number of executed action steps over successful `proposed` episodes

For online evaluation, commands with missing required params are rejected before execution instead of falling back to runtime defaults.

## Gold-label policy

- The offline set covers the 11 thesis primitives:
  `TAKEOFF`, `LAND`, `HOVER`, `MOVE_VELOCITY`, `MOVE_REL`, `GOTO_NED`,
  `ORBIT`, `YAW_TO`, `RTL`, `EMERGENCY_STOP`, `SET_SPEED`
- `FIND_AND_GOTO` is excluded from Section 5.4 to keep the experiment focused on text-command decision performance.
- For `TPA`, action type must be correct and key numeric/string parameters must match within the tolerance encoded in the task file.

## Output artifacts

The experiment writes:

- `results/offline_raw_outputs.jsonl`
- `results/online_raw_outputs.jsonl`
- `results/pilot_records.jsonl`
- `results/offline_records.jsonl`
- `results/offline_summary.csv`
- `results/online_records_baseline.jsonl`
- `results/online_records_proposed.jsonl`
- `results/online_summary.csv`
- `results/table_5_1.csv`
- `results/discussion_notes.md`
