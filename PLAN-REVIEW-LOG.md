# PLAN-REVIEW-LOG — PLAN_HL_FMA_2026.md

- Started: 2026-10-25
- Host / coordinator: Claude Code (Opus 5.5)
- Plan reviewer: Codex CLI (model: CLI default, no override requested)
- Mode: review (existing plan; no requirements interview)
- Max rounds: 1
- Authorization: review only — no build authorized
- Scope note: plan is dated 2026-09-02 for the 2026-09-20 event, which has already taken place (commit 8972d35). Reviewed as-is on user request.

## Round 1 — Codex review

- Result: `C:/Users/USER/AppData/Local/Temp/claude/C--dev-racing-ws/4ea22a73-b86e-402e-b9c6-75dc37efdcba/scratchpad/review-r1/claudex-70wkrf52/result.json`
- CLI: codex-cli 0.164.0; requested model: CLI default; effective_model (client-applied): gpt-6.1-sol; sandbox read-only; session 01a17263-1ad9-7f91-882d-3d9283a5292a
- Plan SHA256: 4f96115053e83731b06d0ff827d6f5e05664dec5972262ec2e5798f088b941de
- Verdict: **REVISE**

```json
{
  "verdict": "REVISE",
  "summary": "Three concrete control-integration defects undermine the plan��s completion-first strategy. The supplied plan hash matches the file.",
  "findings": [
    {
      "id": "FMA-001",
      "severity": "high",
      "path": "src/mission_perception/mission_perception/parking_node.py",
      "evidence": "start_cb ignores False, but mission_sequencer.disarm sends False to /parking/start. A parking timeout therefore cannot cancel an active maneuver. After completion or follower abort, tick continuously publishes zero commands on /teleop/cmd_vel; vehicle_cmd_mux prioritizes these fresh commands over autonomous driving indefinitely. Enabling Track B can consequently prevent resuming the lap and exceed the plan��s one-minute stationary limit.",
      "fix": "Implement explicit parking cancellation and ownership release. Handle disarm during every maneuver state, stop safely, then cease override publication so autonomous control can resume. Make completion and abort release ownership as well."
    },
    {
      "id": "FMA-002",
      "severity": "high",
      "path": "src/gps_localization/launch/bringup.launch.py",
      "evidence": "traffic_light_bridge and crosswalk_stop_node both publish /stop_line_distance. The bridge publishes continuously and has no arm subscription, while launch only supplies it stop_points and state_topic. With traffic and crosswalk enabled, the bridge��s unconstrained value 999 can overwrite a crosswalk HOLD value 0 in longitudinal_controller.stop_cb. Sequencer gating of crosswalk does not gate the traffic bridge. This prevents reliable combined execution of Tracks A and D.",
      "fix": "Give each mission a separate constraint channel and arbitrate them centrally, or implement exclusive ownership and arm handling for every writer, including traffic_light_bridge. Include the integration requirement explicitly in the plan."
    },
    {
      "id": "FMA-003",
      "severity": "high",
      "path": "src/velocity_controller/velocity_controller/longitudinal_controller_node.py",
      "evidence": "stop_cb stores stop_line_dist without a timestamp or expiry. If a stop adapter dies after publishing 0 during HOLD, the controller continues publishing fresh zero target speeds indefinitely while curvature remains fresh. The mux��s input timeout cannot detect this because /target_speed continues arriving. Crosswalk��s explicit release on normal disarm does not cover process failure, leaving the plan��s completion-first guarantee unfulfilled.",
      "fix": "Add an explicit constraint lease and adapter-health policy, with bounded recovery for abandoned mission constraints. Specify safe recovery behavior for missing perception separately from normal mission completion; do not rely solely on mux command freshness."
    }
  ],
  "coverage": [
    "Verified PLAN_HL_FMA_2026.md SHA256 against the supplied hash.",
    "Inspected vehicle command mux arbitration and freshness handling.",
    "Traced stop constraint writers through traffic bridge, crosswalk adapter, launch wiring, and longitudinal controller.",
    "Traced parking start, completion, abort, and timeout handling through mission sequencer and mux.",
    "Inspected mission configuration, origin configuration and reconciliation helpers, and origin-switching writers.",
    "Inspected sudden-stop dwell configuration."
  ],
  "limitations": [
    "The referenced Downloads competition regulations were unavailable; competition facts and penalty interpretations could not be independently verified.",
    "Static inspection only; no tests were run, files edited, or work delegated.",
    "Hardware behavior, RTK coverage, physical clearances, simulation success claims, and lap-time estimates were not validated.",
    "Repository contents include work dated after the plan; findings describe the inspected current integration."
  ]
}
```

### Host dispositions (Claude)

- **FMA-001 — accepted as a code defect, confirmed.** `parking_node.start_cb` returns on `False` (line 201), so `mission_sequencer.disarm` (line 407) can't cancel a maneuver. `tick` keeps publishing 0 on `/teleop/cmd_vel` while in DONE/ABORT/FINISHED, and the mux gives fresh teleop priority over autonomous commands (vehicle_cmd_mux_node.py:310).
- **FMA-002 — accepted, partly mitigated by procedure.** There are three `/stop_line_distance` writers (traffic_light_bridge, crosswalk_stop_node, sudden_stop_node). The launch file warns against enabling them together, and crosswalk counts publishers and warns. That is a startup check only; nothing enforces it at runtime.
- **FMA-003 — accepted, confirmed.** `longitudinal_controller.stop_cb` (line 180) stores the value with no timestamp. sudden_stop_node.py:164 notes the same issue.
- **Common note:** these are implementation and integration defects in code written after the plan. The plan is a pre-competition roadmap, and the event (2026-09-20) is over. The plan was not revised.

### Outcome
- Round budget (1) used up. Final verdict: REVISE, plan **not approved**. No build authorized or done.

---

# Loop 2 — FMA-001 + FMA-003 수정 (PLAN_FMA001_003.md)

- Started: 2026-10-25
- Host/planner/builder: Claude Code (Opus 5.5). Plan reviewer: Codex (CLI default). Final inspector: fresh Codex session.
- Mode: full (recon done in-session; no interview — user specified scope). Max review rounds: 2. Inspect: on.
- Authorization: plan **and implement** (user request). Commit/push: not authorized.
- Pre-build commit: see build section.

## Loop 2 Round 1 — Codex review

- Result: `C:/Users/USER/AppData/Local/Temp/claude/C--dev-racing-ws/4ea22a73-b86e-402e-b9c6-75dc37efdcba/scratchpad/loop2-r1/claudex-xg_9ffnr/result.json`
- codex-cli 0.164.0; requested model: CLI default; effective_model: gpt-6.1-sol; session 01a1726c-97ec-7db0-a6e5-734e0a92af64
- Plan SHA256 reviewed: 0200c1102223ac43d5767d5c1d1fd4614c74f138cdfcd10c3f47a6da76098aeb
- Verdict: **REVISE**

```json
```

### Host dispositions

- FMA-R01 accepted. A4 now covers every start failure: plan failure, missing pose file and missing odometry all go to a follower-less ABORT that publishes done=False and no teleop. Retry is still only via /parking/reset. A7 now says the teleop sequence is identical to the old one and that only these start-failure cases differ, in the done output. Test scenario 3 covers all three failures through the real _trigger→begin path.
- FMA-R02 accepted. New A9 adds _owns_control(): the shutdown zero command is published only while parking owns control. Test scenario 8 covers it.
- FMA-R03 accepted. B1 now uses time.monotonic() for the stop-line lease. Existing time.time() fields are left as they are. Lease test 6 covers a backward wall-clock jump.
- Revised plan SHA256: 3125d9e3ca105e02e9e28438a74e17a8671c9d0c0fe3b5fed0304ee6f350e6bc

(정정: 위 Round 1 JSON 블록은 콘솔 인코딩 오류로 비어 있다. 전체 응답:)
```json
{
  "verdict": "REVISE",
  "summary": "The plan addresses the main failure paths, but misses startup failures and a publisher that can reclaim control after release. Its lease clock also permits stale stops to persist after clock corrections.",
  "findings": [
    {
      "id": "FMA-R01",
      "severity": "medium",
      "path": "src/mission_perception/mission_perception/parking_node.py:338",
      "evidence": "begin() returns without setting ABORT when the selected pose file is unavailable or odometry has not arrived (lines 339–346). The trigger has already latched _started=True. The proposed follower-less ABORT notification therefore never runs for these failures, repeated True messages cannot retry, and the sequencer remains occupied until its timeout.",
      "fix": "Make these failed start attempts enter a follower-less failure state that publishes done=False, or define an explicit bounded retry policy. Cover missing slot data and missing odometry using the actual trigger and begin methods."
    },
    {
      "id": "FMA-R02",
      "severity": "medium",
      "path": "src/mission_perception/mission_perception/parking_node.py:458",
      "evidence": "The plan gates publishing in tick(), but main() unconditionally calls node._publish_cmd(0.0, 0.0) during shutdown. Shutting down a RELEASED node therefore publishes teleop again, violating A2. vehicle_cmd_mux_node.py:310 gives this fresh command priority over autonomous driving for the teleop timeout.",
      "fix": "Suppress the shutdown command after ownership has been released, while retaining the shutdown stop when parking owns control. Add coverage for shutdown after RELEASED."
    },
    {
      "id": "FMA-R03",
      "severity": "medium",
      "path": "PLAN_FMA001_003.md",
      "evidence": "The proposed lease records time.time() and expires by elapsed wall time. If the host clock moves backward after receiving HOLD=0 and the publisher dies, the supposedly one-second lease persists until wall time catches up. A sufficiently large correction recreates the prolonged stop that FMA-003 is intended to prevent.",
      "fix": "Use time.monotonic() consistently for stop-line reception and expiry, independently of the controller's existing wall-clock fields. Include backward wall-clock adjustment coverage."
    }
  ],
  "coverage": [
    "Verified the on-disk plan SHA256 matches the supplied hash.",
    "Inspected parking initialization, triggers, reset, planning, exit, tick, path publishing, and shutdown.",
    "Inspected longitudinal input callbacks, target selection, profiling, and early-return paths.",
    "Traced sequencer arm publication, completion handling, timeout, and disarm.",
    "Inspected all three stop-line writers: crosswalk_stop_node, sudden_stop_node, and traffic_light_bridge.",
    "Traced mux teleop priority and timeout; searched goal-reached producers and consumers.",
    "Inspected relevant launch/config wiring and the existing parking test harness."
  ],
  "limitations": [
    "Reviewed a proposed change; the repository does not yet contain the planned implementation or new test files.",
    "No tests, compilation, ROS nodes, or HIL checks were run.",
    "Repository comments contained encoding corruption in tool output; executable source and the supplied plan remained inspectable.",
    "Physical stopping behavior and autonomous recovery from a parking bay remain unverified."
  ]
}
```

## Loop 2 Round 2 — Codex review (resumed same session)

- Result: `C:/Users/USER/AppData/Local/Temp/claude/C--dev-racing-ws/4ea22a73-b86e-402e-b9c6-75dc37efdcba/scratchpad/loop2-r2/claudex-ygs4sdqr/result.json`
- session 01a1726c-97ec-7db0-a6e5-734e0a92af64; effective_model gpt-6.1-sol
- Plan SHA256 reviewed: 3125d9e3ca105e02e9e28438a74e17a8671c9d0c0fe3b5fed0304ee6f350e6bc
- Verdict: **REVISE**
```json
{
  "verdict": "REVISE",
  "summary": "The revision resolves FMA-R01, FMA-R02, and FMA-R03 at the plan level. One release-state edge case remains unresolved.",
  "findings": [
    {
      "id": "FMA-R04",
      "severity": "medium",
      "path": "PLAN_FMA001_003.md",
      "evidence": "The proposed release() enters RELEASING whenever a follower exists, without guarding RELEASED or RELEASING. After release, the follower remains present. A subsequent True message updates _arm_level even though _started prevents restarting; the next False then calls release() and resumes zero teleop publication. This violates A2 and A8 and temporarily overrides autonomous driving again. Repeated True→False edges during RELEASING can also restart _release_since and defeat the three-second bound.",
      "fix": "Make release() idempotent in RELEASING and RELEASED: return without changing state or _release_since. Check ownership before beginning release. Add True→False cycles during RELEASING and after RELEASED to the planned tests."
    }
  ],
  "coverage": [
    "Verified the revised on-disk plan SHA256 matches the supplied hash.",
    "Checked all three prior findings against the revised acceptance criteria, implementation approach, and test scenarios.",
    "Re-inspected parking trigger latching, reset, begin failure paths, and tick publication ordering.",
    "Re-inspected longitudinal control-loop placement and early returns.",
    "Used the preceding review's inspection of sequencer arm/disarm callers, mux priority, all three stop-line writers, and launch/config wiring to assess the revision."
  ],
  "limitations": [
    "This is a plan review; the proposed implementation and new tests have not been inspected because they are not yet implemented in the reviewed source.",
    "No tests, compilation, ROS execution, or delegation were performed.",
    "Physical stopping and autonomous recovery remain unverified."
  ]
}
```

### Host dispositions
- FMA-R04 accepted. release() is now idempotent in RELEASING/RELEASED (A10, test 9).
- Revised plan SHA256: f99815ce50d4ccd2e7d81bc52d9fa9149b3ed78b9b7c7129e211cb0d6e449709. **This revision has not been reviewed** — round budget (2) is used up, so it is not approved.
- User authorized one extra review round (round 3) — 2026-10-25.

## Loop 2 Round 3 — Codex review (user-authorized extra round)
- Result: `C:/Users/USER/AppData/Local/Temp/claude/C--dev-racing-ws/4ea22a73-b86e-402e-b9c6-75dc37efdcba/scratchpad/loop2-r3/claudex-chb6j6ni/result.json`; session 01a1726c-97ec-7db0-a6e5-734e0a92af64
- Plan SHA256: f99815ce50d4ccd2e7d81bc52d9fa9149b3ed78b9b7c7129e211cb0d6e449709
- Verdict: **APPROVED**
```json
{
  "verdict": "APPROVED",
  "summary": "The revised plan resolves FMA-R01 through FMA-R04. A10 and test scenario 9 explicitly prevent repeated edges from extending release or reclaiming control. No material unresolved defects were identified within the planned scope.",
  "findings": [],
  "coverage": [
    "Verified the on-disk plan SHA256 matches the supplied hash.",
    "Reviewed revised acceptance criteria, implementation approach, and planned verification against all prior findings.",
    "Rechecked parking trigger, reset, planning, tick, and shutdown references; longitudinal lease integration; sequencer disarm writers; and mux teleop priority.",
    "Carried forward earlier inspection of all three stop-line publishers and relevant launch/config wiring."
  ],
  "limitations": [
    "Approval applies to the plan, not an implemented change.",
    "No tests, compilation, ROS execution, or delegation were performed.",
    "Physical stopping, parking-bay recovery, and the explicitly accepted release-on-expiry behavior require field or HIL validation."
  ]
}
```

## Build (host = Claude)
- Pre-build commit: 375250591a592fe0026931884e6f9a57abba1184 (tree clean except loop's PLAN/LOG files)
- Builder: Claude (host), all code written by Claude.
- Changed: parking_node.py, longitudinal_controller_node.py. Added: tools/ros_stub.py, tools/test_parking_release.py, tools/test_stop_line_lease.py.
- Proof (host-run, Windows, Python 3.12, no rclpy → ros_stub):
  - `python tools/test_parking_release.py` → ALL PASS (9 scenarios, incl. old-vs-new identical teleop/done sequences)
  - `python tools/test_stop_line_lease.py` → ALL PASS (6 scenarios, incl. old-vs-new identical 580 /target_speed values)
  - `python -m py_compile` on both changed nodes → OK
  - Negative check: scenarios 1·2 (parking) and 1·6 (lease) FAIL against HEAD code → the tests detect the original bugs.
  - Regression: existing tools/test_ramp_section.py (with ros_stub) → all pass.
  - NOT run: robot/HIL manual checks (plan §5) — need ROS on the vehicle.

## Inspection 1 — fresh Codex session
- Result: `C:/Users/USER/AppData/Local/Temp/claude/C--dev-racing-ws/4ea22a73-b86e-402e-b9c6-75dc37efdcba/scratchpad/loop2-inspect1/claudex-nocfgvq7/result.json`; session 01a17279-0a9c-7200-a3cc-f52a624e1827; snapshot 337f7e4c…
- Verdict: **REVISE**
```json
{
  "verdict": "REVISE",
  "summary": "The node changes follow the planned release and lease behavior, but the regression tests use a moving baseline that breaks verification after committing the change.",
  "findings": [
    {
      "id": "FMA-I01",
      "severity": "medium",
      "path": "tools/test_parking_release.py:load_old; tools/test_stop_line_lease.py:load_old",
      "evidence": "Both load_old() functions read git show HEAD rather than the supplied pre-build commit. Once this implementation is committed, OLD becomes the new implementation. Parking scenario t6_old_vs_new then deterministically fails its missing-pose assertion: it expects OLD to publish no done messages, but the committed implementation publishes False. The remaining comparisons also become self-comparisons and lose their regression coverage.",
      "fix": "Pin both historical imports to commit 375250591a592fe0026931884e6f9a57abba1184, or store a stable baseline fixture. Ensure the comparisons retain their meaning after the implementation is committed."
    }
  ],
  "coverage": [
    "Verified the on-disk plan SHA256 matches the supplied hash.",
    "Read all seven files in the change manifest.",
    "Reviewed parking arm and goal callbacks, release idempotency, reset, planning failures, maneuver and exit handling, publication ordering, and shutdown ownership.",
    "Reviewed longitudinal lease timestamps, expiry, control-loop early returns, and unchanged target-speed calculations.",
    "Traced sequencer completion, timeout, and repeated arm/disarm publication; parking topic configuration; launch wiring; and mux teleop priority and freshness.",
    "Inspected all three stop-line writers: traffic_light_bridge, crosswalk_stop_node, and sudden_stop_node.",
    "Statically inspected the ROS stubs and both added test suites."
  ],
  "limitations": [
    "No tests, compilation, ROS execution, edits, or delegation were performed. Builder-reported test results were not independently verified.",
    "Physical stopping, autonomous recovery from parking, and hardware/HIL behavior remain unverified.",
    "Shared stop-line publisher arbitration and sequencer process failure remain explicit non-goals.",
    "Release-on-expiry behavior, including possible departure during a failed traffic-light publisher, is an explicitly accepted plan tradeoff."
  ]
}
```
### Host disposition
- FMA-I01 accepted (fix round 1). Both load_old() now pin BASE=375250591a59 instead of HEAD. Plan §5 tests 6 and 5 were changed to match. Both tests rerun → ALL PASS.
- Plan SHA256 after fix: 358f0c106cc61b3799812d643c3da709d5b6b83f04844bf0113b0ffdfbe850b4. The only change is the verification wording. Plan approval was for f99815ce…; this edit was not re-reviewed by the plan reviewer, and inspection 2 judges against it.

## Inspection 2 — fresh Codex session
- Result: `C:/Users/USER/AppData/Local/Temp/claude/C--dev-racing-ws/4ea22a73-b86e-402e-b9c6-75dc37efdcba/scratchpad/loop2-inspect2/claudex-8f_kltjw/result.json`; session 01a1727a-5b7e-79d1-a4a2-31b03ef400a7; snapshot 7ec3976c…
- codex-cli 0.164.0; effective_model: gpt-6.1-sol; sandbox read-only
- Plan SHA256 inspected: 358f0c106cc61b3799812d643c3da709d5b6b83f04844bf0113b0ffdfbe850b4
- Verdict: **APPROVED**
```json
{
  "verdict": "APPROVED",
  "summary": "No material unresolved defects identified within the planned scope. The historical regression comparisons now use the fixed pre-build commit.",
  "findings": [],
  "coverage": [
    "Verified the plan SHA256 matches the supplied hash.",
    "Read all seven files in the change manifest.",
    "Reviewed parking triggers, release idempotency, failure handling, reset, exit handling, publication ordering, and shutdown ownership.",
    "Reviewed stop-line lease timestamps, expiration, control-loop placement, and target-speed calculations.",
    "Traced sequencer completion and timeout disarm, repeated arm publication, mux priority, launch wiring, and mission configuration.",
    "Inspected all three stop-line publishers and statically reviewed both test suites and ROS stubs."
  ],
  "limitations": [
    "Static review only; no tests, compilation, ROS execution, edits, or delegation performed. Builder-reported results were not independently verified.",
    "Physical stopping, autonomous recovery, and hardware/HIL behavior remain unverified.",
    "Shared stop-line arbitration and sequencer process failure are explicit non-goals.",
    "Departure after a failed traffic-light publisher is an explicitly accepted plan tradeoff."
  ]
}
```
### Host verification (separate Claude Code app session, after the loop ended)
- Before this entry, all 7 manifest files matched the inspection-2 snapshot by SHA256. After the snapshot, only this log changed, by this entry.
- Reran on this PC (Python 3.12, ros_stub): `test_parking_release.py` ALL PASS, `test_stop_line_lease.py` ALL PASS, `py_compile` on both nodes OK, `test_ramp_section.py` all pass.
- NOT run: robot/HIL manual checks (plan §5).

### Outcome
- Final verdict: **APPROVED** (plan approved in round 3; implementation approved in inspection 2).
- User authorized the commit on 2026-10-25. Push not authorized.
