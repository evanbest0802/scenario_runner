# OpenSCENARIO 1.0 Stochastic Extension — Implementation Notes

## Background

OpenSCENARIO 1.0 is fully deterministic: every `value` attribute in `SpeedAction`
is a single scalar constant. This project extends the standard with a new XML element
`<StochasticTargetSpeed>` that enables **intra-run dynamic speed resampling** for
pedestrians (walkers) — meaning within a single simulation run, the pedestrian's
speed fluctuates around a base value, not just a fixed value chosen once per run.

This is NOT the same as OpenSCENARIO 1.1+ `ParameterValueDistribution`, which only
samples once per scenario run. We want the speed to keep changing throughout the run.

Validation backend: **CARLA simulator** via Python API (`carla.WalkerControl`).

---

## New XML Syntax Design

### Placement

`<StochasticTargetSpeed>` is a new alternative child of `<SpeedActionTarget>`,
alongside the existing `<AbsoluteTargetSpeed>` and `<RelativeTargetSpeed>`.

Original 1.0 schema (XSD):
```xml
<xsd:complexType name="SpeedActionTarget">
  <xsd:choice>
    <xsd:element name="AbsoluteTargetSpeed" type="AbsoluteTargetSpeed"/>
    <xsd:element name="RelativeTargetSpeed" type="RelativeTargetSpeed"/>
  </xsd:choice>
</xsd:complexType>
```

Extended schema:
```xml
<xsd:complexType name="SpeedActionTarget">
  <xsd:choice>
    <xsd:element name="AbsoluteTargetSpeed"   type="AbsoluteTargetSpeed"/>
    <xsd:element name="RelativeTargetSpeed"   type="RelativeTargetSpeed"/>
    <xsd:element name="StochasticTargetSpeed" type="StochasticTargetSpeed"/>  <!-- NEW -->
  </xsd:choice>
</xsd:complexType>
```

### `StochasticTargetSpeed` Element Definition

```xml
<xsd:complexType name="StochasticTargetSpeed">
  <xsd:sequence>
    <xsd:element name="Distribution" type="SpeedDistribution"/>
    <xsd:element name="ResampleTrigger" type="ResampleTrigger"/>
  </xsd:sequence>
</xsd:complexType>

<xsd:complexType name="SpeedDistribution">
  <xsd:attribute name="type"     type="DistributionType" use="required"/>
  <xsd:attribute name="base"     type="xsd:double"       use="required"/>  <!-- center value, m/s -->
  <xsd:attribute name="noise"    type="xsd:double"       use="required"/>  <!-- half-range or stddev, m/s -->
  <xsd:attribute name="minSpeed" type="xsd:double"       use="optional"/>  <!-- hard lower clamp, m/s -->
  <xsd:attribute name="maxSpeed" type="xsd:double"       use="optional"/>  <!-- hard upper clamp, m/s -->
</xsd:complexType>

<xsd:simpleType name="DistributionType">
  <xsd:restriction base="xsd:string">
    <xsd:enumeration value="uniform"/>   <!-- uniform in [base-noise, base+noise] -->
    <xsd:enumeration value="gaussian"/>  <!-- normal(mean=base, stddev=noise), clipped to [min,max] -->
  </xsd:restriction>
</xsd:simpleType>

<xsd:complexType name="ResampleTrigger">
  <xsd:attribute name="type"  type="ResampleTriggerType" use="required"/>
  <xsd:attribute name="value" type="xsd:double"          use="required"/>
</xsd:complexType>

<xsd:simpleType name="ResampleTriggerType">
  <xsd:restriction base="xsd:string">
    <xsd:enumeration value="time"/>      <!-- resample every N seconds -->
    <xsd:enumeration value="distance"/>  <!-- resample every N metres travelled -->
  </xsd:restriction>
</xsd:simpleType>
```

### Full XML Usage Example

```xml
<!-- Pedestrian walks at 10 km/h ± 1 km/h (uniform), resampling every 1.5 seconds -->

<Private entityRef="Pedestrian_01">
  <PrivateAction>
    <LongitudinalAction>
      <SpeedAction>
        <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="rate"/>
        <SpeedActionTarget>
          <StochasticTargetSpeed>
            <Distribution type="uniform"
                          base="2.778"
                          noise="0.278"
                          minSpeed="2.0"
                          maxSpeed="3.5"/>
            <!-- base  = 10 km/h = 2.778 m/s -->
            <!-- noise = 1  km/h = 0.278 m/s  -->
            <ResampleTrigger type="time" value="1.5"/>
          </StochasticTargetSpeed>
        </SpeedActionTarget>
      </SpeedAction>
    </LongitudinalAction>
  </PrivateAction>
</Private>
```

```xml
<!-- Gaussian variant: stddev = 0.2 m/s, resample every 10 metres -->
<StochasticTargetSpeed>
  <Distribution type="gaussian"
                base="2.778"
                noise="0.2"
                minSpeed="1.5"
                maxSpeed="4.0"/>
  <ResampleTrigger type="distance" value="10.0"/>
</StochasticTargetSpeed>
```

---

## Semantics / Runtime Behaviour

1. When the `SpeedAction` is triggered (Init or Story), the runtime creates a
   `StochasticActorSpeed` behavior node that runs alongside the scenario tree.

2. **Time-based trigger (`type="time"`)**: every `value` simulation seconds,
   draw a new speed sample from the distribution and push it to the actor's
   controller via `ActorControl.update_target_speed()`.

3. **Distance-based trigger (`type="distance"`)**: accumulate displacement
   (diff of `CarlaDataProvider.get_location()` between ticks); when accumulated
   distance ≥ `value` metres, resample.

4. **Sampling rules**:
   - `uniform`: sample ∈ `[base - noise, base + noise]`, then clip to `[minSpeed, maxSpeed]`.
   - `gaussian`: sample from `Normal(mean=base, stddev=noise)`, then clip to `[minSpeed, maxSpeed]`.

5. The behavior stays `RUNNING` until the enclosing `Event` ends or another
   longitudinal command supersedes it (supersede detection via `get_last_longitudinal_command()`).

6. `SpeedActionDynamics` is still parsed but for `StochasticTargetSpeed` the
   recommended value is `dynamicsShape="step"` (instantaneous application per
   resample tick). Other shapes are ignored.

---

## Implementation — What Was Actually Done

The feature is integrated **directly into the existing scenario_runner source files**.
No separate module or folder is created. Three files were modified, one example added.

### 1. XSD Schema Extension

**File:** `srunner/openscenario/OpenSCENARIO.xsd`

`SpeedActionTarget` was changed from a two-way choice to a three-way choice, and
five new type definitions were appended after the existing `SpeedActionTarget` block:
- `StochasticTargetSpeed` (sequence of `Distribution` + `ResampleTrigger`)
- `SpeedDistribution` (attributes: `type`, `base`, `noise`, `minSpeed`, `maxSpeed`)
- `DistributionType` (enumeration: `"uniform"` | `"gaussian"`)
- `ResampleTrigger` (attributes: `type`, `value`)
- `ResampleTriggerType` (enumeration: `"time"` | `"distance"`)

Note: attributes use the existing `Double` type alias already defined in the schema,
not the raw `xsd:double`.

### 2. Parser

**File:** `srunner/tools/openscenario_parser.py`

Two changes:
1. **Import line (~line 33):** `StochasticActorSpeed` added to the import from
   `srunner.scenariomanager.scenarioatomics.atomic_behaviors`.
2. **Parsing branch (~line 1353):** After the `RelativeTargetSpeed` parsing block,
   a new `if` branch checks for `StochasticTargetSpeed` under `SpeedActionTarget`
   and constructs a `StochasticActorSpeed` atomic behavior with the parsed parameters.
   Parser defaults: `minSpeed → 0`, `maxSpeed → inf`, `type → "time"`,
   `value → inf` (never resamples, stays at initial sample forever).

### 3. Runtime Behavior

**File:** `srunner/scenariomanager/scenarioatomics/atomic_behaviors.py`

New class `StochasticActorSpeed(AtomicBehavior)` added at end of file (~line 4935).

Architecture mirrors `ChangeActorTargetSpeed` exactly:
- Uses the **ActorsWithController blackboard** (`py_trees.blackboard.Blackboard().ActorsWithController`)
  to reach the actor's `ActorControl` instance.
- **Supersede mechanism**: stores `_start_time = GameTime.get_time()` in `initialise()`,
  calls `update_target_speed(..., start_time=_start_time)`.  Each `update()` checks
  `get_last_longitudinal_command() != _start_time` → if true, another speed command
  took over, returns `SUCCESS` immediately.
- **Accumulator**: `_acc` tracks elapsed time (time mode) or accumulated distance
  (distance mode). Resets to 0 after each resample.
- **Distance mode**: uses `CarlaDataProvider.get_location()` position diff (not
  velocity × dt) to avoid floating-point drift.
- **`_sample_speed()`**: calls `numpy.random.uniform` or `numpy.random.normal`,
  then `numpy.clip` to enforce `[minSpeed, maxSpeed]`. The `random` and `np` names
  here refer to `numpy.random` and `numpy` respectively (imported at top of file).

Key safety property: `minSpeed` defaults to 0 at the parser level, and
`PedestrianControl.run_step()` raises `NotImplementedError` on negative
`_target_speed` as a second-layer guard.

### 4. Example Scenario

**File:** `srunner/examples/pedestrian_stochastic.xosc`

- Map: `Town01`
- Ego (`hero`): static VW T2 van at `(120, -200, 0.3)`, assigned `external_control`
  module so keyboard input works from `manual_control.py`.
- Pedestrian (`Pedestrian_01`): walker at `(135, -200, 0.3)`, heading north (`h=1.5708`).
- Stochastic speed: `gaussian, base=0.3, noise=0.2, minSpeed=0.1, maxSpeed=0.5`,
  resampled every `0.5 s`.
- Pedestrian starts at `SimTime > 10 s`; scenario ends at `SimTime > 60 s`.

---

## How to Run

```bash
# Terminal 1 — launch CARLA
./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen

# Terminal 2 — run the scenario
python3 scenario_runner.py \
    --openscenario srunner/examples/pedestrian_stochastic.xosc \
    --reloadWorld \
    --trafficManagerPort 8100

# Terminal 3 — keyboard control for ego (optional)
python3 manual_control.py --trafficManagerPort 8100
```

Required environment:
```bash
export CARLA_ROOT=/path/to/carla
export SCENARIO_RUNNER_ROOT=/path/to/scenario_runner
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
```

---

## Dependencies

- Python 3.8+
- `carla` Python package (matching your CARLA server version, e.g. 0.9.15)
- `numpy` (for gaussian sampling and `np.clip`)
- No additional third-party libraries needed beyond what scenario_runner already uses.

---

## Constraints & Notes

- **Unit**: all speed values in the XML are in **m/s**.
- **Backward compatibility**: `.xosc` files using only `<AbsoluteTargetSpeed>` or
  `<RelativeTargetSpeed>` continue to work without modification.
- **No OpenSCENARIO 1.1+ features**: stays within 1.0 semantics except for the
  new element.
- **Pedestrian only (in practice)**: `StochasticActorSpeed` calls
  `update_target_speed()` which routes through `PedestrianControl` or
  `NpcVehicleControl` transparently. Vehicles accept the command but their PID
  controller smooths out speed changes — abrupt resampling is most visible on walkers.
- **`WalkerAIController` is NOT used**: speed is controlled manually via
  `WalkerControl.speed`, giving precise per-tick control.
