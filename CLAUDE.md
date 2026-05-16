# OpenSCENARIO 1.0 Stochastic Extension — Implementation Spec

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

1. When the `SpeedAction` is triggered (Init or Story), the runtime parses
   `StochasticTargetSpeed` and enters a continuous resample loop.

2. **Time-based trigger (`type="time"`)**: every `value` simulation seconds,
   draw a new speed sample from the distribution and call
   `walker.apply_control(carla.WalkerControl(speed=sampled_speed, ...))`.

3. **Distance-based trigger (`type="distance"`)**: accumulate displacement
   since last resample; when accumulated distance ≥ `value` metres, resample.

4. **Sampling rules**:
   - `uniform`: sample ∈ `[base - noise, base + noise]`, then clip to `[minSpeed, maxSpeed]`.
   - `gaussian`: sample from `Normal(mean=base, stddev=noise)`, then clip to `[minSpeed, maxSpeed]`.

5. The loop runs until the enclosing `Event` ends or the `Act` ends.

6. `SpeedActionDynamics` is still parsed but for `StochasticTargetSpeed` the
   recommended value is `dynamicsShape="step"` (instantaneous application per
   resample tick). Other shapes are ignored with a warning.

---

## Implementation Tasks

### 1. XSD Schema Extension (`openscenario_stochastic.xsd`)

- Start from the official OpenSCENARIO 1.0 XSD
  (`scenario_runner/srunner/openscenario/OpenSCENARIO.xsd` in CARLA's scenario_runner).
- Add the new types: `StochasticTargetSpeed`, `SpeedDistribution`,
  `ResampleTrigger`, `DistributionType`, `ResampleTriggerType`.
- Ensure `SpeedActionTarget` becomes a `xsd:choice` of three elements.

### 2. Parser (`stochastic_parser.py`)

Parse `<StochasticTargetSpeed>` from an `.xosc` file using Python's `xml.etree.ElementTree`.

Expected output (Python dataclass or dict):
```python
@dataclass
class StochasticSpeedConfig:
    distribution_type: str    # "uniform" | "gaussian"
    base: float               # m/s
    noise: float              # m/s
    min_speed: float          # m/s (default: 0.0)
    max_speed: float          # m/s (default: inf)
    resample_type: str        # "time" | "distance"
    resample_value: float     # seconds or metres
```

The parser should fall back gracefully: if the element is `<AbsoluteTargetSpeed>`,
return a `StochasticSpeedConfig` with `noise=0` and `resample_value=inf` (never resamples),
so the rest of the runtime code is uniform.

### 3. CARLA Runtime Controller (`stochastic_walker_controller.py`)

```python
class StochasticWalkerController:
    """
    Wraps a CARLA Walker actor and drives it with dynamically resampled speed
    according to a StochasticSpeedConfig.
    """

    def __init__(self, walker: carla.Actor, config: StochasticSpeedConfig):
        ...

    def tick(self, delta_time: float):
        """
        Call this every simulation tick.
        delta_time: seconds elapsed since last tick (from world snapshot).
        Internally tracks time/distance accumulator and resamples when due.
        Calls walker.apply_control() with the current sampled speed.
        """
        ...

    def _sample_speed(self) -> float:
        """Draw a new speed from the configured distribution."""
        ...

    def _get_direction(self) -> carla.Vector3D:
        """Return normalised forward vector of the walker."""
        ...
```

Key implementation notes:
- Use `world.on_tick(callback)` or a synchronous `world.tick()` loop.
- For distance-based resample: compute displacement using
  `walker.get_velocity()` integrated over `delta_time`, OR diff of
  `walker.get_location()` between ticks.
- Thread safety: CARLA callbacks run in a separate thread; use a lock or
  move logic into the main loop.

### 4. Integration Test (`test_stochastic_walker.py`)

Write a pytest-based test that:
1. Loads a minimal `.xosc` file containing one `<StochasticTargetSpeed>` element.
2. Parses it with `stochastic_parser.py`.
3. Spawns a walker in CARLA (requires a running CARLA server on `localhost:2000`).
4. Runs the controller for 30 simulation seconds.
5. Asserts that the observed speeds stay within `[base - noise, base + noise]`
   (and within `[minSpeed, maxSpeed]`).
6. Asserts that the speed changes at least `floor(30 / resample_value) - 1` times
   (allowing for one missed tick at boundaries).

### 5. Example Scenario File (`pedestrian_stochastic.xosc`)

A complete runnable `.xosc` file with:
- One ego vehicle (static or slow-moving).
- One pedestrian (`Pedestrian_01`) using `<StochasticTargetSpeed>` with
  `uniform` distribution, `base=2.778`, `noise=0.278`, `resample type="time" value="1.5"`.
- Trigger: SimulationTime > 0 (starts immediately).
- End condition: SimulationTime > 60.

---

## File Structure

```
openscenario_stochastic/
├── CLAUDE.md                          ← this file
├── schema/
│   └── openscenario_stochastic.xsd   ← extended XSD
├── src/
│   ├── stochastic_parser.py           ← XML parser
│   └── stochastic_walker_controller.py← CARLA runtime controller
├── scenarios/
│   └── pedestrian_stochastic.xosc    ← example scenario
└── tests/
    └── test_stochastic_walker.py      ← integration test
```

---

## Dependencies

- Python 3.8+
- `carla` Python package (matching your CARLA server version, e.g. 0.9.15)
- `pytest` for tests
- No third-party XML libraries needed; `xml.etree.ElementTree` (stdlib) is sufficient.
- `numpy` recommended for `gaussian` sampling (`numpy.random.normal`).

---

## Constraints & Notes

- **Unit**: all speed values in the XML are in **m/s**. The comment in the example
  shows km/h for human readability only.
- **Backward compatibility**: a `.xosc` file using only `<AbsoluteTargetSpeed>` or
  `<RelativeTargetSpeed>` must still parse and run without modification.
- **No OpenSCENARIO 1.1+ features**: do not use `ParameterValueDistribution`,
  `Variables`, or expression syntax — stay within 1.0 semantics except for the
  new element.
- **CARLA version**: tested against CARLA 0.9.x. The `WalkerAIController` is NOT
  used here; we use manual `WalkerControl` for precise speed control.
