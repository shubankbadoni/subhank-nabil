# Intelligent Traffic Signal Optimization Agent

This is a starter Python implementation based on the uploaded CS 502 report.

## Files
- `traffic_controller.py`: TraCI-based adaptive controller.
- `config.json`: configuration template. Replace every `REPLACE_WITH...` value.
- `requirements.txt`: Python dependency note.

## Before running
1. Install SUMO (Simulation of Urban Mobility).
2. Create or obtain a SUMO network (`.net.xml`), route file (`.rou.xml`),
   and SUMO configuration (`.sumocfg`).
3. Open the network in netedit or inspect the `.net.xml` to find:
   - the traffic-light ID,
   - inbound lane IDs for north, south, east and west,
   - the traffic-light state string for NS green,
   - the NS yellow transition state,
   - the EW green state,
   - the EW yellow transition state,
   - the all-red state.
4. Put those exact values into `config.json`.

**Important:** SUMO signal-state strings have one character per controlled link.
Do not guess these strings. They must match your junction's controlled links and
safe movement plan. The two-stage grouping is valid only if the movements are
compatible in your network.

## Install / environment
SUMO includes TraCI Python tools. Use the Python environment configured for your
SUMO installation. If `import traci` fails, add SUMO's `tools` directory to
`PYTHONPATH` or use the Python environment bundled/configured with SUMO.

## Run
```bash
python traffic_controller.py --sumocfg path/to/your.sumocfg --config config.json --gui
```
Omit `--gui` for the command-line SUMO simulator.

The controller writes `adaptive_run.csv` by default.

## What the script implements
- Queue = sum of stopped vehicles on the configured inbound lanes.
- Longest wait = maximum current waiting time among stopped vehicles on those lanes.
- Normalization capped at 1 using reference values 50 vehicles, 120 seconds,
  and 90 seconds (configurable).
- Priority = 0.55 * queue + 0.30 * wait + 0.15 * time without service.
- Minimum green, maximum green, switching margin and decision interval.
- Yellow then all-red then target green transition.
- Per-decision CSV log.

## Important limitations / checks
- This is a controller starter, not a complete SUMO scenario. It does not generate
  the road network or vehicle routes for you.
- Check your SUMO version's TraCI API and your junction's signal states.
- The report describes the algorithm and evaluation plan; it does not provide the
  actual network, route files, lane IDs, traffic-light ID or signal-state strings.
- Validate no conflicting greens, no early switch, no green beyond max, and
  eventual service of a low-demand phase before interpreting any performance data.
- The report states that no simulation has yet been run. Do not claim performance
  improvements until matched adaptive and fixed-time experiments have been completed.
