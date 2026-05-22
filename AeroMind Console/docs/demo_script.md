# Demo Script

## Goal

Demonstrate that AeroMind Console is not a traditional button-heavy ground station. It is a natural-language mission console for UAV tasks.

## Suggested Flow

1. Open AeroMind Console.
2. Point out the primary video area for AirSim / UE camera feed.
3. Select the area-search template.
4. Submit this mission:

```text
Search the front 60 meters and hover after finding a vehicle.
```

5. Show the generated mission plan.
6. Explain that the next step is connecting the plan to AirSim execution.
7. Show the event stream and safety actions: pause, return-to-launch, stop.

## Final Target Flow

1. User gives a natural-language or voice mission.
2. The planner generates a structured task.
3. The executor controls the AirSim UAV.
4. The camera stream appears in the main view.
5. The detector finds the target.
6. The UAV hovers and reports coordinates.
7. The console generates a mission report.

