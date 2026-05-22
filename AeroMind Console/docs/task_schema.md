# Mission Task Schema

## MissionPlan

```json
{
  "task": "Search the front area and hover after finding a vehicle",
  "intent": "area_search",
  "target": "vehicle",
  "altitude_m": 8,
  "strategy": "lawnmower",
  "area": {
    "x_min": 0,
    "x_max": 60,
    "y_min": -20,
    "y_max": 20
  },
  "plan": [
    {
      "name": "Understand search target",
      "detail": "Extract target type, search area, and altitude constraints.",
      "action": "parse"
    }
  ]
}
```

## Supported Intents

- `area_search`
- `object_or_area_scan`
- `formation_flight`
- `obstacle_aware_navigation`
- `path_planning`
- `return_to_launch`
- `general_uav_task`

