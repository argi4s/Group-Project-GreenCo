   Telemetry properties in droneclass

Telemetry properties in droneclass

| Property | Returns | Description |
| --- | --- | --- |
| `armed` | `bool` | Armed status |
| `mode` | `str` | Current flight mode |
| `gps_fix` | `int` | GPS fix type (0-6) |
| `position` | `(lon, lat, alt)` | Current position |
| `heading` | `float` | Yaw in degrees |
| `ground_speed` | `float` | Ground speed in m/s |
| `air_speed` | `float` | Air speed in m/s |
| `vertical_speed` | `float` | Vertical velocity in m/s |