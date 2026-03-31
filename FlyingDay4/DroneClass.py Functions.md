   DroneClass.py Functions

DroneClass.py Functions

DroneClass.py Functions

DroneClass.py Functions

NOTES:

- Co ordinates are ordered according to mavlink standards: Lon then Lat order.
- RTL and Emergency flags need to be able to block new commands while automated
- Takeoff, Land, RTL and move_to_wp are set up to block until completion but mustbe at a lower priority to aforementioned interrupts
- Flight commands are not thread-safe, this means do not call the flight commands such as takeoff(), move_to_wp() or land() from multiple threads simultaneously. This is okay as FSM is single threaded and these functions are not expected to be called at the same time.
- Properties use locks, `self._lock` this ensures that the properties are not changed while being actively modified elsewhere

Initialisation

`Drone(mavlink_uri="udp:127.0.0.1:14552")`

`Input: <MAVPROXY IP address>:<PORT>`

`set_mode(mode: str)`

Input: "GUIDED" or "AUTO"  
Action: Changes flight mode  
Raises: ValueError if mode not supported  
Blocking: No, sends command and returns

`arm()`

Input: None  
Action: Arms the drone (motors enable)  
Checks: Interrupt flags (RTL/emergency)  
Blocking: No

`disarm()`

Input: None  
Action: Disarms the drone (motors disable)  
Checks: Nothing  
Blocking: No

`takeoff(target_altitude_m: float`

Input: Target altitude in metres  
Action: Takes off to specified altitude, waits until reached  
Requires: GUIDED mode  
Raises: RuntimeError if interrupted by RTL/emergency  
Blocking: Yes - waits until altitude reached  
TODO: ensure that emergency and RTL states can be triggered during this action

`land()`  
Input: None  
Action: Initialises landing, waits until altitude <0.3m  
Blocking: Yes - waits until near ground  
TODO: Duplicate modified function to enable descent to approx. 1m altutude for deploying payload

`rtl()`  
Input: None  
Action: Sends Mavlink RTL command, sets rtl_flag, waits until landed  
Blocking: Yes - waits until landed  
Note: Clears rtl_flag only after successful completion  
TODO: maintain as 'real emergency' function and install a secondary 'rtl' that uses the synthetic geofences for our mission. As our task is a demonstration we may need a real RTL function that we can utilise to follow university drone flying procedure

`emergency()`  
Input: None  
Action: Sets emergency_flag, sends mavlink command - disarm with param 211 (emergency stop)  
Blocking: No  
Note: Highest priority and blocks all other commands

set_speed(speed_m_s: fload)  
Input: Ground speed in m/s  
Action: Sets drone's ground speed via MAV_CMD_DO_CHANGE_SPEED  
Blocking: No

`move_to_wp(lon: float, lat: float, alt: float, heading: Optional[float] = None)`  
Input:  
lon: Longitude in deg  
lat: Latitude in deg  
alt: altitude in m (relative to take off)  
heading: Optional fixed heading (None = auto-rotate) Not yet implemented  
Action: Sends position target, then waits until the drone is within 2m horizontal and 1m vertical  
Requires: GUIDED mode  
Raises: RuntimeError if interrupted  
Blocking: Yes - waits until position reached (to be changed)

`move_to_wp_queue(waypoint_generator`  
input: Generator function that returns list of waypoints  
Action: Iterates through waypoints, calls move_to_wp for each  
Blocking: Yes - processes all waypoints sequentially (to be changed)

`upload_mission(waypoints: List[Tuple[lon,lat,alt]])`  
Input: List of (lon, lat, alt) tuples  
Action: Uploads waypoints to autopilot for AUTO mode execution  
Blocking: Yes - waits for ACK after each waypoint

`send_position_target(lon:float, lat: float, alt: float, heading:Optional[float] = None)`  
Input: Same as move_to_wp  
Action: Sends single position target without watiing  
Debug: Writes to waypoint_debug.txt  
Blocking: No (But includes delay and response checks)

Utilities:

`distance_to(lon: float, lat: float) returns float`  
Input: Target longitude, latitude  
Action: Calculates Haversine distance to target  
Returns: Distance in metres

`landed() returns bool`  
Input: None  
Action: Checks if altitude <0.5m AND not armed  
Returns: bool

Internal Functions:

`_check_interrupts()`  
Action: Raises RuntimeError if emergency_flag or rtl_flag is True  
Called by: All blocking flight commands

`_wait_until_altitude(target_alt: float, tolerance: float = 0.5`  
Action: Loops until altitude within tolerance  
TODO: Needs to subscribe to interrupts\*\* Must check

`_distance_m(a: LonLatAlt, b: LonLatAlt) returns float`  
Input: Two (lon, lat, alt) tuples  
Returns: Distance between points in metres

Unused Functions:

`upload_mission(waypoints)`  
Note: For AUTO missions, mostly using GUIDED for ease  
`move_to_wp_queue(waypoint_generator)`  
Note: Helper function, not yet used  
`_distance_m(a, b)`  
Note: using public function, was having issues utilising due to unfamiliarity with classes at the time  
`send_position_target(lon, lat, alt, heading`  
Note: called only by `move_to_wp`, keep

PI_State_Machine.py  
`heartbeat_loop()`  
Note: `Clogged up UDP and unsure whether necessary`