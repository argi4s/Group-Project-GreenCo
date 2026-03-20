   Network Configuration

Network Configuration

## Network Configuration

| Component | IP Address | Port | Protocol | Direction |
| --- | --- | --- | --- | --- |
| Pi FSM → GCS | 192.168.1.69 | 9050 | UDP | Status messages |
| GCS → Pi FSM | 192.168.1.69 | 9051 | UDP | Commands |
| Pi Video → GCS | 192.168.1.17 | 5800 | UDP RTP | H.264 video |
| Pi Detection → GCS | 192.168.1.17 | 5801 | UDP JSON | Vector data |
| Pi → MAVProxy | 192.168.1.69 | 14401 | UDP | Drone telemetry |
| GCS → MAVProxy | 192.168.1.17 | 14402 | UDP | Drone telemetry |
| Mission Planner → MAVProxy | 192.168.1.17 | 14400 | UDP | Optional GCS |

## MAVProxy Launch Command

```bash
python3 -m MAVProxy.mavproxy --master=/dev/ttyAMA0,115200 \
  --out=udp:192.168.1.17:14400 \
  --out=udp:192.168.1.17:14402 \
  --out=udp:192.168.1.69:14401 \
  --logfile=/home/flightlab-user/Desktop/flightday2/flight.tlog \
  --mav20
```