# Flight Route Planner (Design and Analysis of Algorithms Lab - Mini Project 4)

Airports are the **nodes** of a graph and flights are the **weighted edges**. The program finds the best
route between two airports using **Dijkstra's algorithm** (Experiment 5) and then analyses its performance.

## What it does
| Feature (from the project description) | Where |
| --- | --- |
| Shortest paths with Dijkstra | `dijkstra_heap` (binary heap, O((V+E) log V)) and `dijkstra_array` (O(V^2)) |
| Multi-objective optimisation | goal = distance / time / fuel / balanced; balanced uses a weight between time and fuel |
| Weather integration | a weather factor on a route (1 = clear, 2 = storm) makes the flight slower and burns more fuel |
| Air-traffic / airspace restrictions | closed airports are skipped, so the route detours or "no route available" |
| Performance analysis | table of running time vs number of airports, heap vs array, plus graph (`output/scalability.png`) |
| Visualisation | map images of routes (`output/*.png`) |
| Correctness check | heap and array versions are compared on every pair of airports and every goal |

## Web dashboard (recommended for the demo)
```
python dashboard.py
```
It opens http://localhost:8000 in your browser (if the port is busy it tries 8001, 8002 ...). No extra install
is needed for the dashboard, only Python. In the page you can:
- pick From / To (or click airports on the map) and choose the goal: shortest, fastest, least fuel or balanced
- close airports (red) and put weather (storm, rain) on a route, and watch the route change
- compare all four goals in a table and see each flight leg
- press **Run analysis** to see the heap vs array Dijkstra timing table and graph
Stop it with Ctrl+C in the terminal. Screenshots of the dashboard are in `output/dashboard_*.png`.

## Files
- `dashboard.py` + `static/index.html`   the web dashboard
- `planner.py`   the algorithms, terminal menu and terminal demo
- `airports.csv` 15 Indian airports with latitude/longitude
- `routes.csv`   34 flight routes and weather factor (edit this file to add routes)
- `output/`      images created by the program
- `expected_output.txt`  real output of the demo

## Requirements and how to run
Python 3.8+ and matplotlib (only for pictures).
```
pip install matplotlib
python planner.py demo        # everything automatically, saves images in output/
python planner.py             # interactive menu
```
On Windows use `python` (or `py`). It runs in VS Code terminal, PyCharm, IDLE or a Linux terminal.
If `python` is not found, install Python from python.org and tick "Add to PATH".

## Notes
- Distances are real great-circle distances computed from the coordinates (haversine formula).
- The list of routes, the speed (800 km/h), the 0.5 h turnaround and the fuel burn (2.6 kg/km) are
  **sample values for the project**, not real airline schedules.
- Complexity result: heap version grows almost linearly with network size, the array version grows
  quadratically; in the demo the heap version is about 40x faster at 1600 airports.
