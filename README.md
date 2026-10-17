# ✈️ Flight Route Planner

**Design and Analysis of Algorithms (DAA) Lab — Mini Project 4**

A Python-based flight route planning application that uses graph algorithms to find optimal routes between airports. It implements two versions of Dijkstra's algorithm and compares their performance.

## 🚀 Features

- **Shortest Path:** Find routes using Dijkstra's algorithm.
- **Algorithm Comparison:** Compare binary heap and array-based implementations.
- **Multiple Route Goals:** Optimize for distance, time, fuel, or a balanced objective.
- **Weather Simulation:** Simulate weather conditions that affect flight time and fuel consumption.
- **Airspace Restrictions:** Close airports and observe alternative routes.
- **Interactive Dashboard:** Explore routes and compare optimization goals in a browser.
- **Performance Analysis:** Compare algorithm execution times and visualize scalability.
- **Correctness Validation:** Compare the two implementations across available airport pairs and objectives.

## 🛠️ Technologies Used

- Python
- Graphs and weighted edges
- Dijkstra's shortest-path algorithm
- Binary heap and array-based priority selection
- CSV data files
- HTML, CSS and JavaScript for the dashboard
- Matplotlib for generated charts and images

## 📁 Project Structure

```text
Flight-Route-Planner/
├── planner.py
├── dashboard.py
├── airports.csv
├── routes.csv
├── expected_output.txt
├── static/
│   └── index.html
└── output/
    └── Generated charts and route images
```

## ⚙️ Requirements

- Python 3.8 or later
- Matplotlib (required for generating plots and images)

## ▶️ Installation and Usage

**1. Clone the repository**

```bash
git clone https://github.com/Anuj25-cyber/Flight-Route-Planner.git
cd Flight-Route-Planner
```

**2. Install the optional plotting dependency**

```bash
python -m pip install matplotlib
```

**3. Run the web dashboard**

```bash
python dashboard.py
```

Open `http://localhost:8000` in your browser. If the port is busy, check the terminal for the alternative port.

**4. Run the algorithm demo**

```bash
python planner.py demo
```

**5. Run the interactive terminal menu**

```bash
python planner.py
```

On Windows, you can use `py` instead of `python` if required.

## 📊 Algorithm Complexity

| Implementation | Time Complexity |
|---|---|
| Dijkstra with binary heap | O((V + E) log V) |
| Dijkstra with array-based selection | O(V²) |

Here, V represents the number of vertices (airports), and E represents the number of edges (routes). Actual runtime depends on graph size, implementation details and the test environment.

## 📌 Sample Data Disclaimer

Airport coordinates are used to calculate great-circle distances. Flight speeds, turnaround times, fuel consumption and weather factors are illustrative values for academic demonstration, not real airline schedules or operational flight-planning data.

## 🎓 Learning Outcomes

- Applying graph representations to a real-world-inspired problem
- Implementing and comparing shortest-path algorithms
- Understanding time complexity and performance trade-offs
- Building a browser-based interface for an algorithmic project

## 👨‍💻 Author

**Anuj25-cyber**

GitHub: https://github.com/Anuj25-cyber

---

*Created as an academic mini project for Design and Analysis of Algorithms.*
