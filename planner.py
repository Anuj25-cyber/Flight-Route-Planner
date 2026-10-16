"""Flight Route Planner (Design and Analysis of Algorithms Lab - Mini Project 4)

Airports are graph nodes, flights are weighted edges.  Routes are found with Dijkstra's algorithm
(Experiment 5) for different goals: shortest distance, least time, least fuel, or a balance of both
(multi-objective).  Weather factors and closed airports (airspace restrictions) change the result.
Part of the project measures how the running time grows with the number of airports.

Run:  python planner.py          (menu)
      python planner.py demo     (everything automatically, saves images in output/)
"""
import csv
import heapq
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output")

CRUISE_KMH = 800        # average cruise speed
TURNAROUND_H = 0.5      # taxi + take-off + landing time added to every flight
FUEL_KG_PER_KM = 2.6    # sample fuel burn of a narrow-body aircraft


# ----------------------------------------------------------------------------- graph
def haversine(lat1, lon1, lat2, lon2):
    """Great-circle distance in km."""
    r = 6371.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin((p2 - p1) / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * r * math.asin(math.sqrt(a))


class Graph:
    def __init__(self):
        self.airports = {}   # code -> (name, lat, lon)
        self.adj = {}        # code -> {neighbour: [km, weather_factor]}

    def add_airport(self, code, name, lat, lon):
        self.airports[code] = (name, lat, lon)
        self.adj.setdefault(code, {})

    def add_route(self, a, b, weather=1.0, km=None):
        if km is None:
            km = haversine(*self.airports[a][1:], *self.airports[b][1:])
        self.adj[a][b] = [km, weather]
        self.adj[b][a] = [km, weather]

    def set_weather(self, a, b, factor):
        self.adj[a][b][1] = factor
        self.adj[b][a][1] = factor

    def edge_count(self):
        return sum(len(v) for v in self.adj.values()) // 2


def load_graph():
    g = Graph()
    with open(os.path.join(HERE, "airports.csv"), newline="") as f:
        for r in csv.DictReader(f):
            g.add_airport(r["code"], r["name"], float(r["lat"]), float(r["lon"]))
    with open(os.path.join(HERE, "routes.csv"), newline="") as f:
        for r in csv.DictReader(f):
            g.add_route(r["src"], r["dst"], float(r["weather"]))
    return g


# ----------------------------------------------------------------------------- edge metrics
def flight_time_h(km, weather):
    """Weather factor > 1 means headwind/storm: slower flight."""
    return km / (CRUISE_KMH / weather) + TURNAROUND_H


def flight_fuel_kg(km, weather):
    return km * FUEL_KG_PER_KM * weather


def make_cost(g, mode, w_time=0.5):
    """Return a function cost(km, weather) for the chosen goal.
    balanced = w_time * (time / max time) + (1 - w_time) * (fuel / max fuel)  (multi-objective)"""
    max_t = max(flight_time_h(km, wf) for nb in g.adj.values() for km, wf in nb.values())
    max_f = max(flight_fuel_kg(km, wf) for nb in g.adj.values() for km, wf in nb.values())
    if mode == "distance":
        return lambda km, wf: km
    if mode == "time":
        return flight_time_h
    if mode == "fuel":
        return flight_fuel_kg
    if mode == "balanced":
        return lambda km, wf: w_time * flight_time_h(km, wf) / max_t + (1 - w_time) * flight_fuel_kg(km, wf) / max_f
    raise ValueError("mode must be distance, time, fuel or balanced")


# ----------------------------------------------------------------------------- Dijkstra
def dijkstra_heap(adj, src, cost, closed=frozenset()):
    """Dijkstra with a binary min-heap.  Time O((V + E) log V).
    Returns (dist, prev, relaxations)."""
    dist = {src: 0.0}
    prev = {}
    heap = [(0.0, src)]
    done = set()
    relax = 0
    while heap:
        d, u = heapq.heappop(heap)
        if u in done:
            continue
        done.add(u)
        for v, (km, wf) in adj[u].items():
            if v in closed:
                continue
            relax += 1
            nd = d + cost(km, wf)
            if nd < dist.get(v, math.inf):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))
    return dist, prev, relax


def dijkstra_array(adj, src, cost, closed=frozenset()):
    """Textbook Dijkstra with a plain list/array to find the minimum.  Time O(V^2)."""
    dist = {u: math.inf for u in adj}
    dist[src] = 0.0
    prev = {}
    done = set()
    for _ in range(len(adj)):
        u, best = None, math.inf
        for x in adj:                       # linear scan for the nearest unfinished node
            if x not in done and dist[x] < best:
                u, best = x, dist[x]
        if u is None:
            break
        done.add(u)
        for v, (km, wf) in adj[u].items():
            if v in closed:
                continue
            nd = best + cost(km, wf)
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
    return {k: v for k, v in dist.items() if v < math.inf}, prev


def path_to(prev, src, dst):
    if dst != src and dst not in prev:
        return None
    path = [dst]
    while path[-1] != src:
        path.append(prev[path[-1]])
    return path[::-1]


def plan(g, src, dst, mode="distance", w_time=0.5, closed=frozenset()):
    """Find the best route; returns dict with path and totals, or None if no route exists."""
    if src in closed or dst in closed:
        return None
    dist, prev, relax = dijkstra_heap(g.adj, src, make_cost(g, mode, w_time), closed)
    path = path_to(prev, src, dst)
    if path is None:
        return None
    km = hours = fuel = 0.0
    for a, b in zip(path, path[1:]):
        d, wf = g.adj[a][b]
        km += d
        hours += flight_time_h(d, wf)
        fuel += flight_fuel_kg(d, wf)
    return dict(path=path, km=km, hours=hours, fuel=fuel, relax=relax)


def fmt_route(r):
    return (" -> ".join(r["path"]) + f"   | {r['km']:.0f} km, {r['hours']:.2f} h, "
            f"fuel {r['fuel']:.0f} kg, {len(r['path']) - 1} flight(s)")


# ----------------------------------------------------------------------------- performance analysis
def random_graph(n, avg_degree=4, seed=1):
    """Connected random 'airport network' with n nodes (a ring plus random extra routes)."""
    rnd = random.Random(seed)
    g = Graph()
    for i in range(n):
        g.add_airport(f"A{i}", f"Airport {i}", rnd.uniform(8, 35), rnd.uniform(68, 97))
    for i in range(n):
        g.add_route(f"A{i}", f"A{(i + 1) % n}")
    for _ in range(n * (avg_degree - 2) // 2):
        a, b = rnd.sample(range(n), 2)
        if f"A{b}" not in g.adj[f"A{a}"]:
            g.add_route(f"A{a}", f"A{b}")
    return g


def performance_analysis(sizes=(100, 200, 400, 800, 1600), repeats=3):
    rows = []
    for n in sizes:
        g = random_graph(n)
        cost = make_cost(g, "distance")
        t_heap = t_arr = 1e9
        for _ in range(repeats):
            t0 = time.perf_counter()
            d1, _, relax = dijkstra_heap(g.adj, "A0", cost)
            t_heap = min(t_heap, time.perf_counter() - t0)
            t0 = time.perf_counter()
            d2, _ = dijkstra_array(g.adj, "A0", cost)
            t_arr = min(t_arr, time.perf_counter() - t0)
        same = all(abs(d1[k] - d2[k]) < 1e-6 for k in d1) and len(d1) == len(d2)
        rows.append((n, g.edge_count(), t_heap * 1000, t_arr * 1000, same))
    return rows


def print_table(rows):
    print(f"{'Airports(V)':>11} | {'Routes(E)':>9} | {'Heap ms':>9} | {'Array ms':>9} | {'Array/Heap':>10} | same answer")
    print("-" * 78)
    for n, e, th, ta, same in rows:
        print(f"{n:>11} | {e:>9} | {th:>9.2f} | {ta:>9.2f} | {ta / th:>9.1f}x | {'yes' if same else 'NO'}")


# ----------------------------------------------------------------------------- pictures
def draw_map(g, routes, title, filename):
    """routes: list of (path, colour, label). Saves a PNG map (longitude on x, latitude on y)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(OUT, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 8))
    seen = set()
    for a in g.adj:
        for b in g.adj[a]:
            if (b, a) in seen:
                continue
            seen.add((a, b))
            ax.plot([g.airports[a][2], g.airports[b][2]], [g.airports[a][1], g.airports[b][1]],
                    color="#cbd5e1", lw=1, zorder=1)
    for path, colour, label in routes:
        xs = [g.airports[c][2] for c in path]
        ys = [g.airports[c][1] for c in path]
        ax.plot(xs, ys, color=colour, lw=3.2, label=label, zorder=3, alpha=.9)
    for code, (name, lat, lon) in g.airports.items():
        ax.scatter(lon, lat, s=70, color="#312e81", zorder=4)
        ax.annotate(code, (lon, lat), textcoords="offset points", xytext=(-24, -13) if code == "KNU" else (6, 5), fontsize=9, weight="bold")
    ax.set_title(title)
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.grid(alpha=.25)
    if routes:
        ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, filename), dpi=130)
    plt.close(fig)
    return os.path.join("output", filename)


def draw_scalability(rows, filename="scalability.png"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(OUT, exist_ok=True)
    n = [r[0] for r in rows]
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(n, [r[2] for r in rows], "o-", color="#4f46e5", label="Dijkstra with heap  O((V+E) log V)")
    ax.plot(n, [r[3] for r in rows], "s-", color="#dc2626", label="Dijkstra with array  O(V^2)")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Number of airports (V)")
    ax.set_ylabel("Time (ms, log scale)")
    ax.set_title("Running time vs network size")
    ax.grid(alpha=.3, which="both")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, filename), dpi=130)
    plt.close(fig)
    return os.path.join("output", filename)


# ----------------------------------------------------------------------------- demo
def demo():
    g = load_graph()
    print(f"=== STEP 1: Network: {len(g.airports)} airports, {g.edge_count()} routes ===")
    print("Airports:", ", ".join(f"{c} ({v[0]})" for c, v in g.airports.items()))
    print()

    print("=== STEP 2: Best route Bengaluru (BLR) -> Lucknow (LKO) for different goals (Dijkstra) ===")
    colours = {"distance": "#2563eb", "time": "#16a34a", "fuel": "#dc2626", "balanced": "#d97706"}
    drawn = []
    for mode in ("distance", "time", "fuel", "balanced"):
        r = plan(g, "BLR", "LKO", mode)
        print(f"{mode:<9}: {fmt_route(r)}")
        drawn.append((r["path"], colours[mode], mode))
    print("Map saved:", draw_map(g, [drawn[1], drawn[2]], "Fastest vs most fuel-efficient route BLR -> LKO", "route_blr_lko.png"))
    print()

    print("=== STEP 3: Multi-objective: change the weight between time and fuel (BLR -> LKO) ===")
    for w in (1.0, 0.75, 0.5, 0.25, 0.0):
        r = plan(g, "BLR", "LKO", "balanced", w_time=w)
        print(f"time weight {w:.2f}: {fmt_route(r)}")
    print()

    print("=== STEP 4: Weather: a storm (factor 2.0) on DEL-BLR slows the flight and burns more fuel ===")
    before = plan(g, "DEL", "BLR", "time")
    g.set_weather("DEL", "BLR", 2.0)
    after = plan(g, "DEL", "BLR", "time")
    print("clear sky :", fmt_route(before))
    print("storm     :", fmt_route(after))
    g.set_weather("DEL", "BLR", 1.0)
    print()

    print("=== STEP 5: Airspace restriction: HYD and BOM are closed ===")
    normal = plan(g, "DEL", "MAA", "distance")
    closed = plan(g, "DEL", "MAA", "distance", closed=frozenset({"HYD", "BOM"}))
    print("normal    :", fmt_route(normal))
    print("closed    :", fmt_route(closed))
    print("Map saved:", draw_map(g, [(normal["path"], "#2563eb", "normal"),
                                     (closed["path"], "#dc2626", "HYD and BOM closed")],
                              "Detour around closed airports DEL -> MAA", "route_closed_airports.png"))
    none = plan(g, "KNU", "DEL", "distance", closed=frozenset({"DEL"}))
    print("Destination closed:", "no route available" if none is None else fmt_route(none))
    print()

    print("=== STEP 6: Correctness check - heap Dijkstra vs array Dijkstra on the real network ===")
    ok = True
    for mode in colours:
        cost = make_cost(g, mode)
        for s in g.airports:
            d1, _, _ = dijkstra_heap(g.adj, s, cost)
            d2, _ = dijkstra_array(g.adj, s, cost)
            ok &= len(d1) == len(d2) and all(abs(d1[k] - d2[k]) < 1e-9 for k in d1)
    print("All source/destination pairs and all 4 goals give identical distances:", "YES" if ok else "NO")
    print()

    print("=== STEP 7: Performance analysis (random networks, best of 3 runs) ===")
    rows = performance_analysis()
    print_table(rows)
    print("Graph saved:", draw_scalability(rows))
    print("(Times depend on your computer; the growth pattern is what matters.)")


# ----------------------------------------------------------------------------- menu
def ask_code(g, text):
    while True:
        c = input(text).strip().upper()
        if c in g.airports:
            return c
        print("Unknown airport code. Choose from:", " ".join(g.airports))


def menu():
    g = load_graph()
    closed = set()
    while True:
        print("\n===== FLIGHT ROUTE PLANNER =====")
        print("1. List airports       2. Find a route        3. Compare all goals for a trip")
        print("4. Close/open airport  5. Set weather on route 6. Performance analysis")
        print("7. Save map of a route 0. Exit")
        ch = input("Choice: ").strip()
        if ch == "0":
            break
        elif ch == "1":
            for c, (n, la, lo) in g.airports.items():
                print(f"{c}  {n:<10} ({la:.2f}, {lo:.2f})  routes: {', '.join(g.adj[c])}")
            print("Closed airports:", ", ".join(sorted(closed)) or "none")
        elif ch in ("2", "3", "7"):
            s, d = ask_code(g, "From (code): "), ask_code(g, "To (code): ")
            if ch == "3":
                for m in ("distance", "time", "fuel", "balanced"):
                    r = plan(g, s, d, m, closed=frozenset(closed))
                    print(f"{m:<9}:", fmt_route(r) if r else "no route")
                continue
            m = input("Goal (distance/time/fuel/balanced) [distance]: ").strip().lower() or "distance"
            if m not in ("distance", "time", "fuel", "balanced"):
                print("Invalid goal")
                continue
            r = plan(g, s, d, m, closed=frozenset(closed))
            if not r:
                print("No route available (airport closed or not connected).")
            else:
                print(fmt_route(r))
                if ch == "7":
                    print("Saved:", draw_map(g, [(r["path"], "#2563eb", m)], f"{s} -> {d} ({m})", f"route_{s}_{d}.png"))
        elif ch == "4":
            c = ask_code(g, "Airport code: ")
            closed.symmetric_difference_update({c})
            print(c, "is now", "CLOSED" if c in closed else "OPEN")
        elif ch == "5":
            a, b = ask_code(g, "Route from: "), ask_code(g, "Route to: ")
            if b not in g.adj[a]:
                print("No direct route between them.")
                continue
            try:
                g.set_weather(a, b, float(input("Weather factor (1 = clear, 2 = storm): ")))
            except ValueError:
                print("Enter a number")
        elif ch == "6":
            rows = performance_analysis()
            print_table(rows)
            print("Graph saved:", draw_scalability(rows))
        else:
            print("Invalid choice")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "demo":
        demo()
    else:
        menu()
