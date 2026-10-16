"""Web dashboard for the Flight Route Planner (uses only the Python standard library).

Run:   python dashboard.py        then open the address it prints (usually http://localhost:8000)
The browser page calls small JSON APIs in this file, which use planner.py (Dijkstra) to find routes.
"""
import json
import os
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import planner

HERE = os.path.dirname(os.path.abspath(__file__))
MODES = ("distance", "time", "fuel", "balanced")


def build_graph(weather_param):
    """weather_param looks like 'DEL-BLR:2.0,BOM-GOI:1.5' (route:factor)."""
    g = planner.load_graph()
    for item in filter(None, (weather_param or "").split(",")):
        pair, _, factor = item.partition(":")
        a, _, b = pair.partition("-")
        if a in g.adj and b in g.adj[a]:
            try:
                g.set_weather(a, b, max(0.5, min(5.0, float(factor))))
            except ValueError:
                pass
    return g


def route_json(g, r):
    if r is None:
        return None
    legs = []
    for a, b in zip(r["path"], r["path"][1:]):
        km, wf = g.adj[a][b]
        legs.append(dict(a=a, b=b, km=round(km), hours=round(planner.flight_time_h(km, wf), 2),
                         fuel=round(planner.flight_fuel_kg(km, wf)), weather=wf))
    return dict(path=r["path"], km=round(r["km"]), hours=round(r["hours"], 2), fuel=round(r["fuel"]),
                flights=len(r["path"]) - 1, legs=legs)


def api_network(_q):
    g = planner.load_graph()
    airports = [dict(code=c, name=v[0], lat=v[1], lon=v[2]) for c, v in g.airports.items()]
    seen, routes = set(), []
    for a in g.adj:
        for b, (km, _wf) in g.adj[a].items():
            if (b, a) not in seen:
                seen.add((a, b))
                routes.append(dict(a=a, b=b, km=round(km)))
    return dict(airports=airports, routes=routes)


def api_plan(q):
    g = build_graph(q.get("weather", [""])[0])
    src = q.get("src", [""])[0].upper()
    dst = q.get("dst", [""])[0].upper()
    if src not in g.airports or dst not in g.airports:
        return dict(error="Unknown airport code")
    try:
        w = max(0.0, min(1.0, float(q.get("w", ["0.5"])[0])))
    except ValueError:
        w = 0.5
    closed = frozenset(c for c in q.get("closed", [""])[0].upper().split(",") if c in g.airports)
    results = {m: route_json(g, planner.plan(g, src, dst, m, w_time=w, closed=closed)) for m in MODES}
    note = None
    if src in closed or dst in closed:
        note = "Source or destination airport is closed."
    elif all(v is None for v in results.values()):
        note = "No route available (closed airports cut the network)."
    return dict(results=results, note=note)


def api_perf(_q):
    rows = planner.performance_analysis()
    return dict(rows=[dict(v=r[0], e=r[1], heap=round(r[2], 3), array=round(r[3], 3), same=r[4]) for r in rows])


API = {"/api/network": api_network, "/api/plan": api_plan, "/api/perf": api_perf}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):   # keep the console quiet
        pass

    def send(self, code, body, ctype):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path in API:
            try:
                self.send(200, json.dumps(API[url.path](parse_qs(url.query))), "application/json")
            except Exception as e:      # never crash the server on a bad request
                self.send(500, json.dumps(dict(error=str(e))), "application/json")
        elif url.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "static", "index.html"), "rb") as f:
                self.send(200, f.read(), "text/html; charset=utf-8")
        else:
            self.send(404, "Not found", "text/plain")


def main():
    open_browser = "--no-browser" not in sys.argv
    for port in range(8000, 8011):
        try:
            server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            print(f"Port {port} is busy, trying {port + 1} ...")
    else:
        print("No free port between 8000 and 8010. Close the old dashboard and run again.")
        return
    url = f"http://localhost:{port}"
    print(f"Flight Route Planner dashboard running -> open {url}   (Ctrl+C to stop)")
    if open_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
