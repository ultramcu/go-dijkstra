# go-dijkstra

A small directed-graph + **Dijkstra shortest-path** library for Go,
with optional vertex blocking and optional `X` / `Y` / `Heading`
metadata for spatial graphs (e.g. robot pathfinding on a 2D map).

## Install

```sh
go get github.com/ultramcu/go-dijkstra
```

Requires Go 1.18 or newer. No third-party dependencies.

## Quick example

```go
package main

import (
    "fmt"

    dijkstra "github.com/ultramcu/go-dijkstra"
)

func main() {
    var g dijkstra.StGraph

    g.VertexAdd("A", 0, 0, 0,
        dijkstra.StEdge{ToVertexName: "B", Weight: 1, IsOneWay: true},
        dijkstra.StEdge{ToVertexName: "C", Weight: 5, IsOneWay: true},
    )
    g.VertexAdd("B", 1, 0, 0,
        dijkstra.StEdge{ToVertexName: "C", Weight: 2, IsOneWay: true},
    )
    g.VertexAdd("C", 2, 0, 0)

    ok, path := g.DijkstraSearch("A", "C")
    if !ok {
        fmt.Println("no path")
        return
    }
    for _, p := range path {
        fmt.Printf("%s (cost %.0f)\n", p.Name, p.Cost)
    }
    // A (cost 0)
    // B (cost 1)
    // C (cost 3)
}
```

## Features

- **Directed graph** with one-way edges supported via
  `StEdge{IsOneWay: true}`.
- **Dijkstra shortest path** between any two named vertices.
  `DijkstraSearch` returns an ordered slice of `StPath` nodes with
  cumulative cost on each entry.
- **Vertex blocking** — mark a vertex as off-limits and any path
  that would route through it is rejected. Useful for routing around
  closed corridors, charging stations, etc.
- **Optional spatial metadata** — every vertex has `X`, `Y`,
  `Heading` fields. Leave them at zero for non-spatial graphs, or
  use them with the bundled helpers (`NearPoint`,
  `DistanceCMToVertex`, `Distance`, `DistanceCM`).
- **Zero dependencies** — pure standard library.

## API at a glance

| Type | Purpose |
| --- | --- |
| `StGraph`            | The graph; holds vertices, edges, and the blocked set. |
| `StVertex`           | One node; `Name`, `X`, `Y`, `Heading`, `Edges`, `Weight`, `Visited`, `Parent`, `MaskSearch`. |
| `StEdge`             | Outgoing edge: `ToVertexName`, `Weight`, `IsOneWay`. |
| `StPath`             | One node in a returned path: `Name`, `X`, `Y`, `Heading`, `Cost`. |
| `StPriorityQueue`    | Ascending-by-weight priority queue used by `DijkstraRun`. Exported for reuse. |
| `StStack`            | LIFO stack of strings. Exported for reuse. |

| Function / method | Purpose |
| --- | --- |
| `g.VertexAdd(name, x, y, heading, edges...)` | Add a vertex with outgoing edges. |
| `g.VertexFind(name) int`                     | Slice index of a vertex, or `-1`. |
| `g.VertexIsExist(name) bool`                 | Whether the vertex exists. |
| `g.VertexBLockLoad([]string) / VertexBLock(name) / VertexBLockRemove(name) / VertexIsBLock(name) / VertexBLockClear()` | Manage the blocked set (see note below). |
| `g.DijkstraSearch(from, to) (bool, []StPath)`| Find the shortest path. |
| `g.DijkstraRun(from) bool`                   | Run only the relaxation pass; populates `Weight` / `Parent` / `Visited` on every reachable vertex. Useful when you want all shortest paths from a single source. |
| `g.NearPoint(x, y) (string, float64)`        | Vertex with `(X, Y)` closest to `(x, y)`, returned with its distance in cm. |
| `g.DistanceCMToVertex(x, y, name) (bool, float64)` | Distance in cm from `(x, y)` to a named vertex. |
| `Distance(x1, y1, x2, y2) float64`           | Euclidean distance helper. |
| `DistanceCM(x1, y1, x2, y2) float64`         | Same, scaled by 100. |
| `g.Debug(true)`                              | Enable verbose logging on all subsequent operations. |

## Notes and limitations

- **Block before add.** Vertex blocking applies a per-edge cost
  penalty at `VertexAdd` time, so calls to `VertexBLock` /
  `VertexBLockLoad` need to happen *before* the matching `VertexAdd`
  calls in order to take effect.
- **Goroutine-safe.** Every `StGraph` method is safe to call from
  multiple goroutines. Internally a `sync.RWMutex` protects the
  graph: read methods take the read lock, mutations and searches
  take the write lock. Concurrent searches on the same graph
  therefore *serialise* (a search mutates per-vertex working state),
  but parallel reads of an idle graph run freely. For typical
  fleet workloads the serialised throughput is well above what one
  graph instance ever needs.
- **Sized for small graphs.** Vertex lookup is O(n) and the
  priority queue is O(n) per insert. Comfortable up to a few
  thousand vertices; for tens of thousands you'd want indexing
  by name and `container/heap`.
- **Naming.** The `St` prefix on exported types and the
  `(bool, T)` success-flag return convention are the author's
  intentional house style.

## License

[Mozilla Public License 2.0](LICENSE) — `SPDX-License-Identifier: MPL-2.0`.

You can drop this module into any project (open or closed),
commercial or otherwise. If you modify any of the files in this
package, the modified files must remain under MPL 2.0 and their
source must be made available alongside the binary. Application
code that simply imports the package is not affected.
