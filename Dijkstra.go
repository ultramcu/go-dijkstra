// SPDX-License-Identifier: MPL-2.0
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package dijkstra

import "fmt"

// DijkstraInit clears per-vertex Dijkstra state on every vertex
// (Visited, MaskSearch, Weight, Parent, and the per-edge isShort
// flag). Called automatically by DijkstraRun; the caller usually
// does not need to invoke it directly.
func (g *StGraph) DijkstraInit() {
	for i := range g.vertex {
		g.vertex[i].Visited = false
		g.vertex[i].MaskSearch = false
		g.vertex[i].Weight = 0
		g.vertex[i].Parent = ""
		for j := range g.vertex[i].Edges {
			g.vertex[i].Edges[j].isShort = false
		}
	}
}

// DijkstraRun does the main relaxation pass starting from
// startVertex. After it returns, every reachable vertex has its
// Visited flag set, its cumulative Weight populated, its Parent
// pointer set, and the edges belonging to the shortest-path tree
// have their isShort flag set.
//
// Returns false only if startVertex doesn't exist in the graph.
func (g *StGraph) DijkstraRun(startVertex string) bool {
	if !g.VertexIsExist(startVertex) {
		return false
	}

	g.DijkstraInit()

	var pq StPriorityQueue
	pq.EnQueue(startVertex, startVertex, 0)

	debugPath := ""

	for pq.NotEmpty() {
		hasData, q := pq.DeQueue()
		if !hasData {
			break
		}
		if g.debugEn {
			fmt.Printf("DeQueue : %v\r\n", q)
		}

		if g.VertexIsVisited(q.toVertex) {
			if g.debugEn {
				fmt.Printf("\tVertex has been visited %s\r\n", q.toVertex)
			}
			continue
		}

		// Mark the edge that brought us here as part of the SP tree
		// and set the cumulative weight on the discovered vertex.
		// (For the source itself there is no real from->to edge, so
		// MaskShortEdge returns false and we skip; weight stays 0.)
		if g.MaskShortEdge(q.fromVertex, q.toVertex) {
			w := g.VertexGetWeight(q.fromVertex) + g.EdgeGetWeight(q.fromVertex, q.toVertex)
			g.VertexSetWeight(q.toVertex, w)
			g.VertexSetParent(q.toVertex, q.fromVertex)
			debugPath += " " + q.toVertex
			if g.debugEn {
				fmt.Printf("\t\tSet Weight on Vertex %s = %1.2f\r\n", q.toVertex, w)
				fmt.Printf("\t\t\t\tShort edge from %s to %s has been masked\r\n",
					q.fromVertex, q.toVertex)
			}
		}

		// If the reverse edge exists and is two-way, it is also part
		// of the SP tree (used by the path-walk in DijkstraSearch).
		if !g.EdgeIsOneWay(q.toVertex, q.fromVertex) {
			if g.MaskShortEdge(q.toVertex, q.fromVertex) && g.debugEn {
				fmt.Printf("\t\t\t\tShort edge from %s to %s has been masked\r\n",
					q.toVertex, q.fromVertex)
			}
		}

		if g.debugEn {
			fmt.Printf("\tVisited vertex %s\r\n", q.toVertex)
		}
		g.VertexSetVisited(q.toVertex)

		for _, e := range g.getAdjacencyVertices(q.toVertex) {
			if g.VertexIsVisited(e.ToVertexName) {
				continue
			}
			w := g.VertexGetWeight(q.toVertex) + e.Weight
			pq.EnQueue(q.toVertex, e.ToVertexName, w)
			if g.debugEn {
				fmt.Printf("\t\t\tEnQueue [%s,%s,%1.2f]\r\n",
					q.toVertex, e.ToVertexName, w)
			}
		}
	}

	if g.debugEn {
		fmt.Printf("End\r\n\r\nPath : %s\r\n", debugPath)
	}
	return true
}

// DijkstraSearch finds the shortest path from fromVertex to toVertex.
//
// On success returns ok = true and a slice of StPath nodes ordered
// from source to destination, with cumulative Cost on each entry.
// On failure (either endpoint missing, no path, or the only path
// crosses a blocked vertex) returns ok = false and a nil slice.
//
// Calling DijkstraSearch implicitly resets all per-vertex Dijkstra
// state on the graph; running it concurrently on the same graph is
// not safe.
func (g *StGraph) DijkstraSearch(fromVertex, toVertex string) (bool, []StPath) {
	if !g.VertexIsExist(fromVertex) || !g.VertexIsExist(toVertex) {
		return false, nil
	}

	if !g.DijkstraRun(fromVertex) {
		return false, nil
	}

	if !g.VertexIsVisited(toVertex) {
		if g.debugEn {
			fmt.Printf("No path to target vertex\r\n")
		}
		return false, nil
	}

	// Walk parent pointers from toVertex back to fromVertex.
	var allPath []StPath
	name := toVertex
	for {
		ok, p := g.VertexToStPath(name)
		if !ok {
			return false, nil
		}
		allPath = append([]StPath{p}, allPath...)
		if name == fromVertex {
			break
		}
		next := g.VertexGetParent(name)
		if next == "" || next == name {
			// Parent chain broken before reaching the source --
			// state is inconsistent; treat as no-path.
			return false, nil
		}
		name = next
	}

	// If the route had to go through a blocked vertex, the cumulative
	// cost on the destination will exceed _BLOCK_WEIGHT_; treat that
	// as no-path.
	if allPath[len(allPath)-1].Cost >= _BLOCK_WEIGHT_ {
		return false, nil
	}

	if g.debugEn {
		fmt.Printf("Path : %s\r\n", joinArrow(allPath))
	}
	return true, allPath
}

// joinArrow renders a path as "A -> B -> C" for debug logging.
func joinArrow(path []StPath) string {
	out := ""
	for i, p := range path {
		if i > 0 {
			out += " -> "
		}
		out += p.Name
	}
	return out
}
