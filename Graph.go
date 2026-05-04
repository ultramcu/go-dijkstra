// SPDX-License-Identifier: MIT
// Copyright (c) 2026 MaIII Themd

// Package dijkstra is a small directed-graph + Dijkstra-shortest-path
// library with optional vertex blocking and optional X/Y/Heading
// metadata for spatial graphs (e.g. robot pathfinding on a map).
//
// The API uses an St prefix on exported types (StGraph, StVertex,
// StEdge, StPath) and a (bool, T) success-flag return convention --
// both kept intentionally consistent with the author's other Go code.
//
// Goroutine safety: all StGraph methods are safe for concurrent use.
// A sync.RWMutex inside StGraph protects every public operation; read
// operations take the read lock, mutations and searches take the
// write lock. This means concurrent searches on the same graph
// serialise (a search mutates per-vertex state), but parallel
// reads of a graph that's not currently being searched are free.
package dijkstra

import (
	"fmt"
	"sync"
)

// _BLOCK_WEIGHT_ is the per-edge cost penalty added to every edge
// pointing to a blocked vertex. Any path whose final cumulative cost
// reaches or exceeds this value is treated as "no path" by
// DijkstraSearch.
const _BLOCK_WEIGHT_ = 10000000.0

// StPath is one node in a returned shortest path. Cost is the
// cumulative weight from the source up to and including this node.
type StPath struct {
	Name    string
	X       float64
	Y       float64
	Heading float64
	Cost    float64
}

// StEdge is a directed edge from the owning vertex to ToVertexName.
//
// IsOneWay = true means the edge may only be traversed from the
// owning vertex to ToVertexName during search; the reverse direction
// will not be promoted to the shortest-path tree.
//
// isShort is set internally by DijkstraRun; it is unexported and not
// part of the public contract.
type StEdge struct {
	ToVertexName string
	Weight       float64
	isShort      bool
	IsOneWay     bool
}

// StVertex is one node in the graph.
//
// X / Y / Heading are optional metadata for spatial graphs. They are
// returned verbatim in StPath but are not consulted by the algorithm
// itself, so non-geometric users can leave them at zero.
type StVertex struct {
	Name       string
	Weight     float64
	Visited    bool
	X          float64
	Y          float64
	Heading    float64
	Edges      []StEdge
	MaskSearch bool
	Parent     string
}

// StGraph is a directed graph supporting Dijkstra shortest-path
// search and optional vertex blocking. The zero value is a usable
// empty graph.
//
// All exported methods are safe for concurrent use. Internally,
// methods named with a "Locked" suffix assume the caller already
// holds g.mu and are used by the search internals to avoid
// re-entrant locking.
type StGraph struct {
	mu            sync.RWMutex
	vertex        []StVertex
	debugEn       bool
	blockedVertex []string
}

// ---------------------------------------------------------------- //
// Vertex blocking
// ---------------------------------------------------------------- //
//
// Blocked vertices are not removed from the graph; instead, every
// edge pointing to a blocked vertex gets a +_BLOCK_WEIGHT_ cost
// penalty applied at VertexAdd time. DijkstraSearch then rejects
// any returned path whose total cost crosses that threshold.
//
// IMPORTANT: VertexBLock / VertexBLockLoad must be called BEFORE
// the matching VertexAdd calls, since the penalty is baked into
// edges as they are added.

// VertexBLockClear empties the blocked-vertex set.
// (Existing edge weights are not adjusted retroactively.)
func (g *StGraph) VertexBLockClear() {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.blockedVertex = nil
}

// VertexBLockLoad appends names to the blocked-vertex set.
// Always returns true.
func (g *StGraph) VertexBLockLoad(name []string) bool {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.blockedVertex = append(g.blockedVertex, name...)
	return true
}

// VertexIsBLock reports whether name is currently in the blocked set.
func (g *StGraph) VertexIsBLock(name string) bool {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.vertexIsBLockLocked(name)
}

func (g *StGraph) vertexIsBLockLocked(name string) bool {
	for _, a := range g.blockedVertex {
		if a == name {
			return true
		}
	}
	return false
}

// VertexBLock adds an existing vertex to the blocked set.
// Returns false if name does not exist or is already blocked.
func (g *StGraph) VertexBLock(name string) bool {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.vertexIsBLockLocked(name) {
		return false
	}
	if !g.vertexIsExistLocked(name) {
		return false
	}
	g.blockedVertex = append(g.blockedVertex, name)
	return true
}

// VertexBLockRemove removes name from the blocked set.
// Returns true if the set was already empty or the name was removed,
// false if name was present in neither case.
func (g *StGraph) VertexBLockRemove(name string) bool {
	g.mu.Lock()
	defer g.mu.Unlock()
	if len(g.blockedVertex) == 0 {
		return true
	}
	for i := 0; i < len(g.blockedVertex); i++ {
		if g.blockedVertex[i] == name {
			g.blockedVertex[i] = g.blockedVertex[len(g.blockedVertex)-1]
			g.blockedVertex = g.blockedVertex[:len(g.blockedVertex)-1]
			return true
		}
	}
	return false
}

// ---------------------------------------------------------------- //
// Debug + printing
// ---------------------------------------------------------------- //

// Debug toggles verbose printing during graph operations and search.
func (g *StGraph) Debug(debugEn bool) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.debugEn = debugEn
}

// Print writes every vertex and its edges to stdout.
func (g *StGraph) Print() {
	g.mu.RLock()
	defer g.mu.RUnlock()
	for _, v := range g.vertex {
		fmt.Printf("Vertex %s (%1.2f,%1.2f), w = %1.2f, visited %v\r\n",
			v.Name, v.X, v.Y, v.Weight, v.Visited)
		fmt.Printf("\tEdge ")
		for _, e := range v.Edges {
			fmt.Printf("(%s, %1.2f) | ", e.ToVertexName, e.Weight)
		}
		fmt.Printf("\r\n\r\n")
	}
}

// PrintDijkstra writes every vertex with its post-Dijkstra state and
// only the edges that belong to the shortest-path tree.
func (g *StGraph) PrintDijkstra() {
	g.mu.RLock()
	defer g.mu.RUnlock()
	for _, v := range g.vertex {
		fmt.Printf("Vertex %s, w %1.2f, (%1.2f,%1.2f), visited %v, parent %s\r\n",
			v.Name, v.Weight, v.X, v.Y, v.Visited, v.Parent)
		fmt.Printf("\tEdge ")
		for _, e := range v.Edges {
			if e.isShort {
				fmt.Printf("(%s, %1.2f) | ", e.ToVertexName, e.Weight)
			}
		}
		fmt.Printf("\r\n\r\n")
	}
}

// ---------------------------------------------------------------- //
// Vertex management
// ---------------------------------------------------------------- //

// VertexLength returns the number of vertices in the graph.
func (g *StGraph) VertexLength() int {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return len(g.vertex)
}

// VertexFind returns the slice index of the vertex named `name`,
// or -1 if not found.
func (g *StGraph) VertexFind(name string) int {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.vertexFindLocked(name)
}

func (g *StGraph) vertexFindLocked(name string) int {
	for i := 0; i < len(g.vertex); i++ {
		if g.vertex[i].Name == name {
			return i
		}
	}
	return -1
}

// VertexIsExist reports whether a vertex with that name exists.
func (g *StGraph) VertexIsExist(name string) bool {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.vertexIsExistLocked(name)
}

func (g *StGraph) vertexIsExistLocked(name string) bool {
	return g.vertexFindLocked(name) >= 0
}

// VertexAdd inserts a new vertex with the given name, optional
// (x, y, heading) metadata, and zero or more outgoing edges.
//
// Returns false if a vertex with this name already exists.
//
// Edges whose ToVertexName is currently in the blocked set get a
// +_BLOCK_WEIGHT_ penalty added to their Weight at insertion time.
func (g *StGraph) VertexAdd(name string, x, y, heading float64, edges ...StEdge) bool {
	g.mu.Lock()
	defer g.mu.Unlock()

	if g.debugEn {
		fmt.Printf("name = %s\r\n", name)
	}
	if g.vertexIsExistLocked(name) {
		if g.debugEn {
			fmt.Printf("Duplicate vertex %s\r\n", name)
		}
		return false
	}

	v := StVertex{
		Name:    name,
		X:       x,
		Y:       y,
		Heading: heading,
	}

	for _, e := range edges {
		if g.vertexIsBLockLocked(e.ToVertexName) {
			e.Weight += _BLOCK_WEIGHT_
			if g.debugEn {
				fmt.Printf("BLock %s set Weight = %f\r\n", e.ToVertexName, e.Weight)
			}
		}
		v.Edges = append(v.Edges, e)
		if g.debugEn {
			fmt.Printf("edge add %s weight %f\r\n", e.ToVertexName, e.Weight)
		}
	}

	g.vertex = append(g.vertex, v)
	return true
}

// VertexSetXY updates the X and Y coordinates of an existing vertex.
// No-op if the vertex doesn't exist.
func (g *StGraph) VertexSetXY(name string, x, y float64) {
	g.mu.Lock()
	defer g.mu.Unlock()
	i := g.vertexFindLocked(name)
	if i >= 0 {
		g.vertex[i].X = x
		g.vertex[i].Y = y
	}
}

// VertexToStPath builds a snapshot StPath from the named vertex.
// Cost is taken from the vertex's current Weight (cumulative cost
// after a Dijkstra pass).
func (g *StGraph) VertexToStPath(name string) (bool, StPath) {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.vertexToStPathLocked(name)
}

func (g *StGraph) vertexToStPathLocked(name string) (bool, StPath) {
	i := g.vertexFindLocked(name)
	if i < 0 {
		return false, StPath{}
	}
	return true, StPath{
		Name:    g.vertex[i].Name,
		X:       g.vertex[i].X,
		Y:       g.vertex[i].Y,
		Heading: g.vertex[i].Heading,
		Cost:    g.vertex[i].Weight,
	}
}

// ---------------------------------------------------------------- //
// Per-vertex Dijkstra state accessors
// ---------------------------------------------------------------- //

// VertexGetParent returns the parent name set during the last
// DijkstraRun, or "" if name doesn't exist or has no parent.
func (g *StGraph) VertexGetParent(name string) string {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.vertexGetParentLocked(name)
}

func (g *StGraph) vertexGetParentLocked(name string) string {
	i := g.vertexFindLocked(name)
	if i < 0 {
		return ""
	}
	return g.vertex[i].Parent
}

// VertexSetParent records that `child`'s parent in the shortest-path
// tree is `parent`. Returns false if child doesn't exist.
func (g *StGraph) VertexSetParent(child, parent string) bool {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.vertexSetParentLocked(child, parent)
}

func (g *StGraph) vertexSetParentLocked(child, parent string) bool {
	i := g.vertexFindLocked(child)
	if i < 0 {
		return false
	}
	g.vertex[i].Parent = parent
	return true
}

// VertexGetWeight returns the cumulative cost recorded on the vertex
// during the last DijkstraRun, or 0 if the vertex doesn't exist.
func (g *StGraph) VertexGetWeight(name string) float64 {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.vertexGetWeightLocked(name)
}

func (g *StGraph) vertexGetWeightLocked(name string) float64 {
	i := g.vertexFindLocked(name)
	if i < 0 {
		return 0
	}
	return g.vertex[i].Weight
}

// VertexSetWeight overwrites the cumulative cost on a vertex.
func (g *StGraph) VertexSetWeight(name string, weight float64) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.vertexSetWeightLocked(name, weight)
}

func (g *StGraph) vertexSetWeightLocked(name string, weight float64) {
	i := g.vertexFindLocked(name)
	if i >= 0 {
		g.vertex[i].Weight = weight
	}
}

// VertexIsVisited reports whether the vertex has been settled by
// the last DijkstraRun.
func (g *StGraph) VertexIsVisited(name string) bool {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.vertexIsVisitedLocked(name)
}

func (g *StGraph) vertexIsVisitedLocked(name string) bool {
	i := g.vertexFindLocked(name)
	if i < 0 {
		return false
	}
	return g.vertex[i].Visited
}

// VertexSetVisited marks a vertex as settled.
func (g *StGraph) VertexSetVisited(name string) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.vertexSetVisitedLocked(name)
}

func (g *StGraph) vertexSetVisitedLocked(name string) {
	i := g.vertexFindLocked(name)
	if i >= 0 {
		g.vertex[i].Visited = true
	}
}

// VertexIsMasked reports whether a vertex has been touched by the
// path-reconstruction stack walk.
func (g *StGraph) VertexIsMasked(name string) bool {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.vertexIsMaskedLocked(name)
}

func (g *StGraph) vertexIsMaskedLocked(name string) bool {
	i := g.vertexFindLocked(name)
	if i < 0 {
		return false
	}
	return g.vertex[i].MaskSearch
}

// VertexSetMask marks a vertex as touched by the path-reconstruction
// stack walk.
func (g *StGraph) VertexSetMask(name string) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.vertexSetMaskLocked(name)
}

func (g *StGraph) vertexSetMaskLocked(name string) {
	i := g.vertexFindLocked(name)
	if i >= 0 {
		g.vertex[i].MaskSearch = true
	}
}

// ---------------------------------------------------------------- //
// Edge accessors
// ---------------------------------------------------------------- //

// getAdjacencyVerticesLocked returns the slice of edges leaving `name`.
// Returns nil if the vertex doesn't exist. Caller must hold g.mu.
//
// The returned slice aliases the graph's internal storage; only
// safe to read while the lock is still held.
func (g *StGraph) getAdjacencyVerticesLocked(name string) []StEdge {
	i := g.vertexFindLocked(name)
	if i < 0 {
		return nil
	}
	return g.vertex[i].Edges
}

// EdgeIsOneWay reports whether the edge from fromVertex to toVertex
// is marked one-way. Returns false if no such edge exists.
func (g *StGraph) EdgeIsOneWay(fromVertex, toVertex string) bool {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.edgeIsOneWayLocked(fromVertex, toVertex)
}

func (g *StGraph) edgeIsOneWayLocked(fromVertex, toVertex string) bool {
	i := g.vertexFindLocked(fromVertex)
	if i < 0 {
		return false
	}
	for _, e := range g.vertex[i].Edges {
		if e.ToVertexName == toVertex {
			return e.IsOneWay
		}
	}
	return false
}

// MaskShortEdge marks the edge from fromVertex to toVertex as part
// of the shortest-path tree. Returns false if no such edge exists.
func (g *StGraph) MaskShortEdge(fromVertex, toVertex string) bool {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.maskShortEdgeLocked(fromVertex, toVertex)
}

func (g *StGraph) maskShortEdgeLocked(fromVertex, toVertex string) bool {
	i := g.vertexFindLocked(fromVertex)
	if i < 0 {
		return false
	}
	for j := range g.vertex[i].Edges {
		if g.vertex[i].Edges[j].ToVertexName == toVertex {
			g.vertex[i].Edges[j].isShort = true
			return true
		}
	}
	return false
}

// EdgeExist reports whether an edge from fromVertex to toVertex exists.
func (g *StGraph) EdgeExist(fromVertex, toVertex string) bool {
	g.mu.RLock()
	defer g.mu.RUnlock()
	i := g.vertexFindLocked(fromVertex)
	if i < 0 {
		return false
	}
	for _, e := range g.vertex[i].Edges {
		if e.ToVertexName == toVertex {
			return true
		}
	}
	return false
}

// EdgeGetWeight returns the weight of the edge from fromVertex to
// toVertex, or 0 if no such edge exists.
func (g *StGraph) EdgeGetWeight(fromVertex, toVertex string) float64 {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.edgeGetWeightLocked(fromVertex, toVertex)
}

func (g *StGraph) edgeGetWeightLocked(fromVertex, toVertex string) float64 {
	i := g.vertexFindLocked(fromVertex)
	if i < 0 {
		return 0
	}
	for _, e := range g.vertex[i].Edges {
		if e.ToVertexName == toVertex {
			return e.Weight
		}
	}
	return 0
}

// ---------------------------------------------------------------- //
// Spatial helpers (only useful when X / Y are populated)
// ---------------------------------------------------------------- //

// NearPoint returns the name and distance (in cm) of the vertex
// whose (X, Y) is closest to (x, y). Returns "" and a very large
// number on an empty graph.
func (g *StGraph) NearPoint(x, y float64) (string, float64) {
	g.mu.RLock()
	defer g.mu.RUnlock()
	min := 1e9
	minName := ""
	for _, v := range g.vertex {
		d := DistanceCM(v.X, v.Y, x, y)
		if d < min {
			min = d
			minName = v.Name
		}
	}
	return minName, min
}

// DistanceCMToVertex returns the cm distance from (x, y) to the
// vertex named `name`. Returns false, 0 if the vertex doesn't exist.
func (g *StGraph) DistanceCMToVertex(x, y float64, name string) (bool, float64) {
	g.mu.RLock()
	defer g.mu.RUnlock()
	i := g.vertexFindLocked(name)
	if i < 0 {
		return false, 0
	}
	return true, DistanceCM(g.vertex[i].X, g.vertex[i].Y, x, y)
}
