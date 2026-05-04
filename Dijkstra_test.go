// SPDX-License-Identifier: MIT
// Copyright (c) 2026 MaIII Themd

package dijkstra

import "testing"

// buildSimpleGraph returns the small fixture used by most tests.
// Weights are chosen so every shortest path is unique:
//
//	  A --1--> B --2--> C --1--> D
//	  |        |                  ^
//	  +--100---+----100------+----+
//
// All edges one-way. A->C direct is 100 (vs. 1+2 via B), and B->D
// direct is 100 (vs. 2+1 via C), so the only shortest A->D goes
// A -> B -> C -> D for total cost 4.
func buildSimpleGraph() *StGraph {
	var g StGraph
	g.VertexAdd("A", 0, 0, 0,
		StEdge{ToVertexName: "B", Weight: 1, IsOneWay: true},
		StEdge{ToVertexName: "C", Weight: 100, IsOneWay: true},
	)
	g.VertexAdd("B", 1, 0, 0,
		StEdge{ToVertexName: "C", Weight: 2, IsOneWay: true},
		StEdge{ToVertexName: "D", Weight: 100, IsOneWay: true},
	)
	g.VertexAdd("C", 2, 0, 0,
		StEdge{ToVertexName: "D", Weight: 1, IsOneWay: true},
	)
	g.VertexAdd("D", 3, 0, 0)
	return &g
}

func pathNames(path []StPath) []string {
	out := make([]string, len(path))
	for i, p := range path {
		out[i] = p.Name
	}
	return out
}

func TestDijkstraSearch(t *testing.T) {
	cases := []struct {
		name     string
		from, to string
		wantOK   bool
		wantPath []string
		wantCost float64
	}{
		{"direct one-hop", "A", "B", true, []string{"A", "B"}, 1},
		{"prefers two-hop over direct", "A", "C", true, []string{"A", "B", "C"}, 3},
		{"three-hop best", "A", "D", true, []string{"A", "B", "C", "D"}, 4},
		{"unreachable backwards (one-way edges)", "D", "A", false, nil, 0},
		{"same node", "B", "B", true, []string{"B"}, 0},
		{"missing source", "X", "A", false, nil, 0},
		{"missing target", "A", "X", false, nil, 0},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			g := buildSimpleGraph()
			ok, path := g.DijkstraSearch(tc.from, tc.to)
			if ok != tc.wantOK {
				t.Fatalf("ok = %v, want %v (path=%v)", ok, tc.wantOK, pathNames(path))
			}
			if !ok {
				return
			}
			got := pathNames(path)
			if len(got) != len(tc.wantPath) {
				t.Fatalf("path = %v, want %v", got, tc.wantPath)
			}
			for i := range got {
				if got[i] != tc.wantPath[i] {
					t.Errorf("path[%d] = %q, want %q (full %v)", i, got[i], tc.wantPath[i], got)
				}
			}
			if cost := path[len(path)-1].Cost; cost != tc.wantCost {
				t.Errorf("total cost = %v, want %v", cost, tc.wantCost)
			}
		})
	}
}

// TestDijkstraInit_ResetsState is a regression test for a bug in the
// pre-fork code where DijkstraInit ranged over copies and never reset
// per-vertex state. Re-running the search on the same graph would
// reuse the previous run's Visited / Weight / Parent.
func TestDijkstraInit_ResetsState(t *testing.T) {
	g := buildSimpleGraph()
	if ok, _ := g.DijkstraSearch("A", "D"); !ok {
		t.Fatal("first search failed")
	}

	// Re-run from a different source. If state wasn't reset, B would
	// still be marked Visited with a non-zero Weight from the previous
	// pass and the second search would either return the wrong path
	// or fail outright.
	ok, path := g.DijkstraSearch("B", "D")
	if !ok {
		t.Fatal("second search failed")
	}
	got := pathNames(path)
	want := []string{"B", "C", "D"}
	if len(got) != len(want) {
		t.Fatalf("second search path = %v, want %v", got, want)
	}
	for i := range got {
		if got[i] != want[i] {
			t.Fatalf("second search path = %v, want %v", got, want)
		}
	}
	if cost := path[len(path)-1].Cost; cost != 3 {
		t.Errorf("second search cost = %v, want 3", cost)
	}
}

// TestVertexBLockRemove_NotPresent is a regression test for a bug
// where VertexBLockRemove(missing) used to remove the wrong (last)
// element instead of returning false.
func TestVertexBLockRemove_NotPresent(t *testing.T) {
	var g StGraph
	g.VertexAdd("A", 0, 0, 0)
	g.VertexAdd("B", 1, 0, 0)
	g.VertexAdd("C", 2, 0, 0)
	g.VertexBLock("A")
	g.VertexBLock("B")
	g.VertexBLock("C")

	if g.VertexBLockRemove("notblocked") {
		t.Fatal("VertexBLockRemove(notblocked) = true, want false")
	}
	for _, n := range []string{"A", "B", "C"} {
		if !g.VertexIsBLock(n) {
			t.Fatalf("removing a missing name corrupted the blocked set (%q gone)", n)
		}
	}
}

func TestVertexBlock(t *testing.T) {
	var g StGraph
	g.VertexAdd("A", 0, 0, 0)
	g.VertexAdd("B", 1, 0, 0)

	if !g.VertexBLock("A") {
		t.Error("VertexBLock(A) = false, want true")
	}
	if g.VertexBLock("A") {
		t.Error("VertexBLock(A) twice = true, want false")
	}
	if g.VertexBLock("nonexistent") {
		t.Error("VertexBLock(nonexistent) = true, want false")
	}
	if !g.VertexIsBLock("A") {
		t.Error("VertexIsBLock(A) = false, want true")
	}
	if g.VertexIsBLock("B") {
		t.Error("VertexIsBLock(B) = true, want false")
	}
	if !g.VertexBLockRemove("A") {
		t.Error("VertexBLockRemove(A) = false, want true")
	}
	if g.VertexIsBLock("A") {
		t.Error("A still blocked after remove")
	}
}

// TestDijkstraSearch_BlockedVertex verifies that a vertex blocked
// before VertexAdd is unreachable, and that the only remaining path
// around it still resolves. (Blocking after VertexAdd is documented
// to have no effect on the corresponding existing edges.)
func TestDijkstraSearch_BlockedVertex(t *testing.T) {
	var g StGraph
	g.VertexBLockLoad([]string{"C"}) // must precede VertexAdd
	g.VertexAdd("A", 0, 0, 0,
		StEdge{ToVertexName: "B", Weight: 1, IsOneWay: true},
		StEdge{ToVertexName: "C", Weight: 100, IsOneWay: true},
	)
	g.VertexAdd("B", 1, 0, 0,
		StEdge{ToVertexName: "C", Weight: 2, IsOneWay: true},
		StEdge{ToVertexName: "D", Weight: 100, IsOneWay: true},
	)
	g.VertexAdd("C", 2, 0, 0,
		StEdge{ToVertexName: "D", Weight: 1, IsOneWay: true},
	)
	g.VertexAdd("D", 3, 0, 0)

	if ok, _ := g.DijkstraSearch("A", "C"); ok {
		t.Error("path to blocked vertex C should be rejected")
	}

	// With C blocked, the only viable A -> D path is the direct
	// A -> B -> D edges (cost 1 + 100 = 101).
	ok, path := g.DijkstraSearch("A", "D")
	if !ok {
		t.Fatal("path A -> D should be reachable via A -> B -> D")
	}
	got := pathNames(path)
	want := []string{"A", "B", "D"}
	if len(got) != len(want) {
		t.Fatalf("path = %v, want %v", got, want)
	}
	for i := range got {
		if got[i] != want[i] {
			t.Errorf("path[%d] = %q, want %q (full %v)", i, got[i], want[i], got)
		}
	}
	if cost := path[len(path)-1].Cost; cost != 101 {
		t.Errorf("cost = %v, want 101", cost)
	}
}

func TestPriorityQueue_Order(t *testing.T) {
	var pq StPriorityQueue
	pq.EnQueue("a", "b", 5)
	pq.EnQueue("a", "c", 1)
	pq.EnQueue("a", "d", 3)
	pq.EnQueue("a", "e", 1) // tie with c, should come AFTER c
	pq.EnQueue("a", "f", 7)

	wantOrder := []string{"c", "e", "d", "b", "f"}
	for i, w := range wantOrder {
		ok, q := pq.DeQueue()
		if !ok {
			t.Fatalf("DeQueue %d returned !ok", i)
		}
		if q.toVertex != w {
			t.Errorf("DeQueue %d: toVertex = %q, want %q", i, q.toVertex, w)
		}
	}
	if pq.NotEmpty() {
		t.Error("queue should be empty after draining")
	}
}

func TestStack_LIFO(t *testing.T) {
	var st StStack
	if !st.Empty() {
		t.Error("zero-value stack not Empty")
	}
	st.Push("a")
	st.Push("b")
	st.Push("c")
	if got := st.Pop(); got != "c" {
		t.Errorf("Pop = %q, want %q", got, "c")
	}
	if got := st.Pop(); got != "b" {
		t.Errorf("Pop = %q, want %q", got, "b")
	}
	if st.Len() != 1 {
		t.Errorf("Len = %d, want 1", st.Len())
	}
	st.Clear()
	if !st.Empty() {
		t.Error("Empty after Clear should be true")
	}
	if got := st.Pop(); got != "" {
		t.Errorf("Pop on empty = %q, want \"\"", got)
	}
}

func TestDistance(t *testing.T) {
	cases := []struct {
		x1, y1, x2, y2 float64
		want           float64
	}{
		{0, 0, 3, 4, 5},
		{0, 0, 0, 0, 0},
		{1, 1, 4, 5, 5},
	}
	for _, tc := range cases {
		if got := Distance(tc.x1, tc.y1, tc.x2, tc.y2); got != tc.want {
			t.Errorf("Distance(%v,%v,%v,%v) = %v, want %v",
				tc.x1, tc.y1, tc.x2, tc.y2, got, tc.want)
		}
		if got := DistanceCM(tc.x1, tc.y1, tc.x2, tc.y2); got != tc.want*100 {
			t.Errorf("DistanceCM(%v,%v,%v,%v) = %v, want %v",
				tc.x1, tc.y1, tc.x2, tc.y2, got, tc.want*100)
		}
	}
}

func TestNearPoint(t *testing.T) {
	g := buildSimpleGraph() // vertices at (0,0), (1,0), (2,0), (3,0)
	name, dist := g.NearPoint(2.1, 0)
	if name != "C" {
		t.Errorf("NearPoint name = %q, want %q", name, "C")
	}
	// expected distance: |2.1 - 2.0| in the same units * 100 cm/unit ≈ 10
	if dist < 9.99 || dist > 10.01 {
		t.Errorf("NearPoint dist = %v, want ~10", dist)
	}
}
