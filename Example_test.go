// SPDX-License-Identifier: MPL-2.0
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package dijkstra_test

import (
	"fmt"

	dijkstra "github.com/ultramcu/go-dijkstra"
)

func ExampleStGraph_DijkstraSearch() {
	var g dijkstra.StGraph

	// A directed graph:
	//
	//   A --1--> B --2--> C
	//   |                  ^
	//   +---------5--------+   (direct A->C, but slower)
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
	// Output:
	// A (cost 0)
	// B (cost 1)
	// C (cost 3)
}
