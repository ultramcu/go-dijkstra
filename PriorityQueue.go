// SPDX-License-Identifier: MIT
// Copyright (c) 2026 MaIII Themd

package dijkstra

import "fmt"

// StPriorityQueueData is one entry in the priority queue used by
// the Dijkstra relaxation pass: an edge (fromVertex -> toVertex)
// together with the cumulative cost to reach toVertex via that edge.
type StPriorityQueueData struct {
	fromVertex string
	toVertex   string
	weight     float64
}

// StPriorityQueue is an ascending-by-weight priority queue of
// StPriorityQueueData. EnQueue is O(n); fine for the small graphs
// this library is sized for. Equal weights keep insertion order.
type StPriorityQueue struct {
	q []StPriorityQueueData
}

// Print writes the queue's contents to stdout in head-to-tail order.
func (pq *StPriorityQueue) Print() {
	fmt.Printf("Priority Queue : ")
	for _, pqd := range pq.q {
		fmt.Printf("(%s,%s,%1.2f), ", pqd.fromVertex, pqd.toVertex, pqd.weight)
	}
	fmt.Printf("\r\n")
}

// EnQueue inserts a new entry, maintaining ascending-by-weight order.
// Entries with the same weight as existing ones are placed AFTER them
// (insertion order is preserved within an equivalence class).
func (pq *StPriorityQueue) EnQueue(fromVertex string, toVertex string, weight float64) {
	elem := StPriorityQueueData{
		fromVertex: fromVertex,
		toVertex:   toVertex,
		weight:     weight,
	}

	// Find the first slot whose weight is strictly greater; insert there.
	insertAt := len(pq.q)
	for i, pqd := range pq.q {
		if pqd.weight > weight {
			insertAt = i
			break
		}
	}

	pq.q = append(pq.q, StPriorityQueueData{})
	copy(pq.q[insertAt+1:], pq.q[insertAt:])
	pq.q[insertAt] = elem
}

// DeQueue removes and returns the front (lowest-weight) entry.
// Returns (false, zero-value) if the queue is empty.
func (pq *StPriorityQueue) DeQueue() (bool, StPriorityQueueData) {
	if pq.Len() == 0 {
		return false, StPriorityQueueData{}
	}
	dq := pq.q[0]
	pq.q = pq.q[1:]
	return true, dq
}

// Head returns the front entry without removing it.
// Returns (false, zero-value) if the queue is empty.
func (pq *StPriorityQueue) Head() (bool, StPriorityQueueData) {
	if pq.Len() == 0 {
		return false, StPriorityQueueData{}
	}
	return true, pq.q[0]
}

// Len returns the number of entries currently in the queue.
func (pq *StPriorityQueue) Len() int { return len(pq.q) }

// Clear empties the queue.
func (pq *StPriorityQueue) Clear() { pq.q = nil }

// NotEmpty is the negation of empty; convenient as a loop predicate.
func (pq *StPriorityQueue) NotEmpty() bool { return pq.Len() > 0 }
