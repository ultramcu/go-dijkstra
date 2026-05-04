// SPDX-License-Identifier: MIT
// Copyright (c) 2026 MaIII Themd

package dijkstra

import "fmt"

// StStack is a simple LIFO stack of strings. Exported for users who
// want to do their own graph traversal on top of the StGraph types.
type StStack struct {
	data []string
}

// Push appends data to the top of the stack. Always returns true.
func (st *StStack) Push(data string) bool {
	st.data = append(st.data, data)
	return true
}

// Pop removes and returns the top of the stack.
// Returns "" if the stack is empty.
func (st *StStack) Pop() string {
	n := len(st.data)
	if n == 0 {
		return ""
	}
	v := st.data[n-1]
	st.data = st.data[:n-1]
	return v
}

// Len returns the current depth of the stack.
func (st *StStack) Len() int { return len(st.data) }

// Clear empties the stack.
func (st *StStack) Clear() { st.data = nil }

// Empty reports whether the stack contains no entries.
func (st *StStack) Empty() bool { return len(st.data) == 0 }

// Print writes the stack's contents to stdout in bottom-to-top order.
func (st *StStack) Print() {
	fmt.Printf("Stack : ")
	for _, s := range st.data {
		fmt.Printf("%s, ", s)
	}
	fmt.Printf("\r\n")
}
