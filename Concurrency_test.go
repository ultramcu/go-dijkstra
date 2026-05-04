// SPDX-License-Identifier: MPL-2.0
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package dijkstra

import (
	"sync"
	"sync/atomic"
	"testing"
)

// TestConcurrentSearches runs many DijkstraSearch goroutines on the
// same graph and asserts every search returns the same correct path.
// Run with `go test -race` to catch any data race that would slip
// past the lock.
func TestConcurrentSearches(t *testing.T) {
	g := buildSimpleGraph()

	const goroutines = 64
	const perGoroutine = 50
	var wg sync.WaitGroup
	var failures int64

	for i := 0; i < goroutines; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < perGoroutine; j++ {
				ok, path := g.DijkstraSearch("A", "D")
				if !ok || len(path) != 4 {
					atomic.AddInt64(&failures, 1)
					return
				}
				want := []string{"A", "B", "C", "D"}
				for k, p := range path {
					if p.Name != want[k] {
						atomic.AddInt64(&failures, 1)
						return
					}
				}
				if path[len(path)-1].Cost != 4 {
					atomic.AddInt64(&failures, 1)
					return
				}
			}
		}()
	}
	wg.Wait()

	if failures > 0 {
		t.Fatalf("%d/%d searches returned a wrong result",
			failures, int64(goroutines)*int64(perGoroutine))
	}
}

// TestConcurrentReadsDuringSearch verifies that pure read methods
// remain safe to call concurrently while searches are also running.
// Useful primarily as a -race check.
func TestConcurrentReadsDuringSearch(t *testing.T) {
	g := buildSimpleGraph()

	stop := make(chan struct{})
	var searchers sync.WaitGroup

	// Long-running searchers, bounded by the stop channel.
	for i := 0; i < 4; i++ {
		searchers.Add(1)
		go func() {
			defer searchers.Done()
			for {
				select {
				case <-stop:
					return
				default:
				}
				g.DijkstraSearch("A", "D")
			}
		}()
	}

	// Fixed-work readers.
	var readers sync.WaitGroup
	for i := 0; i < 8; i++ {
		readers.Add(1)
		go func() {
			defer readers.Done()
			for j := 0; j < 500; j++ {
				_ = g.VertexFind("C")
				_ = g.VertexIsExist("D")
				_, _ = g.NearPoint(2.1, 0)
			}
		}()
	}

	readers.Wait()
	close(stop)
	searchers.Wait()
}

// TestConcurrentMutationAndSearch verifies that VertexBLock and
// VertexBLockRemove can run concurrently with searches without
// data races. The point of this test is that the race detector
// stays quiet -- we don't assert path correctness because the
// blocked set is being toggled mid-flight.
func TestConcurrentMutationAndSearch(t *testing.T) {
	g := buildSimpleGraph()

	stop := make(chan struct{})
	var searchers sync.WaitGroup

	for i := 0; i < 4; i++ {
		searchers.Add(1)
		go func() {
			defer searchers.Done()
			for {
				select {
				case <-stop:
					return
				default:
				}
				g.DijkstraSearch("A", "D")
			}
		}()
	}

	var mutators sync.WaitGroup
	for i := 0; i < 4; i++ {
		mutators.Add(1)
		go func() {
			defer mutators.Done()
			for j := 0; j < 500; j++ {
				g.VertexBLock("C")
				g.VertexBLockRemove("C")
			}
		}()
	}

	mutators.Wait()
	close(stop)
	searchers.Wait()
}
