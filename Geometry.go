// SPDX-License-Identifier: MPL-2.0
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package dijkstra

import "math"

// Distance returns the Euclidean distance between (x1, y1) and
// (x2, y2). The unit of the result matches the unit of the inputs.
func Distance(x1, y1, x2, y2 float64) float64 {
	dx := x2 - x1
	dy := y2 - y1
	return math.Sqrt(dx*dx + dy*dy)
}

// DistanceCM is Distance multiplied by 100, useful when the input
// coordinates are in metres and the caller wants centimetres.
func DistanceCM(x1, y1, x2, y2 float64) float64 {
	return Distance(x1, y1, x2, y2) * 100.0
}
