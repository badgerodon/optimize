package pso

import (
	"math"
	"testing"
)

func Test(t *testing.T) {
	s := New(func(xs []float64) float64 {
		return math.Abs(math.Cos(xs[0]))
	})
	s.AddDimension(-2*math.Pi, 2*math.Pi)
	vs, v := s.Solve()
	if !nearlyEqual(v, 0, 0.01) {
		t.Errorf("Expected value to be %v, got %v", 0, v)
	}
	if !nearlyEqual(vs[0], -1.57, 0.01) && !nearlyEqual(vs[0], 1.57, 0.01) {
		t.Errorf("Expected value to be %v, got %v", 4.71, vs[0])
	}
}

func nearlyEqual(x, y, e float64) bool {
	return math.Abs(y-x) < e
}
