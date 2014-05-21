package pso

import (
	"math"
	"math/rand"
	"time"
)

type (
	particle struct {
		position, best, velocity []float64
		bestCost                 float64
	}
	dimension struct {
		min, max float64
	}
	ParticleSwarmOptimizer struct {
		Iterations, Size     int
		Ω, Φp, Φg            float64
		Rand                 *rand.Rand
		Dimensions           []dimension
		OptimizationFunction OptimizationFunction
	}
	OptimizationFunction func([]float64) float64
)

func New(f OptimizationFunction) *ParticleSwarmOptimizer {
	return &ParticleSwarmOptimizer{
		Iterations:           500,
		Size:                 -1,
		Ω:                    math.NaN(),
		Φp:                   math.NaN(),
		Φg:                   math.NaN(),
		Rand:                 rand.New(rand.NewSource(time.Now().UnixNano())),
		Dimensions:           []dimension{},
		OptimizationFunction: f,
	}
}

func (pso *ParticleSwarmOptimizer) AddDimension(min, max float64) {
	pso.Dimensions = append(pso.Dimensions, dimension{min, max})
}

func (pso *ParticleSwarmOptimizer) Solve() ([]float64, float64) {
	pso.setDefaults()

	pop := make([]particle, pso.Iterations)
	globalBest := make([]float64, len(pso.Dimensions))
	globalBestCost := 0.0
	f := pso.OptimizationFunction

	for i := range pop {
		pop[i].position = make([]float64, len(pso.Dimensions))
		pop[i].best = make([]float64, len(pso.Dimensions))
		pop[i].velocity = make([]float64, len(pso.Dimensions))
		for j, dim := range pso.Dimensions {
			pop[i].position[j] = dim.min + (dim.max-dim.min)*pso.Rand.Float64()
			pop[i].best[j] = pop[i].position[j]
			pop[i].velocity[j] = -(dim.max - dim.min) + (dim.max-dim.min)*2*pso.Rand.Float64()
		}
		cost := f(pop[i].position)
		if i == 0 || cost < globalBestCost {
			copy(globalBest, pop[i].position)
			globalBestCost = cost
		}
	}

	for it := 0; it < pso.Iterations; it++ {
		for i := range pop {
			rP, rG := pso.Rand.Float64(), pso.Rand.Float64()
			for j, dim := range pso.Dimensions {
				pop[i].velocity[j] = pso.Ω*pop[i].velocity[j] +
					pso.Φp*rP*(pop[i].best[j]-pop[i].position[j]) +
					pso.Φg*rG*(globalBest[j]-pop[i].position[j])
				pop[i].position[j] += pop[i].velocity[j]
				if pop[i].position[j] > dim.max {
					pop[i].position[j] = dim.max
				} else if pop[i].position[j] < dim.min {
					pop[i].position[j] = dim.min
				}
			}
			cost := f(pop[i].position)
			if cost < pop[i].bestCost {
				copy(pop[i].best, pop[i].position)
				pop[i].bestCost = cost
			}
			if cost < globalBestCost {
				copy(globalBest, pop[i].position)
				globalBestCost = cost
			}
		}
	}

	return globalBest, globalBestCost
}

func (pso *ParticleSwarmOptimizer) setDefaults() {
	type parameter struct {
		dimensions, evaluations, size int
		ω, φp, φg                     float64
	}
	// this table is from:
	// http://www.cof.orst.edu/cof/teach/fe640/Class_Materials/Particle%20Swarm/PSO%20parameters.pdf
	parameters := []parameter{
		{2, 400, 25, 0.3925, 2.5586, 1.3358},
		{2, 4000, 156, 0.4091, 2.1304, 1.0575},
		{5, 1000, 63, -0.3593, -0.7238, 2.0289},
		{5, 10000, 223, -0.3699, -0.1207, 3.3657},
		{10, 2000, 63, 0.6571, 1.6319, 0.6239},
		{10, 20000, 53, -0.3488, -0.2746, 4.8976},
		{20, 40000, 69, -0.4438, -0.2699, 3.3950},
		{20, 400000, 149, -0.3236, -0.1136, 3.9789},
		{30, 600000, 95, -0.6031, -0.6485, 2.6475},
		{50, 100000, 106, -0.2256, -0.1564, 3.8876},
		{100, 200000, 161, -0.2089, -0.0787, 3.7637},
	}
	dim := parameters[0].dimensions
	evs := parameters[0].evaluations
	for _, p := range parameters {
		if distance(len(pso.Dimensions), p.dimensions) < distance(len(pso.Dimensions), dim) {
			dim = p.dimensions
			evs = p.evaluations
		}
	}
	sz, ω, φp, φg := 0, 0.0, 0.0, 0.0
	for _, p := range parameters {
		if p.dimensions == dim &&
			distance(pso.Iterations, p.evaluations) <= distance(pso.Iterations, evs) {
			evs = p.evaluations
			sz, ω, φp, φg = p.size, p.ω, p.φp, p.φg
		}
	}

	if pso.Size < 0 {
		pso.Size = sz
	}
	if math.IsNaN(pso.Ω) {
		pso.Ω = ω
	}
	if math.IsNaN(pso.Φp) {
		pso.Φp = φp
	}
	if math.IsNaN(pso.Φg) {
		pso.Φg = φg
	}
}

func distance(x, y int) int {
	return int(math.Abs(float64(y - x)))
}
