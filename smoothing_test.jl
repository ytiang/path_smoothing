using JuMP, Ipopt, NLopt
using PyPlot
using PyCall
using CSV

FILE = "/home/yangt/catkin_ws/src/path_smoothing/astar_path.csv"
df = CSV.read(FILE; header = true)
px = Array(df[:x])
py = Array(df[:y])
numPoint = length(px)


mdl = Model(solver = IpoptSolver())  # set model
# mdl = Model(solver = NLoptSolver(algorithm = :LD_LBFGS))
@variable(mdl, x[i=1:numPoint])
@variable(mdl, y[i=1:numPoint])
for i=1:numPoint
    setvalue(x[i], px[i])
    setvalue(y[i], py[i])
end
@NLexpression(mdl, Δx[i=1:numPoint-1], x[i+1]-x[i])
@NLexpression(mdl, Δy[i=1:numPoint-1], y[i+1]-y[i])
@NLexpression(mdl, normΔP[i = 1:numPoint-1], sqrt(Δx[i]^2 + Δy[i]^2))
@NLexpression(mdl, cosθ[i=1:numPoint - 2], (Δx[i]*Δx[i+1]+Δy[i]*Δy[i+1])/(normΔP[i]*normΔP[i+1]))
@NLexpression(mdl, Δθ[i=1:numPoint-2], acos(cosθ[i]>1.0?1.0:cosθ[i]))
@NLexpression(mdl, F1, sum( ((Δx[i+1]-Δx[i])^2+(Δy[i+1]-Δy[i])^2) for i=1:numPoint-2))
@NLexpression(mdl, F2, sum( (Δθ[i]/normΔP[i])^2 for i=1:numPoint-2))
@NLobjective(mdl, Min, F1 + F2)

@time result = solve(mdl)
X = getvalue(x)
Y = getvalue(y)


pfig = figure("path")
pax = pfig[:add_subplot](1,1,1)
pax[:set_aspect]("equal")
# pax[:set_xlim](-1.1, 1.1)
# pax[:set_ylim](-1.1, 1.1)
pax[:plot](px, py)
pax[:plot](X, Y)
