"""
This is just a scratch file that contains some checks on calculations
"""

# Solve the system of equations for a and vmax (tt, tr, tvmax and d are known, the rest are unknown):
# I   tt = tr + tvmax + tr
# II  d = dr + dvmax + dr
# III tr = sqrt(2 * dr / a)
# IV  tvmax = dvmax / vmax
# V   vmax = a * tr
#
# V in IV: tvmax = dvmax / (a * tr)
# VI  dvmax = tvmax * (a * tr)
#
# VI in II: d = dr + tvmax * (a * tr) + dr = 2*dr + tvmax * a * tr
# 2*dr = d - tvmax * a * tr
# VII dr = (d - tvmax * a * tr) / 2
#
# VII in III: tr = sqrt(2 * (d - tvmax * a * tr) / 2 / a) = sqrt((d - tvmax * a * tr) / a)
# tr^2 = (d - tvmax * a * tr) / a  | * a
# a * tr^2 = d - tvmax * a * tr  | +tvmax * a * tr
# d = a * tr^2 + tvmax * a * tr = a (tr^2 + tvmax * tr)
# VIII a = d / (tr^2 + tvmax * tr)
#
# VIII in V:
# vmax = tr * d / (tr^2 + tvmax * tr) = d / (tr + tvmax)


# Calculating the true unit of Acceleration values

# So the acceleration value 'acc' determines the TIME before RPM is incremented/decremented by 1
# Example: 50s/1000000 * (256-236) = 1ms  (where acc = 236)
# td = (256-acc) * 0.00005s
# a = dv / td = 1 / ((256-acc) * 0.00005s)



from sympy import symbols, Eq, solve, sqrt


# Define all variables again including unknowns
a, vmax, tt, tr, tvmax, d, dr, dvmax = symbols('a vmax tt tr tvmax d dr dvmax')


# I   tt = tr + tvmax + tr
# II  d = dr + dvmax + dr
# III tr = sqrt(2 * dr / a)
# IV  tvmax = dvmax / vmax
# V   vmax = a * tr

# Original equations
eq1 = Eq(tt, 2*tr + tvmax)
eq2 = Eq(d, 2*dr + dvmax)
eq3 = Eq(tr, sqrt(2*dr/a))
eq4 = Eq(tvmax, dvmax/vmax)
eq5 = Eq(vmax, a*tr)

# Substitute eq5 into eq4 to express dvmax in terms of a, tr, and tvmax
eq6 = eq4.subs(vmax, eq5.rhs)

# Now substitute eq6 into eq2 to express d in terms of dr, a, tr, and tvmax
eq7 = eq2.subs(dvmax, eq6.rhs * tvmax)

# Express dr in terms of d, a, tr, and tvmax
eq8 = Eq(dr, solve(eq7, dr)[0])

# Substitute eq8 into eq3 to find a relationship involving a, tr, d, and tvmax
eq9 = eq3.subs(dr, eq8.rhs)

# Solve eq9 for a to find its expression in terms of known quantities
a_solution = solve(eq9, a)

# Use the found expression for a to find vmax
vmax_solution = solve(eq5.subs(a, a_solution[0]), vmax)

a_solution, vmax_solution