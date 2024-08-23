#!/usr/bin/env python3

# analytical solution:
# t = -1/2 sqrt((2.25689e-20 (14123424280463317916104064499712 y - 15409551594649655757462803841024 K))/(477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3) + (477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3)/K - 89539.2) - 1/2 sqrt(-(2.25689e-20 (14123424280463317916104064499712 y - 15409551594649655757462803841024 K))/(477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3) - (477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3)/K - (1.8551e9)/sqrt((2.25689e-20 (14123424280463317916104064499712 y - 15409551594649655757462803841024 K))/(477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3) + (477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3)/K - 89539.2) - 179078.) + 25

import matplotlib.pyplot as plt
import numpy as np
import scipy

n_samples = 500_000

# Define parameters for the lookup table generation
min_temp = -200
max_temp = 900
max_err = 0.0001  # Desired maximum error in °C

# Constants for Callendar-Van Dusen equation (Platinum RTD, typical values)
R0 = 100  # Resistance at 0°C
A =  3.9083e-3
B = -5.775e-7
C = -4.183e-12  # Only valid for temperatures below 0°C

# Function to calculate resistance based on temperature
def resistance(t):
    if t < 0:
        return R0 * (1 + A * t + B * t**2 + C * (t - 100) * t**3)
    else:
        return R0 * (1 + A * t + B * t**2)

def approx_order_1(r, r1, t1, r2, t2):
    return np.interp(r, [r1, r2], [t1, t2])

def parabola(r, a, b, c):
    return a * r**2 + b * r + c

def cubic(r, a, b, c, d):
    return a * r**3 + b * r**2 + c * r + d

def o4(r, a, b, c, d, e):
    return a * r**4 + b * r**3 + c * r**2 + d * r + e

def o5(r, a, b, c, d, e, f):
    return e * r**5 + a * r**4 + b * r**3 + c * r**2 + d * r + e

def o6(r, a, b, c, d, e, f, g):
    return f * r**6 + e * r**5 + a * r**4 + b * r**3 + c * r**2 + d * r + e
Fs = [parabola, cubic, o4, o5, o6]
Frange = list(range(2, 2 + len(Fs)))

approx_order = 4

print("preparing values")
# Generate equidistant temperature positions
temperatures = np.linspace(min_temp, max_temp, n_samples, dtype=np.float32)

# Calculate resistances for the generated temperatures
vresistance = np.vectorize(resistance)
resistances = vresistance(temperatures)
resistances = np.float32(resistances)
print("done preparing values")

# Function to generate lookup table
def generate_lookup_table(min_temp, max_temp, desired_max_err):
    lookup_table = [(0, resistances[0], temperatures[0], None)]  # Start with first entry
    first_index = 1

    while first_index < n_samples - 1:
        # print(f"Lookup table construction at {100.0 * first_index / n_samples}% ({first_index=})")
        current_end_index = n_samples - 1
        next_change = (current_end_index - first_index) // 2

        # Binary search to find the best position
        while True:
            fit_params = None
            if approx_order in Frange:
                F = Fs[approx_order - 2]
                fit_params, pcov = scipy.optimize.curve_fit(F, resistances[first_index:current_end_index+1], temperatures[first_index:current_end_index+1])
                f_approx = np.vectorize(lambda r: F(r, *fit_params))
            else:
                f_approx = np.vectorize(lambda r: approx_order_1(r, resistances[first_index], temperatures[first_index], resistances[current_end_index], temperatures[current_end_index]))
            # print(f"Check range [{first_index},{current_end_index}]")

            approx_temps = f_approx(resistances[first_index:current_end_index+1])
            max_error = np.max(np.abs(approx_temps - temperatures[first_index:current_end_index+1]))

            if next_change == 0:
                # print(f"Found optimum at index={current_end_index} with max_error={max_error:.4f}")
                print(f"{100.0 * current_end_index / (n_samples - 1)}% - creating segment at index={current_end_index:6}, max_error={max_error:.4f}")
                break

            if max_error > desired_max_err:
                # print(f" -> Max error {max_error:.4f} °C is too high, reducing range")
                current_end_index -= next_change
            else:
                # print(f" -> Max error {max_error:.4f} °C may be too low, increasing range")
                current_end_index += next_change

            if current_end_index >= n_samples - 1:
                # print(f"Encountered last piece. Finishing table.")
                current_end_index = n_samples - 1
                print(f"{100.0 * current_end_index / (n_samples - 1)}% - creating segment at index={current_end_index:6}, max_error={max_error:.4f}")
                break

            next_change = next_change // 2

        # Add the best position to the lookup table
        lookup_table.append((current_end_index+1, resistances[current_end_index], temperatures[current_end_index], fit_params))
        first_index = current_end_index + 1

    return lookup_table

# Generate the lookup table
lookup_table = generate_lookup_table(min_temp, max_temp, max_err)

# t_low = np.linspace(min_temp - 5, min_temp, 500, dtype=np.float32)
# t_high = np.linspace(max_temp, max_temp + 5, 500, dtype=np.float32)
# r_low = vresistance(t_low)
# r_high = vresistance(t_high)
# Ts = np.concatenate([t_low, temperatures, t_high], axis=0)
# Rs = np.concatenate([r_low, resistances, r_high], axis=0)
Ts = temperatures
Rs = resistances

if approx_order in Frange:
    F = Fs[approx_order - 2]
    approx_temps = []
    # low extend
    # _,_,_,params = lookup_table[1]
    # f_approx = np.vectorize(lambda r: F(r, *params))
    # approx_temps.append(f_approx(r_low))
    # valid range
    for k,(li,lr,lt,_) in enumerate(lookup_table[0:-1]):
        ri,rr,rt,params = lookup_table[k+1]
        f_approx = np.vectorize(lambda r: F(r, *params))
        approx_temps.append(f_approx(resistances[li:ri]))
    # high extend
    # _,_,_,params = lookup_table[-1]
    # f_approx = np.vectorize(lambda r: F(r, *params))
    # approx_temps.append(f_approx(r_high))
    # concat
    approx_temps = np.concatenate(approx_temps, axis=0)
elif approx_order == 1:
    approx_temps = np.interp(Rs, [x[1] for x in lookup_table], [x[2] for x in lookup_table])

error = approx_temps - Ts
max_error = np.max(np.abs(error))
print(f"Table calculation done!")
print(f"Entries:       ", len(lookup_table))
print(f"Maximum Error: {max_error:.4f}", )
print(f"Mean Error:    {error.mean():.4f}")
for i,r,t,params in lookup_table:
    print(f"{i:20},{r:20},{t:20},{params}")

print(f"""
// 4th order piecewise approximation table, requires {len(lookup_table) - 1}*6*4 = {(len(lookup_table) - 1)*6*4} bytes.
// - Values for R0={R0}Ω
// - maximum error: +- {max_err}°C
// - valid range: [{min_temp}°C, {max_temp}°C]
// - no need for binary search
#[rustfmt::skip]
const LOOKUP_TABLE_R{R0}_RESISTANCES: [f32; {len(lookup_table) - 1}] = [
    // maximum valid resistance for the params at the same index
{"\n".join(f"    {r}," for _,r,_,_ in lookup_table[1:])}
];

#[rustfmt::skip]
const LOOKUP_TABLE_R{R0}_PARAMS: [[f32; 5]; {len(lookup_table) - 1}] = [
    // polynomial params [a, b, c, d, e])
{"\n".join("    [" + ", ".join(f"{str(x):25}" for x in params) + " ]," for _,_,_,params in lookup_table[1:])}
];
"""
)

print(f"""
#[cfg(test)]
mod test {{
    #[test]
    fn test_lookup_table_r{R0}() {{""")
N = 100
for i in range(N):
    idx = len(resistances) * i // (N + 1)
    r = resistances[idx]
    t = temperatures[idx]
    print(f"        assert_abs_diff_eq!(resistance_to_temperature_r{R0}({r}), {t}, epsilon = {max_err});")
print("""    }
}
""")

# Create the plot
fig, ax1 = plt.subplots(figsize=(10, 6))

# Plot predicted and real temperatures
ax1.plot(Rs, approx_temps, label="Predicted Temperature", color='tab:blue')
ax1.plot(Rs, Ts, label="Real Temperature", linestyle="--", color='tab:orange')
ax1.set_xlabel("Resistance (Ω)")
ax1.set_ylabel("Temperature (°C)")
ax1.set_title("Piecewise Linear Approximation of Callendar-Van Dusen Equation")
ax1.grid(True)
ax1.legend(loc='upper left')

# Create a second y-axis for the error
ax2 = ax1.twinx()
ax2.plot(Rs, error, label="Error", color='tab:red')
ax2.set_ylabel("Error")

# Combine legends from both y-axes
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines + lines2, labels + labels2, loc='upper left')

plt.show()
