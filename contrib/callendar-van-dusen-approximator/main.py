#!/usr/bin/env python3

# analytical solution:
# t = -1/2 sqrt((2.25689e-20 (14123424280463317916104064499712 y - 15409551594649655757462803841024 K))/(477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3) + (477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3)/K - 89539.2) - 1/2 sqrt(-(2.25689e-20 (14123424280463317916104064499712 y - 15409551594649655757462803841024 K))/(477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3) - (477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3)/K - (1.8551e9)/sqrt((2.25689e-20 (14123424280463317916104064499712 y - 15409551594649655757462803841024 K))/(477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3) + (477244327538622144 K^3 - 42810999909758488 K^2 y + 1.69526e-30 K^(3/2) sqrt(93888020965527858323701368714067466232986277692359736916684766077860651474130183296704764182528 K^3 - 54462502494335639718685321165686266165384797958199188415987855307271839840242508360456431730688 K^2 y + 37522856856107613087317656212462989703632371986859477818291109746388036214224728942788644700160 K y^2 - 11268860665316324488520037218937802013667426589820931462978601746317432642597497101502847647744 y^3))^(1/3)/K - 89539.2) - 179078.) + 25

import numpy as np
import matplotlib.pyplot as plt

# Constants for Callendar-Van Dusen equation (Platinum RTD, typical values)
R0 = 100  # Resistance at 0°C
A = 3.9083e-3
B = -5.775e-7
C = -4.183e-12  # Only valid for temperatures below 0°C

# Function to calculate resistance based on temperature
def resistance(t):
    if t < 0:
        return R0 * (1 + A * t + B * t**2 + C * (t - 100) * t**3)
    else:
        return R0 * (1 + A * t + B * t**2)

print("preparing values")
# Generate equidistant temperature positions
n_samples = 500_000
temperatures = np.linspace(-200, 900, n_samples, dtype=np.float32)

# Calculate resistances for the generated temperatures
vresistance = np.vectorize(resistance)
resistances = vresistance(temperatures)
print("done preparing values")

# Function to generate lookup table
def generate_lookup_table(min_temp, max_temp, desired_max_err):
    lookup_table = [(0, resistances[0], temperatures[0])]  # Start with first entry
    first_index = 1

    while first_index < n_samples - 1:
        # print(f"Lookup table construction at {100.0 * first_index / n_samples}% ({first_index=})")
        current_end_index = n_samples - 1
        next_change = (current_end_index - first_index) // 2

        # Binary search to find the best position
        while True:
            f_approx = np.vectorize(lambda r: np.interp(r, [resistances[first_index], resistances[current_end_index]], [temperatures[first_index], temperatures[current_end_index]]))
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
        lookup_table.append((current_end_index+1, resistances[current_end_index], temperatures[current_end_index]))
        first_index = current_end_index + 1

    return lookup_table

# Define parameters for the lookup table generation
min_temp = -200
max_temp = 900
max_err = 0.001  # Desired maximum error in °C

# Generate the lookup table
lookup_table = generate_lookup_table(min_temp, max_temp, max_err)
approx_temps = np.interp(resistances, [x[1] for x in lookup_table], [x[2] for x in lookup_table])
error = approx_temps - temperatures
# abs_error = np.abs(approx_temps - temperatures)
max_error = np.max(np.abs(approx_temps - temperatures))
print(f"Table calculation done!")
print(f"Entries:       ", len(lookup_table))
print(f"Maximum Error: {max_error:.4f}", )
print(f"Mean Error:    {error.mean():.4f}")
for i,r,t in lookup_table:
    print(f"{i:20},{r:20},{t:20}")

# Create the plot
fig, ax1 = plt.subplots(figsize=(10, 6))

# Plot predicted and real temperatures
ax1.plot(resistances, approx_temps, label="Predicted Temperature", color='tab:blue')
ax1.plot(resistances, temperatures, label="Real Temperature", linestyle="--", color='tab:orange')
ax1.set_xlabel("Resistance (Ω)")
ax1.set_ylabel("Temperature (°C)")
ax1.set_title("Piecewise Linear Approximation of Callendar-Van Dusen Equation")
ax1.grid(True)
ax1.legend(loc='upper left')

# Create a second y-axis for the error
ax2 = ax1.twinx()
ax2.plot(resistances, error, label="Error", color='tab:red')
ax2.set_ylabel("Error")

# Combine legends from both y-axes
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines + lines2, labels + labels2, loc='upper left')

plt.show()
