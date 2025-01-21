import matplotlib.pyplot as plt
import csv


import argparse

parser = argparse.ArgumentParser(description='Draw a graph from simulation results')
parser.add_argument('--file', '-f', type=str, help='File containing simulation results', default='simulation_results.csv')
args = parser.parse_args()

data = []
with open(args.file, 'r') as f :
    reader = csv.reader(f)
    data = [r for r in reader]

titles = data[0]
print(titles)
data = [{title: val for title, val in zip(titles, line)} for line in data[1:]]

x = [int(d['max_k_s']) for d in data]
# T_avg is a list of int => '[12,3]' => [12,3]

reject_1 = [float(d['alpha_s'][:-1]) for d in data]
reject_2 = [float(d['alpha_f'][:-1]) for d in data]
reject = [ t1 + t2 for t1, t2 in zip(reject_1, reject_2)]

fig, ax1 = plt.subplots()

colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
print(colors)

# Plot T_avg on the left y-axis
ax1.plot(x, reject, label='reject_total', color=colors[0])
ax1.plot(x, reject_1, label='reject_arrival', color=colors[1])
ax1.plot(x, reject_2, label='reject_results', color=colors[2])
ax1.set_xlabel('Size of second queue (kf)')
ax1.set_ylabel('Reject rate (%)')
ax1.tick_params(axis='y')

# Set x and y limits to ensure lines touch the axes
ax1.set_xlim([min(x), max(x)])
ax1.set_ylim([0, max(reject)*1.1])


# Add a dotted line at 0.1% as a target
ax1.axhline(y=0.1, color='gray', linestyle='--', linewidth=1, label='Target 0.1%')
# ax1.axhline(y=0.05, color='gray', linestyle='--', linewidth=1, label='Target 0.05%')

for i in range(1, len(reject_1)):
    if reject_1[i-1] >= 0.1 and reject_1[i] < 0.1:
        # Interpolate the x position where reject_1 crosses 0.1%
        x_interp = x[i-1] + (x[i] - x[i-1]) * (0.1 - reject_1[i-1]) / (reject_1[i] - reject_1[i-1])
        ax1.axvline(x=x_interp, color='gray', linestyle='--', linewidth=1)
        ax1.annotate(f'{x_interp:.2f}', xy=(x_interp, 1), xytext=(x_interp, 0.74), horizontalalignment='right')

# for i in range(1, len(reject_2)):
#     if reject_2[i-1] >= 0.05 and reject_2[i] < 0.05:
#         print
#         # Interpolate the x position where reject_2 crosses 0.05%
#         x_interp = x[i-1] + (x[i] - x[i-1]) * (0.05 - reject_2[i-1]) / (reject_2[i] - reject_2[i-1])
#         ax1.axvline(x=x_interp, color='gray', linestyle='--', linewidth=1)
#         ax1.annotate(f'{x_interp:.2f}', xy=(x_interp, 1), xytext=(x_interp, 1.3), horizontalalignment='right')

# Add a title and legend
plt.title('Relation entre la taille de la deuxième file (kf) et les taux de rejet')
fig.tight_layout()
fig.subplots_adjust(right=0.75)
fig.legend(loc='upper right', bbox_to_anchor=(1, 0.9))

plt.show()
