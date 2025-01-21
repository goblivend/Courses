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

x = [int(d['K']) for d in data]
# T_avg is a list of int => '[12,3]' => [12,3]

T_avg = [float(d['T_avg'][1:-1].split(',')[0]) for d in data]
waiting1_avg = [float(d['T_wait1_avg'][1:-1].split(',')[0]) for d in data]
waiting2_avg = [float(d['T_wait2_avg'][1:-1].split(',')[0]) for d in data]
waiting_avg = [ t1 + t2 for t1, t2 in zip(waiting1_avg, waiting2_avg)]
load1 = [float(d['load1_avg'])*100 for d in data]
load2 = [float(d['load2_avg'])*100 for d in data]

print(T_avg)
print(load1)
print(load2)

fig, ax1 = plt.subplots()

colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
print(colors)

# Plot T_avg on the left y-axis
ax1.plot(x, T_avg, label='T_avg', color=colors[0])
ax1.plot(x, waiting_avg, label='T_waiting_avg', color=colors[1])
ax1.plot(x, waiting1_avg, label='T_waiting1_avg', color=colors[2])
ax1.plot(x, waiting2_avg, label='T_waiting2_avg', color=colors[3])
ax1.set_xlabel('Nb of Servers (K)')
ax1.set_ylabel('T_avg (minutes)')
ax1.tick_params(axis='y')

# Set x and y limits to ensure lines touch the axes
ax1.set_xlim([min(x), max(x)])
ax1.set_ylim([0, max(T_avg + waiting_avg)*1.1])


# Create a second y-axis for load1 and load2
ax2 = ax1.twinx()
ax2.plot(x, load1, label='load1', color=colors[4])
ax2.plot(x, load2, label='load2', color=colors[5])
ax2.set_ylabel('Load (%)')
ax2.tick_params(axis='y')

ax2.set_ylim([0, 110])


# Add a dotted line at 80% as a target
ax2.axhline(y=80, color='gray', linestyle='--', linewidth=1, label='Target 80%')

for i in range(1, len(load1)):
    if load1[i-1] >= 80 and load1[i] < 80:
        # Interpolate the x position where load1 crosses 80%
        x_interp = x[i-1] + (x[i] - x[i-1]) * (80 - load1[i-1]) / (load1[i] - load1[i-1])
        ax2.axvline(x=x_interp, color='gray', linestyle='--', linewidth=1)
        ax2.annotate(f'{x_interp:.2f}', xy=(x_interp, 80), xytext=(x_interp, 0.74), horizontalalignment='right')

# Add a title and legend
plt.title('Relation entre le nombre de serveur (K) et les performances du système')
fig.tight_layout()
fig.subplots_adjust(right=0.75)
fig.legend(loc='upper right', bbox_to_anchor=(1, 0.9))

plt.show()
