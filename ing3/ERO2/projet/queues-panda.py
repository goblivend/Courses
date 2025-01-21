import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class QueueBase:
    """
    Base class for queuing systems.
    Contains shared methods and attributes for MM1, MM1k, and MMKK.
    """

    def __init__(self, lamda, mu, gen, test_z=100):
        self.lamda = lamda
        self.mu = mu
        self.gen = gen
        self.test_z = test_z
        self.tops = pd.DataFrame(columns=['t_arval_queue', 't_arval_srv', 't_depart_sys'])

    def compute_counts(self, nb_tops):
        """
        Computes the number of agents in the system at any given time.
        """
        tops = self.tops
        t_range = np.linspace(0., tops['t_depart_sys'].max(), nb_tops)
        delta = tops['t_depart_sys'].max() / nb_tops
        counts = pd.DataFrame(index=t_range, columns=['ag_in_sys', 'ag_in_queue', 'ag_in_service', 'blocked'])

        for t in counts.index:
            counts.loc[t, 'ag_in_sys'] = ((tops['t_arval_queue'] <= t) & (tops['t_depart_sys'] >= t)).sum()
            counts.loc[t, 'ag_in_queue'] = ((tops['t_arval_queue'] <= t) & (tops['t_arval_srv'] > t)).sum()
            counts.loc[t, 'ag_in_service'] = ((tops['t_arval_srv'] <= t) & (tops['t_depart_sys'] >= t)).sum()

            counts.loc[t, 'blocked'] = ((t <= tops['t_arval_queue']) &  (tops['t_arval_queue'] <= t+delta) & tops['blocked']).sum()
        return counts

    def compute_stats(self):
        """
        Computes key statistics for the simulation.
        """
        tops = self.tops
        total_served = (~tops['blocked']).sum()
        stats = {
            'mean_sojourn_time': tops['t_sojourn'].sum() / total_served,
            'mean_waiting_time': tops['t_waiting'].sum() / total_served,
            'mean_service_time': tops['t_service'].sum() / total_served,
            'waiting_proportion': (tops['t_waiting'] > 0).sum() / total_served,
            'blocked_proportion': tops['blocked'].sum() / self.test_z if 'blocked' in tops else 0.0
        }
        return stats

    def plot(self, n, title) :
        counts = self.compute_counts(n)
        plt.plot(counts.index, counts['ag_in_sys'], label='Agents in System')
        plt.plot(counts.index, counts['ag_in_queue'], label='Agents in Queue')
        plt.plot(counts.index, counts['ag_in_service'], label='Agents in Service')
        plt.plot(counts.index, counts['blocked'], label='Agents Blocked')

        plt.xlabel('Time')
        plt.ylabel('Number of Agents')
        plt.title(title)
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

class MM1(QueueBase):
    """
    MM1 Queue: Single server, unlimited queue size.
    """

    def run(self, arrival_times=None):
        tops = self.tops

        # Initialize times and server availability
        if arrival_times is None:
            tops['t_arval_queue'] = np.cumsum(self.gen.exponential(1. / self.lamda, self.test_z))
        else:
            tops['t_arval_queue'] = arrival_times
            self.test_z = len(arrival_times)
        tops['t_arval_srv'] = tops['t_arval_queue'].copy()
        tops['t_depart_sys'] = tops['t_arval_srv'] + self.gen.exponential(1. / self.mu, self.test_z)

        for i in range(1, self.test_z):
            if tops.loc[i, 't_arval_queue'] < tops.loc[i - 1, 't_depart_sys']:
                tops.loc[i, 't_arval_srv'] = tops.loc[i - 1, 't_depart_sys']
            tops.loc[i, 't_depart_sys'] = tops.loc[i, 't_arval_srv'] + self.gen.exponential(1. / self.mu)

        tops['t_sojourn'] = tops['t_depart_sys'] - tops['t_arval_queue']
        tops['t_waiting'] = tops['t_arval_srv'] - tops['t_arval_queue']
        tops['t_service'] = tops['t_depart_sys'] - tops['t_arval_srv']

class MMKK(QueueBase):
    """
    MMKK Queue: K servers, limited queue size.
    """

    def __init__(self, lamda, mu, gen, num_servers, queue_size, test_z=100):
        super().__init__(lamda, mu, gen, test_z)
        self.num_servers = num_servers
        self.queue_size = queue_size

    def run(self, arrival_times=None):
        tops = self.tops
        gen = self.gen

        # Initialize times and server availability
        if arrival_times is None:
            tops['t_arval_queue'] = np.cumsum(self.gen.exponential(1. / self.lamda, self.test_z))
        else:
            tops['t_arval_queue'] = arrival_times
            self.test_z = len(arrival_times)
        tops['t_arval_srv'] = -1
        tops['t_depart_sys'] = -1

        # Track when each server becomes available
        server_available_times = [0] * self.num_servers

        for agent_ix in range(self.test_z):
            arrival_time = tops.loc[agent_ix, "t_arval_queue"]

            # Check if there's room in the system (servers + queue)
            in_system = sum(
                (tops.loc[:agent_ix, 't_arval_queue'] <= arrival_time) &
                (tops.loc[:agent_ix, 't_depart_sys'] > arrival_time)
            )
            if self.queue_size is not None and in_system >= self.num_servers + self.queue_size:
                # Blocked
                tops.loc[agent_ix, "t_depart_sys"] = -1
                tops.loc[agent_ix, "blocked"] = True
                print("BLOCKED", agent_ix, arrival_time)
                continue

            # Find the earliest available server
            earliest_server_idx = np.argmin(server_available_times)
            service_start_time = max(arrival_time, server_available_times[earliest_server_idx])

            # Compute departure time
            departure_time = service_start_time + gen.exponential(1.0 / self.mu)

            # Update the server's availability
            server_available_times[earliest_server_idx] = departure_time

            # Record times
            tops.loc[agent_ix, "t_arval_srv"] = service_start_time
            tops.loc[agent_ix, "t_depart_sys"] = departure_time
            tops.loc[agent_ix, "blocked"] = False

        # Compute additional metrics
        tops["t_sojourn"] = np.where(tops["t_depart_sys"] == -1, 0, tops["t_depart_sys"] - tops["t_arval_queue"])
        tops["t_waiting"] = np.where(tops["t_depart_sys"] == -1, 0, tops["t_arval_srv"] - tops["t_arval_queue"])
        tops["t_service"] = np.where(tops["t_depart_sys"] == -1, 0, tops["t_depart_sys"] - tops["t_arval_srv"])
        tops["blocked"] = tops["t_depart_sys"] == -1

class MM1k(QueueBase):
    """
    MM1k Queue: Single server, limited queue size.
    """

    def __init__(self, lamda, mu, gen, queue_size, test_z=100):
        super().__init__(lamda, mu, gen, test_z)
        self.queue_size = queue_size

    def run(self, arrival_times=None):
        tops = self.tops
        if arrival_times is None:
            tops['t_arval_queue'] = np.cumsum(self.gen.exponential(1. / self.lamda, self.test_z))
        else:
            tops['t_arval_queue'] = arrival_times
            self.test_z = len(arrival_times)

        tops['blocked'] = False

        for i in range(len(arrival_times)):
            in_system = ((tops.loc[:i, 't_arval_queue'] <= tops.loc[i, 't_arval_queue']) &
                         (tops.loc[:i, 't_depart_sys'] > tops.loc[i, 't_arval_queue'])).sum()
            if in_system > self.queue_size:
                tops.loc[i, 'blocked'] = True
                continue

            if i > 0 and tops.loc[i, 't_arval_queue'] < tops.loc[i - 1, 't_depart_sys']:
                tops.loc[i, 't_arval_srv'] = tops.loc[i - 1, 't_depart_sys']
            else:
                tops.loc[i, 't_arval_srv'] = tops.loc[i, 't_arval_queue']
            tops.loc[i, 't_depart_sys'] = tops.loc[i, 't_arval_srv'] + self.gen.exponential(1. / self.mu)

        tops['t_sojourn'] = tops['t_depart_sys'] - tops['t_arval_queue']
        tops['t_waiting'] = tops['t_arval_srv'] - tops['t_arval_queue']
        tops['t_service'] = tops['t_depart_sys'] - tops['t_arval_srv']
        tops["blocked"] = tops["t_depart_sys"] == -1
        # print(tops['t_arval_queue'])

class CompoundQueue:

    def __init__(self, gen, queues):
        self.gen = gen
        self.queues = []
        self.titles = []
        for (title, params) in queues :
            self.titles.append(title)
            self.queues.append(MMKK(*params))

    def run(self) :
        assert len(self.queues) > 0

        self.queues[0].run()
        print(0, self.queues[0].compute_stats())
        for i in range(1, len(self.queues)):
            prev_depart = self.queues[i-1].tops.loc[~self.queues[i-1].tops['blocked'], 't_depart_sys'].values
            prev_depart.sort()
            self.queues[i].run(prev_depart)
            print(i, self.queues[i].compute_stats())

    def compute_counts(self, nb_tops):
        return [q.compute_counts(nb_tops) for q in self.queues]

    def plot(self, n) :
        plt.figure(figsize=(12, 6))

        for i in range(len(self.queues)):
            plt.subplot(len(self.queues), 1, i+1)
            self.queues[i].plot(n, self.titles[i])

        plt.show()


def run_config(config) :
    gen = np.random.default_rng(42)
    queues = [
        ('Évolution de la file de tests au cours du temps', (config['lambda_mu1'][0][0], config['lambda_mu1'][0][1], gen, config['K'], config['max_queue1'], 5000)),
        ('Évolution de la file d\'envoi des résultats au cours du temps', (5, config['mu2'], gen, 1, config['max_queue2']))
    ]
    compound_queue = CompoundQueue(gen, queues)
    compound_queue.run()
    compound_queue.plot(5000)


if __name__ == '__main__':
    from configurations import configs
    import argparse

    parser = argparse.ArgumentParser(description='Run compound queue simulations')
    parser.add_argument('--config', '-c', type=str, help='Name of the configuration to run', required=True, choices=configs.keys())

    args = parser.parse_args()


    for config in configs[args.config]:
        run_config(config)
