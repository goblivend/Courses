import random
import statistics
import heapq
import csv

class Event:
    def __init__(self, event_type, time, activity, arrival_time=None, arrival_time2=None):
        """
        Initialise un événement dans la simulation.

        Paramètres:
        ----------
        event_type : str
            Type de l'événement ("arrival", "departure1", "departure2", "reinsertion").
        time : float
            Temps auquel l'événement se produit.
        arrival_time : float, optional
            Temps d'arrivée du tag dans la 1re file (pour "departure1").
        arrival_time2 : float, optional
            Temps d'arrivée du tag dans la 2e file (pour "departure2").
        """
        self.event_type = event_type
        self.time = time
        self.activity = activity
        self.arrival_time = arrival_time
        self.arrival_time2 = arrival_time2

    def __lt__(self, other):
        return self.time < other.time

def exponential_rate(mu):
    """
    Retourne un temps aléatoire tiré d'une loi exponentielle de paramètre mu.

    Paramètres:
    ----------
    mu : float
        Paramètre de la loi exponentielle.

    Retourne:
    --------
    float
        Temps aléatoire.
    """
    return random.expovariate(mu)

def simulate_waterfall(lambda_mu1, K, mu2, sim_time, max_queue1=None, max_queue2=None, backup=False, max_backup_attempts=3):
    """
    Simule le système Waterfall avec gestion des files finies et back-up.

    Paramètres:
    ----------
    lambda_rate : float
        Taux d'arrivée (processus de Poisson).
    mu1 : float
        Taux de service par serveur dans la 1re file.
    K : int
        Nombre de serveurs dans la 1re file.
    mu2 : float
        Taux de service de la 2e file.
    sim_time : float
        Durée de la simulation.
    max_queue1 : int or None
        Capacité maximale de la 1re file (None => infinie).
    max_queue2 : int or None
        Capacité maximale de la 2e file (None => infinie).
    backup : bool
        Si True, active le back-up pour la 2e file.
    max_backup_attempts : int
        Nombre maximal de tentatives de back-up par résultat.

    Retourne:
    --------
    dict
        Statistiques de la simulation.
    """
    nb_activities = len(lambda_mu1)
    # Statistiques
    sojourn_times = [[] for _ in range(nb_activities)]
    waiting1_times = [[] for _ in range(nb_activities)]
    service1_times = [[] for _ in range(nb_activities)]
    waiting2_times = [[] for _ in range(nb_activities)]
    service2_times = [[] for _ in range(nb_activities)]
    arrivals_rejected = 0
    results_lost = 0
    total_arrivals = 0
    total_results_sent = 0
    busy_servers = 0
    queue1 = []
    queue2 = []
    current_time = 0.0
    load1 = [(0, 0)] # (time, nb_active_servers)
    load2 = [(0, 0)] # (time, nb_active_servers)

    # Priority queue pour les événements
    events = []

    # Prochaine arrivée

    next_arrival_times= [exponential_rate(lambda_rate) for lambda_rate, _ in lambda_mu1]

    for i in range(len(lambda_mu1)) :
        total_arrivals += 1
        heapq.heappush(events, (next_arrival_times[i], Event("arrival", next_arrival_times[i], i)))

    # Dictionnaire pour suivre les tentatives de back-up
    backup_attempts = {}

    while events :
        event_time, event = heapq.heappop(events)
        current_time = event_time

        if event.event_type == "arrival":
            # Gestion de l'arrivée
            if max_queue1 is not None and (busy_servers + len(queue1)) >= (K + max_queue1):
                # La file 1 est pleine => rejet
                arrivals_rejected += 1
            else:
                if busy_servers < K:
                    # Démarrer service immédiatement
                    if total_arrivals < sim_time :
                        busy_servers += 1
                        load1.append((current_time, busy_servers))
                        service_time = exponential_rate(lambda_mu1[event.activity][1])
                        departure_time = current_time + service_time
                        waiting1_times[event.activity].append(0)
                        service1_times[event.activity].append(service_time)
                        heapq.heappush(events, (departure_time, Event("departure1", departure_time, event.activity, arrival_time=current_time)))
                else:
                    # Ajouter à la queue1
                    queue1.append((current_time, event.activity))

            # Programmer la prochaine arrivée
            next_arrival_time = current_time + exponential_rate(lambda_mu1[event.activity][0])
            if total_arrivals < sim_time:
                total_arrivals += 1
                heapq.heappush(events, (next_arrival_time, Event("arrival", next_arrival_time, event.activity)))

        elif event.event_type == "departure1":
            # Fin de service en 1re file
            busy_servers -= 1
            load1.append((current_time, busy_servers))
            arrival_time = event.arrival_time

            # Envoi vers la 2e file
            if max_queue2 is not None and len(queue2) >= max_queue2:
                if backup:
                    # Tentative de sauvegarde en back-up
                    attempts = backup_attempts.get(arrival_time, 0)
                    if attempts < max_backup_attempts:
                        backup_attempts[arrival_time] = attempts + 1
                        re_insertion_time = current_time + exponential_rate(0.1)  # Taux de réessai arbitraire
                        heapq.heappush(events, (re_insertion_time, Event("reinsertion", re_insertion_time, event.activity, arrival_time=arrival_time)))
                    else:
                        # Trop de tentatives => perdre le résultat
                        results_lost += 1
                else:
                    # Résultat perdu
                    results_lost += 1
            else:
                # Ajouter à la queue2
                queue2.append((arrival_time, current_time, event.activity))
                total_results_sent += 1
                # Si le serveur 2 est libre, démarrer le service
                if len(queue2) == 1:
                    load2.append((current_time, 1))
                    service_time = exponential_rate(mu2)
                    departure_time2 = current_time + service_time
                    service2_times[event.activity].append(service_time)
                    waiting2_times[event.activity].append(0)
                    heapq.heappush(events, (departure_time2, Event("departure2", departure_time2, event.activity, arrival_time2=arrival_time)))

            # Gestion de la queue1
            if queue1 :
                next_in_queue, activity = queue1.pop(0)
                busy_servers += 1
                load1.append((current_time, busy_servers))
                service_time = exponential_rate(lambda_mu1[event.activity][1])
                departure_time = current_time + service_time
                heapq.heappush(events, (departure_time, Event("departure1", departure_time, activity, arrival_time=next_in_queue)))
                waiting1_times[event.activity].append(departure_time - next_in_queue)
                service1_times[event.activity].append(service_time)

        elif event.event_type == "reinsertion":
            # Réessayer d'insérer le job dans la 2e file
            arrival_time = event.arrival_time
            attempts = backup_attempts.get(arrival_time, 0)
            if attempts < max_backup_attempts:
                backup_attempts[arrival_time] = attempts + 1
                if max_queue2 is not None and len(queue2) >= max_queue2:
                    # Encore pleine => perdre le résultat
                    results_lost += 1
                else:
                    queue2.append((arrival_time, current_time, event.activity))
                    total_results_sent += 1
                    if len(queue2) == 1:
                        departure_time2 = current_time + exponential_rate(mu2)
                        heapq.heappush(events, (departure_time2, Event("departure2", departure_time2, event.activity, arrival_time2=arrival_time)))
            else:
                # Trop de tentatives => perdre le résultat
                results_lost += 1

        elif event.event_type == "departure2":
            # Fin de service en 2e file
            arrival_time2 = event.arrival_time2
            sojourn_time = current_time - arrival_time2
            sojourn_times[event.activity].append(sojourn_time)
            keeps_running = False

            # Enlever le job de la queue2
            if queue2:
                queue2.pop(0)
                keeps_running = True

            # Démarrer le service pour le prochain dans la queue2
            if queue2:
                if not keeps_running :
                    load2.append((current_time, 1))
                service_time = exponential_rate(mu2)
                next_departure_time2 = current_time + service_time
                service2_times[event.activity].append(service_time)
                waiting2_times[event.activity].append(current_time - queue2[0][1])
                next_arrival_time2 = queue2[0][0]
                heapq.heappush(events, (next_departure_time2, Event("departure2", next_departure_time2, queue2[0][2], arrival_time2=next_arrival_time2)))
            elif keeps_running :
                load2.append((current_time, 0))


    # Calcul des statistiques
    avg_sojourns = [statistics.mean(times) for times in sojourn_times]
    var_sojourns = [statistics.pvariance(times) for times in sojourn_times]

    # Calcul des taux
    alpha_s = (arrivals_rejected / total_arrivals) * 100 if total_arrivals > 0 else 0.0
    alpha_f = (results_lost / total_results_sent) * 100 if total_results_sent > 0 else 0.0

    # Calcul des taux d'activité des serveurs
    avg_load1 = sum(load1[i][1] * (load1[i+1][0] - load1[i][0]) for i in range(len(load1)-1)) / load1[-1][0] / K
    avg_load2 = sum(load2[i][1] * (load2[i+1][0] - load2[i][0]) for i in range(len(load2)-1)) / load2[-1][0]

    return {
        'arrivals_rejected': arrivals_rejected,
        'results_lost': results_lost,
        'total_arrivals': total_arrivals,
        'total_results_sent': total_results_sent,
        'sojourn_times': sojourn_times,
        'service1_times': service1_times,
        'waiting1_times': waiting1_times,
        'service2_times': service2_times,
        'waiting2_times': waiting2_times,
        'avg_load1' : avg_load1,
        'avg_load2' : avg_load2,
        'avg_sojourns': avg_sojourns,
        'var_sojourns': var_sojourns,
        'count_completed': len(sojourn_times),
        'alpha_s': alpha_s,
        'alpha_f': alpha_f
    }

def run_simulations(configurations, sim_time=10000, repetitions=30):
    """
    Exécute les simulations pour différentes configurations et répète chaque simulation plusieurs fois pour obtenir des statistiques fiables.

    Paramètres:
    ----------
    configurations : list of dict
        Liste des configurations à simuler.
    sim_time : float
        Durée de chaque simulation.
    repetitions : int
        Nombre de répétitions par configuration.

    Retourne:
    --------
    list of dict
        Liste des résultats moyens par configuration.
    """
    results = []
    total_configs = len(configurations)
    for idx, config in enumerate(configurations, 1):
        print(f"Configuration {idx}/{total_configs}: {config['name']}")
        nb_activities = len(config['lambda_mu1'])
        total_rejected = 0
        total_lost = 0
        total_arrivals = 0
        total_results_sent = 0
        sojourn_times_all = [[] for _ in range(nb_activities)]

        waiting_times1_all =  [[] for _ in range(nb_activities)]
        service_times1_all =  [[] for _ in range(nb_activities)]

        waiting_times2_all = [[] for _ in range(nb_activities)]
        service_times2_all = [[] for _ in range(nb_activities)]

        loads1 = []
        loads2 = []

        for rep in range(1, repetitions + 1):
            print(f"  Répétition {rep}/{repetitions}")
            sim_result = simulate_waterfall(
                lambda_mu1=config['lambda_mu1'],
                K=config['K'],
                mu2=config['mu2'],
                sim_time=sim_time,
                max_queue1=config.get('max_queue1'),
                max_queue2=config.get('max_queue2'),
                backup=config.get('backup', False)
            )
            total_rejected += sim_result['arrivals_rejected']
            total_lost += sim_result['results_lost']
            total_arrivals += sim_result['total_arrivals']
            total_results_sent += sim_result['total_results_sent']
            for i in range(nb_activities) :
                sojourn_times_all[i].extend(sim_result['sojourn_times'][i])
                waiting_times1_all[i].extend(sim_result['waiting1_times'][i])
                service_times1_all[i].extend(sim_result['service1_times'][i])
                waiting_times2_all[i].extend(sim_result['waiting2_times'][i])
                service_times2_all[i].extend(sim_result['service2_times'][i])
            loads1.append(sim_result['avg_load1'])
            loads2.append(sim_result['avg_load2'])

        # Calcul des moyennes
        alpha_s = (total_rejected / total_arrivals) * 100 if total_arrivals > 0 else 0.0
        alpha_f = (total_lost / total_results_sent) * 100 if total_results_sent > 0 else 0.0

        avgs_sojourn_mean = [statistics.mean(times) for times in sojourn_times_all]
        avgs_sojourn_var = [statistics.pvariance(times) for times in sojourn_times_all]

        avgs_service1_times = [statistics.mean(times) for times in service_times1_all]
        avgs_waiting1_times = [statistics.mean(times) for times in waiting_times1_all]

        avgs_service2_times = [statistics.mean(times) for times in service_times2_all]
        avgs_waiting2_times = [statistics.mean(times) for times in waiting_times2_all]

        avg_load1 = statistics.mean(loads1) if loads1 else 0.0
        avg_load2 = statistics.mean(loads2) if loads2 else 0.0

        results.append({
            'configuration': config['name'],
            'lambda_mu1': config['lambda_mu1'],
            'K': config['K'],
            'mu2': config['mu2'],
            'max_k_s': config.get('max_queue1', 'None'),
            'max_k_f': config.get('max_queue2', 'None'),
            'backup': 'Oui' if config.get('backup', False) else 'Non',
            'alpha_s': f"{alpha_s:.1f}%",
            'alpha_f': f"{alpha_f:.1f}%",
            'T_avg': [round(avg_sojourn_mean, 2) for avg_sojourn_mean in avgs_sojourn_mean],
            'T_var': [round(avg_sojourn_var, 2) for avg_sojourn_var in avgs_sojourn_var],

            "T_wait1_avg" : [round(avg_waiting1_times, 2) for avg_waiting1_times in avgs_waiting1_times],
            "T_service1_avg" : [round(avg_waiting1_times, 2) for avg_waiting1_times in avgs_service1_times],
            "T_wait2_avg" : [round(avg_waiting2_times, 2) for avg_waiting2_times in avgs_waiting2_times],
            "T_service2_avg" : [round(avg_service2_times, 2) for avg_service2_times in avgs_service2_times],
            "load1_avg" : f"{avg_load1:.2f}",
            "load2_avg" : f"{avg_load2:.2f}",

        })

    return results

if __name__ == "__main__":
    from configurations import configs
    import argparse

    parser = argparse.ArgumentParser(description='Simulate Queueing system')
    parser.add_argument('--sim_time', '-s', type=int, default=5000, help='Simulation time')
    parser.add_argument('--repetitions', '-r', type=int, default=10, help='Number of repetitions')
    parser.add_argument('--config', '-c', type=str, help='Configuration list', choices=configs.keys(), required=True)
    args = parser.parse_args()

    # Exécuter les simulations avec des paramètres réduits pour tester
    simulation_results = run_simulations(configs[args.config], sim_time=args.sim_time, repetitions=args.repetitions)  # Réduire sim_time et repetitions pour tester

    # Sauvegarder les résultats dans un fichier CSV
    if simulation_results:
        keys = simulation_results[0].keys()
        with open('simulation_results.csv', 'w', newline='') as output_file:
            dict_writer = csv.DictWriter(output_file, keys)
            dict_writer.writeheader()
            dict_writer.writerows(simulation_results)

    # Afficher les résultats
    for res in simulation_results:
        print(res)
