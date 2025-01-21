configs = {
    'Infinite_Sans_Backup_Piscine_K_17': [
        {
            'name': f'Infinite_Sans_Backup_Piscine_K17',
            'lambda_mu1': [(16.6, 1.0)],
            'K': 17,
            'mu2': 20.0,
            'max_queue1': None,
            'max_queue2': None,
            'backup': False
        }
    ],

    'Infinite_Sans_Backup_Piscine_K_21': [
        {
            'name': f'Infinite_Sans_Backup_Piscine_K21',
            'lambda_mu1': [(16.6, 1.0)],
            'K': 21,
            'mu2': 20.0,
            'max_queue1': None,
            'max_queue2': None,
            'backup': False
        }
    ],

    'Infinite_Sans_Backup_Piscine_K_10to26': [
        {
            'name': f'Infinite_Sans_Backup_Piscine_K{k}',
            'lambda_mu1': [(16.6, 1.0)],
            'K': k,
            'mu2': 20.0,
            'max_queue1': None,
            'max_queue2': None,
            'backup': False
        } for k in range(10, 27)
    ],

    'Infinite_Sans_Backup_Ouverture_Piscine': [
        {
            'name': f'Infinite_Sans_Backup_Ouverture_Piscine',
            'lambda_mu1': [(100, 1.0)],
            'K': 125,
            'mu2': 20.0,
            'max_queue1': None,
            'max_queue2': None,
            'backup': False
        }
    ],

    'Finite_Sans_Backup_Piscine_Ks_5to30': [
        {
            'name': f'Finite_Sans_Backup_Piscine_Ks_{ks}',
            'lambda_mu1': [(16.6, 1.0)],
            'K': 21,
            'mu2': 20.0,
            'max_queue1': ks,
            'max_queue2': None,
            'backup': False
        } for ks in range(5, 31)
    ],

    'Finite_Sans_Backup_Piscine_Kf_5to40': [
        {
            'name': f'Finite_Sans_Backup_Piscine_Kf_{kf}',
            'lambda_mu1': [(16.6, 1.0)],
            'K': 21,
            'mu2': 20.0,
            'max_queue1': None,
            'max_queue2': kf,
            'backup': False
        } for kf in range(5, 40, 3)
    ],

    'Finite_Sans_Backup_Piscine': [
        {
            'name': f'Finite_Sans_Backup_Piscine',
            'lambda_mu1': [(16.6, 1.0)],
            'K': 21,
            'mu2': 20.0,
            'max_queue1': 20,
            'max_queue2': 30,
            'backup': False
        }
    ],

    'Finite_Sans_Backup_Prepa': [
        {
            'name': f'Finite_Sans_Backup_Prepa',
            'lambda_mu1': [(0.65, 0.2)],
            'K': 4,
            'mu2': 20.0,
            'max_queue1': 15,
            'max_queue2': 1,
            'backup': False
        } for k in range(1, 10)
    ],


    'Finite_Sans_Backup_Piscine_Prepa': [
        {
            'name': f'Finite_Sans_Backup_Piscine_Prepa_{k}',
            'lambda_mu1': [(100.0, 1.0), (0.65, 0.2)],
            'K': k,
            'mu2': 20.0,
            'max_queue1': 150,
            'max_queue2': 150,
            'backup': False
        } for k in range(60, 150, 5)
    ],
}
