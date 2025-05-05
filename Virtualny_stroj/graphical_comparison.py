#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import rosbag
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse

REGULATOR_STYLES = {
    "P":   {'color': '#1f77b4', 'linestyle': '-'},
    "PI":  {'color': '#ff7f0e', 'linestyle': '--'},
    "PID": {'color': '#2ca02c', 'linestyle': '-.'}
}

def read_bag_data(bag_path):
    bag = rosbag.Bag(bag_path)

    auto_drive_log = []

    for _, msg, t in bag.read_messages(topics=['/auto_drive_log']):
        entry = {
            '%time': t.to_nsec(),
            'desired_dist': msg.data[1],
            'actual_dist': msg.data[2],
            'speed': msg.data[3],
            'odom_speed': msg.data[4],
            'regulator_code': msg.data[5]
        }
        auto_drive_log.append(entry)

    bag.close()
    return pd.DataFrame(auto_drive_log)

def process_bag_file(bag_path):
    df = read_bag_data(bag_path)
    df['timestamp'] = df['%time'] / 1e9
    df['time_rel'] = df['timestamp'] - df['timestamp'].min()

    start_time = df[df['desired_dist'] != 0]['time_rel'].min() if any(df['desired_dist']) else df['time_rel'].min()
    df['time_norm'] = df['time_rel'] - start_time
    df = df[(df['time_norm'] >= 0) & (df['time_norm'] <= 15)]

    regulator_map = {1.0: "PID", 2.0: "PI", 3.0: "P"}
    regulator = regulator_map.get(df['regulator_code'].mode()[0], "Neznámy")

    return df, regulator

def plot_comparison(all_data):
    plt.close('all')

    # 1. Hlavný kombinovaný graf
    plt.figure(1, figsize=(14, 10))
    plt.rcParams['font.family'] = 'DejaVu Sans'

    # Rýchlosť
    plt.subplot(2, 2, 1)
    for df, regulator in all_data:
        style = REGULATOR_STYLES[regulator]
        plt.plot(df['time_norm'], df['speed'], label=regulator, **style)
    plt.title("Rýchlosť JetBota")
    plt.xlabel("Čas (s)")
    plt.ylabel("Rýchlosť (m/s)")
    plt.grid(True)
    plt.legend()

    # Chyba regulácie
    plt.subplot(2, 2, 2)
    for df, regulator in all_data:
        style = REGULATOR_STYLES[regulator]
        plt.plot(df['time_norm'], df['desired_dist'] - df['actual_dist'], label=regulator, **style)
    plt.title("Chyba regulácie")
    plt.xlabel("Čas (s)")
    plt.ylabel("Chyba (m)")
    plt.grid(True)
    plt.legend()

    # Vzdialenosť - skutočná vs požadovaná
    plt.subplot(2, 2, 3)
    
    # Najprv všetky regulátory
    for df, regulator in all_data:
        style = REGULATOR_STYLES[regulator]
        plt.plot(df['time_norm'], df['actual_dist'], label=regulator, **style)
    
    # Potom požadovaná vzdialenosť (iba raz)
    if all_data:
        df_first = all_data[0][0]
        plt.plot(df_first['time_norm'], df_first['desired_dist'], 
                linestyle=':', color='gray', alpha=0.7, 
                label='Požadovaná vzdialenosť')
    
    plt.title("Skutočná vs požadovaná vzdialenosť")
    plt.xlabel("Čas (s)")
    plt.ylabel("Vzdialenosť (m)")
    plt.grid(True)
    plt.legend()

    # IAE
    plt.subplot(2, 2, 4)
    for df, regulator in all_data:
        style = REGULATOR_STYLES[regulator]
        iae = np.abs(df['desired_dist'] - df['actual_dist']).cumsum()
        plt.plot(df['time_norm'], iae, label="%s (IAE=%.1f)" % (regulator, iae.max()), **style)
    plt.title("Integrovaná absolútna chyba (IAE)")
    plt.xlabel("Čas (s)")
    plt.ylabel("IAE (m·s)")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()

    # 2. Samostatné grafy
    metrics = [
        ('speed', "Rýchlosť (m/s)", "Rýchlosť"),
        (lambda d: d['desired_dist'] - d['actual_dist'], "Chyba (m)", "Chyba regulácie"),
        ('actual_dist', "Vzdialenosť (m)", "Skutočná vzdialenosť"),
        (lambda d: np.abs(d['desired_dist'] - d['actual_dist']).cumsum(), "IAE (m·s)", "Integrovaná absolútna chyba (IAE)"),
        ('actual_vs_desired', "Vzdialenosť (m)", "Skutočná vs požadovaná vzdialenosť")
    ]

    for idx, (col, ylabel, title) in enumerate(metrics):
        plt.figure(idx + 2, figsize=(8, 5))
        
        if col == 'actual_vs_desired':
            # Najprv regulátory
            for df, regulator in all_data:
                style = REGULATOR_STYLES[regulator]
                plt.plot(df['time_norm'], df['actual_dist'], label=regulator, **style)
            
            # Potom požadovaná vzdialenosť
            if all_data:
                df_first = all_data[0][0]
                plt.plot(df_first['time_norm'], df_first['desired_dist'], 
                        linestyle=':', color='gray', alpha=0.7, 
                        label='Požadovaná vzdialenosť')
        else:
            for df, regulator in all_data:
                style = REGULATOR_STYLES[regulator]
                data = col(df) if callable(col) else df[col]
                
                if 'IAE' in title:
                    iae_value = data.max() if callable(col) else data.values[-1]
                    label = "%s (IAE=%.1f)" % (regulator, iae_value)
                else:
                    label = regulator
                
                plt.plot(df['time_norm'], data, label=label, **style)
        
        plt.title(title)
        plt.xlabel("Čas (s)")
        plt.ylabel(ylabel)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    # 3. Porovnanie rýchlostí
    for df, regulator in all_data:
        plt.figure(figsize=(8, 5))
        plt.plot(df['time_norm'], df['speed'],
                label="Rýchlosť z regulátora", color='#d62728', linestyle='-')
        plt.plot(df['time_norm'], df['odom_speed'],
                label="Rýchlosť z odometrie", color='#2ca02c', linestyle='--')
        plt.title("Porovnanie rýchlostí (%s regulátor)" % regulator)
        plt.xlabel("Čas (s)")
        plt.ylabel("Rýchlosť (m/s)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Porovnanie regulátorov")
    parser.add_argument('bag_files', nargs='+', help="Cesty k .bag súborom")
    args = parser.parse_args()

    all_data = []
    for bag_path in args.bag_files:
        try:
            df, regulator = process_bag_file(bag_path)
            if not df.empty:
                all_data.append((df, regulator))
        except Exception as e:
            print("Chyba pri spracovaní %s: %s" % (bag_path, str(e)))

    if all_data:
        plot_comparison(all_data)
    else:
        print("Žiadne dáta na spracovanie!")
