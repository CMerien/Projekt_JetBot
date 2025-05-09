#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, unicode_literals
import rosbag
import pandas as pd
import matplotlib.pyplot as plt
from sys import argv

STYLY = {
    "P": {'color': 'blue', 'linestyle': '-'},
    "PI": {'color': 'orange', 'linestyle': '--'},
    "PID": {'color': 'green', 'linestyle': '-.'}
}

def spracuj_subor(cesta):
    bag = rosbag.Bag(cesta)
    data = []
    
    for _, msg, t in bag.read_messages(topics=['/auto_drive_log']):
        data.append({
            'cas': t.to_nsec()/1e9,
            'pozadovana': msg.data[1],
            'skutocna': msg.data[2],
            'rychlost': msg.data[3],
            'rychlost_odo': msg.data[4],  # Added odometry speed
            'kod': msg.data[5]
        })
    
    bag.close()
    return pd.DataFrame(data)

def priprav_data(df):
    df['cas_rel'] = df['cas'] - df['cas'].min()
    start = df[df['pozadovana'] != 0]['cas_rel'].min()
    if pd.isnull(start):
        start = df['cas_rel'].min()
    df['cas_norm'] = df['cas_rel'] - start
    return df[(df['cas_norm'] >= 0) & (df['cas_norm'] <= 12)]

def zisti_regulator(df):
    kod = df['kod'].mode()[0]
    return {1: 'PID', 2: 'PI', 3: 'P'}.get(kod, 'Neznámy')

def vytvor_graf(data, nazov, ylabel, funkcia, pridaj_pozadovanu=False):
    plt.figure(figsize=(10,6))
    
    for df, reg in data:
        x = df['cas_norm']
        y = funkcia(df)
        plt.plot(x, y, 
                color=STYLY[reg]['color'], 
                linestyle=STYLY[reg]['linestyle'],
                label=reg)
    
    if pridaj_pozadovanu and data:
        df = data[0][0]
        plt.plot(df['cas_norm'], df['pozadovana'], 
                color='red', linewidth=1.5, linestyle='--',
                label='Požadovaná')
    
    plt.title(nazov)
    plt.xlabel(u"Čas (s)")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

def vytvor_individ_graf(df, regulator, nazov_suboru):
    plt.figure(figsize=(10, 6))
    plt.plot(df['cas_norm'], df['rychlost'], 
             color=STYLY[regulator]['color'], 
             linestyle=STYLY[regulator]['linestyle'],
             label='Rýchlosť (riadenie)')
    plt.plot(df['cas_norm'], df['rychlost_odo'],
             color=STYLY[regulator]['color'],
             linestyle=':',
             linewidth=2,
             label='Rýchlosť (odometria)')
    
    plt.title(u"Porovnanie rýchlostí pre {} ({})".format(nazov_suboru, regulator))
    plt.xlabel(u"Čas (s)")
    plt.ylabel(u"Rýchlosť (m/s)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

if __name__ == "__main__":
    if len(argv) < 2:
        print("Použitie: rosrun balik skript.py subor1.bag subor2.bag ...")
        exit(1)

    vsetky_data = []
    for cesta in argv[1:]:
        try:
            df = spracuj_subor(cesta)
            df = priprav_data(df)
            reg = zisti_regulator(df)
            if not df.empty:
                vsetky_data.append((df, reg))
                print(u"Načítané: %s (%s)" % (cesta.split('/')[-1], reg))
                vytvor_individ_graf(df, reg, cesta.split('/')[-1])  # New plot for each file
        except Exception as e:
            print(u"Chyba v %s: %s" % (cesta, str(e)))

    if not vsetky_data:
        print(u"Žiadne použiteľné dáta!")
        exit()

    # Existing aggregated plots
    vytvor_graf(vsetky_data, 
               u"Porovnanie vzdialeností", 
               u"Vzdialenosť (m)", 
               lambda df: df['skutocna'],
               pridaj_pozadovanu=True)

    vytvor_graf(vsetky_data, 
               u"Porovnanie rýchlostí", 
               u"Rýchlosť (m/s)", 
               lambda df: df['rychlost'])

    vytvor_graf(vsetky_data, 
               u"Regulačné chyby", 
               u"Chyba (m)", 
               lambda df: df['pozadovana'] - df['skutocna'])

    # IAE plot
    plt.figure(figsize=(10,6))
    for df, reg in vsetky_data:
        chyba = (df['pozadovana'] - df['skutocna']).abs()
        iae = chyba.cumsum()
        plt.plot(df['cas_norm'], iae, 
                color=STYLY[reg]['color'], 
                linestyle=STYLY[reg]['linestyle'],
                label=u"%s (%.1f)" % (reg, iae.max()))
    
    plt.title(u"Integrovaná absolútna chyba (IAE)")
    plt.xlabel(u"Čas (s)")
    plt.ylabel(u"IAE (m·s)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()
