import os

import numpy as np
import scipy
import seaborn as sns
import matplotlib.pyplot as plt

import pandas as pd


def df_from_lists(seed, comms_aware, iterations):
    return pd.DataFrame({"seed": seed, "comms_aware": comms_aware, "iterations": iterations})


def get_lists():
    seed = []
    comms_aware = []
    iterations = []
    for file in ['../../../results_iris.txt', '../../../results_peter.txt', '../../../results_jenny.txt']:
        with open(file, "r") as f:
            for line in f:
                if len(line) > 0:
                    split = line.split(" ")
                    seed.append(int(split[1]))
                    comms_aware.append(split[3] == "True")
                    iterations.append(int(split[7]))

    return seed, comms_aware, iterations


def box_plot(df):
    sns.set_theme(style="whitegrid")
    return sns.boxplot(x="comms_aware", y="iterations", data=df)


def get_all_separated_tests(df):
    comms_aware = {x: [] for x in df['seed']}
    comms_unaware = {x: [] for x in df['seed']}
    allaware = []
    allunaware = []
    avgaware = []
    avgunaware = []
    for index, row in df.iterrows():
        seed = row["seed"]
        if row["comms_aware"]:
            comms_aware[seed].append(row["iterations"])
            allaware.append(row["iterations"])
        else:
            comms_unaware[seed].append(row["iterations"])
            allunaware.append(row["iterations"])

    for seed in set(df["seed"]):
        if comms_aware[seed] != [] and comms_unaware[seed] != []:
            avgaware.append(np.average(comms_aware[seed]))
            avgunaware.append(np.average(comms_unaware[seed]))

    return allaware, allunaware, avgaware, avgunaware


def paired_box_plot(avgaware, avgunaware):
    pairs = np.asarray(avgaware) - np.asarray(avgunaware)
    return sns.boxplot(y=pairs)


def t_test(aware, unaware):
    return scipy.stats.ttest_ind(aware, unaware, alternative='less')


def paired_test(aware, unaware):
    return scipy.stats.ttest_rel(aware, unaware, alternative='less')


if __name__ == '__main__':
    df = df_from_lists(*get_lists())
    aware, unaware, avgaware, avgunaware = get_all_separated_tests(df)
    print(len(aware), "comms aware samples, avg of ", np.average(aware))
    print(len(unaware), "comms unaware samples, avg of ", np.average(unaware))
    print(len(avgaware), "unique seeds")
    print(t_test(aware, unaware))
    print(paired_test(avgaware, avgunaware))
    box_plot(df)
    # paired_box_plot(avgaware,avgunaware)
    plt.show()
