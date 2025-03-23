import numpy as np

def load_csv(method, file_name):
    # c = np.array()
    for i in range(5):
        a = np.loadtxt(f"../data/{method}/obj{i+1}_{file_name}", delimiter=",")
        if i == 0:
            b = a
        else:
            b = np.concatenate((b, a), axis=0)
    return b

rrt = load_csv("rrt", "rrt.csv")
print(f"rrt mean time {np.mean(rrt[:, 0]):.4f}")
print(f"rrt median time {rrt[:, 0][24]:.4f}")
print(f"rrt mean path length {np.mean(rrt[:, 1]):.4f}")
print(f"rrt median path length {rrt[:, 1][24]:.4f}")


ours = load_csv("ours", "rrt_exp.csv")
print(f"ours mean time {np.mean(ours[:, 0]):.4f}")
print(f"ours median time {ours[:, 0][24]:.4f}")
print(f"ours mean path length {np.mean(ours[:, 1]):.4f}")
print(f"ours median path length {ours[:, 1][24]:.4f}")
print(f"ours mean retrieving time {np.mean(ours[:, 4]):.4f}")
print(f"ours median retrieving time {ours[:, 4][24]:.4f}")