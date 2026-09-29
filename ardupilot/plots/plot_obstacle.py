import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sh = np.genfromtxt("obstacle_shared_log.csv", delimiter=",", names=True)
ma = np.genfromtxt("obstacle_manual_log.csv", delimiter=",", names=True)
OBS, R, D_SAFE = (15.0, 1.0), 2.0, 7.0

fig, ax = plt.subplots(1, 3, figsize=(17, 6))

# 1. top-down paths
ax[0].plot(ma["east"], ma["north"], "r-", lw=2, label="Human only")
ax[0].plot(sh["east"], sh["north"], "g-", lw=2, label="Shared control")
ax[0].add_patch(plt.Circle((OBS[1], OBS[0]), R, color="k", alpha=0.6, label="Obstacle"))
ax[0].add_patch(plt.Circle((OBS[1], OBS[0]), D_SAFE, fill=False, ls="--", color="gray", label="AI avoidance zone"))
ax[0].set_aspect("equal")
ax[0].set_xlabel("East (m)"); ax[0].set_ylabel("North (m)")
ax[0].set_title("Path around the obstacle (top-down)")
ax[0].legend(loc="lower left")

# 2. distance to obstacle over time
ax[1].plot(ma["t"], ma["dist_obs"], "r-", label="Human only")
ax[1].plot(sh["t"], sh["dist_obs"], "g-", label="Shared control")
ax[1].axhline(R, color="k", ls="--", label="Collision limit")
ax[1].set_xlabel("Time (s)"); ax[1].set_ylabel("Distance to obstacle centre (m)")
ax[1].set_title("Distance to obstacle")
ax[1].legend()

# 3. alpha in the shared run
ax[2].plot(sh["t"], sh["alpha"], "purple")
ax[2].set_ylim(-0.05, 1.05)
ax[2].set_xlabel("Time (s)"); ax[2].set_ylabel("alpha")
ax[2].set_title("Human authority (shared run)")

plt.tight_layout()
plt.savefig("obstacle_comparison.png", dpi=150)
print("Saved obstacle_comparison.png")
print(f"Human only:     min dist {ma['dist_obs'].min():.1f} m, time inside {(ma['dist_obs']<R).sum()*0.1:.1f} s")
print(f"Shared control: min dist {sh['dist_obs'].min():.1f} m, time inside {(sh['dist_obs']<R).sum()*0.1:.1f} s")
