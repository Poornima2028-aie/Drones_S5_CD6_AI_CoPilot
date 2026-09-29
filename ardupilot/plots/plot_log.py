import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

fn = sys.argv[1] if len(sys.argv) > 1 else "keyboard_log.csv"
d = np.genfromtxt(fn, delimiter=",", names=True)
t = d["t"]
target = np.array([30.0, 0.0])

fig, ax = plt.subplots(2, 2, figsize=(13, 9))

# 1. position over time
ax[0, 0].plot(t, d["north"], label="North")
ax[0, 0].plot(t, d["east"], label="East")
ax[0, 0].axhline(30, color="gray", ls="--", label="Target (north)")
ax[0, 0].set_title("Drone position")
ax[0, 0].set_xlabel("Time (s)"); ax[0, 0].set_ylabel("Metres")
ax[0, 0].legend()

# 2. commands (north component)
ax[0, 1].plot(t, d["uh_n"], label="Human", alpha=0.7)
ax[0, 1].plot(t, d["ua_n"], label="AI", alpha=0.7)
ax[0, 1].plot(t, d["us_n"], label="Shared (sent to drone)", lw=2)
ax[0, 1].set_title("Commands, north direction")
ax[0, 1].set_xlabel("Time (s)"); ax[0, 1].set_ylabel("Speed (m/s)")
ax[0, 1].legend()

# 3. alpha
ax[1, 0].plot(t, d["alpha"], color="purple")
ax[1, 0].set_ylim(-0.05, 1.05)
ax[1, 0].set_title("Human authority (alpha)")
ax[1, 0].set_xlabel("Time (s)"); ax[1, 0].set_ylabel("alpha (0 = AI only, 1 = human free)")

# 4. top-down path
sc = ax[1, 1].scatter(d["east"], d["north"], c=d["alpha"], cmap="viridis", s=8)
ax[1, 1].plot(target[1], target[0], "r*", ms=15, label="Target")
ax[1, 1].set_title("Top-down path (colour = alpha)")
ax[1, 1].set_xlabel("East (m)"); ax[1, 1].set_ylabel("North (m)")
ax[1, 1].legend()
fig.colorbar(sc, ax=ax[1, 1], label="alpha")

plt.tight_layout()
out = fn.replace(".csv", "_plots.png")
plt.savefig(out, dpi=150)
print("Saved", out)

# numbers for your slide
dist = np.hypot(d["north"] - target[0], d["east"] - target[1])
human_active = np.hypot(d["uh_n"], d["uh_e"]) > 0.1
print("\n--- Summary ---")
print(f"Duration:                   {t[-1]:.0f} s")
print(f"Mean distance to target:    {dist.mean():.2f} m")
print(f"Max distance to target:     {dist.max():.2f} m")
print(f"Mean alpha (all time):      {d['alpha'].mean():.2f}")
if human_active.any():
    print(f"Mean alpha (human pressing):{d['alpha'][human_active].mean():.2f}")
    print(f"Time human was pressing:    {human_active.mean()*100:.0f} %")
    print(f"AI overrode human (alpha<0.2): {(d['alpha'][human_active] < 0.2).mean()*100:.0f} % of that time")
