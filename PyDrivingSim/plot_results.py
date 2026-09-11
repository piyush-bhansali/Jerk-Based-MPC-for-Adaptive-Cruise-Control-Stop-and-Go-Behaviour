"""Plot MPC tracking and actuation performance from a simulator run.

Reads the per-cycle CSV log written by pydrivingsim/agent.py (default:
agent_log.csv, produced next to wherever simulator.py was run) and renders:
  - tracking.png:   actual vs. reference longitudinal velocity
  - actuation.png:  requested/actual acceleration, pedal command, steering

Usage:
    python plot_results.py [log_csv]
"""
import csv
import sys

import matplotlib.pyplot as plt

DEFAULT_LOG_PATH = "agent_log.csv"


def load_log(path):
    rows = {"time": [], "v_actual": [], "v_ref": [], "a_actual": [],
            "a_requested": [], "pedal": []}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            for key in rows:
                rows[key].append(float(row[key]))
    return rows


def plot_tracking(log, out_path="tracking.png"):
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(log["time"], log["v_ref"], label="reference speed", linestyle="--", color="tab:gray")
    ax.plot(log["time"], log["v_actual"], label="actual speed", color="tab:blue")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("longitudinal velocity [m/s]")
    ax.set_title("MPC tracking: velocity vs. reference")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"wrote {out_path}")


def plot_actuation(log, out_path="actuation.png"):
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    axes[0].plot(log["time"], log["a_requested"], label="requested accel", color="tab:orange")
    axes[0].plot(log["time"], log["a_actual"], label="actual accel", color="tab:blue")
    axes[0].set_ylabel("accel [m/s²]")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(log["time"], log["pedal"], color="tab:green")
    axes[1].set_ylabel("pedal [-1..1]")
    axes[1].set_xlabel("time [s]")
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("MPC actuation commands")
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"wrote {out_path}")


def main():
    log_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    log = load_log(log_path)
    plot_tracking(log)
    plot_actuation(log)


if __name__ == "__main__":
    main()
