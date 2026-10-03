import matplotlib

# IMPORTANT:
# Use a non-GUI backend.
# This avoids Qt/QSocketNotifier problems.
matplotlib.use("Agg")

import pandas as pd
import matplotlib.pyplot as plt
import os
import subprocess


# ============================================================
# FILE PATHS
# ============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

CSV_FILE = os.path.join(
    BASE_DIR,
    "experiment_results.csv"
)

OUTPUT_DIR = os.path.join(
    BASE_DIR,
    "graphs"
)

os.makedirs(OUTPUT_DIR, exist_ok=True)


# ============================================================
# START
# ============================================================

print()
print("========================================")
print("UUV GRAPH GENERATOR")
print("========================================")

print()
print("CSV file:")
print(CSV_FILE)

print()
print("Graph output directory:")
print(OUTPUT_DIR)


# ============================================================
# CHECK CSV
# ============================================================

if not os.path.exists(CSV_FILE):

    print()
    print("ERROR: experiment_results.csv was not found.")
    print()
    print("Expected:")
    print(CSV_FILE)

    exit(1)


# ============================================================
# READ CSV
# ============================================================

print()
print("Reading CSV...")

df = pd.read_csv(CSV_FILE)

print("CSV loaded successfully.")
print("Number of rows:", len(df))

print()
print("Columns found:")

for column in df.columns:
    print("  ", column)


# ============================================================
# REQUIRED COLUMNS
# ============================================================

required_columns = [
    "Time_s",

    "UUV1_X_m",
    "UUV1_Y_m",
    "UUV1_Z_m",

    "UUV2_X_m",
    "UUV2_Y_m",
    "UUV2_Z_m",

    "UUV3_X_m",
    "UUV3_Y_m",
    "UUV3_Z_m",

    "UUV1Speed_mps",
    "UUV2Speed_mps",
    "UUV3Speed_mps",

    "Distance_UUV1_UUV2_m",
    "Distance_UUV1_UUV3_m",

    "WaveAcceleration_mps2",
    "WaveForce_N"
]


missing_columns = []

for column in required_columns:

    if column not in df.columns:
        missing_columns.append(column)


if len(missing_columns) > 0:

    print()
    print("========================================")
    print("ERROR: MISSING COLUMNS")
    print("========================================")

    for column in missing_columns:
        print("Missing:", column)

    print()
    print("The MATLAB CSV does not contain all")
    print("the columns required by this script.")

    exit(1)


print()
print("All required columns found.")


# ============================================================
# REMOVE INVALID DATA
# ============================================================

df = df.dropna()

print("Valid rows:", len(df))


# ============================================================
# GRAPH 1
# UUV SPEEDS
# ============================================================

print()
print("Generating Graph 1: UUV speeds...")

fig = plt.figure(figsize=(10, 6))

plt.plot(
    df["Time_s"],
    df["UUV1Speed_mps"],
    linewidth=1.5,
    label="UUV 1 - Master"
)

plt.plot(
    df["Time_s"],
    df["UUV2Speed_mps"],
    linewidth=1.5,
    label="UUV 2 - Slave"
)

plt.plot(
    df["Time_s"],
    df["UUV3Speed_mps"],
    linewidth=1.5,
    label="UUV 3 - Slave"
)

plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("Speed of Three UUVs")

plt.legend()
plt.grid(True)

plt.tight_layout()


speed_file = os.path.join(
    OUTPUT_DIR,
    "01_uuv_speeds.png"
)

fig.savefig(
    speed_file,
    dpi=300,
    bbox_inches="tight"
)

plt.close(fig)

print("Saved:")
print(speed_file)


# ============================================================
# GRAPH 2
# FORMATION DISTANCES
# ============================================================

print()
print("Generating Graph 2: Formation distances...")

fig = plt.figure(figsize=(10, 6))

plt.plot(
    df["Time_s"],
    df["Distance_UUV1_UUV2_m"],
    linewidth=1.5,
    label="UUV 1 to UUV 2"
)

plt.plot(
    df["Time_s"],
    df["Distance_UUV1_UUV3_m"],
    linewidth=1.5,
    label="UUV 1 to UUV 3"
)

# Desired formation distance
plt.axhline(
    y=10.0,
    linestyle="--",
    linewidth=1.5,
    label="Desired Distance = 10 m"
)

plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")
plt.title("Distance Between Master and Slave UUVs")

plt.legend()
plt.grid(True)

plt.tight_layout()


distance_file = os.path.join(
    OUTPUT_DIR,
    "02_formation_distances.png"
)

fig.savefig(
    distance_file,
    dpi=300,
    bbox_inches="tight"
)

plt.close(fig)

print("Saved:")
print(distance_file)


# ============================================================
# GRAPH 3
# UUV TRAJECTORIES
# ============================================================

print()
print("Generating Graph 3: UUV trajectories...")

fig = plt.figure(figsize=(10, 7))

# NUE coordinate system:
# Position 1 = North
# Position 3 = East

plt.plot(
    df["UUV1_Z_m"],
    df["UUV1_X_m"],
    linewidth=1.5,
    label="UUV 1 - Master"
)

plt.plot(
    df["UUV2_Z_m"],
    df["UUV2_X_m"],
    linewidth=1.5,
    label="UUV 2 - Slave"
)

plt.plot(
    df["UUV3_Z_m"],
    df["UUV3_X_m"],
    linewidth=1.5,
    label="UUV 3 - Slave"
)


# Starting positions

plt.scatter(
    df["UUV1_Z_m"].iloc[0],
    df["UUV1_X_m"].iloc[0],
    marker="o",
    s=70,
    label="Starting position"
)


plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.title("UUV Formation Trajectories")

plt.legend()
plt.grid(True)

plt.axis("equal")

plt.tight_layout()


trajectory_file = os.path.join(
    OUTPUT_DIR,
    "03_uuv_trajectories.png"
)

fig.savefig(
    trajectory_file,
    dpi=300,
    bbox_inches="tight"
)

plt.close(fig)

print("Saved:")
print(trajectory_file)


# ============================================================
# GRAPH 4
# DISTURBANCE FORCE
# ============================================================

print()
print("Generating Graph 4: Wave disturbance...")

fig = plt.figure(figsize=(10, 6))

plt.plot(
    df["Time_s"],
    df["WaveForce_N"],
    linewidth=1.5,
    label="Applied Disturbance Force"
)

plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Applied Wave Disturbance")

plt.legend()
plt.grid(True)

plt.tight_layout()


wave_file = os.path.join(
    OUTPUT_DIR,
    "04_wave_disturbance.png"
)

fig.savefig(
    wave_file,
    dpi=300,
    bbox_inches="tight"
)

plt.close(fig)

print("Saved:")
print(wave_file)


# ============================================================
# FINISHED
# ============================================================

print()
print("========================================")
print("GRAPH GENERATION COMPLETE")
print("========================================")

print()
print("Generated files:")

print()
print("1. UUV Speeds")
print(speed_file)

print()
print("2. Formation Distances")
print(distance_file)

print()
print("3. UUV Trajectories")
print(trajectory_file)

print()
print("4. Wave Disturbance")
print(wave_file)

print()
print("Opening graphs...")


# ============================================================
# OPEN GRAPH FILES
# ============================================================

graph_files = [
    speed_file,
    distance_file,
    trajectory_file,
    wave_file
]

for graph_file in graph_files:

    try:

        subprocess.Popen(
            ["xdg-open", graph_file],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    except Exception as e:

        print("Could not automatically open:", graph_file)
        print(e)


print()
print("Done.")
