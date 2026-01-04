#Easily toggleable avoidance and clamp, closed loop

import numpy as np

# ===============================================================
#  MEC1315 - Travail Synthèse 1 - Automne 2025
#  Générateur de trajectoire pour robot 3R planaire
#  Version avec options séparées pour évitement et clamp
# ===============================================================

# ---------- CONFIGURATION GÉNÉRALE -----------------------------
AVOIDANCE = True   # ⬅️  Active la stratégie d’évitement des limites
CLAMP     = True   # ⬅️  Active la saturation des angles articulaires
STEP = 0.005        # Pas de discrétisation cartésienne

# ---------- 1. Lecture du fichier Robot.par --------------------
def read_robot_par(filename):
    params = []
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            params.append(float(line.split()[0]))

    (
        L1, L2, L3,
        xmin, xmax, ymin, ymax,
        x0, y0,
        t1_dep, t2_dep, t3_dep,
        t1_min, t1_max, t2_min, t2_max, t3_min, t3_max,
        xmur0, xmur1, ymur0, ymur1,
        dt
    ) = params

    robot = {
        "L1": L1, "L2": L2, "L3": L3,
        "x0": x0, "y0": y0,
        "theta_dep": np.array([t1_dep, t2_dep, t3_dep]),
        "theta_min": np.array([t1_min, t2_min, t3_min]),
        "theta_max": np.array([t1_max, t2_max, t3_max]),
        "dt": dt
    }
    return robot

# ---------- 2. Lecture du fichier Trajet.xy --------------------
def read_xy(filename):
    return np.loadtxt(filename)

# ---------- 3. Cinématique directe -----------------------------
def forward_kinematics(theta, robot):
    t1, t2, t3 = theta
    L1, L2, L3 = robot["L1"], robot["L2"], robot["L3"]
    x = L1*np.cos(t1) + L2*np.cos(t1+t2) + L3*np.cos(t1+t2+t3)
    y = L1*np.sin(t1) + L2*np.sin(t1+t2) + L3*np.sin(t1+t2+t3)
    return np.array([x, y])

# ---------- 4. Matrice Jacobienne ------------------------------
def jacobian(theta, robot):
    t1, t2, t3 = theta
    L1, L2, L3 = robot["L1"], robot["L2"], robot["L3"]
    J = np.zeros((2, 3))
    J[0,0] = -L1*np.sin(t1) - L2*np.sin(t1+t2) - L3*np.sin(t1+t2+t3)
    J[1,0] =  L1*np.cos(t1) + L2*np.cos(t1+t2) + L3*np.cos(t1+t2+t3)
    J[0,1] = -L2*np.sin(t1+t2) - L3*np.sin(t1+t2+t3)
    J[1,1] =  L2*np.cos(t1+t2) + L3*np.cos(t1+t2+t3)
    J[0,2] = -L3*np.sin(t1+t2+t3)
    J[1,2] =  L3*np.cos(t1+t2+t3)
    return J

# ---------- 5. Pseudo-inverse gauche ----------------------------
def pseudo_inverse(J):
    return J.T @ np.linalg.inv(J @ J.T)

# ---------- 6. Fonction Reach (avec options indépendantes) -----
def reach(p_target, theta, robot, avoidance=True, clamp=True):
    d = 0.75       # amortissement
    epsilon = 1e-5
    max_iter = 25
    alpha = 0.2    # poids de la composante d’évitement

    theta_min = robot["theta_min"]
    theta_max = robot["theta_max"]
    theta_mid = 0.5 * (theta_max + theta_min)
    W = np.diag(1.0 / (theta_max - theta_min))

    for _ in range(max_iter):
        p = forward_kinematics(theta, robot)
        dp = p_target - p
        if np.linalg.norm(dp) < epsilon:
            break

        J = jacobian(theta, robot)
        Jp = pseudo_inverse(J)

        # -- Solution de norme minimale
        dtheta = Jp @ (d * dp)

        if avoidance:
            # -- Ajout de la composante d’évitement des limites
            h = W @ (theta_mid - theta)
            I = np.eye(3)
            dtheta_H = (I - Jp @ J) @ h
            dtheta = dtheta + alpha * dtheta_H

        theta = theta + d * dtheta

        if clamp:
            theta = np.minimum(np.maximum(theta, theta_min), theta_max)

    return theta

# ---------- 7. Génération de trajectoire articulaire ------------
def generate_joint_trajectory(robot, xy_points, step=0.01, avoidance=True, clamp=True):
    theta_list = [robot["theta_dep"].copy()]
    theta = robot["theta_dep"].copy()

    # Step 1: initial pose → first XY point
    p_init = forward_kinematics(theta, robot)
    p_first = xy_points[0]
    if np.linalg.norm(p_first - p_init) > 1e-6:
        segment = np.linspace(0, 1, int(np.linalg.norm(p_first - p_init)/step))
        for s in segment[1:]:
            p_target = (1-s)*p_init + s*p_first
            theta = reach(p_target, theta, robot, avoidance, clamp)
            theta_list.append(theta.copy())

    # Step 2: main trajectory
    for i in range(len(xy_points)-1):
        p_start = xy_points[i]
        p_end = xy_points[i+1]
        segment = np.linspace(0, 1, int(np.linalg.norm(p_end - p_start)/step))
        for s in segment[1:]:
            p_target = (1-s)*p_start + s*p_end
            theta = reach(p_target, theta, robot, avoidance, clamp)
            theta_list.append(theta.copy())

    # Step 3: close the path (last → first)
    p_last = xy_points[-1]
    p_first = xy_points[0]
    if np.linalg.norm(p_first - p_last) > 1e-6:
        segment = np.linspace(0, 1, int(np.linalg.norm(p_first - p_last)/step))
        for s in segment[1:]:
            p_target = (1-s)*p_last + s*p_first
            theta = reach(p_target, theta, robot, avoidance, clamp)
            theta_list.append(theta.copy())

    # Step 4: return to initial pose
    p_final = forward_kinematics(robot["theta_dep"], robot)
    if np.linalg.norm(p_final - p_first) > 1e-6:
        segment = np.linspace(0, 1, int(np.linalg.norm(p_final - p_first)/step))
        for s in segment[1:]:
            p_target = (1-s)*p_first + s*p_final
            theta = reach(p_target, theta, robot, avoidance, clamp)
            theta_list.append(theta.copy())

    return np.array(theta_list)

# ---------- 8. Sauvegarde du fichier ----------------------------
def save_trj(filename, theta_traj):
    np.savetxt(filename, theta_traj, fmt="%.15e")

# ---------- 9. Main ---------------------------------------------
if __name__ == "__main__":
    robot = read_robot_par("Robot.par")
    xy_points = read_xy("Trajet.xy")

    theta_traj = generate_joint_trajectory(robot, xy_points, step=STEP,
                                           avoidance=AVOIDANCE, clamp=CLAMP)

    save_trj("Trajet.trj", theta_traj)

    print(f"✅ Trajectoire articulaire générée "
          f"(avoidance={'ON' if AVOIDANCE else 'OFF'}, clamp={'ON' if CLAMP else 'OFF'}) -> Trajet.trj")