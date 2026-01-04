import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# 1. Lecture du fichier Robot.par
def lire_fichier_parametres(nom_fichier="Robot.par"):
    valeurs = []
    with open(nom_fichier, "r", encoding="utf-8") as f:
        for ligne in f:
            t = ligne.strip()
            if not t or t.startswith("#"):
                continue
            try:
                valeurs.append(float(t.split()[0]))
            except ValueError:
                pass

    if len(valeurs) < 23:
        raise ValueError("Le fichier Robot.par doit contenir 23 valeurs numériques.")

    # Extraction des valeurs
    L1, L2, L3 = valeurs[0:3]
    x_min, x_max, y_min, y_max = valeurs[3:7]
    x_base, y_base = valeurs[7:9]
    angle1_min, angle1_max = valeurs[12], valeurs[13]
    angle2_min, angle2_max = valeurs[14], valeurs[15]
    angle3_min, angle3_max = valeurs[16], valeurs[17]
    mur_x0, mur_x1, mur_y0, mur_y1 = valeurs[18:22]
    dt_ms = int(valeurs[22])

    return {
        "L1": L1, "L2": L2, "L3": L3,
        "x_base": x_base, "y_base": y_base,
        "x_min": x_min, "x_max": x_max,
        "y_min": y_min, "y_max": y_max,
        "limite_min": np.array([angle1_min, angle2_min, angle3_min]),
        "limite_max": np.array([angle1_max, angle2_max, angle3_max]),
        "mur": np.array([[mur_x0, mur_y0], [mur_x1, mur_y1]]),
        "dt": dt_ms,
    }


# 2. Lecture du fichier Trajet.trj (angles θ1, θ2, θ3)
def lire_fichier_trajet(nom_fichier="Trajet.trj"):
    data = np.loadtxt(nom_fichier, dtype=float)
    data = np.atleast_2d(data)
    if data.shape[1] != 3:
        raise ValueError("Trajet.trj doit contenir 3 colonnes.")
    return data



# Cinématique directe : calcul des points du robot
def calculer_points_articulations(angles, L1, L2, L3, x_base, y_base):
    """Retourne les coordonnées des 4 points du robot"""
    t1, t2, t3 = angles
    c1, s1 = np.cos(t1), np.sin(t1)
    c12, s12 = np.cos(t1 + t2), np.sin(t1 + t2)
    c123, s123 = np.cos(t1 + t2 + t3), np.sin(t1 + t2 + t3)

    x1, y1 = x_base + L1 * c1, y_base + L1 * s1
    x2, y2 = x1 + L2 * c12, y1 + L2 * s12
    x3, y3 = x2 + L3 * c123, y2 + L3 * s123

    return np.array([[x_base, y_base], [x1, y1], [x2, y2], [x3, y3]])


#Fonction principale de simulation
def simuler_robot(nom_par="Robot.par", nom_trajet="Trajet.trj", nom_gif="Animation.gif"):
    """Anime le robot 3R en lisant les fichiers fournis."""

    #Lecture des fichiers
    params = lire_fichier_parametres(nom_par)
    angles_trajet = lire_fichier_trajet(nom_trajet)

    #Raccourcis pour plus de lisibilité
    L1, L2, L3 = params["L1"], params["L2"], params["L3"]
    x_base, y_base = params["x_base"], params["y_base"]
    x_min, x_max, y_min, y_max = params["x_min"], params["x_max"], params["y_min"], params["y_max"]
    limites_min, limites_max = params["limite_min"], params["limite_max"]
    mur, dt = params["mur"], params["dt"]

    #Création de la figure
    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim([x_min, x_max])
    ax.set_ylim([y_min, y_max])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Simulation du robot 3R — TS1")

    # Tracé du mur
    ax.plot(mur[:, 0], mur[:, 1], "k-o", lw=2)

    # Lignes à mettre à jour pendant l’animation
    (ligne_robot,) = ax.plot([], [], "-o", lw=2, color="blue")
    (ligne_trajet,) = ax.plot([], [], "--", lw=1.5, color="red")

    # Liste des positions successives du bout du bras
    path_x, path_y = [], []

    # Initialiser l’animation
    def initialiser():
        """On appelle une seule fois au début de l’animation."""
        path_x.clear()  # ✅ Efface la trajectoire précédente
        path_y.clear()
        ligne_robot.set_data([], [])
        ligne_trajet.set_data([], [])
        return ligne_robot, ligne_trajet

    #Fonction mise à jour pour chaque frame
    def mise_a_jour(frame):
        """on  met à jour la position du robot à chaque étape"""
        angles = angles_trajet[frame]
        points = calculer_points_articulations(angles, L1, L2, L3, x_base, y_base)
        x, y = points[:, 0], points[:, 1]

    #Couleur bleue si dans les limites, rouge sinon
        dans_limite = np.all(angles >= limites_min) and np.all(angles <= limites_max)
        ligne_robot.set_color("blue" if dans_limite else "red")
        ligne_robot.set_data(x, y)

    #Ajoute la position du bout du bras (x3, y3) à la trajectoire
        path_x.append(x[-1])
        path_y.append(y[-1])
        ligne_trajet.set_data(path_x, path_y)

        return ligne_robot, ligne_trajet

    #Création de l’animation
    animation = FuncAnimation(
        fig,
        mise_a_jour,
        frames=len(angles_trajet),  # nombre total d’étapes
        init_func=initialiser,      # fonction d’initialisation
        blit=True,                  # redessine uniquement les objets modifiés
        interval=dt,                # délai entre images (ms)
        repeat=False                # ✅ Ne reboucle pas à la fin
        )

    #Enregistrement de l'animation en format GIF
    animation.save(nom_gif, writer="pillow", fps=25)
    
    #Affiche la fenêtre graphique
    plt.show()
    plt.close(fig)


# -------------------------------------------------------------
# 5. Lancement automatique si exécuté directement
# -------------------------------------------------------------
if __name__ == "__main__":
    simuler_robot("Robot.par", "Trajet.trj", "Animation.gif")
