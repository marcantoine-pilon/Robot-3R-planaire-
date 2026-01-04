# Robot 3R planaire — Génération de trajectoires et simulation (Python)

Projet académique réalisé dans le cadre du cours **MEC1315 – Technologies informationnelles en ingénierie**  
(**Polytechnique Montréal**).

Ce projet implémente un **générateur de trajectoires articulaires** et un **simulateur graphique** pour un robot **3R planaire**, à partir d’une trajectoire cartésienne (x, y), avec prise en compte des **limites articulaires** et d’une stratégie d’**évitement** basée sur le noyau du Jacobien.


##  Fonctionnalités principales

### Générateur de trajectoires (`Generateur_19.py`)
- Lecture du fichier robot `Robot.par`
- Lecture d’une trajectoire cartésienne `Trajet.xy`
- Calcul de la trajectoire articulaire `Trajet.trj` (θ₁, θ₂, θ₃)
- Résolution numérique de l’inverse cinématique via le Jacobien
- Option d’**évitement des limites articulaires** (null-space)

### Simulateur graphique (`Simulateur_19.py`)
- Lecture de `Robot.par` et `Trajet.trj`
- Cinématique directe du robot 3R
- Animation graphique avec `matplotlib`
- Affichage :
  - robot **bleu** (angles valides) / **rouge** (hors limites)
  - mur fixe
  - trajectoire du bout du robot
- Export de l’animation en **GIF**
  
## 🖼️ Résultats — Animations

### Trajectoire sans évitement des limites articulaires
![Simulation sans évitement](Trajet0_19.gif)

### Trajectoire avec évitement des limites articulaires
![Simulation avec évitement](Trajet1_19.gif)

### Cas bonus — Trajectoire complexe
![Simulation bonus](Trajet-bonus_19.gif)


##  Structure du dépôt

```
├── src/
│ ├── Generateur_19.py # Générateur (Trajet.xy → Trajet.trj)
│ ├── Simulateur_19.py # Simulateur (Robot.par + Trajet.trj → animation)
│ └── utils.py # Fonctions communes (lecture fichiers, cinématique, Jacobien)
│
├── data/
│ ├── Robot.par # Paramètres du robot
│ ├── Trajet.xy # Trajectoire cartésienne d’entrée
│ └── Trajet.trj # Trajectoire articulaire générée
│
├── outputs/
│ ├── Trajet0.gif # Animation sans évitement
│ └── Trajet1.gif # Animation avec évitement
│
└── README.md
```




##  Contexte technique (résumé)

Le robot est un manipulateur **3R planaire** défini par trois longueurs et trois angles articulaires.  
La relation vitesse est donnée par :

ṗ = J(θ) · θ̇

bash
Copier le code

La trajectoire articulaire est calculée par pseudo-inverse du Jacobien :

Δθₘ = J⁺ · Δp

bash
Copier le code

Avec évitement des limites articulaires, un terme homogène est ajouté dans le noyau du Jacobien :

Δθ = Δθₘ + α (I − J⁺J) h

où `h` pousse les angles vers le centre de leurs limites.



##  Prérequis

- Python 3.9+
- Bibliothèques :
  - `numpy`
  - `matplotlib`

Installation (si nécessaire) :
```bash
pip install numpy matplotlib
```
##  Répartition des tâches

|  | Tâches |
|---|---|
| Marc‑Antoine Pilon | <ul><li>Écriture collaborative du script pour le générateur (Generateur_19.py) avec l’aide de l’IA</li><li>Contribution majeure à l’écriture du rapport final</li><li>Remise du travail sur Moodle</li></ul> |
