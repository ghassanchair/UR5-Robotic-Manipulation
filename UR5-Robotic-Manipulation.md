# 🦾 Projet de Manipulateur de Robot UR5

Modélisation, simulation et commande d'un bras manipulateur robotique UR5 à 6 degrés de liberté.

---

## 📌 Sommaire
1. [Introduction](#1-introduction)
2. [Objectifs du Projet](#2-objectifs-du-projet)
3. [Étude Théorique](#3-étude-théorique)
   - [3.1. Description et Structure Mécanique du Robot](#31-description-et-structure-mécanique-du-robot)
   - [3.2. Modélisation Cinématique](#32-modélisation-cinématique)
     - [A. Paramètres de Denavit-Hartenberg (DH)](#a-paramètres-de-denavit-hartenberg-dh)
     - [B. Modèle Cinématique Direct (MGD)](#b-modèle-cinématique-direct-mgd)
     - [C. Modèle Cinématique Inverse (MGI)](#c-modèle-cinématique-inverse-mgi)
   - [3.3. Analyse de l'Espace de Travail et Singularités](#33-analyse-de-lespace-de-travail-workspace-et-singularités)
     - [A. Définition de l'espace de travail](#a-définition-de-lespace-de-travail)
     - [B. Analyse des singularités](#b-analyse-des-singularités)
   - [3.4. Modélisation Dynamique](#34-modélisation-dynamique)
     - [A. Équation Fondamentale (Formalisme de Lagrange)](#a-équation-fondamentale-formalisme-de-lagrange)
     - [B. Dynamique Inverse et Directe](#b-dynamique-inverse-et-directe)
4. [Partie Simulation](#4-partie-simulation)
5. [Conclusion et Perspectives](#5-conclusion-et-perspectives)

---

## 1. Introduction

Ce projet se concentre sur la modélisation, la simulation et la commande du bras manipulateur robotique **UR5**, un robot industriel collaboratif à **6 degrés de liberté (6-DOF)** développé par Universal Robots.

L'objectif principal est de comprendre le comportement d'un manipulateur robotique à travers l'étude approfondie de sa **cinématique** et de sa **dynamique**, ainsi que l'implémentation d'une stratégie de commande par couple calculé, validée par simulation sous MATLAB / Simulink.

---

## 2. Objectifs du Projet

Les principaux objectifs de ce projet sont les suivants :
* Comprendre l'architecture mécanique du robot UR5 et ses articulations.
* Développer le modèle cinématique direct (MGD).
* Étudier le problème de la cinématique inverse (MGI).
* Analyser l'espace de travail (workspace) du robot et ses limites géométriques.
* Développer le modèle dynamique complet du robot.
* Implémenter une stratégie de commande adaptée.
* Simuler le robot sous MATLAB / Simulink.
* Analyser et évaluer rigoureusement les résultats de simulation.

---

## 3. Étude Théorique

Cette section présente les fondements mathématiques et théoriques régissant le comportement géométrique, cinématique et dynamique du manipulateur UR5.

### 3.1. Description et Structure Mécanique du Robot
L'UR5 est un manipulateur série doté de **six articulations rotoïdes**. Ses six degrés de liberté lui permettent de contrôler de manière indépendante la position et l'orientation de son effecteur final dans l'espace tridimensionnel (**3** mouvements de translation et **3** mouvements de rotation).

### 3.2. Modélisation Cinématique

#### A. Paramètres de Denavit-Hartenberg (DH)

La convention de Denavit-Hartenberg est utilisée pour décrire la relation géométrique entre les liaisons consécutives du robot à l'aide de quatre paramètres : la longueur du lien (`aᵢ`), la torsion du lien (`αᵢ`), le décalage de lien (`dᵢ`) et l'angle articulaire (`θᵢ`). La matrice de transformation homogène élémentaire entre deux liaisons consécutives `i-1` et `i` s'exprime sous la forme :

```text
        ┌                                                          ┐
        │ cos(θᵢ)   -sin(θᵢ)cos(αᵢ)    sin(θᵢ)sin(αᵢ)   aᵢ·cos(θᵢ) │
ⁱ⁻¹Tᵢ = │ sin(θᵢ)    cos(θᵢ)cos(αᵢ)   -cos(θᵢ)sin(αᵢ)   aᵢ·sin(θᵢ) │
        │    0           sin(αᵢ)           cos(αᵢ)           dᵢ    │
        │    0              0                 0              1     │
        └                                                          ┘
```

#### B. Modèle Cinématique Direct (MGD)
Le MGD permet de déterminer la position et l'orientation de l'effecteur final en fonction des variables articulaires q = [q₁, q₂, q₃, q₄, q₅, q₆]ᵀ. La matrice de transformation globale est obtenue par la multiplication successive des matrices de transformation individuelles :<br>
⁰T₆ = A₁ · A₂ · A₃ · A₄ · A₅ · A₆

#### C. Modèle Cinématique Inverse (MGI)
Le MGI résout le problème inverse consistant à trouver les angles articulaires nécessaires pour atteindre une pose (position et orientation) désirée de l'effecteur. L'approche numérique s'appuie sur la matrice Jacobienne J(q) reliant les vitesses articulaires aux vitesses cartésiennes (Ẋ = J(q)·q̇), en utilisant des méthodes itératives basées sur le pseudo-inverse ou la minimisation des moindres carrés amortis pour gérer la convergence et la stabilité près des singularités.

### 3.3. Analyse de l'Espace de Travail (Workspace) et Singularités

#### A. Définition de l'espace de travail
L'espace de travail représente l'ensemble des positions que l'effecteur du robot peut atteindre en respectant les limites mécaniques et articulaires des liaisons :<br>
W = { P(q) ∈ ℝ³ | q ∈ Q }<br>
Où Q est l'ensemble des configurations articulaires admissibles définies par les limites de chaque articulation :<br>
qᵢ,ₘᵢₙ ≤ qᵢ ≤ qᵢ,ₘₐₓ

On distingue :
* **L'espace accessible** : ensemble de toutes les positions pouvant être atteintes avec au moins une configuration articulaire.
* **L'espace dextre** : ensemble des positions où l'effecteur peut atteindre n'importe quelle orientation requise.

#### B. Analyse des singularités
Une singularité est une configuration géométrique où la matrice Jacobienne J(q) perd son rang (son déterminant s'annule : det(J(q)) = 0).

* **Conséquences** : perte de degrés de liberté instantanés, vitesses articulaires tendant vers l'infini pour des mouvements cartésiens normaux, et instabilité numérique de la cinématique inverse.

* **Principaux types sur l'UR5** :
   1. **Singularité d'épaule** : le centre du poignet se trouve sur l'axe de l'articulation 1.
   2. **Singularité de coude** : alignement du bras et de l'avant-bras (bras tendu ou replié).
   3. **Singularité de poignet** : alignement des axes des articulations 4 et 6.

* **Indicateur de suivi** : l'évaluation du nombre de condition de la Jacobienne κ(J) = σₘₐₓ / σₘᵢₙ permet de détecter l'approche d'une configuration singulière.

### 3.4. Modélisation Dynamique
La dynamique établit la relation mathématique entre les forces/couples appliqués et les mouvements générés, en prenant en compte les masses, les centres d'inertie et les forces de Coriolis.

#### A. Équation Fondamentale (Formalisme de Lagrange)
L'équation générale du mouvement pour un robot à n degrés de liberté s'écrit sous la forme :<br>
M(q)·q̈ + C(q, q̇)·q̇ + G(q) = τ<br>
Où :

* **M(q)** : matrice d'inertie (symétrique et définie positive).
* **C(q, q̇)** : matrice des forces de Coriolis et centrifuges.
* **G(q)** : vecteur des couples de gravité compensant le poids des liens.
* **τ** : vecteur des couples appliqués aux articulations.

#### B. Dynamique Inverse et Directe
* **Dynamique inverse** : calcule les couples τ nécessaires pour suivre une référence (q, q̇, q̈). Très utile pour la commande et le dimensionnement des moteurs.
* **Dynamique directe** : calcule les accélérations résultantes q̈ en fonction des couples appliqués τ :<br>
q̈ = M(q)⁻¹ · [ τ - C(q, q̇)·q̇ - G(q) ]

---
## 4. Partie Simulation

### 4.1. Environnement et objectif

La simulation est réalisée sous **MATLAB / Simulink** avec la **Robotics System Toolbox** et **Simscape Multibody**. Elle porte sur deux aspects du robot UR5 :

* **la cinématique** : calcul de la pose de l'effecteur (MGD), puis retour aux angles articulaires (MGI) ;
* **la dynamique** : commande par couple calculé et réponse du robot sous l'effet des couples appliqués.

![Modèle Simulink du UR5](images/UR5_simulation_environment.png)
*Figure 4.1 : Modèle Simulink (cinématique et dynamique du UR5)*

### 4.2. Architecture du modèle

Le modèle se compose de deux parties reliées en chaîne.

**Partie 1 : Cinématique**

| Bloc | Rôle |
|---|---|
| **q1** (constante 1×6) | Configuration articulaire de départ à tester |
| **Transformation Matrix computation** | MGD : calcule la matrice de transformation ⁰T₆ à partir de q1 |
| **q0** (constante 1×6) | Estimation initiale du solveur de MGI |
| **Weights** (constante 1×6) | Poids de l'orientation et de la position dans la MGI |
| **Inverse Kinematics** (EE : tool0) | MGI : retrouve la configuration articulaire désirée $q_d$ à partir de la pose |

**Partie 2 : Dynamique et commande**

| Bloc | Rôle |
|---|---|
| **Computed Torque Controller** | Calcule les couples à appliquer |
| **UR5 Robot Model** | Dynamique directe du robot |
| **Scope** | Compare les signaux désirés et mesurés |
| **Sorties $\dot q$ et $\ddot q$** | Vitesses et accélérations articulaires du robot |

*Entrées du contrôleur :*
* *Desired Configuration* : $q_d$ fournie par la MGI ;
* *Measured Configuration* : $q$ mesurée sur le robot ;
* *Measured Velocities* : $\dot q$ mesurée sur le robot.

*Sortie du contrôleur :* *Applied Torque* $\tau$, envoyé au robot.

### 4.3. Principe de fonctionnement

1. **MGD** : la configuration q1 est convertie en pose cartésienne de l'effecteur :
   $${}^0T_6 = A_1 A_2 A_3 A_4 A_5 A_6$$
2. **MGI** : cette pose est transmise au solveur, qui retrouve la configuration désirée $q_d$ en partant de q0 :
   $$FK(q_d) \approx {}^0T_6$$
3. **Commande** : le contrôleur compare $q_d$ à la configuration mesurée $q$ et calcule les couples :
   $$\tau = M(q)\,v + C(q,\dot q)\dot q + G(q)$$
   $$v = \ddot q_d + K_d(\dot q_d - \dot q) + K_p(q_d - q)$$
4. **Dynamique** : le robot applique les couples $\tau$ et fournit en retour $q$, $\dot q$ et $\ddot q$ :
   $$\ddot q = M(q)^{-1}\left[\tau - C(q,\dot q)\dot q - G(q)\right]$$
5. **Boucle de retour** : $q$ et $\dot q$ sont renvoyés au contrôleur, ce qui ferme la boucle.

### 4.4. Étapes de la simulation

**Étape 1 : Définir le modèle du robot.** Charger le UR5 et l'affecter aux blocs de cinématique :
```matlab
robot = loadrobot("universalUR5", DataFormat="row");
```

**Étape 2 : Régler les blocs de cinématique.**
* Dans *Transformation Matrix computation* : choisir le corps cible `tool0` (le bloc est actuellement en *Target : world*, ce qui donne toujours l'identité).
* Dans *Inverse Kinematics* : vérifier l'effecteur `tool0` et saisir les poids.

**Étape 3 : Définir les constantes.**
```matlab
q0 = zeros(1,6);                          % estimation initiale
q1 = [pi/4 -pi/3 pi/4 -pi/4 pi/3 0];      % configuration à atteindre
weights = [0.25 0.25 0.25 1 1 1];         % [orientation position]
```

**Étape 4 : Régler le contrôleur.** Choisir les gains $K_p$ et $K_d$ du *Computed Torque Controller*, puis fixer le temps de simulation et le pas de calcul.

**Étape 5 : Valider la cinématique.** Vérifier que la MGD est correcte (pour q = 0 : P = [-0,81725 ; -0,19145 ; -0,005491] m), puis que la MGI redonne une pose cohérente : $FK(q_d) \approx FK(q1)$.

**Étape 6 : Lancer la simulation.** Exécuter le modèle et observer le mouvement du robot.

**Étape 7 : Enregistrer les signaux.** Récupérer $q_d$, $q$, $\dot q$, $\ddot q$ et $\tau$ (Scope, *To Workspace* ou sorties du modèle).

**Étape 8 : Tracer et analyser.** Afficher les courbes décrites en 4.5 et calculer les indicateurs de performance de 4.6.

### 4.5. Résultats à présenter

* **Figure 4.2** : configuration désirée $q_d$ (sortie de la MGI) et configuration mesurée $q$
* **Figure 4.3** : erreur de suivi $e_i(t) = q_{d,i}(t) - q_i(t)$
* **Figure 4.4** : vitesses articulaires $\dot q$
* **Figure 4.5** : accélérations articulaires $\ddot q$
* **Figure 4.6** : couples appliqués $\tau$

### 4.6. Critères d'évaluation

| Indicateur | Description |
|---|---|
| Erreur de la MGI | Écart entre la pose obtenue $FK(q_d)$ et la pose demandée |
| Erreur de suivi | $e_i(t) = q_{d,i}(t) - q_i(t)$ |
| RMSE | $\sqrt{\frac{1}{N}\sum_{k=1}^{N} e_i(k)^2}$ |
| Temps de réponse | Temps pour atteindre et rester dans la bande d'erreur |
| Dépassement | Écart maximal au-delà de la position finale |
| Couple maximal | À comparer aux limites des moteurs du UR5 |

### 4.7. Résultats et discussion

*(À compléter avec tes courbes de simulation.)*

* Les positions mesurées suivent-elles la configuration désirée ?
* Quelle est l'erreur de suivi maximale et le RMSE de chaque articulation ?
* Les vitesses, accélérations et couples restent-ils dans des valeurs acceptables ?
* La MGI converge-t-elle vers une configuration valide ?

> ⚠️ Toutes les valeurs présentées doivent provenir de la simulation.
## 4. Partie Simulation

*(Coller ici la section « 4. Partie Simulation » fournie précédemment, basée sur ton modèle Simulink.)*

---

## 5. Conclusion et Perspectives

*(À rédiger après les résultats de simulation.)*
