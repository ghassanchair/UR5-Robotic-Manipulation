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
   - [3.4. Planification de Trajectoire](#34-planification-de-trajectoire)
     - [A. Trajectoires dans l'Espace Articulaire](#a-trajectoires-dans-lespace-articulaire)
     - [B. Trajectoires dans l'Espace Cartésien](#b-trajectoires-dans-lespace-cartésien)
   - [3.5. Modélisation Dynamique](#35-modélisation-dynamique)
     - [A. Équation Fondamentale (Formalisme de Lagrange)](#a-équation-fondamentale-formalisme-de-lagrange)
     - [B. Dynamique Inverse et Directe](#b-dynamique-inverse-et-directe)
4. [Partie Simulation](#4-partie-simulation)
5. [Conclusion et Perspectives](#5-conclusion-et-perspectives)

---

## 1. Introduction

Ce projet se concentre sur la modélisation, la simulation et la commande du bras manipulateur robotique **UR5**, un robot industriel collaboratif à **6 degrés de liberté (6-DOF)** développé par Universal Robots. 

L'objectif principal est de comprendre le comportement d'un manipulateur robotique à travers l'étude approfondie de sa cinématique et de sa dynamique, la génération de ses trajectoires et l'implémentation de stratégies de commande. Une application de type **Pick and Place** est également développée pour démontrer la capacité du robot à accomplir une tâche de manipulation concrète dans un environnement virtuel.

---

## 2. Objectifs du Projet

Les principaux objectifs de ce projet sont les suivants :
* Comprendre l'architecture mécanique du robot UR5 et ses articulations.
* Développer le modèle cinématique direct (MGD).
* Étudier le problème de la cinématique inverse (MGI).
* Analyser l'espace de travail (workspace) du robot et ses limites géométriques.
* Générer des trajectoires fluides dans l'espace articulaire et dans l'espace cartésien.
* Développer le modèle dynamique complet du robot.
* Implémenter une stratégie de commande adaptée.
* Simuler une tâche de manipulation de type *Pick and Place*.
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
Le MGD permet de déterminer la position et l'orientation de l'effecteur final en fonction des variables articulaires q = [q₁, q₂, q₃, q₄, q₅, q₆]ᵀ. La matrice de transformation globale est obtenue par la multiplication successive des matrices de transformation individuelle :
⁰T₆ = A₁ · A₂ · A₃ · A₄ · A₅ · A₆
#### C. Modèle Cinématique Inverse (MGI)
Le MGI résout le problème inverse consistant à trouver les angles articulaires nécessaires pour atteindre une pose (position et orientation) désirée de l'effecteur. L'approche numérique s'appuie sur la matrice Jacobienne J(q) reliant les vitesses articulaires aux vitesses cartésiennes (Ẋ = J(q)·q̇), en utilisant des méthodes itératives basées sur le pseudo-inverse ou la minimisation des moindres carrés amortis pour gérer la convergence et la stabilité près des singularités.
