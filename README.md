# Kernel-ROS Sensor Dashboard

**Kernel-ROS Sensor Dashboard** est un projet logiciel basé sur **Linux**, où chaque capteur est simulé via un **module du noyau Linux**. Les données des capteurs sont exposées vers l’espace utilisateur, collectées par des **nœuds ROS 2**, publiées sur des **topics**, et affichées **en temps réel** à travers une interface graphique interactive.

## Fonctionnalités

- Simulation de capteurs via des modules Linux.
- Transmission des données du kernel vers l’espace utilisateur.
- Collecte et publication des données par des nœuds **ROS 2**.
- Visualisation des données en **temps réel** avec un tableau de bord interactif.
- Architecture modulable permettant l’ajout facile de nouveaux capteurs.

## Architecture

Linux Kernel Modules (Sensors) --> User Space --> ROS 2 Nodes --> GUI Dashboard (Real-time visualization)

- **Modules Kernel :** Simulation de capteurs.
- **Nœuds ROS 2 :** Collecte et publication des données.
- **Dashboard :** Visualisation des données en temps réel.

## Technologies utilisées

- Linux Kernel Programming
- ROS 2
- Python / PyQt5 (ou PySide2) pour l’interface graphique
- C/C++ pour les modules kernel

## Objectif

Ce projet permet de créer un environnement simulé complet pour tester des capteurs et leur intégration avec **ROS 2**, tout en offrant une visualisation en temps réel des données.


