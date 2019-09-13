---
title: "Pybotics: Python Toolbox for Robotics"
tags:
  - Python
  - robotics
  - automation
  - kinematics
  - geometry
  - optimization
authors:
  - name: Nicholas A. Nadeau
    orcid: 0000-0003-1220-0889
    affiliation: 1
affiliations:
 - name: Department of Automated Manufacturing Engineering, École de Technologie Supérieure, Montréal, QC H3C 1K3, Canada
   index: 1
date: 10 September 2019
bibliography: paper.bib
---

# Summary

<!-- context -->
Modern robotic programming relies on offline simulation to reduce process downtime.
In a virtual environment, application specialists can program, visualize, and test their robotic application before deploying it to the real production environment.
This offline process saves both time and costs while increasing the safety and efficacy of the robotic application.
However, to achieve a high level of fidelity between virtual and production environments, the robot system must be accurate.
According to ISO 5725-1, accuracy refers to closeness or *trueness* of measurements to a specific value, while precision refers to the closeness of the measurements to each other.
Most industrial robots are inherently precise (i.e., repeatable), but they are not necessarily accurate.
One cost-effective approach to obtaining a more accurate robot is through calibration, where the actual kinematic and non-kinematic parameters of the robot model are identified and improved upon when compared to the nominal model.

<!-- functionality -->
`Pybotics` is an open-source Python toolbox for robot kinematics and calibration.
It was designed to provide a simple, clear, and concise interface to quickly simulate and evaluate common robot concepts, such as kinematics, dynamics, trajectory generations, and calibration.
The toolbox is specifically designed for use with the Modified Denavit–Hartenberg parameters convention, introduced and popularized by @Craig2005, which uses four geometric parameters to define reference frames along the links of a robot manipulator.

<!-- uses and other packages -->
As MATLAB® is not necessarily readily available outside of academia, `Pybotics` was originally developed as a fully open-source alternative to the `Robotics Toolbox` by Peter Corke [@corke2017robotics] with the intention of being used in both research and industry.
The `Pybotics` toolbox leverages the `NumPy` package for computational efficiency [@van2011numpy] and offers a flexible interface to model robot manipulators using array-based notation.
The modelling approach allows for the vectorization of the robot model and integration with the robust optimization algorithms contained in the `SciPy` package [@jones2001scipy].
This results in the capability to easily calibrate a robot model and forms the foundation of the research presented in @nadeau2019impedance.
Furthermore, real-time robot optimization applications, an example of which is presented in @nadeau2018evolutionary, can be augmented with machine learning through the vectorial interface of `Pybotics` and the `Scikit-learn` framework [@pedregosa2011scikit].

# Acknowledgements

We thank Professor Ilian Bonev for his supervision throughout the graduate studies during which this package was developed.

# References
