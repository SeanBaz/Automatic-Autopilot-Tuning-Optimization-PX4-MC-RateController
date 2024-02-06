# Automatic Autopilot Tuning Framework using Genetic Algorithm and System Identification

## Overview
This repository contains the code and documentation for an automatic autopilot tuning framework utilizing Genetic Algorithm and System Identification. The framework is designed to optimize tuning parameters offline in unmanned aerial vehicle (UAV) flight controllers. It includes stages for system identification, optimization, and validation.

## Authors
- **Sean Bazzocchi**
  - PhD Candidate, Department of Mechanical Engineering, University of Victoria
  - Email: seanbazzocchi@uvic.ca
- **Afzal Suleman**
  - Principal Investigator, Department of Mechanical Engineering, University of Victoria
  - Email: suleman@uvic.ca

## Table of Contents
1. [Introduction](#introduction)
2. [Methodology](#methodology)
3. [Repository Structure](#repository-structure)
4. [Usage](#usage)
5. [License](#license)

## Introduction
This framework is introduced in the paper titled "Automatic autopilot tuning framework using Genetic Algorithm and System Identification." The goal is to optimize tuning parameters offline in UAV flight controllers, ensuring a balance among metrics such as stability, efficiency, safety, and overall performance.

For more details, refer to the [paper](link-to-paper).

## Methodology
The methodology involves distinct stages, including processing flight data, implementing control laws in the simulation environment, developing a flight dynamics model (FDM) through system identification, and optimizing the flight controller using Genetic Algorithms. The final validation is performed through simulations and flight tests.

For a detailed overview, refer to the [Methodology section](link-to-paper#methodology) of the paper.

## Repository Structure
- **Documentation:** Contains gains used for flight testing and block diagrams of the algorithms.
- **SystemIdentification:** Contains `Main_SID.m` for generating system identification models.
- **TuningOptimization:** Contains `Main_OPT.m` for running the optimization using system identification results and the control law specified in `PX4_angRate_controller_GAoptimization.slx`.
- **Validation:** Contains `Main_VAL.m` for validating all steps in the tuning method and generating additional plots. Each Main generates results and plots in the respective folders.

## Usage
1. Clone the repository: `git clone https://github.com/SeanBaz/Automatic-Autopilot-Tuning-Optimization-PX4-MC-RateController.git`
2. Navigate to the desired folder (e.g., `SystemIdentification`, `TuningOptimization`, `Validation`).
3. Run the respective `Main` script.

## License
This project is licensed under the [GNU General Public License v3.0](LICENSE).

