# Intelligent Agents Simulation - Java & Jason (AgentSpeak)

This repository contains a two-part project developed for the "Intelligent Agents" course at the University of Piraeus, Department of Digital Systems. The project explores autonomous agent behavior, pathfinding, and multi-agent coordination using a Java-based environment and Jason (AgentSpeak) for the agents' BDI (Belief-Desire-Intention) logic.

Author: Emmanouil Sifogiannakis

---

## 🌍 The Environment
Both parts of the project share a common grid-based world:
* **Grid:** 9x9 grid with 4-way movement (up, down, left, right) and 4 fixed, impassable obstacles.
* **Goals:** Agents collect colored goals with varying rewards: Green (0.8), Blue (0.6), Yellow (0.3), and Purple (0.2). Goals dynamically respawn in random free cells once collected.
* **Constraints:** Each episode lasts for 31 moves, and every step costs 0.01. 
* **Objective:** Maximize the final score, calculated as `totalReward - totalCost`.
* **GUI:** A Swing-based GUI visualizes the grid, agent positions, optimal paths (dashed lines), and provides a HUD with live metrics.

---

## 🚀 Part 1: Single Agent BDI Optimization
The first part (`Part1_Single_Agent`) focuses on a single agent optimizing its path to maximize points within the move limit. 

<p align="center">
  <img src="screenshot (1).png" width="300" />
</p>

### Key Features:
* **Advanced Target Evaluation:** The Java environment acts as the sensor/calculator, evaluating all possible goals using the A* algorithm.
* **Smart Detours:** Before committing to a target, the system checks if a nearby goal can be collected with a positive net gain (reward > extra steps cost) and adjusts the path.
* **Lookahead & Horizon Penalty:** The system performs a 1-step lookahead to predict the next best target and applies a horizon penalty to avoid targets the agent doesn't have enough moves to reach.
* **BDI Architecture:** The agent (`r1.asl`) receives `best_target` percepts and forms intentions to move (`!go(dir)`) towards it, dynamically reacting to environment changes.
* **Performance:** Over 1,000 episodes, the agent achieved an average score of 3.95 (Max: 7.29).

---

## 🤖 Part 2: Multi-Agent Coordination (Contract Net Protocol)
The second part (`Part2_Multi_Agent`) expands the system into a multi-agent environment featuring two Workers (Bob, Alice) and one Manager.

<p align="center">
  <img src="screenshot (2).png" width="300" />
</p>

### Key Features:
* **Contract Net Protocol (CNP):** The Manager orchestrates goal assignments. It sends a Call for Proposals (CFP) for available goals. Workers propose based on their calculated net gains (using A*), and the Manager accepts the best proposal.
* **Atomic Selection:** Implemented an atomic selection lock (`+!select_best_atomic`) to prevent race conditions during the auction process.
* **Collision Avoidance:** Agents execute moves asynchronously. If a collision is imminent, the system triggers a rollback and sends a `conflict(Other)` percept, forcing the agents to replan.
* **Worker BDI Logic:** Workers strictly follow the BDI paradigm, reacting to CFPs, pursuing assigned colors, and revising intentions if conflicts or respawns occur.

### Known Issues (Part 2)
* Occasional GUI rendering bugs where dashed path lines do not perfectly match the agents.
* Rare deadlock scenarios caused by complex collision/rollback edge cases during simultaneous crossings.
