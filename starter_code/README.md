# ECE276B SP22 PR2

## Overview
In this assignment, you are required to implement a search-based motion planning algorithm that intercepts a moving target. You should run **main.py** to test your algorithm.  

There are totally 11 scenes you have to test and include in the report. The 11 tests are implemented in **main.py** as functions `test_map0()`, `test_map1()`, ..., `test_map3c()`. You can modify `line 145` of **main.py** to switch among different test scenes.

We already implemented the greedy robot planner described in the project description and you can directly run **main.py** to have some visualizations.


### File description

#### 1. robotplanner.py

You should replace the algorithm in **robotplanner.py** with your own algorithm.

#### 2. targetplanner.py

This file contains the target planner. It uses the [minimax decisiton rule](https://en.wikipedia.org/wiki/Minimax) and tries to maximize the minimal distance the robot can achieve. You should not modify this file.

#### 3. main.py

This file contains test functions.
