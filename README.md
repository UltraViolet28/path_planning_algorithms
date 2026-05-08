# Path Planning Algorithms: A*, RRT, and RRT*

Standalone Python implementations of A*, RRT, RRT*, and goal-biased variants for comparison and study, visualized on 2D workspaces with circular obstacles.

## Overview

This repository collects implementations of three path planning algorithm families: grid-based A* search and sampling-based RRT variants. A* operates on discrete occupancy grids; the RRT family operates in continuous 2D space. RRT* adds asymptotic optimality through cost-based rewiring. Goal-biased variants (RRT-Smart, RRT*-Smart) accelerate convergence by occasionally sampling directly toward the goal. The implementations are written for learning and comparison; they are not optimized for real robot deployment.

## Methods / Approach

- **A\*:** heuristic graph search on a 2D occupancy grid (pixel map); Euclidean distance heuristic; heap-based priority queue for open-set management
- **RRT:** random tree expansion in a continuous 800×600 workspace; fixed step size (0.1); goal check within distance threshold (20 px)
- **RRT\*:** extends RRT with cost-based parent rewiring within a proximity radius (50 px); converges toward optimal path given sufficient iterations (5000)
- **RRT-Smart:** RRT with goal-directed sampling bias (configurable bias parameter, 1–10); faster convergence at the cost of path optimality
- **RRT\*-Smart:** combines RRT\* rewiring with goal bias; best trade-off between convergence speed and path quality
- **Collision detection:** Euclidean distance check against circular obstacles defined as (x, y, radius) tuples
- **Visualization:** OpenCV (A\*); Pygame real-time tree rendering with blue edges, red path, black obstacles (RRT variants)
- **Libraries:** OpenCV, NumPy (A\*); Pygame, Python `math`, `random` (RRT variants)

## Results

A* result images (GitHub CDN):

A* on a synthetic OpenCV map:
![A* OpenCV](https://user-images.githubusercontent.com/88196192/179578813-1c8f3bd3-ccdb-4ceb-9898-2b2bfe9ea8fe.png)

A* on an RViz occupancy grid:
![A* RViz](https://user-images.githubusercontent.com/88196192/179578684-99bce3ec-1fd5-444b-b6ac-fa5e14ce3b51.png)

RRT variant result images are in `Results/` (added after merging RRT-Algorithm-Implementation):
- `RRT.png` — basic RRT (dense, erratic tree structure)
- `RRT_Smart.png` — goal-biased RRT (faster, more directed)
- `RRT_Star_pic1.png`, `RRT_Star_pic2.png` — RRT* optimized paths
- `RRT_StarSmart_pic2.png` — RRT*-Smart (bias=8); best convergence and path quality
- `RRT_StarSmart_bias6.png` — RRT*-Smart with lower bias (60% random, 40% goal-directed)
- `RRT_Iteration Failed.png` — example of failed convergence within iteration limit

No quantitative path-length or runtime comparisons are recorded.

## How to Run

Dependencies:
```
pip install pygame opencv-python numpy
```

A* on a synthetic grid:
```bash
cd a_star
python test.py
```

RRT variants (each opens a Pygame window):
```bash
cd rrt
python rrt_test.py           # basic RRT
python rrt_smart.py          # RRT with goal bias
python rrt_star.py           # RRT*
python rrt_star_smart.py     # RRT*-Smart (recommended starting point)
```

Modify start/goal positions and obstacle list in the `main` block of each script. Adjust the `bias` parameter in smart variants to control goal-directedness (higher value = more random exploration; range 1–10).

## Context

Standalone study implementations, 2022–2023.
