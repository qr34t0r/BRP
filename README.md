# A Bresenham-based Global Path Planning Algorithm on Grid Maps
This repository contains supplementary code for the [Paper](https://doi.org/10.1109/DESSERT61349.2023.10416444).
> Velikzhanin, Artem, and Inna Skarga-Bandurova. "A Bresenham-based Global Path Planning Algorithm on Grid Maps." 2023 13th International Conference on Dependable Systems, Services and Technologies (DESSERT). IEEE, 2023.

## Abstract
This paper presents a novel global path planning algorithm adapted to grid maps extending the Bug algorithm family. During experiments with Bresenham’s line algorithm, it was found that this algorithm allows natural obstacle avoidance. Based on this observed property, a recursive path planning algorithm was developed that operates on the grid maps represented by a masked array and solves potential looping problems using a state machine-based loop breaking mechanism. BRP was compared with existing methods from the Bug family of algorithms as well as with more classical algorithms such as A*, D*, and Dijkstra. The Piano Movers problem was also touched upon. The time performance of the BLA algorithm for different line thicknesses was compared. Based on these experiments, future optimisation of the BRP algorithm for motion planning tasks is possible. Through experiments, BRP was found to show significant improvement in path planning time for certain map cases compared to existing algorithms while acknowledging a slight increase in route length, approximately 0.43% above the optimal path. The presented work improves the field of global path planning by providing efficient and adaptable solutions that can be applied to specific applications. 

## Illustrative animation depicting algorithmic processes
![Applying the algorithm to a grid map](https://github.com/qr34t0r/BRP/assets/146039455/e8330cd5-be08-4713-a404-1249ddae759d)

## Citation
```
@inproceedings{velikzhanin2023bresenham,
  title={A Bresenham-based Global Path Planning Algorithm on Grid Maps},
  author={Velikzhanin, Artem and Skarga-Bandurova, Inna},
  booktitle={2023 13th International Conference on Dependable Systems, Services and Technologies (DESSERT)},
  pages={1--8},
  year={2023},
  organization={IEEE}
}
```
## Licence
MIT
