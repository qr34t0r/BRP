# A Bresenham-based Global Path Planning Algorithm on Grid Maps
This repository contains supplementary code for the [Paper](https://doi.org/10.1109/DESSERT61349.2023.10416444).
> Velikzhanin, Artem, and Inna Skarga-Bandurova. "A Bresenham-based Global Path Planning Algorithm on Grid Maps." 2023 13th International Conference on Dependable Systems, Services and Technologies (DESSERT). IEEE, 2023.

## Abstract
This paper presents a novel global path planning algorithm adapted to grid maps extending the Bug algorithm family. During experiments with Bresenham’s line algorithm, it was found that this algorithm allows natural obstacle avoidance. Based on this observed property, a recursive path planning algorithm was developed that operates on the grid maps represented by a masked array and solves potential looping problems using a state machine-based loop breaking mechanism. BRP was compared with existing methods from the Bug family of algorithms as well as with more classical algorithms such as A*, D*, and Dijkstra. The Piano Movers problem was also touched upon. The time performance of the BLA algorithm for different line thicknesses was compared. Based on these experiments, future optimisation of the BRP algorithm for motion planning tasks is possible. Through experiments, BRP was found to show significant improvement in path planning time for certain map cases compared to existing algorithms while acknowledging a slight increase in route length, approximately 0.43% above the optimal path. The presented work improves the field of global path planning by providing efficient and adaptable solutions that can be applied to specific applications. 

## Illustrative animation depicting algorithmic processes
![Applying the algorithm to a grid map](https://github.com/qr34t0r/BRP/assets/146039455/e8330cd5-be08-4713-a404-1249ddae759d)

## Perfomanse analysis
Testing was performed on 5,000 thousand randomly generated point pairs for the three maps. It is assumed that a route is always possible between any two points in the pair.

> Used symbols: µ — mean; Th – thickness; OD – original distance; TE – execution time; OP – occupancy percentage; RE – successful goal-finding percentage; TD - total distance; SM – trajectory smoothness. 

### BRP algorithm perfomance comparision (MAP1)
| Algorithm	| μ OD	|OP (%)	| μ TD	|μ  SM	|μ  TE|	RE (%)|
| --- | --- | --- | --- | --- | --- | --- |
|`Dijkstra`	|32.39	|9.36	|34.35|	0.121|	0.0225|	100|
|`A*`	|32.39|	9.36	|34.36|	0.24	|0.008|	100|
|`D*`	|32.39|9.36|	33.32	|0.126|	0.055|	100|
|`Bug_0`	|32.33|	9.36|	36.42	|0.152|	0.0078	|99.64|
|`Bug_1`	|32.2	|9.36	|42.26	|0.31|	0.011|	97.62|
|`Bug_2`	|30.13	|9.36|	39.62	|0.267	|0.0062|	52.6|
|`BRP`	|32.39|	9.36	|34.51|	0.366	|0.0029	|100|

### BRP algorithm perfomance comparision (MAP2)
| Algorithm	| μ OD	|OP (%)	| μ TD	|μ  SM	|μ  TE|	RE (%)|
| --- | --- | --- | --- | --- | --- | --- |
|`Dijkstra`	|33.92	|19.08|	79.44|	0.154	|0.018|	100|
|`A*`	|33.92	|19.08	|79.44	|0.215	|0.03	|100|
|`D*`	|33.92	|19.08	|78.42	|0.152	|0.056|	100|
|`Bug_0`	|	33.92	|19.08	|81.22	|0.122	|0.047|	100|
|`Bug_1`	|33.92	|19.08	|107.07|	0.167	|0.044	|100|
|`Bug_2`	|34.07	|19.08	|113.34|	0.148|	0.02|	86.56|
|`BRP`	|33.92	|19.08	|84.09|	0.296|	0.024	|100|

### BRP algorithm perfomance comparision (MAP3)
| Algorithm	| μ OD	|OP (%)	| μ TD	|μ  SM	|μ  TE|	RE (%)|
| --- | --- | --- | --- | --- | --- | --- |
|`Dijkstra`	|32.46	|36.4	|36.52	|0.22	|0.0138|	100|
|`A*`	|32.46	|36.4	|36.52	|0.244|	0.0071|	100|
|`D*`	|32.46	|36.4	|35.43	|0.215|	0.079|	100|
|`Bug_0`	|30.98	|36.4	|45.37|	0.51|	0.089	|53.0|
|`Bug_1`	|38.89	|36.4	|112.48	|0.33	|0.116	|28.98|
|`Bug_2`	|37.64	|36.4	|96.1	|0.343	|0.077	|21.78|
|`BRP`	|32.46	|36.4	|39.21	|0.28	|0.0219	|100|


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
