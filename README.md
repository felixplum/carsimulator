Class car: Base class for specific car models; provides I/O and integration routines
  - Inherited classes: Must override EvalModel() function
  : provides method to apply simulation step

Class map:
    : Map objected is instantiated once
    : loads map from file
    : generating test maps
    : generates center line from grid
    : map is stored as occupancy grid, indices correspond to position [m]
    : local center line coord. or local occupancy grid can be returned
    : provides methods for coord. transformation