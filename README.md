# Purpose
One of the computer engineering projects at Concordia is a autonomous hovercraft where it must pass a challenge where it navigates through a simple maze. Using the arduino library docoks points from the project. To support my team in this development, I created a library which allows for the development of the algorithm to be streamlined as it abstracts away the implementation and running of components. This was used to create the final algorithm, which completed the majority of the maze.

## Potential Problems
The ATMega328p has 3 hardware timers, where multiple components need to use these for either PWM or for IMU calculations. There were bugs regarding this which we didn't have time to fix.

## Development
This was developed on Platform.io using object oriented design.
