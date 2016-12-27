# Robot Affordances

Software for visual robot affordances of objects and tools.

## Introduction

This website illustrates a software framework for experiments in visual robot affordances. It is directed at the robotics, psychophysics and neuroscience communities. We provide documentation, tutorials and videos of some practical applications.

In a nutshell, the pipeline of the framework is as follows. 1) A visual segmentation algorithm is run on an image stream; 2) features of the segmented objects (and their sub-parts in the case of tools) are extracted; 3) features are then used for higher-level Bayesian inference, addressing the actual object affordances. Complete examples are provided.

## Tutorials & Documentation

Dependencies:

- [YARP](https://github.com/robotology/yarp)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [OpenCV](http://opencv.org/downloads.html)

Installation:

  git clone https://github.com/gsaponaro/robot-affordances
  cd robot-affordances && mkdir build && cd build && cmake .. && make

Online documentation is available here: [https://gsaponaro.github.io/robot-affordances/](https://gsaponaro.github.io/robot-affordances/).

## Publications

* Alexandre Antunes, Lorenzo Jamone, Giovanni Saponaro, Alexandre Bernardino, Rodrigo Ventura. *From Human Instructions to Robot Actions: Formulation of Goals, Affordances and Probabilistic Planning*. IEEE International Conference on Robotics and Automation (ICRA 2016).
* Afonso Gonçalves, João Abrantes, Giovanni Saponaro, Lorenzo Jamone, Alexandre Bernardino. *Learning Intermediate Object Affordances: Towards the Development of a Tool Concept*. IEEE International Conference on Development and Learning and on Epigenetic Robotics (ICDL-EpiRob 2014).
* Afonso Gonçalves, Giovanni Saponaro, Lorenzo Jamone, Alexandre Bernardino. *Learning Visual Affordances of Objects and Tools through Autonomous Robot Exploration*. IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC 2014).

## License

Released under the terms of the GPL v2.0 or later. See the file LICENSE for details.
