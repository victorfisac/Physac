# Physac
_Created by Víctor Fisac [www.victorfisac.com]_

Physac is a small 2D physics engine written in pure C. The engine uses a fixed time-step thread loop to simluate physics.
A physics step contains the following phases: get collision information, apply dynamics, collision solving and position correction. It uses
a very simple struct for physic bodies with a position vector to be used in any 3D rendering API.

The header file includes some tweakable define values to fit the results that the user wants with a minimal bad results. Most of those values are commented with a little explanation about their uses.

_Note: The example code uses raylib programming library to create the program window and rendering framework._
