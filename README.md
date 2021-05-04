Robotics initiation class materials by Passault Grégoire, Olivier Ly and Remi Fabre is licensed under a [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/).

# Simulation
A simple simulation based on PyBullet.

## Requirements
Tested on Python 3.6 but should work on any recent version of Python:
```bash
pip install numpy pybullet onshape-to-robot transforms3d scipy
```

## Usage
 ```python sim_hexa.py --help``` gives some info on how to use this.


```bash
python3 sim_hexa.py --mode direct
```

```bash
python3 sim_hexa.py --mode inverse
```

- Configurable walk and position (print in the terminal):
```bash
python3 sim_hexa.py --mode walk
```

- Rotation:
```bash
python3 sim_hexa.py --mode toupie
```

- Move the body of the robot:
```bash
python3 sim_hexa.py --mode move-center
```

- Controle the robot with keyboard:
```bash
python3 sim_hexa.py --mode input
```
Input: ZQSD to move, AE to rotate, +SHIFT to go quicker

You will have to add your file ```kinematics.py``` is in this folder.

## Videos
https://youtu.be/w3psAbh3AoM
```bash
python3 sim_hexa.py --mode frozen-direct
```

# Real robot
Controlling a Bioloid hexapod

## Requirements
Tested on Python 3.6 but should work on any recent version of Python:
```bash
pip install pypot
```

## Usage
**Before** trying to run this on the real robot. Check that:
- You set ```ROBOT_TYPE = BIOLOID``` in ```constants.py```
- Check that your IK and DK functions give correct answers. Some results to check:
```
computeDK(0, -90, -90) -->  [81.70844556523558, 0.0, -33.04585438182947]
computeDK(0, 0, 0) -->  [118.79421209003992, 0.0, -115.14351525756761]
computeDK(90, 0, 0) -->  [7.274047579664959e-15, 118.79421209003992, -115.14351525756761]
computeDK(180, -30.501, -67.819) -->  [-64.13126891250484, 7.853815319895721e-15, -67.79136156948073]
computeDK(0, -30.645, 38.501) -->  [203.22984674011096, 0.0, -14.30057614144421]
computeIK(200, 30, -60) -->  [8.530765609948132, 0.9495664162204136, 64.25]
computeIK(150, -50, -90) -->  [-18.434948822922014, -13.452636970582287, 9.669311402298497]
```
-> If your results don't match check the following:
- Units (mm vs m, rads vs degrees)
- The actual robot pieces are not straight. The angles need to be "compensated" to account for that. The file ```constants.py``` should give you a hint.
- Some (Some? A few? All? None?) points have several IK solutions. How do you chose which one to use?
- Check the direction of Z (I might have inverted it)

```bash
python3 main.py
```