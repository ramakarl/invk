

InvK - Inverse Kinematics Library using Quaternions
------------------------------------------------

by Rama Hoetzlein ([ramakarl.com](http://ramakarl.com))

This is a simple library that demonsrates an efficient solution to
inverse kinematic chains using the Jacobian Transpose method over
quaternions. The goal of this library is to enable a baseline 
implementation with few dependencies upon which to build larger projects.

Some useful reading material:
<br>
Steve Rotenberg, Inverse Kinematics (part 1), UCSB. [Slides](https://cseweb.ucsd.edu/classes/wi17/cse169-a/slides/CSE169_08.pdf)<br>
Steve Rotenberg, Inverse Kinematics (part 2), UCSB. [Slides](https://cseweb.ucsd.edu/classes/wi17/cse169-a/slides/CSE169_09.pdf)<br>
Andreas Aristidou and Joan Lasenby, Inverse Kinematics: a review of existing techniques and introduction of a new fast iterative solver. [Tech Report](http://www.andreasaristidou.com/publications/papers/CUEDF-INFENG,%20TR-632.pdf)<br>

Revision History
--------
May 6, 2019 - v1.0 - Support for hinge and ball joints, with joint limits. 

How to Build
-------
* You will need to install cmake
1. Clone into a folder for invk
2. Create a build folder somewhere, eg. invk/build
3. From inside that folder: invk/build> cmake .. -DCMAKE_HELPERS_PATH=invk/helpers
4. When compile & generate succeed, run 'make'

LICENSE
-------
This library is licensed under the LGPLv3 license.
  https://www.gnu.org/licenses/lgpl-3.0.txt

Short summary:
- Public changes to the library itself must be back-contributed or forked as LGPL
- Use in larger projects that are not LGPL are allowed

Rama Hoetzlein (c) May 2019
