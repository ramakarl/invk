

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

IK and Quaternions
------------------
Quaternions allow for several benefits over Euler angles. First, axis boundaries are greatly simplified as quaternions can interpolate thru two arbitrary vectors. Second, IK requires incremental changes in angles which are well suited to quaternions. Third, quaternions are more efficient to compute for certain operations. 

There are two drawbacks to using quaternions for inverse kinematics. Per-axis angle range limits are more easily computed with Euler angles, so there is a conversion performed in the LimitQuaternion function to handle this. Finally, care must be taken to normalize the quaternions frequently during calculations. 

Quaternions can represent an orientation (a local coordinate system), or they can represent a rotation (amount to rotate around a given axis), which makes it easy to compute changes in orientation. For example, a key operation during IK is to rotate a joint around its local coordinate X,Y or Z axis by an incremental angle. This is easily accomplished by observing that a joint which is oriented by a quaternion is **locally** rotated by performed a post-multiplying with a rotational quaternion.
<br><br>
P.fromAngleAxis ( angle, Vector3DF(0, 1, 0) );    // where angle is a scalar, vec<0,1,0> = Y-axis<br>
Q = Q * P;         // post-multiply to perform a rotation around the **local** Y-axis of Q.<br>
Q = P * Q;         // pre-multiply to perform a rotation around the **global** Y-axis of Q.<br>
<br>
Where Q is a quaternion for local orientation, and P is a rotational quaternion. Notice the output Q is not a point but another quaternion (a new orientation).

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
