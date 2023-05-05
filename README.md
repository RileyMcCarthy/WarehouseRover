# Autonomous Sorting Mobile Robot

University of Guelph 4th year project by: Riley McCarthy, Owen Brook, Kaden Penner, Charles Peters, and Nicholas Virdiramo 

## Introduction

With the new Amazon fulfilment center being built in Cambridge, the need for automation
is ever increasing. As the number of outbound packages increase, so will the number of
returns that need to be processed. This is one example of where using automation will allow
processes to become more streamlined and efficient. Our project demonstrates the ability
of these robots in a small scale environment. We designed and developed an autonomous
sorting robot that takes packages from a starting location, identifies it, and then moves it to
the appropriate final location.

This will help the efficiency of large factories as the need for humans to move parts around
the building will be eliminated. With the new Amazon warehouse being 136,000 sqft, small
electric robots moving packages will be more efficient when compared to humans needing to
walk around.

## Objectives

The team aimed to develop an autonomous robot that sorted multiple package variations
between unique locations one at a time. The robot would receive an input from a package
at the starting location and determine the desired final location. Then autonomously would
deliver the package to the correct destination. A block diagram of the robots functions is
shown below.

**Insert block diagrm ref**

The system is completely automated, with the only external input being the placement of
the package in the starting location by a human. This mimicked the use of a conveyor belt
for accurate package delivery to the initial location. Our initial goal was to a system that
could handle three different package shapes with weight variations. This would ensure the
robot was capable of a wide verity of packages shapes and sizes, as well as a range of different
weights. Each package would have it’s own unique destination determined by an RFID sensor
to mimic a scanner being able to identify the incommoding packages and determining their
final locations.

## Mechanical Design

As the goal of the project is to both pickup and move packages, the mechanical design
needed to allow for both a movable main base, the ability to interact with the different
package sizes, and contain all the necessary parts to make the robot autonomous such as a
battery, sensors, and a micro-controller.
Shown below is the initial design of the Autonomous Sorting Mobile Robot.
While the specifics of the different components will be covered later, the motors and
ultrasonic sensors are shown, as well as the wheels, gripper mechanism, and main base of the
robot.

**Figure 2: Robot Isometric View**

The largest part of the robot is the main base. This houses the four
driving motors, two ultrasonic sensors mounts, the control system electronics, and a base for
the grippers to be fastened. Initially the base was designed to be very compact, but this
limited space under for the electronics and motors so the base was increased in size slightly
to accommodate with out overly compromising the compact design This based is designed
an manufactured to provide a strong, square foundation for the robot to operate from. The
strength of the base allowed for the robot to handle larger packages then original considered.
The square base allows for the mecanum wheels to operate as intended and no adjustment
in software needed for straight driving.

**Figure 3: Robot Base with Drive Motors, Sensors and Control System**

The gripper mechanism is driven by two motors. The first is to open and close the grippers,
while the other allows for tilting gripper assembly to lift the package off the ground. The
RFID sensor is mounted to the gripper mechanism at the base of the arms to allow for
the robot to read the package. This allows the control system to determine the correct
destination.

**Figure 4: Gripper System with RFID Sensor**

The grippers are designed to allow for the manipulation of various package sizes to be
interacted with. The initial design is shown in **Figure 4**. The design was tested and found
not to have adequate holding power for the different shapes and sizes. With the fingers of
the arms in an arc shape they only provided one point of contact and resulted in the robot
dropping packages. The arms were also found to be short not allowing enough room for the
fingers to contact he center of the package. These issues were discovered through testing,
and the solution is explained in Section 2.3.

A rack and pinion gripper design was also tested and compared to
look at the load capacity of our designs. This design is secondary and serves as a backup to
our main gripper assembly. This design may be beneficial because it is more compact and
requires less moving parts, resulting in greater grip strength. However, the rack may seize
due to friction between the rack and slide plate when the gripper is under load. For these
reasons, this gripper was not used on the final robot.

**Figure 5: Rack and Pinion Gripper**

## Fabrication

In order to physically build the robot, all the designed components were fabricated using
a 3D printer. This method was chosen as it allows for a wide range of part geometry and
features, with rapid prototyping. The group had direct access to four 3D printers, so the
fabrication of the components was not a problem. Other methods were considered but with
the accessibility and range of builds the 3D printer was chosen
Since we printed all of our parts, material selection was greatly simplified as we could
only print plastics. Furthermore, the forces involved with this robot were low enough where
material strengths did not limit the function of the robot. Plastic is also beneficial as it is
easily manipulated to fit together. Our design contains interference fits (bearings and gears)
that rely on friction and a high normal force to maintain their position. We used heat to
expanded and soften the plastic to fit the components together. Many components were also
filed and sanded after they were printed to remove imperfections from the print, such as
strings, blobs, and support material. This was especially true for parts where a precise fit
was needed or printing orientation effected quality.
Most components were printed using PLA as it is the easiest and most reliable print
material available. PETG, a more dense plastic, was also used for additional strength, for
example, the tilt shaft. The load bearing components were fabricated with 5 layers of material
on the outside surfaces for additional strength. The rest of the parts only required 2-3 layers.
Furthermore, 20-30 percent infill was used on most parts but was varied for based on the
components needs.

**Figure 6: Cura Slicer Print Estimate**

We got material estimates from Ultimaker Cura, the 3D printing slicer. This software allowed the team to understand and control the fabrication of each part.
Adjusting prints based on their geometry, fit, and function allowed for best performance. An
example of the importance of this is the fabrication of the tilt shaft. The shaft needed a
strong and circular geometry. These qualities conflict in the printing orientation, horizontal
for strength and vertical for circular geometry. This is because when it is printed horizontally
it has continuous strings of plastic along its length, increasing strength against bending.
When printed vertically the print is able to precisely outline the circular geometry and avoid
overhanging. Ultimately horizontally was chosen as the strength is more important and the
circular geometry can be adjusted by sanding after. Below shows supports under the step
in the Tilt Shaft.

**Figure 7: Cura Slicer Shaft Print Estimate**

The proposed design from report one (shown in **Figure 8**) was fabricated, assembled, and
tested. The initial testing showed that the gripper fingers were limited in their ability to pick
up the required variety of packages.

**Figure 8: Original Gripper Design**

The final gripper design was an iteration of the first design, improving on all the issues
that caused reliability problems. In the final design (shown in **Figure 9**), the arms and fingers
were redesigned to have longer reach and a pivoting crescent finger for dual point contact.
These worked flawlessly in picking up and holding the package during transportation for
several packages all with different sizes, geometry, and weight.

**Figure 9: Updated Gripper Design**

Additional mechanical testing was done to test the robot’s grip strength and ability to raise
and lower the package. Deflection of the gripper top plate was observed when the gripper
fingers closed. This is due to the torque generated by the gripper motor during package pick
up. The top plate was reinforced and reprinted to prevent failure.

## Motors and Actuators

3.1.1 Four DC Motors with Encoders
As the design places a priority on movement accuracy rather than speed, motors were
selected that contained rotary encoders, allowing for accurate tracking of the rotation of
each wheel. This allowed for a control system algorithm to be created that ensure accurate
movement. Motion planning was also incorporated ensuring adequate traction is always
maintained by limiting the acceleration of the robot’s velocity. The encoders present in these
motors contain 140 ticks per revolution. The motors were been tested to max out at 190 rpm
while running at 12v.
3.1.2 Two DC Motors with Gearboxes
In order to provide movement to the grippers, motors were used to manipulate the grippers.
To ensure the ability to manipulate packages of different sizes and weights, these motors
required sufficient torque which was provided by the gearbox. Tests were conducted to
calculate the amount of torque they were capable of providing when run at 12v. This was
determined to be 0.583 Nm of torque. These motors were controlled with the amount of
current being drawn and were programmed to hold their position when open, or a package
was sufficiently held.
3.1.3 Motor Drivers
These motors will be driven by an L293 motor driver[1]. These drivers allowed for the
motors to be speed-controlled by using pulse wave modulation (PWM). This also allowed for
the direction to be adjusted using the microcontroller.
8
GROUP 3 A.S.M. Robot
3.2 Sensors
3.2.1 Ultrasonic Sensors
To ensure accurate positioning was maintained over time, an ultrasonic sensor was used to
re-home the robot upon its return to the starting/pick-up location. This allowed the robot
to determine its current location, and adjust to ensure location error didn’t accumulate over
time. An HC-SR04 ultrasonic sensor[2] was used as it had been used by the group before,
and are easily available. It had sufficient accuracy to ensure the robot could return to the
desired location between the delivery of each package. The ultrasonic sensor has a large
range of 2cm to 400cm and an accuracy of 2mm which was adequate accuracy and range for
the purpose of our system.
3.2.2 RFID Sensor
In order to determine the final destination for each individual package, a Radio Frequency
Identification Device (RFID) was decided on. While in the future implementation, a more
widely used technology could be used. For testing, the RFID system was easy to implement,
and the component required belong to members of the group. The specific sensor used was
a MFRC522[3].
3.3 Micro-Controller
3.3.1 Arduino Mega
The microcontroller selected is the Arduino Mega, this controller was selected due to its
high amount of general-purpose input/output (GPIO) pins. Due to our limited budget, it
was also one of the few microcontrollers we had access to. Our other option, the Arduino Uno
was not selected because it would not have enough GPIO pins for all our peripherals. The
Arduino platform was convenient because it has pre-done libraries for both the Ultrasonic
sensors and the RFID sensor. It also has analog pins that were used for current sensing on
our gripper motors to determine when a package was being handled.
3.3.2 Power System
The system power was provided by a 12v LiPo battery which was connected to the motor
drivers in order to provide the required 12 volts. The voltage was also routed through a 5v
linear regulator to create the 5v needed to power the Arduino Mega, the motor driver, and
the sensors
