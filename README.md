# What's all this, then?

Experiments in how to write FRC subsystems.

# What are subsystems?

We divide robot behaviors into "subsystems". A subsystem is a collection of
hardware that works together to achieve a certain goal. For example:

* A drive subsystem manages the wheels of the robot and moves it around
the field in response to autonomous or driver input;

* A shooter subsystem flings a game object at a target (it might include
an intake subsystem to pick up the pieces, or that might be separate);

* An arm subsystem raises and lowers an arm (which might itself have
subsystem attached to it).

Examples:

* In 2021 we had:
  * A drive subsystem that managed the four-wheel swerve drive
  * A combined intake/shooter subsystem that picked up balls and launched them
  * Two different arm subsystems that managed arms:
    * One that could rotate to traverse some monkey bars; and,
    * One to raise/lower the robot

* In 2022 we had
  * A drive subsystem that managed the four-wheel swerve drive
  * An arm subsystem that managed an arm that could raise/lower and extend/retract
  * A claw subsystem that could clamp and release game pieces

* In 2023 we had
  * A drive subsystem that managed the four-wheel swerve drive
  * An arm subsystem that managed an arm that could raise/lower
  * A combined intake/shooter subsystem that picked up rings and launches them

In Java, when you have a few pieces of data related to one another, it's
very common to put them into a `class` together.

# Controlling subsystems

The best first step in robotics programming is to make the hardware move
according to an instruction from your code. For instance, if someone
pushes a joystick.
