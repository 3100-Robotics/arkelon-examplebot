Implementation of team 3100's 2026 robot Arkelon. Used as an example of the LTPS standards.

## Conventions
Subsystems (the order they should go in is this. Eg. when declaring all of the subsystems in RobotContainer):
* Hood : Shooter
* Flywheels : Shooter
* Indexer
* IntakePivot : Intake
* IntakeRoller : Intake

Some subsystems are classified under a multi-subsystem assembly, which is usually what that group of subsystems get called outside of code.
<br/><br/>
Example: The shooter is made up of flywheels and a hood mechanisim. Both of those things combined are usually referred to as the subsystem, but within the code subsystems are what people outside of code would probably call subsystems.
<br/><br/>
Example: When people usually say "the intake subsystem" they mean the roller and the pivot. Programming calls it the IntakePivot and IntakeRoller subsystems, which others would call mechanisims
<br/><br/>
Example:
The Indexer is simply the Indexer because all 3 of its motors will always be doing the same thing as each other (from a command perspective; they run different PID loops)
