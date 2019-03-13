# KnightOwl
Code for the 2019 FRC Season

After downloading this repo, use `./gradlew build` to download all dependencies and build the source

To generate project files for working with IntelliJ, use `./gradlew idea`

Then you can open the IntelliJ project and use all the internal gradle controls

# Feature Summary
* Robot diagnostic logging
* Robot self test mode
* Critical systems monitoring and real-time encoder and system fault detection
* Real-time communication diagnostics
* Autonomous odometry for use with path planning
* Path planning interface to generate auto paths (accurate to within two inches)
* TalonSRX, SparkMAX and other device wrappers (drivers) to extend features and simplify use
* Collision interference avoidance between all subsystems
* LED Strip Controller for message display through a morse code translator