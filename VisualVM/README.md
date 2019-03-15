# VisualVM

[VisualVM](https://visualvm.github.io/) is a tool that allows profiling of code while running on the RoboRIO. This allows us to view how much CPU time certain functions are consuming and helps us to optimize our code.

### Modify *build.gradle* to Enable Profiling


```
def TEAMIP = "10.1.95.2"


frcJavaArtifact('frcJava') {
	targets << "roborio"
	// Debug can be overridden by command line (./gradlew deploy -PdebugMode), for use with VSCode
	debug = frc.getDebugOrDefault(false)
	project.logger.lifecycle('Checking if debug mode...')
	if (debug) {
		project.logger.lifecycle('Debug mode enabled!')
		project.logger.lifecycle("Connect JMX client to ${TEAMIP}:1099 for RoboRIO profiling with visualvm.")
		jvmArgs = [ "-Dcom.sun.management.jmxremote=true",
					"-Dcom.sun.management.jmxremote.port=1099",
					"-Dcom.sun.management.jmxremote.local.only=false",
					"-Dcom.sun.management.jmxremote.ssl=false",
					"-Dcom.sun.management.jmxremote.authenticate=false",
					"-Djava.rmi.server.hostname=${TEAMIP}"
					]
	}
}
```


### Deploying Code With Profiling Enabled

Enable debug mode when deploying like this to enable the profiling setup
`./gradlew deploy  -PdebugMode`

### Connect to VisualVM JMX Target

In VisualVM, setup a JMX target pointing to `TEAMIP:1099`

Make sure to uncheck *Require SSL*



#### Reference:

Based on this [article](https://www.chiefdelphi.com/t/we-are-experiencing-a-lag-issue-in-our-code/350295/11)