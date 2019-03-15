# Limelight Vision

We make a small hardware modification to our Limelight2 cameras in order to help us best identify the target

### LED Brightness Modification

On the LED plug-in board inside the Limelight, there are two 150Ω resistors that are feedback resistors to the two high side driver chips that control the LEDs. These are the two chips that are the [SO-8 package](https://www.diodes.com/assets/Package-Files/SO-8.pdf). The resistors are labeled *151* very close to these chips. They are of type SMD mount in an [0603 package](http://www.resistorguide.com/resistor-sizes-and-packages/).

We replace these two 150Ω resistors with 1kΩ resistors, which lowers the current flowing through our LEDs, thereby decreasing the brightness.

##### Why don't we cover some LEDs with tape instead?

We do not want to destroy the uniformity of light that is provided by having multiple LEDs across a larger area. We found from testing, that at the stock brightness configuration, going down to one LED on each side was what was required for us to reach our desired target exposure. With only one LED on each side, the target had very mottled color and was not ideal for targeting. Lowering the brightness on the entire LED array and having all LEDs on solved this issue for us.