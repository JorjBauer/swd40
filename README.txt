At the moment, there are three Arduino boards in play in this system.

The first is the remote control - the "sender", contained in
arduino_dalek_sender. This has a low-quality analog joystick, buttons
for left/right head control, and an on-off switch. It's an Arduino Pro
Mini 5v, running with an RFM12B, powered by a SparkFun PowerCell and
lithium-ion battery. Rechargable by MicroUSB.

The second is the receiver in the Dalek - arduino_dalek_receiver. This
is an Arduino Uno coupled to a Pololu motor shield, which has been
modified to not conflict with using the RFM12B. The receiver
interprets the received commands, actuates the drive motors, and
signals the tertiary Arduino for other functions via two digial pins.

The protocol between the sending and receiving RF units is described
in protocol.txt. The remote control is the arbitor of what the
joystick position means, converting that in to intended motor
movements; the receiver is the final authority on how that is
translated in to actual motor PWM speeds.

The third is the shoulder motor driver. It's a third-party Arduino Uno
clone, driving another Pololu motor shield. It receives pulses from
the receiver board on two digital pins, telling it to perform left and
right rotation of the shoulder motor.
