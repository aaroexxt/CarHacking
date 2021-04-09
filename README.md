# CarHacking

Hacking my 2018 Subaru Forester by reading accelerator pedal pressure over the CAN bus!

### Project Story

This project was mostly an exercise in reverse engineering, as I had to figure out what CAN packets to send without any documentation on the manufacturer's part. This involved M2Ret (original found [here](https://github.com/collin80/M2RET), but cloned here for convenience), a library to read frames off the bus in real-time. Using M2Ret, I would compare logs of the car idling vs when I was pressing the pedal to figure out what the correct packet ID was, as well as some trial-and-error to figure out packets for "tester present" signals to wake up the bus. Lots of fun when I finally got it working!
