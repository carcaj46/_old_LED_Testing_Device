# _old_LED_Testing_Device
Repository with the scripts used to make the LED tester machine.

Firmware developed for an industrial LED testing device, implemented on ESP32 and Nuvoton N76E003 microcontrollers.
The system controls motor movement, sensor feedback, manual overrides, safety limits and error handling to automate repetitive hardware testing cycles. The project focuses on reliable state management, fault detection, and safe operation under practical constraints.
This repository contains the core firmware logic used in production, including motor control, sensor handling, timing logic, and reset/recovery behaviour.

DETAILED DESCRIPTION:

LED Testing Machine

What the device was: 

An industrial LED testing machine used to repeatedly test LED units in a controlled way.
Built to automate repetitive testing cycles instead of manual checks.
Designed to be reliable, predictable, and safe.


Hardware & setup:

Microcontrollers used:

	- ESP32 for higher-level control and interfacing.
	- Nuvoton N76E003 for low-level embedded control.

Connected to:

	- Motors (forward / reverse movement).
	- Sensors and limit switches.
	- Manual buttons / overrides.

Operated in a real physical environment.


What the firmware had to do:

Control motor movement through defined sequences
React to sensor input to know when to stop, reverse, or reset
Handle manual intervention safely
Enforce safety limits so the machine couldn’t damage itself
Run continuously without getting stuck in bad states


How the logic worked:

Implemented a state-based flow:

	- Idle
	- Start
	- Forward movement
	- Reverse movement
	- Completion / reset
	- Error state

Each state had:

	- Clear entry conditions
	- Clear exit conditions
	- Timeouts to avoid infinite loops
	- Used timing logic instead of blocking delays where possible


Error handling & defensive logic

Detected:

	- Sensor not triggering when expected
	- Motor taking too long
	- Invalid sequences

On errors:

	- Stop movement
	- Reset safely
	- Prevent repeated failures

Focus was on fail-safe behaviour, not just “making it work”


Debugging & challenges

Debugging was mostly:

	- Behavioural (machine doing the wrong thing)
	- Timing-related
	- Hardware + software interaction issues

Required:

	- Testing edge cases
	- Watching how the machine behaved over time
	- Adjusting logic to handle real-world inconsistencies


What this project taught me:

	- How to reason about systems, not just write code
	- Writing logic that survives unexpected inputs
	- Defensive programming in constrained environments
	- Thinking about:
		- What happens when something goes wrong
		- How to recover cleanly
		- How to avoid cascading failures

