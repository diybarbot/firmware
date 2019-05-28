// L9110 Motor driver Library
// Templated from Paul Hanseartic Motor shield 3 code
// And of course Lady Ada original Motor shield 1 library
// By - Scott Beasley 2015. Pubplic domain. Enjoy.

#include "L9110Driver.h"
#include "Arduino.h"

L9110_Motor::L9110_Motor() {}

bool L9110_Motor::initialize(uint8_t ia_pin, uint8_t ib_pin) {
 	motor_ia = ia_pin;
	motor_ib = ib_pin;
	pinMode(motor_ia, OUTPUT);
	pinMode(motor_ib, OUTPUT);
	return true;
}

bool L9110_Motor::run(uint8_t cmds) {
	if (((FORWARD | BACKWARD) & cmds) == (FORWARD | BACKWARD)) {
		return false;
	}
	if (((BRAKE | RELEASE) & cmds) == (BRAKE | RELEASE)) {
		return false;
	}
	if ((cmds & FORWARD) == FORWARD) {
        digitalWrite(motor_ia, HIGH);
		digitalWrite(motor_ib, LOW);
		runstate = (runstate & (BRAKE | RELEASE)) | FORWARD;
	}
	if ((cmds & BACKWARD) == BACKWARD) {
        digitalWrite(motor_ib, HIGH);
		digitalWrite(motor_ia, LOW);
		runstate = (runstate & (BRAKE | RELEASE)) | BACKWARD;
	}
	if ((cmds & RELEASE) == RELEASE) {
  	   if ((runstate & FORWARD) == FORWARD) {
            digitalWrite(motor_ia, HIGH);
		    digitalWrite(motor_ib, LOW);
  	   }

       if ((runstate & BACKWARD) == BACKWARD) {
            digitalWrite(motor_ib, HIGH);
		    digitalWrite(motor_ia, LOW);
        }

        runstate = (runstate & (FORWARD | BACKWARD)) | RELEASE;
	}

	if ((cmds & BRAKE) == BRAKE) {
         digitalWrite(motor_ia, LOW);
         digitalWrite(motor_ib, LOW);
         runstate = (runstate & (FORWARD | BACKWARD)) | BRAKE;
	}

	return true;
}

uint8_t L9110_Motor::getState(void) {
	return runstate & (BRAKE | RELEASE);
}

uint8_t L9110_Motor::getDirection(void) {
	return runstate & (FORWARD | BACKWARD);
}
