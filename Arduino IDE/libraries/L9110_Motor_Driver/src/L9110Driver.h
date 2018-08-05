// L9110 Motor driver Library
// Templated from Paul Hanseartic Motor shield 3 code
// And of course Lady Ada original Motor shield 1 library
// By - Scott Beasley 2015. Pubplic domain. Enjoy.

#ifndef _L9110Driver_h_
#define _L9110Driver_h_

#include <inttypes.h>

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 4
#define RELEASE 8

class L9110_Motor {
public:
       L9110_Motor();
	   bool initialize(uint8_t ia_pin, uint8_t ib_pin);
	   bool run(uint8_t);
	   uint8_t getState(void);
	   uint8_t getDirection(void);

private:
	    uint8_t motor_ia, motor_ib, runstate, currentspeed;
};

#endif
