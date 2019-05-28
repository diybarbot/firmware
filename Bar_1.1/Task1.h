// Distance Sensor PIN
#define PWM_PIN 22
int glassDistance = 0;

void workLoad (void) ;     

void Task1( void * parameter ) {
  // Everything defined on the setup
   pinMode(PWM_PIN, INPUT);
   for (;;) {
     // Everything defined on the loop
     glassDistance = pulseIn(PWM_PIN, HIGH);
     //Serial << "Task 1 complete. Distance " << (glassDistance) << " mm."  << endl;
   }
}
