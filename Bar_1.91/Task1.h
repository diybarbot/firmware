// Distance Sensor PIN
#define PWM_PIN 22
int glassDistance = 0;
int messurment = 0;
int aux = 0;

void workLoad (void) ;     

void Task1( void * parameter ) {
  // Everything defined on the setup
   pinMode(PWM_PIN, INPUT);
   for (;;) {
     // Everything defined on the loop
     messurment = pulseIn(PWM_PIN, HIGH);
     if ((messurment < minGlassDistance && messurment > 0) || (messurment > minGlassDistance && aux > 1)) {
        glassDistance = messurment;
        aux = 0;    
     } else if (glassDistance > 0) {
        aux++;
     }
     delay(100);
   }
}
