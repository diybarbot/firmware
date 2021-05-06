//
void workLoad (void) {
 unsigned long loops1 = 1000 ;
 unsigned long loops2 = 1000 ;
 unsigned long qq = 0 ;
 float t1;
 int t2;
 int t3;
 for ( long i = 0; i < loops1; i++) {
   for (long  j = 1; j < loops2; j++) {
     qq++;
     t1 = 5000.0 * i;
     t2 = 150 * 1234 * i;
     t3 = j % 554 ;
   }
 }
}
