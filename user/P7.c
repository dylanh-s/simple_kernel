
#include "P7.h"

void main_P7() {
  while (1) {
    while (sem_wait(0) == 0) {
    }
    //P3 code used for 'eating'
    int i = 0;
    while( i<40 ) {
      //write( STDOUT_FILENO, "P7", 2 );
      uint32_t lo = 1 <<  8;
      uint32_t hi = 1 << 20;

      for( uint32_t x = lo; x < hi; x++ ) {
        uint32_t r = weight( x );
      }
      i++;
    }
    sem_post(0);
    yield();
  }
}
