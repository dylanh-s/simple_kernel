
#include "P6.h"

extern void main_P7();

void main_P6() {
  for( uint32_t i = 0; i < 16; i++ ) {
    pid_t pid = fork();
    if( 0 == pid ) {
      exec( &main_P7 );
    }
  }
  exit( EXIT_SUCCESS );
}
