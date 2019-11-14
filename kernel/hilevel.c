/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

/* We assume there will be 2 user processes, stemming from the 2 user programs,
 * and so can
 *
 * - allocate a fixed-size process table (of PCBs), and then maintain an index
 *   into it to keep track of the currently executing process, and
 * - employ a fixed-case of round-robin scheduling: no more processes can be
 *   created, and neither can be terminated, so assume both are always ready
 *   to execute.
 */

pcb_t pcb[ 32 ];
pcb_t* current = NULL;
uint32_t sems[ 32 ];
uint32_t no_of_processes = 1;
uint32_t jth_scheduling = 1;
extern uint32_t tos_svc;
extern uint32_t tos_user;

void dispatch( ctx_t* ctx, pcb_t* prev, pcb_t* next ) {
  char prev_pid = '?', next_pid = '?';


  if( NULL != prev ) {
    memcpy( &prev->ctx, ctx, sizeof( ctx_t ) ); // preserve execution context of P_{prev}
    prev_pid = '0' + prev->pid;
  }

  if( NULL != next ) {
    memcpy( ctx, &next->ctx, sizeof( ctx_t ) ); // restore  execution context of P_{next}
    next_pid = '0' + next->pid;
  }

    PL011_putc( UART0, '[',      true );
    PL011_putc( UART0, prev_pid, true );
    PL011_putc( UART0, '-',      true );
    PL011_putc( UART0, '>',      true );
    PL011_putc( UART0, next_pid, true );
    PL011_putc( UART0, ']',      true );
    current = next;                             // update   executing index   to P_{next}
  return;
}

uint32_t choose_priority() {
  uint32_t next = 0;
  for (uint32_t i = 1; i <= no_of_processes; i++) {
      if (pcb[ i ].status == STATUS_READY && pcb[ i ].priority > pcb[ next ].priority) {    //if higher priority
        next = i;
      }
      else if ((pcb[ i ].status == STATUS_READY) && (pcb[ i ].priority == pcb[ next ].priority) && (pcb[ i ].last_accessed < pcb[next].last_accessed)) {   //if last access was earlier
        next = i;
      }
  }
  return next;
}

uint32_t find_next_available_pcb() {
    for (uint32_t i = 0; i < no_of_processes; i++) {
        if (pcb[i].status == STATUS_TERMINATED) {
            return i;
        }
    }
    return no_of_processes;
}

void schedule_n(ctx_t* ctx){
  uint32_t next = choose_priority();
  jth_scheduling += 1;
  if (current->pid != next) { //do not switch current process into itself
      if (current->status == STATUS_EXECUTING) {
        pcb[ current->pid ].status = STATUS_READY;
      }
      dispatch( ctx, &pcb[ current->pid ], &pcb[ next ] );      // context switch current process -> next process
      pcb[ next ].status = STATUS_EXECUTING;         // update   execution status  of next process
  }
  current->last_accessed = jth_scheduling;
  return;
}

void hilevel_exit(ctx_t* ctx) {
    PL011_putc( UART0, 'X',     true );
    PL011_putc( UART0, current->pid+'0',      true );
    no_of_processes -= 1;
    current->status = STATUS_TERMINATED;
    schedule_n(ctx);
    return;
}

void hilevel_kill(ctx_t* ctx, uint32_t pid) {
    PL011_putc( UART0, 'K',      true );
    PL011_putc( UART0, pid+'0',      true );
    no_of_processes -= 1;
    pcb[pid].status = STATUS_TERMINATED;
    schedule_n(ctx);
    return;
}

void hilevel_yield(ctx_t* ctx) {
    PL011_putc( UART0, 'Y',      true );
    PL011_putc( UART0, current->pid+'0',      true );
    current->last_accessed = jth_scheduling;
    schedule_n( ctx ); //call scheduler with new process
    return;
}

void hilevel_exec(ctx_t* ctx, uint32_t start) {
  PL011_putc( UART0, 'e',      true );
  jth_scheduling += 1;
  //set pc of current context to value passed from gpr[0]
  ctx->pc = (uint32_t) (start);
  //reset stack pointer
  ctx->sp = current->tos;
  //set stack to 0
  memset(current->tos - 0x1000, 0, 0x1000);
  current->last_accessed = jth_scheduling;
  return;
}

void hilevel_fork(ctx_t* ctx ) {
  uint32_t n = find_next_available_pcb();
  PL011_putc( UART0, 'f',      true ); //fork from
  PL011_putc( UART0, current->pid+'0',      true ); //parent PID
  PL011_putc( UART0, '~',      true ); //into
  PL011_putc( UART0, n+'0',      true ); //child PID
  uint32_t tos_Pnew = (uint32_t) &tos_user - (0x00001000 * n); //calculate new top of stack for child process
  pcb_t* parent = current;
  pcb_t* child = &pcb[n];
  no_of_processes += 1;
  jth_scheduling += 1;
  //copy stack of parent
  memcpy(child->tos - 0x1000, parent->tos - 0x1000, 0x1000); //copy parent stack to child stack
  memcpy( child, parent, sizeof( pcb_t ) ); //copy pcb of parent process into pcb of next process
  memcpy( &child->ctx, ctx, sizeof (ctx_t) ); //copy ctx passed in (parent process' pc will change as is currently executing so has to be parameter ctx)
  uint32_t parent_offset = parent->tos - ctx->sp;
  child->pid      = n;
  child->status   = STATUS_READY;
  child->last_accessed = jth_scheduling;
  child->tos = tos_Pnew;
  child->ctx.sp          = child->tos - parent_offset;
  child->parent_pid      = parent->pid;
  //set r0 = 0 === returning 0 for child process;
  child->ctx.gpr[0] = 0;
  //set parent gpr[0] to child PID, returning > 0 for parent process
  ctx->gpr[0] = child->pid;
  return;
}

extern uint32_t main_console();

void hilevel_handler_rst( ctx_t* ctx              ) {

  //load console
  memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );     // initialise 0-th PCB = P_1
  pcb[ 0 ].pid      = 0;
  pcb[ 0 ].status   = STATUS_CREATED;
  pcb[ 0 ].ctx.cpsr = 0x50;
  pcb[ 0 ].priority = 5;
  pcb[ 0 ].last_accessed = 0;
  pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
  pcb[ 0 ].tos      = ( uint32_t )( &tos_user) ;
  pcb[ 0 ].ctx.sp   = pcb[ 0 ].tos;
  pcb[ 0 ].parent_pid = -1;

  TIMER0->Timer1Load  = 0x000100000; // select period = 2^20 ticks ~= 1 sec
  TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
  TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
  TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
  TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

  GICC0->PMR          = 0x000000F0; // unmask all            interrupts
  GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
  GICC0->CTLR         = 0x00000001; // enable GIC interface
  GICD0->CTLR         = 0x00000001; // enable GIC distributor
  for (int i = 0; i < 32; i++) { //initialise semaphore array for chopsticks to 1s
    sems[i] = 1;
  }

  jth_scheduling += 1;
  uint32_t start = choose_priority(); //will always == 0 as only one process running
  dispatch( ctx, NULL, &pcb[ start ] );

  return;
}

void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {
  /* Based on the identifier (i.e., the immediate operand) extracted from the
   * svc instruction,
   *
   * - read  the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call, then
   * - write any return value back to preserved usr mode registers.
   */
  switch( id ) {
    case 0x00 : { // 0x00 => yield()
      hilevel_yield(ctx);
      break;
    }

    case 0x01 : { // 0x01 => write( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        PL011_putc( UART0, *x++, true );
      }

      ctx->gpr[ 0 ] = n;

      break;
    }

    case 0x02 : { // 0x02 => read(fd, x, n); reads n bytes into x from the file descriptor fd; return bytes read

    }

    case 0x03 : { //=> fork call
       hilevel_fork(ctx);
       break;
    }

    case 0x04 : { //=> exit call
      hilevel_exit(ctx);
      break;
    }

    case 0x05 : { //=> exec call
      uint32_t start = ( uint32_t )( ctx->gpr[0] );
      hilevel_exec(ctx, start);
      break;
    }

    case 0x06 : { //=> kill call
      uint32_t pid = (uint32_t)(ctx->gpr[0]);
      hilevel_kill(ctx, pid);
      break;
    }

    case 0x07 : { //=> nice call
      uint32_t pid = (uint32_t)(ctx->gpr[0]);
      uint32_t pri = (uint32_t)(ctx->gpr[1]);
      pcb[pid].priority = pri;
      break;
    }
    case 0x08 : { // => sem_wait
      //calculate left and right semaphor indices
      uint32_t indexL = ((current->pid - current->parent_pid) - 1)%16;
      uint32_t indexR = (current->pid - current->parent_pid)%16;
      if (sems[indexL] > 0 && sems[indexR] > 0) {
        /*
        PL011_putc( UART0, 'U',      true );
        PL011_putc( UART0, indexL+'0',      true );
        PL011_putc( UART0, 'U',      true );
        PL011_putc( UART0, indexR+'0',      true );
        */
        PL011_putc( UART0, 'U',      true );
        PL011_putc( UART0, current->pid + '0',      true );
        sems[indexL]--;
        sems[indexR]--;
        ctx->gpr[0] = (uint32_t) 1;
        return;
      }
      ctx->gpr[0] = (uint32_t) 0;
      return;
      break;
    }
    case 0x09 : { // => sem_post
      //drop both sem[(current->parent_pid - current->pid)] and sem[(current->parent_pid - current->pid) + 1]
      //calculate left and right semaphor indices
      uint32_t indexL = ((current->pid - current->parent_pid) - 1)%16;
      uint32_t indexR = (current->pid - current->parent_pid)%16;
      if (sems[indexL] == 0 && sems[indexR] == 0) {
        PL011_putc( UART0, 'D',      true );
        PL011_putc( UART0, current->pid + '0',  true );
        sems[indexL]++;
        sems[indexR]++;
      }
      break;
    }
    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

  return;
}


void hilevel_handler_irq(ctx_t* ctx) {
  uint32_t id = GICC0->IAR;
  if( id == GIC_SOURCE_TIMER0 ) {
    schedule_n(ctx);
    TIMER0->Timer1IntClr = 0x01;
  }
  GICC0->EOIR = id;
  return;
}
