// RTOS Framework - Spring 2018
// J Losh

// Student Name: Banadahalli Mallesh,Suhas
// TO DO: Add your name on this line.  Do not include your ID number.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// 02_rtos.c   Single-file with your project code
// (xx is a unique number that will be issued in class)
// Please do not include .intvecs section in your code submissions
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include <hw_nvic.h>
#include <hw_types.h>

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

#define PB0 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define PB1 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PB2 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PB3 (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define PB4 (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();  //Returns no left hand and no right hand parameters

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5

#define MAX_CHARS       50

struct semaphore
{
  uint16_t count;
  uint16_t queueSize;
  uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer
#define STATE_RUN        5 // Is Currently Running

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

uint64_t tasktime=0;        //For Time Calculations
uint64_t totaltime=0;       //

//For Shell command
bool command_entered = false;
bool valid = false;
bool values_taken = false;
bool create_thread_flag = false;
uint8_t count = 0;
uint8_t e,k,j,p,l,fields,length;
char str[MAX_CHARS];
uint8_t position[MAX_CHARS];
char type[MAX_CHARS];
uint8_t value[MAX_CHARS];


bool priority_inheritance = true;   //Flag for Priority Inheritance
bool preemption = false;            //Flag for Preemption

//Variables
uint8_t foundsem;
char num_str[10];
uint32_t *sys_stack;
uint32_t *temp1;
uint32_t temp2;
uint32_t temp3;
void *temp4;
uint16_t temp5;
uint16_t process_pid;
uint8_t q;
struct semaphore *tempSemaphore = 0;
struct semaphore *tempsvcSemaphore = 0;



struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify thread
  void *sp;                      // location of stack pointer for thread
  uint8_t priority;              // 0=highest, 7=lowest
  uint8_t currentPriority;       // used for priority inheritance
  uint32_t ticks;                // ticks until sleep complete
  char name[16];                 // name of task used in ps command
  void *semaphore;               // pointer to the semaphore that is blocking the thread
  uint8_t skip_count;            //For skipping the tasks for priority scheduling
  uint64_t start;                //For keeping track of the thread time
  uint64_t end;                  //
  uint64_t processtime;
  uint64_t percentage;
  uint64_t filtered_percentage_final;
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------


// Function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

//Function that gets the string from respective positions
char* get_string(uint8_t field)
{
    return &str[position[field]];
}

//To get the number from Character
int get_number(uint16_t z)
{
    uint16_t p;
    p = atoi(&str[position[z]]);
    return p;
}

//To display the number
void putnUart0(uint32_t number)
{
    if(number == 0)
        putsUart0("0     ");
    else
        {
        uint8_t rem=0;
        uint8_t digits=0;
        uint8_t v;
        for(v=0;v<10;v++)
            num_str[v]=0;
        while(number!=0)
            {
                rem=number%10;
                num_str[digits] = rem + '0';
                number=number/10;
                digits++;
            }
        num_str[digits] = '\0';
        for (v = (strlen(num_str)-1); ;v--)
            {
                while (UART0_FR_R & UART_FR_TXFF);
                UART0_DR_R = num_str[v];
                if(v==0)
                    break;
            }
        }
}
/*
void putnpUart0(uint32_t number)
{
    uint8_t rem=0;
    uint8_t digits=0;
    uint8_t v;
    for(v=0;v<10;v++)
        num_str[v]=0;
    while(number!=0)
    {
        rem=number%10;
        num_str[digits] = rem + '0';
        number=number/10;
        digits++;
    }
    num_str[digits] = '\0';
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = num_str[1];
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = num_str[0];
}*/

//To display floating point number
void putnfUart0(uint32_t number)
{
    uint8_t rem=0;
    uint8_t digits=0;
    uint8_t v;
    for(v=0;v<10;v++)
        num_str[v]=0;
    while(digits < 5)
        {

            if(digits==2)
                {
                    num_str[digits] = '.';
                    digits++;
                }
            rem=(uint8_t) number%10;
            num_str[digits] = rem + '0';
            number=number/10;
            digits++;
        }
    num_str[digits] = '\0';
    while (UART0_FR_R & UART_FR_TXFF);
            UART0_DR_R = num_str[4];
    while (UART0_FR_R & UART_FR_TXFF);
            UART0_DR_R = num_str[3];
    while (UART0_FR_R & UART_FR_TXFF);
            UART0_DR_R = num_str[2];
    while (UART0_FR_R & UART_FR_TXFF);
            UART0_DR_R = num_str[1];
    while (UART0_FR_R & UART_FR_TXFF);
            UART0_DR_R = num_str[0];
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
        yield();
    return UART0_DR_R & 0xFF;
}

//Function used to check whether the command and parameters entered are matching
int is_command(char* strx, int fieldx)
{
    if(strcmp(strx, &str[position[0]])==0)
    {
        if(fieldx<=fields-1)
            return 1;
    }
    return 0;
}

//Function that sets Stack Pointer to the passed argument address
void set_sp(void *a)
{  //sp subtracts 8 every time this function is called
   __asm(" ADD SP, #0x08"); //Correction. See if we can do 2 times pop lr and r3.
   __asm(" MOV SP, R0");
   __asm(" BX LR");
}

/*
void set_pc(void *a)
{
    __asm(" ADD SP, #0x08"); //Correction. See if we can do 2 times pop lr and r3.
    __asm(" MOV SP, R0");
    __asm(" BX LR");
}*/

//Function to get the Current Stack Pointer address
uint32_t* get_sp()
{
   __asm(" MOV R0, SP");
   __asm(" BX LR");
   return 0;                //Dummy, Just to remove warning.
}

void rtosInit()
{
  uint8_t i;
  // no tasks running
  taskCount = 0;
  // clear out tcb records
  for (i = 0; i < MAX_TASKS; i++)
  {
    tcb[i].state = STATE_INVALID;
    tcb[i].pid = 0;
  }
  // REQUIRED: initialize systick for 1ms system timer
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;
  NVIC_ST_RELOAD_R = 0x00009C40;
}


void rtosStart()
{   //r2,r3,r7,lr are getting pushed
  // REQUIRED: add code to call the first task to be run
  _fn fn;
  bool go_ahead = false;
  uint8_t a,count = 0;
  for(a=0;a<MAX_TASKS;a++)
      {
          if((tcb[a].state == STATE_INVALID) | (tcb[a].state == STATE_UNRUN))
              count++;
          if(count == 10)
              go_ahead = true;
      }

  if(go_ahead)
      {
          taskCurrent = rtosScheduler();
          sys_stack = get_sp();
          set_sp(tcb[taskCurrent].sp);
          fn = (_fn) tcb[taskCurrent].pid;
          (*fn)();
      }
  // Add code to initialize the SP with tcb[task_current].sp;
}

bool createThread(_fn fn, char name[], int priority)
{
    __asm("    SVC #0x0A");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm("    SVC #0x09");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    uint8_t m;
    for(m=0;m<taskCount;m++)
        if(tcb[m].pid == fn)
        {
            tcb[m].priority = priority;
            tcb[m].currentPriority = priority;
        }
}

struct semaphore* createSemaphore(uint8_t count)
{
  struct semaphore *pSemaphore = 0;
  if (semaphoreCount < MAX_SEMAPHORES)
  {
    pSemaphore = &semaphores[semaphoreCount++];
    pSemaphore->count = count;
  }
  return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
  // push registers, call scheduler, pop registers, return to new function
    __asm("    SVC #0x06");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
  // push registers, set state to delayed, store timeout, call scheduler, pop registers,
  // return to new function (separate unrun or ready processing)
    __asm("    SVC #0x05");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm("    SVC #0x07");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm("    SVC #0x08");
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{   //r2, r3, r7, lr getting pushed
  bool ok;
  static uint8_t task = 0xFF;
  ok = false;
  while (!ok)
  {
    task++;
    if (task >= MAX_TASKS)
      task = 0;
    if(tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN)
    {
        if (tcb[task].skip_count >= tcb[task].currentPriority)
        {
            ok = true;
            tcb[task].skip_count = 0;
        }
        else tcb[task].skip_count++;
    }
  }
  return task;
}


// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t b;
    temp5++;
    if(temp5 == 100)
    {
       for(b=0;b<10;b++)
       {
           tcb[b].percentage = (tcb[b].processtime)*10000/totaltime;
           tcb[b].filtered_percentage_final = 0.9*tcb[b].filtered_percentage_final + 0.1*tcb[b].percentage;
           values_taken = true;
       }
        temp5 = 0;
    }

    for(b=0;b<MAX_TASKS;b++)
    {
        if(tcb[b].state == STATE_DELAYED)
        {
            tcb[b].ticks--;
            if(tcb[b].ticks == 0)
                tcb[b].state = STATE_READY;
        }
    }

    if(preemption)
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{//pushes r3 and lr
    __asm(" ADD SP, #0x08");
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_UNPEND_SV;
    __asm(" PUSH {R4-R11}");

    tcb[taskCurrent].sp = get_sp();

    tcb[taskCurrent].end = TIMER1_TAV_R;

        if(tcb[taskCurrent].end > tcb[taskCurrent].start)
            {
            tasktime = tcb[taskCurrent].end - tcb[taskCurrent].start;
            tcb[taskCurrent].processtime = tcb[taskCurrent].processtime + tasktime;
            totaltime = totaltime + tasktime;
            }

        if(values_taken)
        {
            values_taken = false;
            for(q=0;q<MAX_TASKS;q++)
            {
            tcb[q].processtime = 0;
            tcb[q].percentage = 0;
            }
            totaltime=0;
        }

    set_sp(sys_stack);
    taskCurrent = rtosScheduler();

    TIMER1_TAV_R = 0x00000000;
    tcb[taskCurrent].start = TIMER1_TAV_R;

    if(tcb[taskCurrent].state == STATE_READY)
    {
        set_sp(tcb[taskCurrent].sp);
        __asm(" POP {R4-R11}");
    }


    if(tcb[taskCurrent].state == STATE_UNRUN)
    {
        tcb[taskCurrent].state = STATE_RUN;
        set_sp(tcb[taskCurrent].sp);

        temp3 = 0x01000000;                         //Pushing xPSR(Important)
        __asm(" PUSH {R0}");

        temp1 = (uint32_t*) tcb[taskCurrent].pid;   //Pushing Program Counter(important)
        __asm(" PUSH {R0}");

        temp2 = 0xFFFFFFF9;                         //Pushing LR
        __asm(" MOV LR,R0");
        __asm(" PUSH {LR}");

        __asm(" PUSH {R0-R3,R12}");                 //Pushing normal registers
    }

    temp2 = 0xFFFFFFF9;
    __asm(" MOV LR,R0");
    __asm(" BX LR");
}

uint32_t R0_value()
{
}

uint32_t* R1_value()
{
    __asm("    MOV R0, R1");
}

uint32_t R2_value()
{
    __asm("    MOV R0, R2");
}

uint8_t svc_value()
{
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    uint32_t R0 = R0_value();
    uint32_t* R1 = R1_value();  //Pointer type to get the char parameter in create Thread function
    uint32_t R2 = R2_value();
    __asm("    MOV R0, SP");
    __asm("    ADD R0, #64");
    __asm("    LDR R0, [R0]");
    __asm("    SUB R0, #2");
    __asm("    LDR R0, [R0]");
    uint8_t svc;
    svc = svc_value();

    switch(svc)
    {
        //SLEEP Function
        case 5:
        {
            tcb[taskCurrent].ticks = R0;
            tcb[taskCurrent].state = STATE_DELAYED;
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;
        }

        //YIELD Function
        case 6:
        {
            tcb[taskCurrent].state = STATE_READY;
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;
        }

        //WAIT Function
        case 7:
        {
            tempsvcSemaphore = (struct semaphore*) R0;
            uint8_t e;

            if(tempsvcSemaphore->count > 0)
                tempsvcSemaphore->count--;
            else
            {
                tempsvcSemaphore->processQueue[tempsvcSemaphore->queueSize] = (uint32_t) tcb[taskCurrent].pid;
                tempsvcSemaphore->queueSize++;
                tcb[taskCurrent].semaphore = tempsvcSemaphore;
                tcb[taskCurrent].state = STATE_BLOCKED;

                if(priority_inheritance)
                {
                    for(e=0;e<MAX_TASKS;e++)
                    {
                        if((tcb[e].semaphore == tempsvcSemaphore))
                            if(tcb[taskCurrent].currentPriority < tcb[e].currentPriority)
                            {
                                //Elevate current priority of i to current priority of task Current
                                tcb[e].currentPriority = tcb[taskCurrent].currentPriority;
                            }
                    }
                }

                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            }
            break;
        }

        //POST function
        case 8:
        {
            tempsvcSemaphore = (struct semaphore*) R0;
            tempsvcSemaphore->count++;
            uint8_t c;
            uint8_t d;
                if(tempsvcSemaphore->queueSize > 0)
                {
                    temp4= (void*) tempsvcSemaphore->processQueue[0];
                    tempsvcSemaphore->processQueue[0]=0;
                    tempsvcSemaphore->queueSize--;
                    tempsvcSemaphore->count--;
                    for(c=0;c<tempsvcSemaphore->queueSize;c++)
                        {
                            tempsvcSemaphore->processQueue[c]=tempsvcSemaphore->processQueue[c+1];
                        }
                    for(d=0;d<10;d++)
                        {
                            if(tcb[d].pid == temp4)
                            {
                                tcb[d].currentPriority = tcb[d].priority;
                                tcb[d].state = STATE_READY;
                            }
                        }
                    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
                }
            break;
        }

        //DESTROY THREAD
        case 9:
        {
            uint8_t x,z,t,r;
            uint32_t temp6;
            for(x=0;x<taskCount;x++)
                if(tcb[x].pid == R0)
                {
                    tcb[x].state = STATE_INVALID;
                    for(foundsem=0;foundsem<semaphoreCount;foundsem++)
                    {
                        tempSemaphore = &semaphores[foundsem];
                        for(z=0;z<(tempSemaphore->queueSize);z++)
                        {
                            if(tempSemaphore->processQueue[z] == (uint32_t) tcb[x].pid)
                                {
                                    //Remove that process from the semaphore
                                    tempSemaphore->processQueue[z] = 0;
                                    for(t=z;t<(tempSemaphore->queueSize);t++)
                                        {
                                        tempSemaphore->processQueue[t] = tempSemaphore->processQueue[t+1];
                                        }
                                    tempSemaphore->queueSize--;
                                }
                            else if(tempSemaphore == (&semaphores[3]))                                                      //Seems Big, as Important and LengthyFn share same Semaphores
                                {
                                    if((strcmp(tcb[x].name,"Important") == 0) || (strcmp(tcb[x].name,"LengthyFn") == 0))
                                    {
                                        temp6 = tempSemaphore->processQueue[z];
                                        for(r=0;r<taskCount;r++)
                                        {
                                            if(temp6 == tcb[r].pid)
                                                {
                                                tcb[r].state = STATE_READY;
                                                tempSemaphore->processQueue[z] = 0;
                                                tempSemaphore->queueSize--;
                                                }
                                        }
                                    }
                                }
                        }
                    }
                    tcb[x].pid = 0;
                    tcb[x].filtered_percentage_final=0;
                }
            break;
        }

        //Create Thread
        case 10:
        {
            bool ok = false;
            uint8_t i = 0;
            bool found = false;
            // REQUIRED: store the thread name
            // add task if room in task list
            if (taskCount < MAX_TASKS)
                {
                  // make sure fn not already in list (prevent reentrancy)
                  while (!found && (i < MAX_TASKS))
                      {
                        found = (tcb[i++].pid ==  R0);
                      }
                  if (!found)
                      {
                        // find first available tcb record
                        i = 0;
                        while (tcb[i].state != STATE_INVALID) {i++;}
                        tcb[i].state = STATE_UNRUN;
                        tcb[i].pid = R0;
                        tcb[i].sp = &stack[i][255];
                        tcb[i].priority = R2;
                        tcb[i].currentPriority = R2;
                        strcpy (tcb[i].name, R1);
                        // increment task count
                        taskCount++;
                        ok = true;
                      }
                }
            break;
            // REQUIRED: allow tasks switches again
        }
    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           5 pushbuttons, and uart
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO ports F, A, B peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB ;

    // Configure LED pins
    GPIO_PORTF_DIR_R |= 0x0E;      // make bits 3,2,1 an output
    GPIO_PORTF_DR2R_R |= 0x0E;     // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x1E;      // enable Green, Red, Blue LEDs on board
    GPIO_PORTF_PUR_R |= 0x10;       // PF4 for Push Button

    GPIO_PORTA_PUR_R |= 0x1C;
    GPIO_PORTB_PUR_R |= 0x40;


    // Configure the solder board LED pins
    GPIO_PORTA_DIR_R |= 0xE0;  // make bit 5,6,7 an output
    GPIO_PORTA_DR2R_R |= 0xE0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= 0xFC;
    GPIO_PORTB_DIR_R |= 0x10;
    GPIO_PORTB_DR2R_R |= 0x10;
    GPIO_PORTB_DEN_R |= 0x50;

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;          // configure for periodic mode (count Up)
    TIMER1_TAV_R = 0x00000000;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;

}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{ //subtracts sp by 8
                                                // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             B    WMS_LOOP1");       // 5*3
  __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    uint8_t a=0;
    if(PB0==0) a=1;
    if(PB1==0) a=a+2;
    if(PB2==0) a=a+4;
    if(PB3==0) a=a+8;
    if(PB4==0) a=a+16;
    return a;
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(1000);
    ORANGE_LED = 0;
    yield();
  }
}

void idle2()
{
    while(true)
    {
      YELLOW_LED = 1;
      waitMicrosecond(1000);
      YELLOW_LED = 0;
      yield();
    }
}

void idle3()
{
    while(true)
    {
      RED_LED = 1;
      waitMicrosecond(1000);
      RED_LED = 0;
      yield();
    }
}

void flash4Hz()
{
  while(true)
  {
    GREEN_LED ^= 1;
    sleep(125);
  }
}

void oneshot()
{
  while(true)
  {
    wait(flashReq);
    YELLOW_LED = 1;
    sleep(1000);
    YELLOW_LED = 0;
  }
}

void partOfLengthyFn()
{
  // represent some lengthy operation
  waitMicrosecond(1000);
  // give another process a chance to run
  yield();
}

void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    wait(resource);
    for (i = 0; i < 4000; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
    post(resource);
  }
}

void readKeys()
{
  uint8_t buttons;
  while(true)
  {
    wait(keyReleased);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = readPbs();
      yield();
    }
    post(keyPressed);
    if ((buttons & 1) != 0)
    {
      YELLOW_LED ^= 1;
      RED_LED = 1;
    }
    if ((buttons & 2) != 0)
    {
      post(flashReq);
      RED_LED = 0;
    }
    if ((buttons & 4) != 0)
    {
      createThread(flash4Hz, "Flash4Hz", 0);
      taskCount--;                                                    //One small Modification !Sorry.
    }
    if ((buttons & 8) != 0)
    {
      destroyThread(flash4Hz);
    }
    if ((buttons & 16) != 0)
    {
      setThreadPriority(lengthyFn, 4);
    }
    yield();
  }
}

void debounce()
{
  uint8_t count;
  while(true)
  {
    wait(keyPressed);
    count = 10;
    while (count != 0)
    {
      sleep(10);
      if (readPbs() == 0)
        count--;
      else
        count = 10;
    }
    post(keyReleased);
  }
}

void uncooperative()
{
  while(true)
  {
    while (readPbs() == 8)
    {
    }
    yield();
  }
}

void important()
{
    while(true)
    {
      wait(resource);
      BLUE_LED = 1;
      sleep(1000);
      BLUE_LED = 0;
      post(resource);
    }
}

void shell()
{
  while (true)
      {
        // REQUIRED: add processing for the shell commands through the UART here
              if(!command_entered)
              {
                if(count<MAX_CHARS)                         //This loop is to get the valid character from the user and to store it in a string
                {
                    str[count]=getcUart0();
                    while(str[0]==8)                        //To avoid negative count if the first character entered is backspace
                        str[0]=getcUart0();

                    count++;
                    if(str[count-1]==8)                       //To delete the previous character if Backspace is entered
                        {
                        str[count]=0x00;
                        str[count-1]=0x00;
                        if(count>1)
                            count=count-2;
                        else
                            count=0;
                        }

                    if(count>=45)                                                    //To show error message if the entered command is too lengthy
                        {
                            count = 0;
                            command_entered = true;
                            putsUart0("Your command is too lengthy\r\n");
                        }
                    if(str[count-1]==13)                                            //To terminate the loop if the Character Return key is pressed
                        {
                            str[count-1]=0x00;
                            count = 0;
                            command_entered = true;
                        }
                }
              }

              if(command_entered)
                  {
                        for(k=0 ; k < strlen(str) ; k++)
                            {
                                if(str[k] == 38)
                                    create_thread_flag = true;
                            }

                        for(k=0 ; k < strlen(str) ; k++)                                  //To get the positions, type and fields of the entered string
                            {
                               if(str[k] >= 48 && str[k] <= 57)
                               {
                                   position[j]=k;
                                   type[j]='n';
                                   j=j+1;
                                   while(str[k+1]>=48 & str[k+1]<=57)
                                   k=k+1;
                               }
                               if((str[k]>=65 && str[k]<=90) | (str[k]>=97 && str[k]<=122))
                               {
                                   position[j]=k;
                                   type[j]='a';
                                   j=j+1;
                                   while((str[k+1]>=65 && str[k+1]<=90) || (str[k+1]>=97 && str[k+1]<=122))
                                   k=k+1;
                                }
                             }
                            fields=j;

                        for(p=0; p < strlen(str); p++)                                    //To convert all data from Upper case to lower case
                            {
                                if(str[p]>=65 && str[p]<=90)
                                str[p]=str[p]+32;
                            }

                        for(l=0;l<=k;l++)                                                  //To pad zeros for all the delimiters
                            {
                                   if((str[l]>=48 && str[l]<=57) || (str[l]>=65 && str[l]<=90) || (str[l]>=97 && str[l]<=122))
                                       str[l]=str[l];
                                   else
                                       str[l]=0x00;
                            }

                        //COMMAND PROCESSING
                        if(is_command("ps",0))
                            {
                                uint8_t w;
                                valid = true;
                                putsUart0("\r\n     PID          NAME          CPU_PERCENTAGE          STATUS\r\n");
                                putsUart0("------------------------------------------------------------------\r\n");
                                for(e=0;e<9;e++)
                                {
                                    //PID
                                    putsUart0("     ");
                                    if(tcb[e].pid == 0) putsUart0("0000");
                                    else putnUart0((uint32_t) tcb[e].pid);
                                    putsUart0("         ");

                                    //NAME
                                    for(w=0;w<strlen(tcb[e].name);w++)
                                        putcUart0(tcb[e].name[w]);
                                    for(w=0;w<(19-strlen(tcb[e].name));w++)
                                        putsUart0(" ");

                                    //CPU_PERCENTAGE
                                    if(tcb[e].filtered_percentage_final == 0)
                                        putsUart0("00.00");
                                    else putnfUart0(tcb[e].filtered_percentage_final);
                                        putsUart0("              ");

                                    //STATUS
                                    switch(tcb[e].state)
                                    {
                                    case 0:
                                        {
                                            putsUart0("INVALID");
                                            break;
                                        }
                                    case 1:
                                        {
                                            putsUart0("UNRUN");
                                            break;
                                        }
                                    case 2:
                                        {
                                            putsUart0("READY");
                                            break;
                                        }
                                    case 3:
                                        {
                                            putsUart0("BLOCKED");
                                            break;
                                        }
                                    case 4:
                                        {
                                            putsUart0("DELAYED");
                                            break;
                                        }
                                    case 5:
                                        {
                                            putsUart0("RUN");
                                            break;
                                        }
                                    }
                                    putsUart0("\r\n");

                                }
                                putsUart0("------------------------------------------------------------------\r\n");
                            }

                        if(is_command("kill",1))
                        {
                            uint8_t y;
                            valid = true;
                            if(type[1]=='n')
                            {
                                process_pid = get_number(1);
                                for(y=0;y<taskCount;y++)
                                {
                                    if((uint32_t) process_pid == (uint32_t) tcb[y].pid)
                                    {
                                        destroyThread((_fn) tcb[y].pid);
                                        putsUart0("\r\nProcess Killed\r\n");
                                        break;
                                    }
                                    if(y == (taskCount-1))
                                        putsUart0("\r\nProcess doesn't exist\r\n");

                                }
                            }
                            else putsUart0("Enter the command with numbers as Parameters\r\n");
                        }

                        if(is_command("pidof",1))
                        {
                                putsUart0("\r\n    ");
                                if(strcmp(get_string(1),"idle")==0)
                                    {valid = true;
                                    putnUart0((uint32_t) idle);}
                                if(strcmp(get_string(1),"flash4hz")==0)
                                    {valid = true;
                                    putnUart0((uint32_t) flash4Hz);}
                                if(strcmp(get_string(1),"lengthyfn")==0)
                                    {valid = true;
                                    putnUart0((uint32_t) lengthyFn);}
                                if(strcmp(get_string(1),"oneShot")==0)
                                    {valid = true;
                                    putnUart0((uint32_t) oneshot);}
                                if(strcmp(get_string(1),"readkeys")==0)
                                    {valid = true;
                                    putnUart0((uint32_t) readKeys);}
                                if(strcmp(get_string(1),"debounce")==0)
                                    {valid = true;
                                    putnUart0((uint32_t) debounce);}
                                if(strcmp(get_string(1),"important")==0)
                                    {valid = true;
                                    putnUart0((uint32_t) important);}
                                if(strcmp(get_string(1),"shell")==0)
                                    {valid = true;
                                    putnUart0((uint32_t) shell);}
                                if(strcmp(get_string(1),"uncoop")==0)
                                    {valid = true;
                                    putnUart0((uint32_t) uncooperative);}
                                putsUart0("\r\n");
                        }

                        if(create_thread_flag)
                        {
                            uint8_t y,e;
                            create_thread_flag = false;
                            putsUart0("\r\n");
                            if(strcmp(get_string(0),"idle")==0)
                                {valid = true;
                                createThread(idle, "Idle", 7);
                                taskCount--;}
                            if(strcmp(get_string(0),"flash4hz")==0)
                                {valid = true;
                                createThread(flash4Hz, "Flash4Hz", 2);
                                taskCount--;}
                            if(strcmp(get_string(0),"lengthyfn")==0)                    //Seems Big, as Important and LengthyFn share same Semaphores
                                {valid = true;
                                for(y=0;y<taskCount;y++)
                                    {
                                        if(strcmp(tcb[y].name,"LengthyFn") == 0)
                                        if(tcb[y].state == STATE_INVALID)
                                        {
                                            for(e=0;e<taskCount;e++)
                                            {
                                                if(strcmp(tcb[e].name,"Important") == 0)
                                                if(tcb[e].state == STATE_INVALID)
                                                {
                                                    semaphoreCount--;
                                                    resource = createSemaphore(1);
                                                }
                                            }
                                        }
                                    }
                                createThread(lengthyFn, "LengthyFn", 6);
                                taskCount--;}
                            if(strcmp(get_string(0),"oneshot")==0)
                                {valid = true;
                                createThread(oneshot, "OneShot", 2);
                                post(flashReq);
                                taskCount--;}
                            if(strcmp(get_string(0),"readkeys")==0)
                                {valid = true;
                                createThread(readKeys, "ReadKeys", 6);
                                taskCount--;}
                            if(strcmp(get_string(0),"debounce")==0)
                                {valid = true;
                                createThread(debounce, "Debounce", 6);
                                taskCount--;}
                            if(strcmp(get_string(0),"important")==0)                    //Seems Big, as Important and LengthyFn share same Semaphores
                                {valid = true;
                                for(y=0;y<taskCount;y++)
                                {
                                    if(strcmp(tcb[y].name,"LengthyFn") == 0)
                                    if(tcb[y].state == STATE_INVALID)
                                    {
                                        for(e=0;e<taskCount;e++)
                                        {
                                            if(strcmp(tcb[e].name,"Important") == 0)
                                            if(tcb[e].state == STATE_INVALID)
                                            {
                                                semaphoreCount--;
                                                resource = createSemaphore(1);
                                            }
                                        }
                                    }
                                }
                                createThread(important, "Important", 0);
                                taskCount--;}
                            if(strcmp(get_string(0),"shell")==0)
                                {valid = true;
                                createThread(shell, "Shell", 4);
                                taskCount--;}
                            if(strcmp(get_string(0),"uncoop")==0)
                                {valid = true;
                                createThread(uncooperative, "Uncoop", 4);
                                taskCount--;}
                            if(!valid) putsUart0("  Thread doesn't exist");
                            else putsUart0(" Thread created successfully");
                            putsUart0("\r\n");
                        }

                        if(is_command("ipcs",0))
                        {
                            valid = true;
                            putsUart0("\r\n     SEM_NAME          QUEUE_SIZE          PROCESS_QUEUE          COUNT\r\n");
                            putsUart0("----------------------------------------------------------------------------\r\n");
                            uint8_t n;
                            for(foundsem=0;foundsem<semaphoreCount;foundsem++)
                            {
                                //SEM_NAME
                                if(foundsem == 0) putsUart0("     KEYPRESSED ");
                                if(foundsem == 1) putsUart0("     KEYRELEASED");
                                if(foundsem == 2) putsUart0("     FLASHREQ   ");
                                if(foundsem == 3) putsUart0("     RESOURCE   ");

                                tempSemaphore = &semaphores[foundsem];
                                putsUart0("           ");

                                //QUEUE_SIZE
                                putnUart0(tempSemaphore->queueSize);
                                putsUart0("                    ");

                                //PROCESS_QUEUE
                                for(n=0;n<tempSemaphore->queueSize;n++)
                                    {
                                        putnUart0((uint32_t) tempSemaphore->processQueue[n]);
                                        putsUart0(" ");
                                    }
                                putsUart0("              ");

                                //COUNT
                                putnUart0((uint32_t) tempSemaphore->count);
                                putsUart0("\r\n");
                            }
                            putsUart0("----------------------------------------------------------------------------\r\n");
                        }

                        if(is_command("pi",1))
                        {
                            if(strcmp(get_string(1),"off") == 0)
                                {
                                    uint8_t y;
                                    valid = true;
                                    priority_inheritance = false;
                                    for(y=0;y<taskCount;y++)
                                        {
                                            tcb[y].currentPriority = tcb[y].priority;
                                        }
                                    putsUart0("\r\nPriority Inheritance turned OFF\r\n");
                                }
                            else if(strcmp(get_string(1),"on")==0)
                                {
                                    valid = true;
                                    priority_inheritance = true;
                                    putsUart0("\r\nPriority Inheritance turned ON\r\n");
                                }
                        }

                        if(is_command("preempt",1))
                        {
                            if(strcmp(get_string(1),"off")==0)
                            {
                                uint8_t y;
                                valid = true;
                                preemption = false;
                                for(y=0;y<taskCount;y++)
                                {
                                    tcb[y].currentPriority = tcb[y].priority;
                                }
                                putsUart0("\r\nPreemptive Scheduling turned OFF\r\n");
                            }
                            else if(strcmp(get_string(1),"on")==0)
                            {
                                valid = true;
                                preemption = true;
                                putsUart0("\r\nPreemptive Scheduling turned ON\r\n");
                            }
                        }

                        if(is_command("reboot",0))
                        {
                            HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
                        }



                        //if(is_command("reboot",0))

                        if(!valid)
                             putsUart0("Please enter a valid command\r\n");

                        putsUart0(">");
                        valid = false;
                        command_entered = false;
                        for(e=0;e<MAX_CHARS;e++)
                              {
                                  str[e] = 0;
                                  position[e] = 0;
                                  type[e]=0;
                                  value[e]=0;
                              }
                        process_pid = 0;
                        length = 0;
                        fields = 0;
                        j=0;
                        count = 0;

                  }
      }
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
  bool ok;

  // Initialize hardware
  initHw();
  rtosInit();

  // Power-up flash
  GREEN_LED = 1;
  waitMicrosecond(250000);
  GREEN_LED = 0;
  waitMicrosecond(250000);

  putsUart0("READY  \r\n");
  putsUart0(">");
  // Initialize semaphores
  keyPressed = createSemaphore(1);
  keyReleased = createSemaphore(0);
  flashReq = createSemaphore(5);
  resource = createSemaphore(1);

  // Add required idle process
  ok =  createThread(idle, "Idle", 7); //r4,r7,lr are getting pushed?

  // Add other processes
  ok &= createThread(lengthyFn, "LengthyFn", 6);
  ok &= createThread(flash4Hz, "Flash4Hz", 2);
  ok &= createThread(oneshot, "OneShot", 2);
  ok &= createThread(readKeys, "ReadKeys", 6);
  ok &= createThread(debounce, "Debounce", 6);
  ok &= createThread(important, "Important", 0);
  ok &= createThread(uncooperative, "Uncoop", 5);
  ok &= createThread(shell, "Shell", 4);

  // Start up RTOS
  if (ok)
    rtosStart(); // never returns
  else
    RED_LED = 1;

return 0;
}

