// *************os.c**************
// EE445M/EE380L.6 Labs 1, 2, 3, and 4 
// High-level OS functions
// Students will implement these functions as part of Lab
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 
// Jan 12, 2020, valvano@mail.utexas.edu


#include <stdint.h>
#include <stdio.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/Timer4A.h"
#include "../inc/Timer5A.h"
#include "../inc/WTimer0A.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../inc/ADCT0ATrigger.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../inc/Timer1A.h"
#include "../inc/Switch.h"
#include "../inc/Timer3A.h"

// Performance Measurements 
int32_t MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
uint32_t const JitterSize=JITTERSIZE;
uint32_t JitterHistogram[JITTERSIZE]={0,};
uint32_t Mstime = 0;


typedef	struct	tcb	tcbType; 
tcbType	threadPool[NUMTHREADS]; 
tcbType	*RunPt; 
tcbType *SleepNextRunPt;
tcbType *LatestThreadCreated;
long	Stacks[NUMTHREADS][STACKSIZE];
uint32_t thread_counter = 0;

tcbType* SleepThreadPt; //linked list of all threads asleep

uint32_t sleepThreadCount;	//tracks the number of threads asleep
void StartOS(void); 
void ContextSwitch(void); 
void OS_ClearTCBAndStack(void);
void OS_AddToQueue(uint32_t sleepTime);
void OS_CheckQueue(void);
void OS_WakeUpThread(tcbType* awokenThread);




/*------------------------------------------------------------------------------
  Systick Interrupt Handler
  SysTick interrupt happens every 10 ms
  used for preemptive thread switch
 *------------------------------------------------------------------------------*/

void ContextSwitch(void){
	NVIC_INT_CTRL_R = 0x10000000; // trigger PendSV
}



void SysTick_Handler(void) {
	//before we context switch, check if any threads need waking up 
		//Mstime+=2;	
		
		OS_CheckQueue();
		PF1^=0x02;
		ContextSwitch(); 
} 

unsigned long OS_LockScheduler(void){
  // lab 4 might need this for disk formating
  return 0;// replace with solution
}
void OS_UnLockScheduler(unsigned long previous){
  // lab 4 might need this for disk formating
}


void SysTick_Init(unsigned long period){
  STCTRL = 0; //disable timer
	STCURRENT = 0; //clear current val
	SYSPRI3 =(SYSPRI3&0x00FFFFFF)|0xC0000000; // priority 7
	STRELOAD = period - 1; // reload value
	STCTRL = 0x00000007; // enable, core clock and interrupt arm
}

/**
 * @details  Initialize operating system, disable interrupts until OS_Launch.
 * Initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers.
 * Interrupts not yet enabled.
 * @param  none
 * @return none
 * @brief  Initialize OS
 */
void OS_Init(void){
  // put Lab 2 (and beyond) solution here
	DisableInterrupts();
	PLL_Init(Bus80MHz);
	LaunchPad_Init();
	
	SleepThreadPt = NULL;
	sleepThreadCount = 0;
	LatestThreadCreated = NULL;
	
	
    uint8_t threadcount = 0;

    for(uint32_t tcbindex = 0; tcbindex < NUMTHREADS; tcbindex++){
        threadPool[tcbindex].nextTCB = NULL;
				threadPool[tcbindex].prevTCB = NULL;
        threadPool[tcbindex].sp = NULL;
				threadPool[tcbindex].status = 1; //set status to dead
    }

    RunPt = NULL;
		*((volatile uint32_t*)0xE000ED20) |= (0xFF << 16);	// set pendsv priority to lowest
};

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, int32_t value){
  // put Lab 2 (and beyond) solution here
	semaPt->Value = value; //set sema4 value
}; 

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
  DisableInterrupts();

	while(semaPt->Value <= 0){
			EnableInterrupts();
			//OS_Suspend(); 
			DisableInterrupts();
	}

			semaPt->Value--; 
			EnableInterrupts();
}; 

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
		long status;
		status = StartCritical(); 
    semaPt->Value++; 
    EndCritical(status);
}; 

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
		DisableInterrupts();

    while(semaPt->Value <= 0){
        EnableInterrupts();
        OS_Suspend(); 
        DisableInterrupts();
    }

        semaPt->Value = 0; 
        EnableInterrupts();
}; 

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
		long status;
		status = StartCritical(); 
    semaPt->Value = 1; 
    EndCritical(status);
}; 


//derived from textbook

void SetInitialStack(uint32_t tid){
	Stacks[tid][STACKSIZE - 1] = 0x01000000; //thumb bit
	//PC not pushed... will be pushed later
	Stacks[tid][STACKSIZE - 3] = 0x14141414; //R14 
	Stacks[tid][STACKSIZE - 4] = 0x12121212; //R12
	Stacks[tid][STACKSIZE - 5] = 0x03030303; //R3
	Stacks[tid][STACKSIZE - 6] = 0x02020202; //R2
	Stacks[tid][STACKSIZE - 7] = 0x01010101; //R1
	Stacks[tid][STACKSIZE - 8] = 0x00000000; //R0
	Stacks[tid][STACKSIZE - 9] = 0x11111111; //R11
	Stacks[tid][STACKSIZE - 10]= 0x10101010; //R10
	Stacks[tid][STACKSIZE - 11]= 0x09090909; //R9
	Stacks[tid][STACKSIZE - 12]= 0x08080808; //R8
	Stacks[tid][STACKSIZE - 13]= 0x07070707; //R7
	Stacks[tid][STACKSIZE - 14]= 0x06060606; //R6
	Stacks[tid][STACKSIZE - 15]= 0x05050505; //R5
	Stacks[tid][STACKSIZE - 16]= 0x04040404; //R4
}


//** OS_AddThread ** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(task)(void), uint32_t stackSize, uint32_t priority){
    //find unused thread
	long status = StartCritical();
	uint32_t stackindex = 0;
	if(thread_counter==NUMTHREADS){
		EndCritical(status);
		return 0;
	}
    tcbType* freethread = NULL;
    if(RunPt == NULL){        //means that one thread hasnt been initialized
			freethread = &threadPool[0];
			freethread->prevTCB = freethread;
      freethread->nextTCB = freethread;
			RunPt = freethread;
			LatestThreadCreated = freethread;
    }
    else{
        //use the thread counter to start the process of searching through the array of threads
        for(uint32_t threadindex = 0; threadindex < NUMTHREADS; threadindex++){
            if(threadPool[threadindex].status ==1){
                freethread = &threadPool[threadindex];
								stackindex = threadindex;
                break;
            }
        }
        //Bottom two lines help create circular linked list
				
				freethread->prevTCB = LatestThreadCreated;
        freethread->nextTCB = LatestThreadCreated->nextTCB;
				LatestThreadCreated->nextTCB->prevTCB = freethread;
				LatestThreadCreated->nextTCB = freethread;
				LatestThreadCreated = freethread;
    }

    freethread->tid = thread_counter;

    freethread->sleep_counter = 0;

    //initialize unused stack from pre-defined buffer

    freethread->sp = &Stacks[stackindex][STACKSIZE-16];

    SetInitialStack(stackindex); 
    Stacks[stackindex][STACKSIZE-2] = (int32_t)(task);    //    PC
		
		freethread->status = 0; //set the status to ready

    thread_counter++;
		
		EndCritical(status);
	
  return 1; // replace this line with solution
};



//******** OS_AddProcess *************** 
// add a process with foregound thread to the scheduler
// Inputs: pointer to a void/void entry point
//         pointer to process text (code) segment
//         pointer to process data segment
//         number of bytes allocated for its stack
//         priority (0 is highest)
// Outputs: 1 if successful, 0 if this process can not be added
// This function will be needed for Lab 5
// In Labs 2-4, this function can be ignored
int OS_AddProcess(void(*entry)(void), void *text, void *data, 
  unsigned long stackSize, unsigned long priority){
  // put Lab 5 solution here

     
  return 0; // replace this line with Lab 5 solution
}


//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
uint32_t OS_Id(void){
  // put Lab 2 (and beyond) solution here
  
  return 0; // replace this line with solution
};


//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In lab 1, this command will be called 1 time
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), 
   uint32_t period, uint32_t priority){
  // put Lab 2 (and beyond) solution here 
	
  Timer3A_Init(task, period, priority);
	
     
  return 1; // replace this line with solution
};


/*----------------------------------------------------------------------------
  PF1 Interrupt Handler
 *----------------------------------------------------------------------------*/
//void GPIOPortF_Handler(void){

//}



//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here
//	if(thread_counter == NUMTHREADS){
//		return 0; 
//	}
	Switch_Init(task, priority);  //init switch interupts with user task
  return 1; // replace this line with solution
};

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here
    
  return 0; // replace this line with solution
};


// ** OS_Sleep **
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  // put Lab 2 (and beyond) solution here

    long status = StartCritical(); 
	
		OS_AddToQueue(sleepTime);

    EndCritical(status);
	OS_Suspend();
	
};

//this function will add a thread to the sleeping queue
void OS_AddToQueue(uint32_t sleepTime){
	
	tcbType* newSleepThread = RunPt;
	
	if(LatestThreadCreated == RunPt){
		if(RunPt->nextTCB == RunPt){ //checks if there is only one element in the linked list left
				LatestThreadCreated = NULL;
			}
		else{
			LatestThreadCreated = RunPt->prevTCB;
		}
	}
	//now alter the flag in threadpool of this variable
	newSleepThread->status = 2; //set the flag to the sleep flag
	
	//now alter the linked list in the array
	newSleepThread->prevTCB->nextTCB = newSleepThread->nextTCB;
	newSleepThread->nextTCB->prevTCB = newSleepThread->prevTCB;
	
	SleepNextRunPt = newSleepThread->nextTCB;

	
	if(sleepThreadCount == 0){
		SleepThreadPt = newSleepThread;
		SleepThreadPt->nextTCB = SleepThreadPt;
		SleepThreadPt->prevTCB = SleepThreadPt;
	}
	else{
		tcbType* current = SleepThreadPt;
		while(current->nextTCB != SleepThreadPt){ //traverse to end of linked list
			current = SleepThreadPt->nextTCB;
		}
		//now add new sleep thread to the linked list 
		newSleepThread->nextTCB = current->nextTCB;
		newSleepThread->prevTCB = current;
		current->nextTCB = newSleepThread;
		newSleepThread->nextTCB->prevTCB = newSleepThread;	
	}
	
	
	//sleep counter adjustment
	uint32_t callTime = Mstime; //get OS time when sleep is called
	newSleepThread->sleep_time = callTime; //sets the original time when this thread was put to sleep
	newSleepThread->sleep_counter = sleepTime; //sleep counter is given the timeslice
	
	
	sleepThreadCount++;
	thread_counter--;
}

//this function will check the sleep queue and determine if any threads need to be reactivated
//need to implement timing updates for this as well.
//Called from systick handler
void OS_CheckQueue(void){

	tcbType* current = SleepThreadPt;
	if(current == NULL){
		return;
	}
	
	do{
		tcbType* temp_next_tcb = current->nextTCB;
		if(Mstime-(current->sleep_time) > current->sleep_counter){
			if(current == SleepThreadPt){
				SleepThreadPt = current->nextTCB;
			}
			current->prevTCB->nextTCB = current->nextTCB;
			current->nextTCB->prevTCB = current->prevTCB;
			
			//find a free thread available and occupy it
			OS_WakeUpThread(current);
			//remove the current element from queue
			if(sleepThreadCount == 0){ //checks if there is only one element in the linked list left
				SleepThreadPt = NULL;
				return;
			}
		}
		current = temp_next_tcb;
	}while(current->nextTCB != SleepThreadPt);
}

//this function adds an awoken thread to the threadpool
void OS_WakeUpThread(tcbType* awokenThread){
	if(LatestThreadCreated == NULL){
		LatestThreadCreated = awokenThread;
		awokenThread->prevTCB = LatestThreadCreated; 
		awokenThread->nextTCB = LatestThreadCreated;
	}
	else{
		awokenThread->prevTCB = LatestThreadCreated; 
		awokenThread->nextTCB = LatestThreadCreated->nextTCB;
		LatestThreadCreated->nextTCB->prevTCB = awokenThread;
		LatestThreadCreated->nextTCB = awokenThread;
		LatestThreadCreated = awokenThread;
	}

    awokenThread->sleep_counter = 0;
		awokenThread->sleep_time = 0;

		
		awokenThread->status = 0; //set the status to ready
	thread_counter++;
	sleepThreadCount--;
}


//clears the tcb and stack for a killed thread
void OS_ClearTCBAndStack(void){
	if(LatestThreadCreated == RunPt){
		if(RunPt->nextTCB == RunPt){ //checks if there is only one element in the linked list left
				LatestThreadCreated = NULL;
			}
		else{
			LatestThreadCreated = RunPt->prevTCB;
		}
	}
	RunPt->sleep_counter = 0;
	RunPt->tid = NULL;
	RunPt->sp = NULL;
	//set the nextcb of the previous tcb to be the runpt's next tcb
	RunPt->prevTCB->nextTCB = RunPt->nextTCB;
	RunPt->nextTCB->prevTCB = RunPt->prevTCB;
	RunPt->prevTCB = NULL;
	SleepNextRunPt = RunPt->nextTCB;
	RunPt->nextTCB = NULL;
	//set kill flag
	RunPt->status = 1; //make sure to add this as a condition in add thread
	thread_counter--;
}



// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
  // put Lab 2 (and beyond) solution here
	long status = StartCritical();
	
	OS_ClearTCBAndStack();
 
  EndCritical(status); // end of atomic section 
	
	OS_Suspend(); // context switches to the next thread
	
  for(;;){};        // can not return
    
}; 

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
  // put Lab 2 (and beyond) solution here
	ContextSwitch(); 
};
  
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128

uint32_t volatile *PutPt; // put next
uint32_t volatile *GetPt; // get next
uint32_t static Fifo[64];
Sema4Type CurrentSize; // 0 means FIFO empty
Sema4Type  RoomLeft; 
Sema4Type FIFOmutex; // exclusive access to FIFO
void OS_Fifo_Init(uint32_t size){
  // put Lab 2 (and beyond) solution here
   PutPt = GetPt = &Fifo[0]; //init both pt for empty fifo 
	 OS_InitSemaphore(&FIFOmutex, 1); 
	 OS_InitSemaphore(&RoomLeft, 16); 
	 OS_InitSemaphore(&CurrentSize, 0); //0 is empty  
};

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(uint32_t data){
  // put Lab 2 (and beyond) solution here
	if(CurrentSize.Value == 16){
		return 0; //fifo is full 
	}
	
	OS_Wait(&RoomLeft);
	OS_Wait(&FIFOmutex);
	*(PutPt) = data; // Put
	PutPt++; // place to put next
	if(PutPt == &Fifo[16]){
		PutPt = &Fifo[0]; // wrap
	}
	OS_Signal(&FIFOmutex);
	OS_Signal(&CurrentSize);
	
  return 1; // replace this line with solution
};  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
uint32_t OS_Fifo_Get(void){
  // put Lab 2 (and beyond) solution here
	uint32_t data; 
	OS_Wait(&CurrentSize);
	OS_Wait(&FIFOmutex);
	data = *(GetPt); // get data
	GetPt++; // points to next data to get
	if(GetPt == &Fifo[16]){
		GetPt = &Fifo[0]; // wrap
	}
	OS_Signal(&FIFOmutex);
	OS_Signal(&RoomLeft);
	return data;
	
};

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
int32_t OS_Fifo_Size(void){
  // put Lab 2 (and beyond) solution here
   
  return 0; // replace this line with solution
};


uint32_t mail;
uint8_t mailFlag;
Sema4Type mailBoxFree;  //sema4 to signal if mailbox is free
Sema4Type dataReady;    //sem4 to signal if data in mailbox is ready     
// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
  // put Lab 2 (and beyond) solution here
  OS_InitSemaphore(&mailBoxFree,1);  //mail box is set to free 
	OS_InitSemaphore(&dataReady,0);		 //mail box is empty 
	mail = 0;
	mailFlag = 0;

};

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(uint32_t data){
  // put Lab 2 (and beyond) solution here
  // put solution here
  OS_bWait(&mailBoxFree); //wait for mailbox to be free 
	mail = data; //pass data 
	OS_bSignal(&dataReady);  //signal that data is ready 
	

};

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
uint32_t OS_MailBox_Recv(void){
  // put Lab 2 (and beyond) solution here
	uint32_t data;
	OS_bWait(&dataReady);	//wait for data to be in mailbox
	data = mail;		//collect mail 
	OS_bSignal(&mailBoxFree); //signal the box is free 
	return data;  
	
};

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
uint32_t OS_Time(void){
  // put Lab 2 (and beyond) solution here
	uint32_t constant = 1;
  uint32_t curtime = (160000*constant)- NVIC_ST_CURRENT_R;
	if(NVIC_ST_CURRENT_R == 0) {
		curtime--;
		constant++;
	}
	return curtime;
};

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
uint32_t OS_TimeDifference(uint32_t start, uint32_t stop){
  // put Lab 2 (and beyond) solution here

  if (stop >= start) {
    return stop - start;
  } 
  // If stop < start, a timer overflow occurred
  else {
    // Assuming a 32-bit timer, the maximum possible value is 2^32 - 1
    return (0xFFFFFFFF - start + 1) + stop;
  }
};



//count global time in MS
void mscounter(void){
	Mstime++;
}
// ** OS_ClearMsTime **
// sets the system time to zero (solve for Lab 1), and start a periodic interrupt
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
  // put Lab 1 solution here
	Mstime = 0; 
	Timer1A_Init(&mscounter, 80000, 0);
};

void (*PeriodicTask123)(void);   // user function

// ***************** TIMER1A_Init ****************
// Activate TIMER1A interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
//          priority 0 (highest) to 7 (lowest)
// Outputs: none
void Timer1A_Init(void(*task)(void), uint32_t period, uint32_t priority){
  SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER1
  PeriodicTask123 = task;         // user function
  TIMER1_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
  TIMER1_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER1_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER1_TAILR_R = period-1;    // 4) reload value
  TIMER1_TAPR_R = 0;            // 5) bus clock resolution
  TIMER1_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
  TIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  //NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|(priority<<13); // priority 
	NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x80000000;
// interrupts enabled in the main program after all devices initialized
// vector number 37, interrupt number 21
  NVIC_EN0_R = 1<<21;           // 9) enable IRQ 21 in NVIC
  TIMER1_CTL_R = 0x00000001;    // 10) enable TIMER1A
}
// write 1 to TIMER1_ICR_R
// will clear bit 0 TIMER1_RIS_R
void Timer1A_Handler(void){
	// acknowledging
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER1A timeout
  (*PeriodicTask123)();               // execute user task
}
void Timer1A_Stop(void){
  NVIC_DIS0_R = 1<<21;        // 9) disable IRQ 21 in NVIC
  TIMER1_CTL_R = 0x00000000;  // 10) disable timer1A

}

// ******** OS_MsTime ************
// reads the current time in msec (solve for Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// For Labs 2 and beyond, it is ok to make the resolution to match the first call to OS_AddPeriodicThread
uint32_t OS_MsTime(void){
  // put Lab 1 solution here
  return Mstime; // replace this line with solution
};


//** OS_Launch *** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(uint32_t theTimeSlice){
  // put Lab 2 (and beyond) solution here
    //from textbook 
	SysTick_Init(theTimeSlice);
	
	OS_ClearMsTime(); 
	
	StartOS(); // start on the first task
};

//************** I/O Redirection *************** 
// redirect terminal I/O to UART or file (Lab 4)

int StreamToDevice=0;                // 0=UART, 1=stream to file (Lab 4)

int fputc (int ch, FILE *f) { 
  if(StreamToDevice==1){  // Lab 4
    if(eFile_Write(ch)){          // close file on error
       OS_EndRedirectToFile(); // cannot write to file
       return 1;                  // failure
    }
    return 0; // success writing
  }
  
  // default UART output
  UART_OutChar(ch);
  return ch; 
}

int fgetc (FILE *f){
  char ch = UART_InChar();  // receive from keyboard
  UART_OutChar(ch);         // echo
  return ch;
}

int OS_RedirectToFile(const char *name){  // Lab 4
  eFile_Create(name);              // ignore error if file already exists
  if(eFile_WOpen(name)) return 1;  // cannot open file
  StreamToDevice = 1;
  return 0;
}

int OS_EndRedirectToFile(void){  // Lab 4
  StreamToDevice = 0;
  if(eFile_WClose()) return 1;    // cannot close file
  return 0;
}

int OS_RedirectToUART(void){
  StreamToDevice = 0;
  return 0;
}

int OS_RedirectToST7735(void){
  
  return 1;
}

