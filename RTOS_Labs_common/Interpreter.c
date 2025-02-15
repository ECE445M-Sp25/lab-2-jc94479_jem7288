// *************Interpreter.c**************
// Students implement this as part of EE445M/EE380L.12 Lab 1,2,3,4 
// High-level OS user interface

#include <stdint.h>
#include <string.h> 
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../inc/ADCT0ATrigger.h"
#include "../inc/ADCSWTrigger.h"
#include "../RTOS_Labs_common/ADC.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"

char* stringparser(char* instring);
char* stringforlcd(char* instring);
int extractNum(char* instring);
char* removebackspaces(char* input);
extern uint32_t NumCreated;   // number of foreground threads created
extern int32_t MaxJitter;             // largest time jitter between interrupts in usec


void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}


// Print jitter histogram
void Jitter(int32_t MaxJitter, uint32_t const JitterSize, uint32_t JitterHistogram[]){
  // write this for Lab 3 (the latest)
	
}

// *********** Command line interpreter (shell) ************
void Interpreter(void){ 
  // write this  
	//call uart_instring to get the current contents of the fifo until it's empty
	//parse the string to figure out which commands it is. Potentially use slew of if-else statements
	char initcommand[32]; 
  uint16_t buff = 31;
  while(1){
		//read string from Uart (uart instring)
    UART_InString(initcommand, buff); OutCRLF(); 
		
		char* command = removebackspaces(initcommand);
		
    //determine if it's a recognized command (if-else chain)
		char* firstword = stringparser(command);
		
		//if string parse function returns an error
		if(firstword == 0){
			continue;
		}
		
		if(strcmp(firstword, "lcdtop") == 0){
			uint32_t ypos = extractNum(command); //get the line number
			char* message = stringforlcd(command); //get the message to be printed out
			ST7735_Message(0, ypos, message, NULL);
			UART_OutString("LCD Ypos: "); UART_OutUDec(ypos); 
			OutCRLF(); 
			UART_OutString("Message: ");UART_OutString(message); 
		}
		
		else if(strcmp(firstword, "lcdbot") == 0){
			uint32_t ypos = extractNum(command); //get the line number
			char* message = stringforlcd(command); //get the message to be printed out
			ST7735_Message(1, ypos, message, NULL);
			
			UART_OutString("LCD Ypos: "); UART_OutUDec(ypos); 
			OutCRLF(); 
			UART_OutString("Message: ");UART_OutString(message); 
		}
		
		else if(strcmp(firstword, "adcin") == 0){
			UART_OutUDec(ADC_In()); 
		}
		
		else if(strcmp(firstword, "cursystime") == 0){
			UART_OutUDec(OS_MsTime()); 
		}
		
		else if(strcmp(firstword, "cleartime") == 0){
			OS_ClearMsTime();
			UART_OutString("Clear Successful"); 
		}
		
		else if(strcmp(firstword, "clc") == 0){
			//clear LCD screen 
			 ST7735_FillScreen(0);     
		}
				
		else if(strcmp(firstword, "numcreated") == 0){
			UART_OutUDec(NumCreated);
		}
		
		else if(strcmp(firstword, "maxjitter") == 0){
			UART_OutSDec(MaxJitter);
		}
		
		else{
			UART_OutString("Error: command not found"); 
		}
		OutCRLF(); 
	}
}


char* removebackspaces(char* input) {
    char* output = input;
    int writeindex = 0;
    int readindex = 0;

    while (input[readindex] != '\0') {
        if (input[readindex] != 0x7F) {
            output[writeindex] = input[readindex];
            writeindex++;
        } else if (writeindex > 0) {
            writeindex--;
        }
        readindex++;
    }
    output[writeindex] = '\0';

    return output;
}


//extract the first word from the string command given by the user. This is the primary command for the intepreter
//Inputs: instring		the entire input command from the user
//Output: the first word in the string
char* stringparser(char* instring) {
    static char firstword[50];  // Static buffer to store the first word
    int i = 0;
    int whitespacecount = 0;

    // Skip leading whitespace and count it
    while ((*instring == ' ' || *instring == '\t') && whitespacecount < 6) {
        instring++;
        whitespacecount++;
    }

    // Check if leading whitespace exceeds 5
    if (whitespacecount > 5) {
        return 0;
    }

    // Extract characters until a space or null terminator is encountered
    while (*instring != ' ' && *instring != '\0' && i < 49) {
        firstword[i++] = *instring++;
    }

    // Null-terminate the extracted word
    firstword[i] = '\0';

    // If no word was extracted, return an error
    if (i == 0) {
        return "ERROR: No word found";
    }

    return firstword;
}

//extract the line number which the string is meant to be printed on
//Inputs: instring		the entire input command from the user
//Output: the extracted number
int extractNum(char* instring) {
    char* ptr = instring;
    int num = -1;

    // Skip the first word
    while (*ptr && !isspace(*ptr)) ptr++;
    while (*ptr && isspace(*ptr)) ptr++;

    // Extract the number from the second word
    if (*ptr) {
        num = atoi(ptr);
    }

    return num;
}

//extract the string to be printed from the user's input command
//Inputs: instring		the entire input command from the user
//Output: the extracted string
char* stringforlcd(char* instring) {
    static char result[50];
    char* ptr = instring;
    int wordcount = 0;

    // Skip the first two words
    while (*ptr && wordcount < 2) {
        if (isspace(*ptr)) {
            wordcount++;
            while (*ptr && isspace(*ptr)) ptr++;
        } else {
            ptr++;
        }
    }

    // Skip any leading spaces after the second word
    while (*ptr && isspace(*ptr)) ptr++;

    // Extract the remaining string
    if (*ptr) {
        int i = 0;
        // Remove leading and trailing quotes if present
        if (*ptr == '"') ptr++;
        while (*ptr && *ptr != '"' && i < sizeof(result) - 1) {
            result[i++] = *ptr++;
        }
        result[i] = '\0';
    } else {
        return NULL;  // Error if no string found
    }

    return result[0] ? result : NULL;
}