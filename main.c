
/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 Robotic Car Project
 *
 * Description: This project is using an MSP432 and a Bluetooth module chip to remotely control a robotic car.
 *              Instructions :
 *              1. Ensure all pins are properly connected
 *              2. Open a Bluetooth Serial Terminal and send any of the characters below to send a command to the robotic car
 *              w = Move car forward
 *              s = Stop the car
 *              x = Move car backward
 *              a = Move car
 *
 *              3. Pressing Switch P1.4 will toggle different distance that the car will travel.
 *                 If LED is green, car will travel 40 steps
 *                 If LED is blue, car will travel 60 steps
 *                 If LED is red, car will travel 80 steps
 *
 *
 *             4. Pressing Switch P1.1 will toggle between 60% PWM and 85% PWM
 *                If LED P1.0 is red, PWM is set to 85%
 *                If LED P1.0 is off, PWM is set to 60%
 *
 *
 *                MSP432P401
 *             ------------------
 *         /|\|      P3.2/UCA2RXD|<----Bluetooth Serial Terminal
 *            |      P3.3/UCA2TXD|---->Bluetooth Serial Terminal
 *          --|RST               |
 *            |                  |
*             |             P2.4 |--> Output PWM for left motor
 *            |             P2.5 |--> Output PWM for right motor
 *            |                  |
 *            |                  |
 * Author:
*******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdio.h>
#include <inttypes.h>

/*Global variables */
volatile static uint32_t counterA = 0;
volatile static uint32_t counterB = 0;
volatile static uint32_t distanceMode = 0;
volatile static uint32_t speedMode = 0;
volatile static uint32_t stoppingDistance = 40;


#define MIN_DISTANCE    15.0f
#define TICKPERIOD      1000
uint32_t SR04IntTimes;

static void Delay(uint32_t loop)
{
    volatile uint32_t i;

    for (i = 0 ; i < loop ; i++);
}

// -------------------------------------------------------------------------------------------------------------------

void Initalise_HCSR04(void)
{
    /* Timer_A UpMode Configuration Parameter */
    const Timer_A_UpModeConfig upConfig =
    {
            TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
            TIMER_A_CLOCKSOURCE_DIVIDER_24,          //
            TICKPERIOD,                             // 1000 tick period
            TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
            TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
            TIMER_A_DO_CLEAR                        // Clear value
    };

    int a = CS_getSMCLK();

    /* Configuring P3.6 as Output */
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);//
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);//

    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P3, GPIO_PIN7);


    /* Configuring Timer_A1 for Up Mode */
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);

    /* Enabling interrupts and starting the timer */
    Interrupt_enableInterrupt(INT_TA1_0);

    Timer_A_clearTimer(TIMER_A1_BASE);

}

// -------------------------------------------------------------------------------------------------------------------

void TA1_0_IRQHandler(void)
{
    /* Increment global variable (count number of interrupt occurred) */
    SR04IntTimes++;

    /* Clear interrupt flag */
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

// -------------------------------------------------------------------------------------------------------------------

static uint32_t getHCSR04Time(void)
{
    uint32_t pulsetime=0;

    /* Number of times the interrupt occurred (1 interrupt = 1000 ticks)    */
    pulsetime = SR04IntTimes * TICKPERIOD;

    /* Number of ticks (between 1 to 999) before the interrupt could occur */
    pulsetime += Timer_A_getCounterValue(TIMER_A1_BASE); //like the remaining number of ticks

    /* Clear Timer */
    Timer_A_clearTimer(TIMER_A1_BASE);

    Delay(3000);

    return pulsetime;
}

// -------------------------------------------------------------------------------------------------------------------

float getHCSR04Distance(void)
{
    uint32_t pulseduration = 0;
    float calculateddistance = 0;

    /* Generate 10us pulse at P3.6 */
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
    Delay(30);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);

    /* Wait for positive-edge */
    while(GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 0);

    /* Start Timer */
    SR04IntTimes = 0;
    Timer_A_clearTimer(TIMER_A1_BASE);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);

    /* Detects negative-edge */
    while(GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 1);

    /* Stop Timer */
    Timer_A_stopTimer(TIMER_A1_BASE);

    /* Obtain Pulse Width in microseconds */
    pulseduration = getHCSR04Time();

    /* Calculating distance in cm */
    calculateddistance = (float)pulseduration / 58.0f;

    return calculateddistance;
}
// -------------------------------------------------------------------------------------------------------------------

void moveForward(){
   /* Motor A move forward direction */
  GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
  GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);

  /* Motor B move forward direction */
  GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
  GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);}

void stop(){
   /* Motor A brake */
  GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);
  GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);

  /* Motor B brake */
  GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
  GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);}

void moveBackward(){
    /* Motor A move reverse direction */
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);

    /* Motor B move reverse direction */
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);}

void moveRight(){
    /* Motor A brake */
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);

    /* Motor B move forward direction */
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);}

void moveLeft(){
    /* Motor B brake */
   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);

   /* Motor A move forward direction */
   GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);}

void stopRightWheel(){
    /* Motor A brake */
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);
}

void stopLeftWheel(){
    /* Motor B brake */
     GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
     GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
}

/* Delay function. Delay 1 ms */
void delayMs(int n){

 int i, j;
 for (j = 0; j < n; j++)
 for (i = 750; i > 0; i--);}

/* Off all LED2 lights */
void set_All_LED2_Low()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);

}

/*Print data to Bluetooth terminal*/
void uPrintf(unsigned char * TxArray)
{
    unsigned short i = 0;
    while(*(TxArray+i))
    {
        UART_transmitData(EUSCI_A2_BASE, *(TxArray+i));  // Write the character at the location specified by pointer
        i++;                                             // Increment pointer to point to the next character
    }
}

/*UART configuration for Bluetooth HC-05 module */
const eUSCI_UART_ConfigV1 bluetooth =
{
     EUSCI_A_UART_CLOCKSOURCE_SMCLK,
     156,
     4,
     0,
     EUSCI_A_UART_NO_PARITY,
     EUSCI_A_UART_LSB_FIRST,
     EUSCI_A_UART_ONE_STOP_BIT,
     EUSCI_A_UART_MODE,
     EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

/* PWM config for motor A (Right Wheel) */
Timer_A_PWMConfig rightWheel =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_24,
    10000,
    TIMER_A_CAPTURECOMPARE_REGISTER_1,
    TIMER_A_OUTPUTMODE_RESET_SET,
    6000
};

/* PWM config for motor B (Left Wheel) */
Timer_A_PWMConfig leftWheel =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_24,
    10000,
    TIMER_A_CAPTURECOMPARE_REGISTER_2,
    TIMER_A_OUTPUTMODE_RESET_SET,
    6000
};

//PID controller section
/*************************************************************/
/*global variable for PID*/
int lastErrorLeft = 0;
int lastErrorRight = 0;
int totalErrorLeft = 0;
int totalErrorRight = 0;
int PWML = 0;
int PWMR = 0;
uint32_t leftCount = 0;
uint32_t rightCount = 0;

/* use to restrict the input between 1 and 0*/
float checkOne(float num){
    if (num > 1) {
        num = 1;
    } else if (num < 0) {
        num = 0;
    }
    return num;
}

/* use to generate the new PWM */
void newPWM(int PWML, int PWMR){
    Timer_A_generatePWM(TIMER_A0_BASE, &rightWheel);
    Timer_A_generatePWM(TIMER_A0_BASE, &leftWheel);
}

/* carPID*/
/* take leftcount and right count as input*/
/* call newPWM() to generate the new PWM */
void CarPID(int leftcount, int rightcount) {

    //Error of previous time
    int lastErrorLeft = 0;
    int lastErrorRight = 0;

    // Set the target count, use as end point (range)
    int targetCount = 50;

    // Calculate the Porportional Control
    float Kp = 0.03; // Proportional gain

    // Get the error for left and right enconder
    int errorLeft = targetCount - leftcount;
    int errorRight = targetCount - rightcount;

    /* check if the difference of error is in acceptable range of  -3 <  error < 3 */
    /* can change accordingly */
    if ((errorLeft < -3 || errorLeft > 3 ) || (errorRight < -3 || errorRight > 3 )) {
        float PoutLeft = Kp * errorLeft;
        float PoutRight = Kp * errorRight;

        PoutLeft = checkOne(PoutLeft);
        PoutRight = checkOne(PoutRight);

        // Calculate the Derivative Control
        float Kd = 0.15; // Derivative gain
        float DoutLeft = Kd * (errorLeft - lastErrorLeft);
        float DoutRight = Kd * (errorRight - lastErrorRight);

        // Update the last error (previous)
        lastErrorLeft = errorLeft;
        lastErrorRight = errorRight;

        // Calculate the Integral Control
        float Ki = 0.005; // Integral gain
        float IoutLeft = Ki * totalErrorLeft;
        float IoutRight = Ki * totalErrorRight;

        // Add error of this round to the total error
        totalErrorLeft += errorLeft;
        totalErrorRight += errorRight;

        // Calculate the PID output
        float leftPIDOutput = PoutLeft + DoutLeft + IoutLeft;
        float rightPIDOutput = PoutRight + DoutRight + IoutRight;

        // Ensure the PID output is within the range of 0 to 1
        leftPIDOutput = checkOne(leftPIDOutput);
        rightPIDOutput = checkOne(rightPIDOutput);


        // Set the PWM for the left and right motor
        PWML = (int)((1+leftPIDOutput) * rightWheel.dutyCycle);
        PWMR = (int)((1+rightPIDOutput) * leftWheel.dutyCycle);

        // Set the PWM for the left and right motor after PID adjustment
        newPWM(PWML, PWMR);
    }

}
/*********************************************************************************/

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    /* Setting DCO to 24MHz*/
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);

    Initalise_HCSR04();

    /* Configuring all LEDs as outpin pins */
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);

    /* Configuring Timer_A to generate PWM in CCR1 and CCR2*/
   Timer_A_generatePWM(TIMER_A0_BASE, &rightWheel);
   Timer_A_generatePWM(TIMER_A0_BASE, &leftWheel);

   /* Configuring P3.2 and P3.3 as RXD and TXD for HC-05 Bluetooth module*/
   GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
   UART_initModule(EUSCI_A2_BASE, &bluetooth);
   UART_enableModule(EUSCI_A2_BASE);
   UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
   Interrupt_enableInterrupt(INT_EUSCIA2);

   /* Configuring P4.4 and P4.5 as Output pins for motor driver */
   GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4);
   GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN5);
   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);
   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);

   /* Configuring P4.2 and P4.0 as Output pins for motor driver */
   GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);
   GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);

   /*Setting P2.4 and P2.5 as peripheral output for PWM for Motor A and Motor B */
   GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
   GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

   /* Configure pin P1.6 as interrupt input for IR sensor */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);

    /* Configure pin P1.7 as interrupt input for IR sensor */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN7);

    /* Configure pin P4.1 as interrupt input for Wheel encoder */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN1);
   GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN1);

   /* Configure pin P3.0 as interrupt input for Wheel encoder */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN0);
    GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN0);
    GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN0);

    /*Set S1 and S2 as input and set up the interrupts*/
     GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
     GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
     GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
     GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
     GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
     GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);

    /*Enabling all ISR */
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableInterrupt(INT_PORT2);
    Interrupt_enableInterrupt(INT_PORT3);
    Interrupt_enableInterrupt(INT_PORT4);
    Interrupt_enableMaster();

    while(1)
        {

        //use this to test what is the new generate pwm by uncommenting and printf statement in carPID fucntion
        //CarPID(22,18);

        Delay(3000);
           //Obtain distance from HCSR04 sensor and check if its less then minimum distance//
          float calculateddistance = getHCSR04Distance();
          printf("%.2f\n",calculateddistance);
          if((getHCSR04Distance() < MIN_DISTANCE)){
              stop();
              GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
          }
          else{
              GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
		  }
        }
}

/*Interrupt Service Routine for UART (Bluetooth) */
void EUSCIA2_IRQHandler(void)
{
	char receivedValue = UART_receiveData(EUSCI_A2_BASE);
	while ((UCA0IFG & 0x02) == 0){};//check TXIFG bit is 1 - > TXBUF is empty

	if (receivedValue == 'w' || receivedValue == 'W'){  // Car moves forward
		counterA = 0;
		counterB = 0;
		moveForward();
		//uPrintf("Car is moving forward.... ");
	}

	if (receivedValue == 's' || receivedValue == 'S'){  // Car stops
		stop();
		//uPrintf("Car has stopped....  ");
	}

	if (receivedValue == 'x' || receivedValue == 'X'){  // Car moves backward
		counterA = 0;
		counterB = 0;
		moveBackward();
		//uPrintf("Car is moving backward....  ");
	}

	if (receivedValue == 'd' || receivedValue == 'D'){  // Car moves right
		counterA = 0;
		counterB = 0;
		moveRight();
		//uPrintf("Car is rotating right....  ");
		delayMs(1500);
		stop();
	}

	if (receivedValue == 'a'|| receivedValue == 'A'){   // Car moves left
		counterA = 0;
		counterB = 0;
		moveLeft();
		//uPrintf("Car is rotating left....  ");
		delayMs(1500);
		stop();
	}
}

/*ISR for IR sensors and P1.1 switch toggles PWM to either 60% or 85% */
void PORT1_IRQHandler(void){
    if ((GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN6) != 0)){
        stop();
        GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN6);

    }

    if (GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN7) != 0){
        stop();
        GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN7);
    }

    if (GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN1) != 0){
           speedMode = 1 - speedMode; //switch between 1 and 0
           if (speedMode == 1){
               rightWheel.dutyCycle = 8500;
               leftWheel.dutyCycle = 8500;
			   }

            else {
              rightWheel.dutyCycle = 6000;
              leftWheel.dutyCycle = 6000;
			  }

           GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
        }

    if (GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN4) != 0){
              
		if (distanceMode == 0){
			stoppingDistance = 40;
			set_All_LED2_Low();
			GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
		}

		else if (distanceMode == 1){
			stoppingDistance = 60;
			set_All_LED2_Low();
			GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
			distanceMode = 0
		}

		else{
			set_All_LED2_Low();
			stoppingDistance = 80;
			GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
			distanceMode = 0;
		}
				   
		distanceMode +=1;
		
		if (distanceMode > 2){
			distanceMode = 0;
		}
		
		GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
	}

}


/*ISR for the wheel encoder (Right Wheel) */
void PORT3_IRQHandler(void){
    //If Wheel Encoder B gets interrupted
    if (GPIO_getInterruptStatus(GPIO_PORT_P3, GPIO_PIN0) != 0){
        counterB+= 1;
		rightCount++;
        //printf("Right Wheel Counter = %" PRIu32 "\n",counterB);

        if (counterB == stoppingDistance){
            stopRightWheel();
            counterB = 0;
        }

        GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN0);
    }
}

/*ISR for the wheel encoder (Left Wheel)*/
void PORT4_IRQHandler(void){
    //If Wheel Encoder B gets interrupted
    if (GPIO_getInterruptStatus(GPIO_PORT_P4, GPIO_PIN1) != 0){
        counterA+= 1;
		leftCount++;
		if(leftCount == 50){
//            GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0); //for debug purpose
			CarPID(leftCount,rightCount);       //call carPID when count reach 50
			if(rightWheel.dutyCycle >= 9000)     //ensure pwd duty circle not excceed the limit
				rightWheel.dutyCycle = 6000;
				leftWheel.dutyCycle = 6000;
				Timer_A_generatePWM(TIMER_A0_BASE, &rightWheel);
				Timer_A_generatePWM(TIMER_A0_BASE, &leftWheel);
			leftCount = 0;                      //reset left count and rightcount after PID applied
			rightCount = 0;
		}
        //printf("Left Wheel Counter = %" PRIu32 "\n",counterA);

        if (counterA == stoppingDistance){
            stopLeftWheel();
            counterA = 0;
        }

        GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN1);
    }
}
