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
    7000
};

/* PWM config for motor B (Left Wheel) */
Timer_A_PWMConfig leftWheel =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_24,
    10000,
    TIMER_A_CAPTURECOMPARE_REGISTER_2,
    TIMER_A_OUTPUTMODE_RESET_SET,
    7000
};

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    /* Setting DCO to 24MHz*/
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);

    /* Configure P1.0 and set it to LOW */
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Configuring Timer_A to generate PWM in CCR1 and CCR2*/
   Timer_A_generatePWM(TIMER_A0_BASE, &rightWheel);
   Timer_A_generatePWM(TIMER_A0_BASE, &leftWheel);

   /* Configuring P3.2 and P3.3 as RXD and TXD for HC-05 Bluetooth module*/
   GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
   UART_initModule(EUSCI_A2_BASE, &bluetooth);
   UART_enableModule(EUSCI_A2_BASE);
   UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
   Interrupt_enableInterrupt(INT_EUSCIA2);

   /* Configuring P4.4 and P4.5 as Output pins */
   GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4);
   GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN5);
   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);
   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);

   /* Configuring P4.2 and P4.0 as Output pins */
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

    /* Configure pin P2.6 as interrupt input for Wheel encoder */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN6);
   GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);

   /* Configure pin P3.0 as interrupt input for Wheel encoder */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN0);
    GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN0);
    GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN0);

    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableInterrupt(INT_PORT2);
    Interrupt_enableInterrupt(INT_PORT3);
    Interrupt_enableMaster();

}

/*Interrupt Service Routine for UART (Bluetooth) */
void EUSCIA2_IRQHandler(void)
{
    char receivedValue = UART_receiveData(EUSCI_A2_BASE);
       while ((UCA0IFG & 0x02) == 0){};//check TXIFG bit is 1 - > TXBUF is empty

       if (receivedValue == 'w' || receivedValue == 'W'){  // Car moves forward
          moveForward();}

      if (receivedValue == 's' || receivedValue == 'S'){  // Car stops
          stop();}

      if (receivedValue == 'x' || receivedValue == 'X'){  // Car moves backward
          moveBackward();}

      if (receivedValue == 'd' || receivedValue == 'D'){  // Car moves right
          moveRight();
          delayMs(1500);
          stop();}

      if (receivedValue == 'a'|| receivedValue == 'A'){   // Car moves left
          moveLeft();
          delayMs(1500);
          stop();}
}

void PORT1_IRQHandler(void){
    if ((GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN6) != 0)){
        stop();
        GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN6);

    }

    if (GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN7) != 0){
        stop();
        GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN7);
    }

    else if (GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN1) != 0){
           moveForward();
           GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
        }

    else if (GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN4) != 0){
              moveBackward();
              GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
           }

}

void PORT2_IRQHandler(void){

    volatile static uint32_t counterA = 0;

    //If Encoder A gets interrupted
    if (GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN6) != 0){
        counterA+= 1;
        printf("Left Wheel Counter = %" PRIu32 "\n",counterA);


        if (counterA == 40){
            stopLeftWheel();
            counterA = 0;
        }

        GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN6);
    }

}

void PORT3_IRQHandler(void){

    volatile static uint32_t counterB = 0;

    //If Encoder B gets interrupted
    if (GPIO_getInterruptStatus(GPIO_PORT_P3, GPIO_PIN0) != 0){
        counterB+= 1;
        printf("Right Wheel Counter = %" PRIu32 "\n",counterB);

        if (counterB == 40){
            stopRightWheel();
            counterB = 0;
        }

        GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN0);
    }
}



