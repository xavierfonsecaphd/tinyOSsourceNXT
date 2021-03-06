/*
 * Copyright (c) 2007 Copenhagen Business School
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of CBS nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * ARCHED ROCK OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE
 */

/**
 * Test of Bluetooth. 
 *
 * @author Rasmus Ulslev Pedersen
 */

#include "BtRadioCountToLeds.h"

configuration TestBtC{}
implementation {
  components MainC, TestBtM, HalBtC, HalLCDC;
  components new AMSenderC(AM_RADIO_COUNT_MSG);
  components new AMReceiverC(AM_RADIO_COUNT_MSG);
  components BTIOMapCommP;
components HalOutputC;
  components HalInputC;
  components HalAT91I2CLSC;  
  TestBtM.Boot   -> MainC.Boot;
  
  TestBtM.Receive -> AMReceiverC;
  
  TestBtM.HalBt -> HalBtC.HalBt; 
  
  TestBtM.HalLCD -> HalLCDC.HalLCD;
  
TestBtM.AMSend -> AMSenderC;
TestBtM.Packet -> AMSenderC;  

TestBtM.BTIOMapComm -> BTIOMapCommP;

TestBtM.Boot   -> MainC.Boot;

  
  HalOutputC.Boot -> MainC.Boot;


  HalInputC.Boot -> MainC.Boot;


  HalAT91I2CLSC.Boot -> MainC.Boot;
}