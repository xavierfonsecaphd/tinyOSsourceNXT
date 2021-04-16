/*
 * Copyright (c) 2007 nxtmote project
 * All rights reserved. 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *	Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *	Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *  
 *   Neither the name of nxtmote project nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE ARCHED
 * ROCK OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */
/**
 * @author Rasmus Pedersen
 */
interface HplAT91SPI 
{
  async command void setSPICR(uint32_t val);

  async command void setSPIMR(uint32_t val);
  async command uint32_t getSPIMR();

  async command uint32_t getSPISR();

  async command uint32_t getSPIRDR();
  
  async command void setSPITDR(uint32_t val);

  async command void setSPIIER(uint32_t val);
  
  async command void setSPIIDR(uint32_t val);
  
  async command uint32_t getSPIIMR();
  
  async command uint32_t getSPICSR0();
  async command void setSPICSR0(uint32_t val);
    
  async command uint32_t getSPICSR1();
  async command void setSPICSR1(uint32_t val);

  async command uint32_t getSPICSR2();
  async command void setSPICSR2(uint32_t val);

  async command uint32_t getSPICSR3();
  async command void setSPICSR3(uint32_t val);
  
  async event void interruptSPI();
}
