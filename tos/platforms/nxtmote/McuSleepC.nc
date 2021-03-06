// $Id: McuSleepC.nc,v 1.3 2007/06/13 09:10:52 rasped Exp $
/*
 * Copyright (c) 2005-2006 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */
/**
 * Dummy implementation to support the null platform.
 */

module McuSleepC {
  provides {
    interface McuSleep;
    interface McuPowerState;
  }
}
implementation {
  async command void McuSleep.sleep() {
  }

  async command void McuPowerState.update() {
  }
}
