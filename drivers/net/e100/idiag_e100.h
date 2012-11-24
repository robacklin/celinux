/*******************************************************************************

  
  Copyright(c) 1999 - 2003 Intel Corporation. All rights reserved.
  
  This program is free software; you can redistribute it and/or modify it 
  under the terms of the GNU General Public License as published by the Free 
  Software Foundation; either version 2 of the License, or (at your option) 
  any later version.
  
  This program is distributed in the hope that it will be useful, but WITHOUT 
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for 
  more details.
  
  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc., 59 
  Temple Place - Suite 330, Boston, MA  02111-1307, USA.
  
  The full GNU General Public License is included in this distribution in the
  file called LICENSE.
  
  Contact Information:
  Linux NICS <linux.nics@intel.com>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
*******************************************************************************/

#ifndef _IDIAG_E100_H_
#define _IDIAG_E100_H_

/* Unique base driver identifier */
#define IDIAG_E100_DRIVER			0x1

/* e100 diagnostic commands */
#define IDIAG_E100_DIAG_RESET_TEST		0x1
#define IDIAG_E100_DIAG_82559_TEST		0x2
#define IDIAG_E100_DIAG_XSUM_TEST	 	0x3
#define IDIAG_E100_DIAG_LOOPBACK_TEST		0x4
#define IDIAG_E100_DIAG_LINK_TEST		0x5

/* Results when failing test */
enum idiag_e100_diag_result{	
    IDIAG_E100_TEST_OK,	
    IDIAG_E100_TEST_NOT_EXEC,
    IDIAG_E100_TEST_FAILED
};

/* Results when failing diag EEPROM checksum test */
struct idiag_e100_eeprom_test{	
	u16 expected_checksum;
	u16 actual_checksum;
};

/* Results when failing diag 8255x self test */
#define IDIAG_E100_SELF_TEST_ROM     0x01
#define IDIAG_E100_SELF_TEST_PR      0x02
#define IDIAG_E100_SELF_TEST_SER     0x04
#define IDIAG_E100_SELF_TEST_TIMEOUT 0x10

struct idiag_e100_self_test{	
    unsigned long test_result;
};

/* Results when failing diag loopback test */
enum idiag_e100_lpbk_type {
	IDIAG_E100_DIAG_NO_LB,
	IDIAG_E100_DIAG_MAC_LB,
	IDIAG_E100_DIAG_PHY_LB
};

struct idiag_e100_lpback_test{	
	enum idiag_e100_lpbk_type mode;
	enum idiag_e100_diag_result result;
};

#endif /* _IDIAG_E100_H_ */ 

