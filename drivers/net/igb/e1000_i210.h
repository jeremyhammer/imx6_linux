/*******************************************************************************

  Intel(R) Gigabit Ethernet Linux driver
  Copyright(c) 2007-2012 Intel Corporation.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Contact Information:
  e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/

#ifndef _E1000_I210_H_
#define _E1000_I210_H_

s32 e1000_update_flash_i210(struct e1000_hw *hw);
s32 e1000_update_nvm_checksum_i210(struct e1000_hw *hw);
s32 e1000_validate_nvm_checksum_i210(struct e1000_hw *hw);
s32 e1000_write_nvm_srwr_i210(struct e1000_hw *hw, u16 offset,
			      u16 words, u16 *data);
s32 e1000_read_nvm_srrd_i210(struct e1000_hw *hw, u16 offset,
			     u16 words, u16 *data);
s32 e1000_read_invm_i211(struct e1000_hw *hw, u8 address, u16 *data);
s32 e1000_acquire_swfw_sync_i210(struct e1000_hw *hw, u16 mask);
void e1000_release_swfw_sync_i210(struct e1000_hw *hw, u16 mask);

#define E1000_STM_OPCODE		0xDB00
#define E1000_EEPROM_FLASH_SIZE_WORD	0x11

#define INVM_DWORD_TO_RECORD_TYPE(invm_dword) \
	(u8)((invm_dword) & 0x7)
#define INVM_DWORD_TO_WORD_ADDRESS(invm_dword) \
	(u8)(((invm_dword) & 0x0000FE00) >> 9)
#define INVM_DWORD_TO_WORD_DATA(invm_dword) \
	(u16)(((invm_dword) & 0xFFFF0000) >> 16)

enum E1000_INVM_STRUCTURE_TYPE {
	E1000_INVM_UNINITIALIZED_STRUCTURE		= 0x00,
	E1000_INVM_WORD_AUTOLOAD_STRUCTURE		= 0x01,
	E1000_INVM_CSR_AUTOLOAD_STRUCTURE		= 0x02,
	E1000_INVM_PHY_REGISTER_AUTOLOAD_STRUCTURE	= 0x03,
	E1000_INVM_RSA_KEY_SHA256_STRUCTURE		= 0x04,
	E1000_INVM_INVALIDATED_STRUCTURE		= 0x0F,
};

#define E1000_INVM_RSA_KEY_SHA256_DATA_SIZE_IN_DWORDS	8
#define E1000_INVM_CSR_AUTOLOAD_DATA_SIZE_IN_DWORDS	1

#define ID_LED_DEFAULT_I210		((ID_LED_OFF1_ON2  << 8) | \
					 (ID_LED_DEF1_DEF2 <<  4) | \
					 (ID_LED_OFF1_OFF2))
#define ID_LED_DEFAULT_I210_SERDES	((ID_LED_DEF1_DEF2 << 8) | \
					 (ID_LED_DEF1_DEF2 <<  4) | \
					 (ID_LED_DEF1_DEF2))

#endif
