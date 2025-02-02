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

#include "e1000_api.h"


static s32 e1000_acquire_nvm_i210(struct e1000_hw *hw);
static void e1000_release_nvm_i210(struct e1000_hw *hw);
static s32 e1000_get_hw_semaphore_i210(struct e1000_hw *hw);
static void e1000_put_hw_semaphore_i210(struct e1000_hw *hw);
static s32 e1000_write_nvm_srwr(struct e1000_hw *hw, u16 offset, u16 words,
				u16 *data);
static s32 e1000_pool_flash_update_done_i210(struct e1000_hw *hw);
static s32 e1000_valid_led_default_i210(struct e1000_hw *hw, u16 *data);
static s32 e1000_read_nvm_i211(struct e1000_hw *hw, u16 offset, u16 words,
			       u16 *data);

/**
 *  e1000_acquire_nvm_i210 - Request for access to EEPROM
 *  @hw: pointer to the HW structure
 *
 *  Acquire the necessary semaphores for exclusive access to the EEPROM.
 *  Set the EEPROM access request bit and wait for EEPROM access grant bit.
 *  Return successful if access grant bit set, else clear the request for
 *  EEPROM access and return -E1000_ERR_NVM (-1).
 **/
static s32 e1000_acquire_nvm_i210(struct e1000_hw *hw)
{
	s32 ret_val;

	DEBUGFUNC("e1000_acquire_nvm_i210");

	ret_val = e1000_acquire_swfw_sync_i210(hw, E1000_SWFW_EEP_SM);

	return ret_val;
}

/**
 *  e1000_release_nvm_i210 - Release exclusive access to EEPROM
 *  @hw: pointer to the HW structure
 *
 *  Stop any current commands to the EEPROM and clear the EEPROM request bit,
 *  then release the semaphores acquired.
 **/
static void e1000_release_nvm_i210(struct e1000_hw *hw)
{
	DEBUGFUNC("e1000_release_nvm_i210");

	e1000_release_swfw_sync_i210(hw, E1000_SWFW_EEP_SM);
}

/**
 *  e1000_acquire_swfw_sync_i210 - Acquire SW/FW semaphore
 *  @hw: pointer to the HW structure
 *  @mask: specifies which semaphore to acquire
 *
 *  Acquire the SW/FW semaphore to access the PHY or NVM.  The mask
 *  will also specify which port we're acquiring the lock for.
 **/
s32 e1000_acquire_swfw_sync_i210(struct e1000_hw *hw, u16 mask)
{
	u32 swfw_sync;
	u32 swmask = mask;
	u32 fwmask = mask << 16;
	s32 ret_val = E1000_SUCCESS;
	s32 i = 0, timeout = 200; /* FIXME: find real value to use here */

	DEBUGFUNC("e1000_acquire_swfw_sync_i210");

	while (i < timeout) {
		if (e1000_get_hw_semaphore_i210(hw)) {
			ret_val = -E1000_ERR_SWFW_SYNC;
			goto out;
		}

		swfw_sync = E1000_READ_REG(hw, E1000_SW_FW_SYNC);
		if (!(swfw_sync & fwmask))
			break;

		/*
		 * Firmware currently using resource (fwmask)
		 */
		e1000_put_hw_semaphore_i210(hw);
		msec_delay_irq(5);
		i++;
	}

	if (i == timeout) {
		DEBUGOUT("Driver can't access resource, SW_FW_SYNC timeout.\n");
		ret_val = -E1000_ERR_SWFW_SYNC;
		goto out;
	}

	swfw_sync |= swmask;
	E1000_WRITE_REG(hw, E1000_SW_FW_SYNC, swfw_sync);

	e1000_put_hw_semaphore_i210(hw);

out:
	return ret_val;
}

/**
 *  e1000_release_swfw_sync_i210 - Release SW/FW semaphore
 *  @hw: pointer to the HW structure
 *  @mask: specifies which semaphore to acquire
 *
 *  Release the SW/FW semaphore used to access the PHY or NVM.  The mask
 *  will also specify which port we're releasing the lock for.
 **/
void e1000_release_swfw_sync_i210(struct e1000_hw *hw, u16 mask)
{
	u32 swfw_sync;

	DEBUGFUNC("e1000_release_swfw_sync_i210");

	while (e1000_get_hw_semaphore_i210(hw) != E1000_SUCCESS)
		; /* Empty */

	swfw_sync = E1000_READ_REG(hw, E1000_SW_FW_SYNC);
	swfw_sync &= ~mask;
	E1000_WRITE_REG(hw, E1000_SW_FW_SYNC, swfw_sync);

	e1000_put_hw_semaphore_i210(hw);
}

/**
 *  e1000_get_hw_semaphore_i210 - Acquire hardware semaphore
 *  @hw: pointer to the HW structure
 *
 *  Acquire the HW semaphore to access the PHY or NVM
 **/
static s32 e1000_get_hw_semaphore_i210(struct e1000_hw *hw)
{
	u32 swsm;
	s32 ret_val = E1000_SUCCESS;
	s32 timeout = hw->nvm.word_size + 1;
	s32 i = 0;

	DEBUGFUNC("e1000_get_hw_semaphore_i210");

	/* Get the FW semaphore. */
	for (i = 0; i < timeout; i++) {
		swsm = E1000_READ_REG(hw, E1000_SWSM);
		E1000_WRITE_REG(hw, E1000_SWSM, swsm | E1000_SWSM_SWESMBI);

		/* Semaphore acquired if bit latched */
		if (E1000_READ_REG(hw, E1000_SWSM) & E1000_SWSM_SWESMBI)
			break;

		usec_delay(50);
	}

	if (i == timeout) {
		/* Release semaphores */
		e1000_put_hw_semaphore_generic(hw);
		DEBUGOUT("Driver can't access the NVM\n");
		ret_val = -E1000_ERR_NVM;
		goto out;
	}

out:
	return ret_val;
}

/**
 *  e1000_put_hw_semaphore_i210 - Release hardware semaphore
 *  @hw: pointer to the HW structure
 *
 *  Release hardware semaphore used to access the PHY or NVM
 **/
static void e1000_put_hw_semaphore_i210(struct e1000_hw *hw)
{
	u32 swsm;

	DEBUGFUNC("e1000_put_hw_semaphore_i210");

	swsm = E1000_READ_REG(hw, E1000_SWSM);

	swsm &= ~E1000_SWSM_SWESMBI;

	E1000_WRITE_REG(hw, E1000_SWSM, swsm);
}

/**
 *  e1000_read_nvm_srrd_i210 - Reads Shadow Ram using EERD register
 *  @hw: pointer to the HW structure
 *  @offset: offset of word in the Shadow Ram to read
 *  @words: number of words to read
 *  @data: word read from the Shadow Ram
 *
 *  Reads a 16 bit word from the Shadow Ram using the EERD register.
 *  Uses necessary synchronization semaphores.
 **/
s32 e1000_read_nvm_srrd_i210(struct e1000_hw *hw, u16 offset, u16 words,
			     u16 *data)
{
	s32 status = E1000_SUCCESS;
	u16 i, count;

	DEBUGFUNC("e1000_read_nvm_srrd_i210");

	/* We cannot hold synchronization semaphores for too long,
	 * because of forceful takeover procedure. However it is more efficient
	 * to read in bursts than synchronizing access for each word. */
	for (i = 0; i < words; i += E1000_EERD_EEWR_MAX_COUNT) {
		count = (words - i) / E1000_EERD_EEWR_MAX_COUNT > 0 ?
			E1000_EERD_EEWR_MAX_COUNT : (words - i);
		if (hw->nvm.ops.acquire(hw) == E1000_SUCCESS) {
			status = e1000_read_nvm_eerd(hw, offset, count,
						     data + i);
			hw->nvm.ops.release(hw);
		} else {
			status = E1000_ERR_SWFW_SYNC;
		}

		if (status != E1000_SUCCESS)
			break;
	}

	return status;
}

/**
 *  e1000_write_nvm_srwr_i210 - Write to Shadow RAM using EEWR
 *  @hw: pointer to the HW structure
 *  @offset: offset within the Shadow RAM to be written to
 *  @words: number of words to write
 *  @data: 16 bit word(s) to be written to the Shadow RAM
 *
 *  Writes data to Shadow RAM at offset using EEWR register.
 *
 *  If e1000_update_nvm_checksum is not called after this function , the
 *  data will not be committed to FLASH and also Shadow RAM will most likely
 *  contain an invalid checksum.
 *
 *  If error code is returned, data and Shadow RAM may be inconsistent - buffer
 *  partially written.
 **/
s32 e1000_write_nvm_srwr_i210(struct e1000_hw *hw, u16 offset, u16 words,
			      u16 *data)
{
	s32 status = E1000_SUCCESS;
	u16 i, count;

	DEBUGFUNC("e1000_write_nvm_srwr_i210");

	/* We cannot hold synchronization semaphores for too long,
	 * because of forceful takeover procedure. However it is more efficient
	 * to write in bursts than synchronizing access for each word. */
	for (i = 0; i < words; i += E1000_EERD_EEWR_MAX_COUNT) {
		count = (words - i) / E1000_EERD_EEWR_MAX_COUNT > 0 ?
			E1000_EERD_EEWR_MAX_COUNT : (words - i);
		if (hw->nvm.ops.acquire(hw) == E1000_SUCCESS) {
			status = e1000_write_nvm_srwr(hw, offset, count,
						      data + i);
			hw->nvm.ops.release(hw);
		} else {
			status = E1000_ERR_SWFW_SYNC;
		}

		if (status != E1000_SUCCESS)
			break;
	}

	return status;
}

/**
 *  e1000_write_nvm_srwr - Write to Shadow Ram using EEWR
 *  @hw: pointer to the HW structure
 *  @offset: offset within the Shadow Ram to be written to
 *  @words: number of words to write
 *  @data: 16 bit word(s) to be written to the Shadow Ram
 *
 *  Writes data to Shadow Ram at offset using EEWR register.
 *
 *  If e1000_update_nvm_checksum is not called after this function , the
 *  Shadow Ram will most likely contain an invalid checksum.
 **/
static s32 e1000_write_nvm_srwr(struct e1000_hw *hw, u16 offset, u16 words,
				u16 *data)
{
	struct e1000_nvm_info *nvm = &hw->nvm;
	u32 i, k, eewr = 0;
	u32 attempts = 100000;
	s32 ret_val = E1000_SUCCESS;

	DEBUGFUNC("e1000_write_nvm_srwr");

	/*
	 * A check for invalid values:  offset too large, too many words,
	 * too many words for the offset, and not enough words.
	 */
	if ((offset >= nvm->word_size) || (words > (nvm->word_size - offset)) ||
	    (words == 0)) {
		DEBUGOUT("nvm parameter(s) out of bounds\n");
		ret_val = -E1000_ERR_NVM;
		goto out;
	}

	for (i = 0; i < words; i++) {
		eewr = ((offset+i) << E1000_NVM_RW_ADDR_SHIFT) |
			(data[i] << E1000_NVM_RW_REG_DATA) |
			E1000_NVM_RW_REG_START;

		E1000_WRITE_REG(hw, E1000_SRWR, eewr);

		for (k = 0; k < attempts; k++) {
			if (E1000_NVM_RW_REG_DONE &
			    E1000_READ_REG(hw, E1000_SRWR)) {
				ret_val = E1000_SUCCESS;
				break;
			}
			usec_delay(5);
		}

		if (ret_val != E1000_SUCCESS) {
			DEBUGOUT("Shadow RAM write EEWR timed out\n");
			break;
		}
	}

out:
	return ret_val;
}

/**
 *  e1000_read_nvm_i211 - Read NVM wrapper function for I211
 *  @hw: pointer to the HW structure
 *  @address: the word address (aka eeprom offset) to read
 *  @data: pointer to the data read
 *
 *  Wrapper function to return data formerly found in the NVM.
 **/
static s32 e1000_read_nvm_i211(struct e1000_hw *hw, u16 offset, u16 words,
			       u16 *data)
{
	s32 ret_val = E1000_SUCCESS;

	DEBUGFUNC("e1000_read_nvm_i211");

	/* Only the MAC addr is required to be present in the iNVM */
	switch (offset) {
	case NVM_MAC_ADDR:
		ret_val = e1000_read_invm_i211(hw, (u8)offset, &data[0]);
		ret_val |= e1000_read_invm_i211(hw, (u8)offset+1, &data[1]);
		ret_val |= e1000_read_invm_i211(hw, (u8)offset+2, &data[2]);
		if (ret_val != E1000_SUCCESS)
			DEBUGOUT("MAC Addr not found in iNVM\n");
		break;
	case NVM_ID_LED_SETTINGS:
	case NVM_INIT_CTRL_2:
	case NVM_INIT_CTRL_4:
	case NVM_LED_1_CFG:
	case NVM_LED_0_2_CFG:
		ret_val = e1000_read_invm_i211(hw, (u8)offset, data);
		break;
	case NVM_COMPAT:
		*data = ID_LED_DEFAULT_I210;
		break;
	case NVM_SUB_DEV_ID:
		*data = hw->subsystem_device_id;
		break;
	case NVM_SUB_VEN_ID:
		*data = hw->subsystem_vendor_id;
		break;
	case NVM_DEV_ID:
		*data = hw->device_id;
		break;
	case NVM_VEN_ID:
		*data = hw->vendor_id;
		break;
	default:
		DEBUGOUT1("NVM word 0x%02x is not mapped.\n", offset);
		*data = NVM_RESERVED_WORD;
		break;
	}
	return ret_val;
}

/**
 *  e1000_read_invm_i211 - Reads OTP
 *  @hw: pointer to the HW structure
 *  @address: the word address (aka eeprom offset) to read
 *  @data: pointer to the data read
 *
 *  Reads 16-bit words from the OTP. Return error when the word is not
 *  stored in OTP.
 **/
s32 e1000_read_invm_i211(struct e1000_hw *hw, u8 address, u16 *data)
{
	s32 status = -E1000_ERR_INVM_VALUE_NOT_FOUND;
	u32 invm_dword;
	u16 i;
	u8 record_type, word_address;

	DEBUGFUNC("e1000_read_invm_i211");

	for (i = 0; i < E1000_INVM_SIZE; i++) {
		invm_dword = E1000_READ_REG(hw, E1000_INVM_DATA_REG(i));
		/* Get record type */
		record_type = INVM_DWORD_TO_RECORD_TYPE(invm_dword);
		if (record_type == E1000_INVM_UNINITIALIZED_STRUCTURE)
			break;
		if (record_type == E1000_INVM_CSR_AUTOLOAD_STRUCTURE)
			i += E1000_INVM_CSR_AUTOLOAD_DATA_SIZE_IN_DWORDS;
		if (record_type == E1000_INVM_RSA_KEY_SHA256_STRUCTURE)
			i += E1000_INVM_RSA_KEY_SHA256_DATA_SIZE_IN_DWORDS;
		if (record_type == E1000_INVM_WORD_AUTOLOAD_STRUCTURE) {
			word_address = INVM_DWORD_TO_WORD_ADDRESS(invm_dword);
			if (word_address == address) {
				*data = INVM_DWORD_TO_WORD_DATA(invm_dword);
				DEBUGOUT2("Read INVM Word 0x%02x = %x",
					  address, *data);
				status = E1000_SUCCESS;
				break;
			}
		}
	}
	if (status != E1000_SUCCESS)
		DEBUGOUT1("Requested word 0x%02x not found in OTP\n", address);
	return status;
}

/**
 *  e1000_validate_nvm_checksum_i210 - Validate EEPROM checksum
 *  @hw: pointer to the HW structure
 *
 *  Calculates the EEPROM checksum by reading/adding each word of the EEPROM
 *  and then verifies that the sum of the EEPROM is equal to 0xBABA.
 **/
s32 e1000_validate_nvm_checksum_i210(struct e1000_hw *hw)
{
	s32 status = E1000_SUCCESS;
	s32 (*read_op_ptr)(struct e1000_hw *, u16, u16, u16 *);

	DEBUGFUNC("e1000_validate_nvm_checksum_i210");

	if (hw->nvm.ops.acquire(hw) == E1000_SUCCESS) {

		/*
		 * Replace the read function with semaphore grabbing with
		 * the one that skips this for a while.
		 * We have semaphore taken already here.
		 */
		read_op_ptr = hw->nvm.ops.read;
		hw->nvm.ops.read = e1000_read_nvm_eerd;

		status = e1000_validate_nvm_checksum_generic(hw);

		/* Revert original read operation. */
		hw->nvm.ops.read = read_op_ptr;

		hw->nvm.ops.release(hw);
	} else {
		status = E1000_ERR_SWFW_SYNC;
	}

	return status;
}


/**
 *  e1000_update_nvm_checksum_i210 - Update EEPROM checksum
 *  @hw: pointer to the HW structure
 *
 *  Updates the EEPROM checksum by reading/adding each word of the EEPROM
 *  up to the checksum.  Then calculates the EEPROM checksum and writes the
 *  value to the EEPROM. Next commit EEPROM data onto the Flash.
 **/
s32 e1000_update_nvm_checksum_i210(struct e1000_hw *hw)
{
	s32 ret_val = E1000_SUCCESS;
	u16 checksum = 0;
	u16 i, nvm_data;

	DEBUGFUNC("e1000_update_nvm_checksum_i210");

	/*
	 * Read the first word from the EEPROM. If this times out or fails, do
	 * not continue or we could be in for a very long wait while every
	 * EEPROM read fails
	 */
	ret_val = e1000_read_nvm_eerd(hw, 0, 1, &nvm_data);
	if (ret_val != E1000_SUCCESS) {
		DEBUGOUT("EEPROM read failed\n");
		goto out;
	}

	if (hw->nvm.ops.acquire(hw) == E1000_SUCCESS) {
		/*
		 * Do not use hw->nvm.ops.write, hw->nvm.ops.read
		 * because we do not want to take the synchronization
		 * semaphores twice here.
		 */

		for (i = 0; i < NVM_CHECKSUM_REG; i++) {
			ret_val = e1000_read_nvm_eerd(hw, i, 1, &nvm_data);
			if (ret_val) {
				hw->nvm.ops.release(hw);
				DEBUGOUT("NVM Read Error while updating checksum.\n");
				goto out;
			}
			checksum += nvm_data;
		}
		checksum = (u16) NVM_SUM - checksum;
		ret_val = e1000_write_nvm_srwr(hw, NVM_CHECKSUM_REG, 1,
						&checksum);
		if (ret_val != E1000_SUCCESS) {
			hw->nvm.ops.release(hw);
			DEBUGOUT("NVM Write Error while updating checksum.\n");
			goto out;
		}

		hw->nvm.ops.release(hw);

		ret_val = e1000_update_flash_i210(hw);
	} else {
		ret_val = E1000_ERR_SWFW_SYNC;
	}
out:
	return ret_val;
}

#if defined(QV_RELEASE) && defined(SPRINGVILLE_FLASHLESS_HW)
/**
 *  e1000_get_flash_presence_i210 - Check if flash device is detected.
 *  @hw: pointer to the HW structure
 *
 **/
static bool e1000_get_flash_presence_i210(struct e1000_hw *hw)
{
	u32 eec = 0;
	bool ret_val = false;

	DEBUGFUNC("e1000_get_flash_presence_i210");

	eec = E1000_READ_REG(hw, E1000_EECD);

	if (eec & E1000_EECD_FLASH_DETECTED_I210)
		ret_val = true;

	return ret_val;
}

#endif /* QV_RELEASE && SPRINGVILLE_FLASHLESS_HW */
/**
 *  e1000_update_flash_i210 - Commit EEPROM to the flash
 *  @hw: pointer to the HW structure
 *
 **/
s32 e1000_update_flash_i210(struct e1000_hw *hw)
{
	s32 ret_val = E1000_SUCCESS;
	u32 flup;

	DEBUGFUNC("e1000_update_flash_i210");

	ret_val = e1000_pool_flash_update_done_i210(hw);
	if (ret_val == -E1000_ERR_NVM) {
		DEBUGOUT("Flash update time out\n");
		goto out;
	}

	flup = E1000_READ_REG(hw, E1000_EECD) | E1000_EECD_FLUPD_I210;
	E1000_WRITE_REG(hw, E1000_EECD, flup);

	ret_val = e1000_pool_flash_update_done_i210(hw);
	if (ret_val == E1000_SUCCESS)
		DEBUGOUT("Flash update complete\n");
	else
		DEBUGOUT("Flash update time out\n");

out:
	return ret_val;
}

/**
 *  e1000_pool_flash_update_done_i210 - Pool FLUDONE status.
 *  @hw: pointer to the HW structure
 *
 **/
s32 e1000_pool_flash_update_done_i210(struct e1000_hw *hw)
{
	s32 ret_val = -E1000_ERR_NVM;
	u32 i, reg;

	DEBUGFUNC("e1000_pool_flash_update_done_i210");

	for (i = 0; i < E1000_FLUDONE_ATTEMPTS; i++) {
		reg = E1000_READ_REG(hw, E1000_EECD);
		if (reg & E1000_EECD_FLUDONE_I210) {
			ret_val = E1000_SUCCESS;
			break;
		}
		usec_delay(5);
	}

	return ret_val;
}

/**
 *  e1000_init_nvm_params_i210 - Initialize i210 NVM function pointers
 *  @hw: pointer to the HW structure
 *
 *  Initialize the i210 NVM parameters and function pointers.
 **/
static s32 e1000_init_nvm_params_i210(struct e1000_hw *hw)
{
	s32 ret_val = E1000_SUCCESS;
	struct e1000_nvm_info *nvm = &hw->nvm;

	DEBUGFUNC("e1000_init_nvm_params_i210");

	ret_val = e1000_init_nvm_params_82575(hw);

	nvm->ops.acquire = e1000_acquire_nvm_i210;
	nvm->ops.release = e1000_release_nvm_i210;
	nvm->ops.read    = e1000_read_nvm_srrd_i210;
	nvm->ops.write   = e1000_write_nvm_srwr_i210;
	nvm->ops.valid_led_default = e1000_valid_led_default_i210;
	nvm->ops.validate = e1000_validate_nvm_checksum_i210;
	nvm->ops.update   = e1000_update_nvm_checksum_i210;

	return ret_val;
}

/**
 *  e1000_init_nvm_params_i211 - Initialize i211 NVM function pointers
 *  @hw: pointer to the HW structure
 *
 *  Initialize the NVM parameters and function pointers for i211.
 **/
static s32 e1000_init_nvm_params_i211(struct e1000_hw *hw)
{
	struct e1000_nvm_info *nvm = &hw->nvm;

	DEBUGFUNC("e1000_init_nvm_params_i211");

	nvm->ops.acquire  = e1000_acquire_nvm_i210;
	nvm->ops.release  = e1000_release_nvm_i210;
	nvm->ops.read     = e1000_read_nvm_i211;
	nvm->ops.valid_led_default = e1000_valid_led_default_i210;
	nvm->ops.write    = e1000_null_write_nvm;
	nvm->ops.validate = e1000_null_ops_generic;
	nvm->ops.update   = e1000_null_ops_generic;

	return E1000_SUCCESS;
}

/**
 *  e1000_init_function_pointers_i210 - Init func ptrs.
 *  @hw: pointer to the HW structure
 *
 *  Called to initialize all function pointers and parameters.
 **/
void e1000_init_function_pointers_i210(struct e1000_hw *hw)
{
	e1000_init_function_pointers_82575(hw);

	switch (hw->mac.type) {
	case e1000_i210:
#if defined(QV_RELEASE) && defined(SPRINGVILLE_FLASHLESS_HW)
		if (e1000_get_flash_presence_i210(hw))
			hw->nvm.ops.init_params = e1000_init_nvm_params_i210;
		else
			hw->nvm.ops.init_params = e1000_init_nvm_params_i211;
#else
		hw->nvm.ops.init_params = e1000_init_nvm_params_i210;
#endif /* QV_RELEASE && SPRINGVILLE_FLASHLESS_HW */
		break;
	case e1000_i211:
		hw->nvm.ops.init_params = e1000_init_nvm_params_i211;
		break;
	default:
		break;
	}
	return;
}

/**
 *  e1000_valid_led_default_i210 - Verify a valid default LED config
 *  @hw: pointer to the HW structure
 *  @data: pointer to the NVM (EEPROM)
 *
 *  Read the EEPROM for the current default LED configuration.  If the
 *  LED configuration is not valid, set to a valid LED configuration.
 **/
static s32 e1000_valid_led_default_i210(struct e1000_hw *hw, u16 *data)
{
	s32 ret_val;

	DEBUGFUNC("e1000_valid_led_default_i210");

	ret_val = hw->nvm.ops.read(hw, NVM_ID_LED_SETTINGS, 1, data);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		goto out;
	}

	if (*data == ID_LED_RESERVED_0000 || *data == ID_LED_RESERVED_FFFF) {
		switch (hw->phy.media_type) {
		case e1000_media_type_internal_serdes:
			*data = ID_LED_DEFAULT_I210_SERDES;
			break;
		case e1000_media_type_copper:
		default:
			*data = ID_LED_DEFAULT_I210;
			break;
		}
	}
out:
	return ret_val;
}
