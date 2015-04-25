#include "main.h"
#include "flash.h"
#include "uart.h"
#include "lpcfwh.h"
#include "spilib.h"
#include "frser.h"
#include "nibble.h"

static uint8_t flash_prot_in_use=0;

void flash_portclear(void) {
}

void flash_set_safe(void) {
	flash_portclear();
}


void flash_select_protocol(uint8_t allowed_protocols) {
	allowed_protocols &= SUPPORTED_BUSTYPES;
	flash_portclear();
	if ((allowed_protocols&CHIP_BUSTYPE_LPC)&&(lpc_test())) {
		flash_prot_in_use = CHIP_BUSTYPE_LPC;
		return;
	}
	flash_portclear();
	if ((allowed_protocols&CHIP_BUSTYPE_FWH)&&(fwh_test())) {
		flash_prot_in_use = CHIP_BUSTYPE_FWH;
		return;
	}
	flash_portclear();
	/* SPI last because it is really independent of FWH/LPC and works even when not selected. */
	if ((allowed_protocols&CHIP_BUSTYPE_SPI)&&(spi_test())) {
		flash_prot_in_use = CHIP_BUSTYPE_SPI;
		return;
	}
	flash_prot_in_use = 0;
	return;
}


uint8_t flash_read(uint32_t addr) {
	switch (flash_prot_in_use) {
		case 0:
		default:
			return 0xFF;
		case CHIP_BUSTYPE_LPC:
			return lpc_read_address(addr);
		case CHIP_BUSTYPE_FWH:
			return fwh_read_address(addr);
		case CHIP_BUSTYPE_SPI:
			return spi_read(addr);
	}
}

void flash_readn(uint32_t addr, uint32_t len) {
	switch (flash_prot_in_use) {
		case 0:
		default:
			while (len--) SEND(0xFF);
			return;
		case CHIP_BUSTYPE_LPC:
			while (len--) SEND(lpc_read_address(addr++));
			return;
		case CHIP_BUSTYPE_FWH:
			while (len--) SEND(fwh_read_address(addr++));
			return;
		case CHIP_BUSTYPE_SPI:
			spi_readn(addr,len);
			return;
	}
}

void flash_write(uint32_t addr, uint8_t data) {
	switch (flash_prot_in_use) {
		case 0:
		default:
			return;
		case CHIP_BUSTYPE_LPC:
			lpc_write_address(addr,data);
			return;
		case CHIP_BUSTYPE_FWH:
			fwh_write_address(addr,data);
			return;
	}
}


void flash_spiop(uint32_t sbytes, uint32_t rbytes) {
	if ((SUPPORTED_BUSTYPES) & CHIP_BUSTYPE_SPI) {
		spi_init_cond();
		spi_spiop(sbytes,rbytes);
		return;
	} else {
		while (sbytes--) RECEIVE();
		SEND(S_ACK);
		while (rbytes--) SEND(0xFF);
		return;
	}
}
