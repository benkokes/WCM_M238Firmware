/*
 * flash.h
 *
 * Created: 9/2/2015 11:09:51 PM
 *  Author: Ben
 */ 


#ifndef FLASH_H_
#define FLASH_H_

#define EXTMEM_SIZE 	8388608UL
#define FLASH_PAGE_SIZE	256

#define FLASH_CS_EN	    (PORTD &= ~(1<<PORTD7))
#define FLASH_CS_DIS	(PORTD |= (1<<PORTD7))

#define FLASH_HOLD_EN	(PORTB &= ~(1<<PORTB0))
#define FLASH_HOLD_DIS  (PORTB |=(1<<PORTB0))

#define FLASH_WP_EN	    (PORTD &= ~(1<<PORTD6))
#define FLASH_WP_DIS    (PORTD |= (1<<PORTD6))


//opcodes
#define FL_WREN  0x06
#define FL_WRDI  0x04
#define FL_RDSR  0x05
#define FL_RDCR  0x15
#define FL_WRSR  0x01
#define FL_READ  0x03
#define FL_WRITE 0x02
#define FL_RDID  0xAB
#define FL_DPD   0xB9
#define FL_SE    0x20
#define FL_CE	 0xC7

extern volatile uint32_t extmembyteindex;

void flash_init(void);
uint8_t read_flash_byte(uint32_t);
uint8_t read_page_flash(uint8_t*, uint32_t, uint16_t);
uint8_t write_flash(uint32_t, uint8_t);
uint8_t write_page_flash(uint32_t, uint8_t*, uint16_t, uint16_t);
void write_enable_flash(void);
void write_disable_flash(void);
uint8_t flash_copy(uint8_t*, uint32_t, uint16_t);
void flash_deep_power_down(void);
uint8_t flash_wakeup(void);
uint16_t  writeEMstorage(uint8_t* datacache, uint16_t length);
void retrieveEM_data(uint8_t *data_buff, uint8_t skip_dist, uint8_t element_offset);
uint8_t flash_status_register(void);
uint8_t flash_chip_erase(void);
uint8_t flash_4kSector_erase(uint32_t address);

#endif /* FLASH_H_ */