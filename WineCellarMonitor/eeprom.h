
#ifndef _EEPROM_H_
#define _EEPROM_H_ 1

#define EEPROM_SIZE 		131072
#define EEPROM_PAGE_SIZE	256

#define EEPROM_CS_EN	(PORTD &= ~(1<<PORTD7))
#define EEPROM_CS_DIS	(PORTD |= (1<<PORTD7))

#define EEPROM_HOLD_EN	(PORTB &= ~(1<<PORTB0))
#define EEPROM_HOLD_DIS (PORTB |=(1<<PORTB0))

#define EEPROM_WP_EN	(PORTD &= ~(1<<PORTD6))
#define EEPROM_WP_DIS   (PORTD |= (1<<PORTD6))
  

//opcodes
#define WREN  0x06
#define WRDI  0x04
#define RDSR  0x05
#define WRSR  0x01
#define READ  0x03
#define WRITE 0x02
#define RDID  0xAB
#define DPD   0xB9

volatile uint32_t eeprom_start;
volatile uint32_t eeprom_avail;
volatile uint32_t eeprom_end;
extern volatile uint32_t eeprombyteindex;
 
uint32_t GetEEPROMStartPointer(void);
uint32_t GetEEPROMAvailPointer(void);
void UpdateEEPROMPointer(uint32_t);

void eeprom_init(void);
uint8_t read_eeprom(uint32_t);
uint8_t read_page_eeprom(uint8_t*, uint32_t, uint16_t);
uint8_t write_eeprom(uint32_t, uint8_t);
uint8_t write_page_eeprom(uint32_t, uint8_t*, uint16_t, uint16_t);
void write_enable_eeprom(void);
void write_disable_eeprom(void);
uint8_t eeprom_copy(uint8_t*, uint32_t, uint16_t);
void eeprom_deep_power_down(void);
uint8_t eeprom_wakeup(void);
uint16_t  writeEEstorage(uint8_t* datacache, uint16_t length);
void retrieve_data(uint8_t *data_buff, uint8_t skip_dist, uint8_t element_offset);
uint8_t eeprom_status_register(void);
//void spi_init();
//uint8_t spi_transfer(uint8_t data);

#endif //_EEPROM_H_
