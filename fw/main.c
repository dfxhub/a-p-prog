#include "inttypes.h"
#include "intrinsics.h"
#include "iostm8s003f3.h"

#define F_CPU 16000000L
#include "asmprocs.h"

//
// UART1 
// PD5 - TX - 2
// PD6 - RX - 3
//

//
// ISP
// MCLR - PC5 - 15
// DAT - PC4 - 14
// CLK - PC3 - 13
//

#define ISP_MCLR_1 PC_ODR_bit.ODR5 = 1;
#define ISP_MCLR_0 PC_ODR_bit.ODR5 = 0;
#define ISP_MCLR_D_I PC_DDR_bit.DDR5 = 0;
#define ISP_MCLR_D_0 PC_DDR_bit.DDR5 = 1;

#define ISP_DAT_1 PC_ODR_bit.ODR4 = 1;
#define ISP_DAT_0 PC_ODR_bit.ODR4 = 0;
#define ISP_DAT_V PC_IDR_bit.IDR4
#define ISP_DAT_D_I PC_DDR_bit.DDR4 = 0;
#define ISP_DAT_D_0 PC_DDR_bit.DDR4 = 1;

#define ISP_CLK_1 PC_ODR_bit.ODR3 = 1;
#define ISP_CLK_0 PC_ODR_bit.ODR3 = 0;
#define ISP_CLK_D_I PC_DDR_bit.DDR3 = 0;
#define ISP_CLK_D_0 PC_DDR_bit.DDR3 = 1;

#define  ISP_CLK_DELAY 1

// #define _delay_us(us) __delay_cycles((F_CPU  *(us)/4000000uL)

void _delay_ms(uint16_t ms) {
     while (ms--) {
         _delay_us(1000);
     }
}

unsigned char rx_state_machine (unsigned char state, unsigned char rx_char);

unsigned char enter_progmode (void);
unsigned char exit_progmode (void);

void isp_send (unsigned int data, unsigned char n);
void isp_send_8_msb (unsigned char data);
void isp_send_config (unsigned int data);

void isp_read_pgm (unsigned int * data, unsigned char n);
void isp_read_eeprom (unsigned char * data, unsigned char n, unsigned char t);
void isp_write_pgm (unsigned int * data, unsigned char n, unsigned char slow);
void isp_mass_erase (void);

void isp_inc_pointer (void);
void isp_reset_pointer (void);
void isp_reset_pointer_16d (void);
void isp_set_pointer_16d(uint32_t addr);

unsigned int isp_read_8 (void);
unsigned int isp_read_8_msb (void);
unsigned int isp_read_16 (void);
unsigned int isp_read_16_msb (void);
unsigned int isp_read_14s (void);

unsigned char p16c_enter_progmode (void);
void p16c_set_pc (unsigned long pc);
void p16c_bulk_erase (void);
unsigned int p16c_read_data_nvm (unsigned char inc);
void p16c_begin_prog (unsigned char cfg_bit);
void p16c_isp_write_cfg (unsigned int data, unsigned long addr);
void p16c_isp_write_pgm (unsigned int * data, unsigned long addr, unsigned char n);
void p16c_isp_read_pgm (unsigned int * data, unsigned long addr, unsigned char n);
void p16c_load_nvm (unsigned int data, unsigned char inc);

void p18q_isp_write_pgm (unsigned int * data, unsigned long addr, unsigned char n);
void p18q_isp_write_cfg (unsigned int data, unsigned long addr);

unsigned char p18_enter_progmode (void);
unsigned int p18_get_ID (void);
void p18_send_cmd_payload (unsigned char cmd, unsigned int payload);
unsigned int p18_get_cmd_payload (unsigned char cmd);
void p18_set_tblptr (unsigned long val);
unsigned char p18_read_pgm_byte (void);
void p18_isp_mass_erase (void);
void p18_isp_write_cfg (unsigned char data1, unsigned char data2, unsigned long addr);
void p18_isp_write_pgm (unsigned int * data, unsigned long addr, unsigned char n);

void p_18_isp_read_pgm (unsigned int * data, unsigned long addr, unsigned char n);
void p_18_modfied_nop (unsigned char nop_long);

void p18fk_isp_mass_erase (unsigned char data1, unsigned char data2, unsigned char data3);
void p18fk_isp_write_cfg (unsigned char data1, unsigned char data2, unsigned long addr);
void p18fk_isp_write_pgm (unsigned int * data, unsigned long addr, unsigned char n);

void p18fj_isp_mass_erase (void);

void usart_tx_b(uint8_t data);
uint8_t usart_rx_rdy(void);
uint8_t usart_rx_b(void);
void usart_tx_s(uint8_t * data);
void usart_tx_hexa_8b (uint8_t value);


unsigned int dat;
unsigned char rx,i,main_state,bytes_to_receive,rx_state;
unsigned char rx_message[140],rx_message_ptr; // 280
unsigned int flash_buffer[64]; // 260
unsigned char *eeprom_buf;
unsigned int test,cfg_val;
unsigned long addr;

void main( void )
{

    //
    // system clock
    //
    CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
    CLK_ICKR_HSIEN = 1;                 //  Enable the HSI.
    CLK_ECKR = 0;                       //  Disable the external clock.
    while (CLK_ICKR_HSIRDY == 0);       //  Wait for the HSI to be ready for use.
    CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
    CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
    CLK_PCKENR2 = 0xff;                 //  Ditto.
    CLK_CCOR = 0;                       //  Turn off CCO.
    CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
    CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
    CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
    CLK_SWCR = 0;                       //  Reset the clock switch control register.
    CLK_SWCR_SWEN = 1;                  //  Enable switching.
    while (CLK_SWCR_SWBSY != 0);        //  Pause while the clock switch is busy.
    
    // unused pins
    // output push-pull low
    PA_DDR = 0xff; // output
    PB_DDR = 0xff;
    PC_DDR = 0xff;
    PD_DDR = 0x9f; // not 5 and 6 - USART pins
    
    PA_CR1 = 0xff; // push-pull
    PB_CR1 = 0xff;
    PC_CR1 = 0xff;
    PD_CR1 = 0x9f; // not 5 and 6 - USART pins
    
    PA_ODR = 0; // low
    PB_ODR = 0;
    PC_ODR = 0;
    PD_ODR = 0;

    //
    // UART
    //
    //UART1_BRR2 = 0x06;      //  Set the baud rate registers to 57600 baud
    //UART1_BRR1 = 0x11;      //  based upon a 16 MHz system clock.
    UART1_BRR2 = 0x0b;      //  115200
    UART1_BRR1 = 0x08;      //  


    // idk what is this - clock polarity, lock phase and last bit clock pulse
    UART1_CR3_CPOL = 1;
    UART1_CR3_CPHA = 1;
    UART1_CR3_LBCL = 1;
    UART1_CR2_TEN = 1; // enable transmitter
    UART1_CR2_REN = 1; // enable receiver

    // already done above
    // PC_CR1_C15 = 1; // make push-pull, default open-drain not working
    // PC_CR1_C14 = 1;
    // PC_CR1_C13 = 1;
    
    ISP_CLK_D_0
    ISP_DAT_D_0
    ISP_DAT_0
    ISP_CLK_0
    ISP_MCLR_D_0
    ISP_MCLR_1
    rx_state = 0;

  while (1) {
    if (usart_rx_rdy()) {
      rx = usart_rx_b();
      rx_state = rx_state_machine (rx_state,rx);
      if (rx_state==3) {
          
            switch (rx_message[0]) {
                
            case 0x01: // prog_enter_progmode
                enter_progmode();
                usart_tx_b (0x81);
                rx_state = 0;
                break;
                
            case 0x02: // prog_exit_progmode
                exit_progmode();
                usart_tx_b (0x82);
                rx_state = 0;
                break;
                
            case 0x03: // p16a_rst_pointer A
                isp_reset_pointer();
                usart_tx_b (0x83);
                rx_state = 0;
                break;
                
            case 0x04: // p16a_load_config
                isp_send_config(0);
                usart_tx_b (0x84);
                rx_state = 0;
                break;
                
            case 0x05: // p16a_inc_pointer
                for (i=0;i<rx_message[2];i++)
                    isp_inc_pointer();
                usart_tx_b (0x85);
                rx_state = 0;
                break;
                
            case 0x06: // p16a_read_page
                usart_tx_b (0x86);
                isp_read_pgm(flash_buffer,rx_message[2]);
                for (i=0;i<rx_message[2];i++) {
                    usart_tx_b (flash_buffer[i]&0xFF);
                    usart_tx_b (flash_buffer[i]>>8);
                }
                rx_state = 0;
                break;
                
            case 0x07: // p16a_mass_erase
                isp_mass_erase();
                usart_tx_b (0x87);
                rx_state = 0;
                break;
                
            case 0x08: // p16a_program_page
                for (i=0;i<rx_message[2]/2;i++)
                    flash_buffer[i] = (((unsigned int)(rx_message[(2*i)+1+4]))<<8) + (((unsigned int)(rx_message[(2*i)+0+4]))<<0);
                isp_write_pgm(flash_buffer,rx_message[2]/2,rx_message[3]);
                usart_tx_b (0x88);
                rx_state = 0;
                break;
                
            case 0x09: // p16a_rst_pointer D
                isp_reset_pointer_16d();
                usart_tx_b (0x89);
                rx_state = 0;
                break;

            case 0x0a: // p16a_read_eeprom A
            case 0x0d: // p16a_read_eeprom D
                eeprom_buf = (unsigned char *)flash_buffer;
                if (rx_message[0] == 0x0a) {
                    usart_tx_b (0x8a);
                    isp_read_eeprom(eeprom_buf, rx_message[2], 'a');
                }
                else if (rx_message[0] == 0x0d) {
                    usart_tx_b (0x8d);
                    isp_read_eeprom(eeprom_buf, rx_message[2], 'd');
                }
                for (i=0;i<rx_message[2];i++) {
                    usart_tx_b (*eeprom_buf++);
                }
                rx_state = 0;
                break;

            case 0x0c: // p16d_set_pointer
                isp_set_pointer_16d((uint32_t)rx_message[2] + (uint32_t)(rx_message[3] << 8));
                usart_tx_b (0x8c);
                rx_state = 0;
                break;

            case 0x10: // prog_enter_progmode
                p18_enter_progmode();
                usart_tx_b (0x90);
                rx_state = 0;
                break;
                
            case 0x11: // p18a_read_page
                usart_tx_b (0x91);
                addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
                p_18_isp_read_pgm (flash_buffer, addr, rx_message[2]);
                for (i=0;i<rx_message[2];i++) {
                    usart_tx_b (flash_buffer[i]&0xFF);
                    usart_tx_b (flash_buffer[i]>>8);
                }
                rx_state = 0;
                break;
                
            case 0x12: // p18a_write_page
                addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
                for (i=0;i<rx_message[2]/2;i++)
                    flash_buffer[i] = (((unsigned int)(rx_message[(2*i)+1+6]))<<8) + (((unsigned int)(rx_message[(2*i)+0+6]))<<0);
                p18_isp_write_pgm (flash_buffer, addr, rx_message[2]/2);
                usart_tx_b (0x92);
                rx_state = 0;
                break;
                
            case 0x13: // p18a_mass_erase
                p18_isp_mass_erase();
                usart_tx_b (0x93);
                rx_state = 0;
                break;
                
            case 0x14: // p18a_write_cfg
                addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
                p18_isp_write_cfg (rx_message[6],rx_message[7], addr);
                usart_tx_b (0x94);
                rx_state = 0;
                break;
                
            case 0x23: // p18b_mass_erase
                p18fj_isp_mass_erase();
                usart_tx_b (0xA3);
                rx_state = 0;
                break;
                
            case 0x30: // p18d_mass_erase_part
                p18fk_isp_mass_erase (rx_message[2], rx_message[3], rx_message[4]);
                usart_tx_b (0xB0);
                rx_state = 0;
                break;
                
            case 0x31: // p18d_write_page
                addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
                for (i=0;i<rx_message[2]/2;i++)
                    flash_buffer[i] = (((unsigned int)(rx_message[(2*i)+1+6]))<<8) + (((unsigned int)(rx_message[(2*i)+0+6]))<<0);
                p18fk_isp_write_pgm (flash_buffer, addr, rx_message[2]/2);
                usart_tx_b (0xB1);
                rx_state = 0;
                break;
                
            case 0x32: // p18d_write_cfg
                addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
                p18fk_isp_write_cfg (rx_message[6],rx_message[7], addr);
                usart_tx_b (0xB2);
                rx_state = 0;
                break;
                
            case 0x40: // prog_enter_progmode
                p16c_enter_progmode();
                usart_tx_b (0xC0);
                rx_state = 0;   
                break;
                
            case 0x41: // p16c_read_page
                usart_tx_b (0xC1);
                addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
                p16c_isp_read_pgm (flash_buffer, addr, rx_message[2]);
                for (i=0;i<rx_message[2];i++) {
                    usart_tx_b (flash_buffer[i]&0xFF);
                    usart_tx_b (flash_buffer[i]>>8);
                }
                rx_state = 0;
                break;
                
            case 0x42: // p16c_write_page
                addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
                for (i=0;i<rx_message[2]/2;i++)
                    flash_buffer[i] = (((unsigned int)(rx_message[(2*i)+1+6]))<<8) + (((unsigned int)(rx_message[(2*i)+0+6]))<<0);
                p16c_isp_write_pgm (flash_buffer, addr, rx_message[2]/2);
                usart_tx_b (0xC2);
                rx_state = 0;
                break;
                
            case 0x43: // p16c_mass_erase
                p16c_set_pc (0x8000);
                p16c_bulk_erase ();
                usart_tx_b (0xC3);
                rx_state = 0;
                break;
                
            case 0x44: // p16c_write_single_cfg
                addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
                cfg_val = rx_message[6];
                cfg_val = (cfg_val<<8) + rx_message[7];
                p16c_isp_write_cfg (cfg_val, addr);
                usart_tx_b (0xC4);
                rx_state = 0;
                break;
                
            case 0x45: // p18q_write_single_cfg
                addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
                cfg_val = rx_message[6];
                cfg_val = (cfg_val<<8) + rx_message[7];
                p18q_isp_write_cfg (cfg_val, addr);
                usart_tx_b (0xC5);
                rx_state = 0;
                break;
                
            case 0x46: // p18q_write_page
                addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
                for (i=0;i<rx_message[2]/2;i++)
                    flash_buffer[i] = (((unsigned int)(rx_message[(2*i)+1+6]))<<8) + (((unsigned int)(rx_message[(2*i)+0+6]))<<0);
                p18q_isp_write_pgm (flash_buffer, addr, rx_message[2]/2);
                usart_tx_b (0xC6);
                rx_state = 0;
                break;
                
            default:
                break;
            }

        } // rc_state == 3
      } // usart received byte
    } // main loop

}

/////////////////////

unsigned char rx_state_machine (unsigned char state, unsigned char rx_char)
{
if (state==0)
  {
    rx_message_ptr = 0;
    rx_message[rx_message_ptr++] = rx_char;
    return 1;
  }
if (state==1)
  {
    bytes_to_receive = rx_char;
    rx_message[rx_message_ptr++] = rx_char;
    if (bytes_to_receive==0) return 3;
    return 2;
  }
if (state==2)
  {
    rx_message[rx_message_ptr++] = rx_char;
    bytes_to_receive--;
    if (bytes_to_receive==0) return 3;
  }
return state;  
}


void isp_read_pgm (unsigned int * data, unsigned char n)
{
unsigned char i;
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<n;i++)
  {
  isp_send(0x04,6);
  data[i] = isp_read_14s();
  isp_send(0x06,6);
  }
}


void isp_read_eeprom (unsigned char * data, unsigned char n, unsigned char t)
{
unsigned char i;
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<n;i++)
  {
  if (t == 'a')
      isp_send(0x05,6); // read from data mem _A
  else if (t == 'd')
      isp_send(0x04,6); // read _D
  data[i] = (unsigned char)(isp_read_14s() & 0xff); // only 8 lsb is valid data - data(8) zero(6)
  isp_send(0x06,6); // inc address
  }
}



void isp_write_pgm (unsigned int * data, unsigned char n, unsigned char slow)
{
unsigned char i;
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<n;i++)  
  {
  isp_send(0x02,6);
  isp_send(data[i]<<1,16);  
  if (i!=(n-1))
    isp_send(0x06,6);
  }
isp_send(0x08,6);
if (slow==1)
  _delay_ms(5);
else
  _delay_ms(3);
isp_send(0x06,6);
}

void isp_send_config (unsigned int data)
{
isp_send(0x00,6);
isp_send(data,16);
}

void isp_mass_erase (void)
{
//_delay_ms(10);
//_delay_us(3*ISP_CLK_DELAY);
//isp_send(0x11,6);
isp_send_config(0);
isp_send(0x09,6);
_delay_ms(10);
//isp_send(0x0B,6);
//_delay_ms(10);
}



void isp_reset_pointer (void)
{
//_delay_us(3*ISP_CLK_DELAY);
isp_send(0x16,6);
}

void isp_reset_pointer_16d (void)
{
//_delay_us(3*ISP_CLK_DELAY);
isp_send(0x1D,6);
isp_send(0x0,8);
isp_send(0x0,8);
isp_send(0x0,8);
}

void isp_set_pointer_16d(uint32_t addr)
{
    // 
    // Load PC Address
    // 0, lsb addr ... msb addr
    // total 24 bits
    
    addr = addr << 1;
    
    isp_send(0x1D,6);
    isp_send(addr & 0xff,8); // 0 + lsb
    isp_send((addr >> 8) & 0xff,8);
    isp_send((addr >> 16) & 0xff,8); // msb
}

void isp_inc_pointer (void)
{
//_delay_us(3*ISP_CLK_DELAY);
isp_send(0x06,6);
}


unsigned int isp_read_16 (void)
{
unsigned char i;
unsigned int out;
out = 0;
ISP_DAT_D_I
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<16;i++)
  {
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_0
  _delay_us(ISP_CLK_DELAY);
  out = out >> 1;
  if (ISP_DAT_V)
    out = out | 0x8000;
  }
 return out;
}

unsigned int isp_read_8 (void)
{
unsigned char i;
unsigned int out;
out = 0;
ISP_DAT_D_I
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<8;i++)
  {
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_0
  _delay_us(ISP_CLK_DELAY);
  out = out >> 1;
  if (ISP_DAT_V)
    out = out | 0x80;
  }
 return out;
}

unsigned int isp_read_14s (void)
{
unsigned int out;
out = isp_read_16();
out = out &0x7FFE;
out = out >> 1;
return out;
}



void isp_send (unsigned int data, unsigned char n)
{
unsigned char i;
ISP_DAT_D_0
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<n;i++)
  {
  if (data&0x01)
    {
    ISP_DAT_1
    }
  else
    {
    ISP_DAT_0
    }
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_1
//  _delay_us(ISP_CLK_DELAY);
  data = data >> 1;
  ISP_CLK_0
  ISP_DAT_0
//  _delay_us(ISP_CLK_DELAY);
  }
}


void isp_send_24_msb (unsigned long data)
{
unsigned char i;
ISP_DAT_D_0
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<23;i++)
  {
  if (data&0x400000)
    {
    ISP_DAT_1
    }
  else
    {
    ISP_DAT_0
    }
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_1
 _delay_us(ISP_CLK_DELAY);
  data = data << 1;
  ISP_CLK_0
//  _delay_us(ISP_CLK_DELAY);
  }
  ISP_DAT_0
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_0
}

void isp_send_8_msb (unsigned char data)
{
unsigned char i;
ISP_DAT_D_0
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<8;i++)
  {
  if (data&0x80)
    {
    ISP_DAT_1
    }
  else
    {
    ISP_DAT_0
    }
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  data = data << 1;
  ISP_CLK_0
  ISP_DAT_0
//  _delay_us(ISP_CLK_DELAY);
  }
}


unsigned int isp_read_8_msb (void)
{
unsigned char i;
unsigned int out;
out = 0;
ISP_DAT_D_I
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<8;i++)
  {
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_0
  _delay_us(ISP_CLK_DELAY);
  out = out << 1;
  if (ISP_DAT_V)
    out = out | 0x1;
  }
 return out;
}

unsigned int isp_read_16_msb (void)
{
unsigned char i;
unsigned int out;
out = 0;
ISP_DAT_D_I
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<16;i++)
  {
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_0
  _delay_us(ISP_CLK_DELAY);
  out = out << 1;
  if (ISP_DAT_V)
    out = out | 0x1;
  }
 return out;
}



unsigned char enter_progmode (void)
{
ISP_MCLR_0
_delay_us(300);
isp_send('P',8); // 0b01010000
isp_send('H',8); // 0b01001000
isp_send('C',8); // 0b01000011
isp_send('M',8); // 0b01001101

isp_send(0,1); // 33 clock
return 0;
}

/**************************************************************************************************************************/

unsigned char p18_enter_progmode (void)
{
ISP_MCLR_0
_delay_us(300);
// think it is the same ;)
isp_send_8_msb('M');
isp_send_8_msb('C');
isp_send_8_msb('H');
isp_send_8_msb('P');
//isp_send(0xB2,8); // 0b10110010
//isp_send(0xC2,8); // 0b11000010
//isp_send(0x12,8); // 0b00010010
//isp_send(0x0a,8); // 0b00001010
_delay_us(300);
ISP_MCLR_1
return 0;
}


void p18_isp_mass_erase (void)
{
p18_set_tblptr(0x3C0005);
p18_send_cmd_payload(0x0C,0x0F0F);
p18_set_tblptr(0x3C0004);
p18_send_cmd_payload(0x0C,0x8F8F);
p18_send_cmd_payload(0,0x0000);
isp_send(0x00,4);
_delay_ms(20);
isp_send(0x00,16);
}

void p18fj_isp_mass_erase (void)
{
p18_set_tblptr(0x3C0005);
p18_send_cmd_payload(0x0C,0x0101);
p18_set_tblptr(0x3C0004);
p18_send_cmd_payload(0x0C,0x8080);
p18_send_cmd_payload(0,0x0000);
isp_send(0x00,4);
_delay_ms(600);
isp_send(0x00,16);
}


void p18fk_isp_mass_erase (unsigned char data1, unsigned char data2, unsigned char data3)
{
unsigned int tmp1, tmp2, tmp3;
tmp1 = data1;
tmp1 = (tmp1<<8)|data1;
tmp2 = data2;
tmp2 = (tmp2<<8)|data2;
tmp3 = data3;
tmp3 = (tmp3<<8)|data3;
p18_set_tblptr(0x3C0004);
p18_send_cmd_payload(0x0C,tmp3);
p18_set_tblptr(0x3C0005);
p18_send_cmd_payload(0x0C,tmp2);
p18_set_tblptr(0x3C0006);
p18_send_cmd_payload(0x0C,tmp1);
p18_send_cmd_payload(0x00,0);
isp_send(0x00,4);
_delay_ms(5);
isp_send(0x00,16);
}

void p18fk_isp_write_pgm (unsigned int * data, unsigned long addr, unsigned char n)
{
unsigned char i;
//_delay_us(3*ISP_CLK_DELAY);
p18_send_cmd_payload(0,0x8E7F);
p18_send_cmd_payload(0,0x9C7F);
p18_send_cmd_payload(0,0x847F);
p18_set_tblptr(addr);
for (i=0;i<n-1;i++)  
  p18_send_cmd_payload(0x0D,data[i]);  
p18_send_cmd_payload(0x0F,data[n-1]);  
p_18_modfied_nop(0);
}

void p18_isp_write_pgm (unsigned int * data, unsigned long addr, unsigned char n)
{
unsigned char i;
//_delay_us(3*ISP_CLK_DELAY);
p18_send_cmd_payload(0,0x8EA6);
p18_send_cmd_payload(0,0x9CA6);
p18_send_cmd_payload(0,0x84A6);
p18_set_tblptr(addr);
for (i=0;i<n-1;i++)  
  p18_send_cmd_payload(0x0D,data[i]);  
p18_send_cmd_payload(0x0F,data[n-1]);  
p_18_modfied_nop(1);
}

void p18_isp_write_cfg (unsigned char data1, unsigned char data2, unsigned long addr)
{
unsigned int i;
//_delay_us(3*ISP_CLK_DELAY);
p18_send_cmd_payload(0,0x8EA6);
p18_send_cmd_payload(0,0x8CA6);
p18_send_cmd_payload(0,0x84A6);
p18_set_tblptr(addr);
p18_send_cmd_payload(0x0F,data1);  
p_18_modfied_nop(1);
_delay_ms(5);
p18_set_tblptr(addr+1);
i = data2;
i = i << 8;
p18_send_cmd_payload(0x0F,i);  
p_18_modfied_nop(1);
_delay_ms(5);
}

void p18fk_isp_write_cfg (unsigned char data1, unsigned char data2, unsigned long addr)
{
unsigned int i;
//_delay_us(3*ISP_CLK_DELAY);
p18_send_cmd_payload(0,0x8E7F);
p18_send_cmd_payload(0,0x8C7F);
p18_set_tblptr(addr);
p18_send_cmd_payload(0x0F,data1);  
p_18_modfied_nop(1);
_delay_ms(5);
p18_set_tblptr(addr+1);
i = data2;
i = i << 8;
p18_send_cmd_payload(0x0F,i);  
p_18_modfied_nop(1);
_delay_ms(5);
}

void p_18_modfied_nop (unsigned char nop_long)
{
unsigned char i;
ISP_DAT_D_0
ISP_DAT_0
for (i=0;i<3;i++)
  {
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
   ISP_CLK_0
  }
_delay_us(ISP_CLK_DELAY);
ISP_CLK_1
if (nop_long==1) _delay_ms(4);
_delay_ms(1);
ISP_CLK_0
_delay_us(ISP_CLK_DELAY);
isp_send(0x00,16);
}

void p_18_isp_read_pgm (unsigned int * data, unsigned long addr, unsigned char n)
{
unsigned char i;
unsigned int tmp1,tmp2;
//_delay_us(3*ISP_CLK_DELAY);
p18_set_tblptr(addr);
for (i=0;i<n;i++)  
  {
  tmp1 =  p18_read_pgm_byte();
  tmp2 =  p18_read_pgm_byte();
  tmp2 = tmp2<<8;
  data[i] = tmp1|tmp2;
  }
}


void p18_set_tblptr (unsigned long val)
{
  p18_send_cmd_payload(0,0x0E00|((val>>16)&0xFF));
  p18_send_cmd_payload(0,0x6EF8);
  p18_send_cmd_payload(0,0x0E00|((val>>8)&0xFF));
  p18_send_cmd_payload(0,0x6EF7);
  p18_send_cmd_payload(0,0x0E00|((val>>0)&0xFF));
  p18_send_cmd_payload(0,0x6EF6);
}


unsigned char p18_read_pgm_byte (void)
{
  isp_send(0x09,4);
  isp_send(0x00,8);
  return isp_read_8();
}

unsigned int p18_get_ID (void)
{
  unsigned int temp;

  p18_set_tblptr(0x3FFFFE);
  temp = p18_read_pgm_byte();
  temp = temp << 8;
  temp = temp | p18_read_pgm_byte();
  return temp;
}

void p18_send_cmd_payload (unsigned char cmd, unsigned int payload)
{
  isp_send(cmd,4);
  isp_send(payload,16);
  _delay_us(30);
}

unsigned int p18_get_cmd_payload (unsigned char cmd)
{
  isp_send(cmd,4);
  return isp_read_16();
}


unsigned char exit_progmode (void)
{
ISP_MCLR_1
_delay_ms(30);
ISP_MCLR_0
_delay_ms(30);
ISP_MCLR_1
return 0;
}

//***********************************************************************************//

unsigned char p16c_enter_progmode (void)
{
ISP_MCLR_0
_delay_us(300);
isp_send_8_msb('M');
isp_send_8_msb('C');
isp_send_8_msb('H');
isp_send_8_msb('P');
_delay_us(300);
return 0;
}

void p16c_set_pc (unsigned long pc)
{
  isp_send_8_msb(0x80);
  _delay_us(2);
  isp_send_24_msb(pc);

}

void p16c_bulk_erase (void)
{
  isp_send_8_msb(0x18);
  _delay_ms(100);
}

void p16c_load_nvm (unsigned int data, unsigned char inc)
{
  if (inc==0) isp_send_8_msb(0x00);
  else isp_send_8_msb(0x02);
  _delay_us(2);
  isp_send_24_msb(data);
  _delay_us(2);
}

unsigned int p16c_read_data_nvm (unsigned char inc)
{
  unsigned int retval;
  unsigned char tmp;
  if (inc==0) isp_send_8_msb(0xFC);
  else isp_send_8_msb(0xFE);
  _delay_us(2);
  tmp = isp_read_8_msb();
  retval = isp_read_16_msb();
  retval = retval >> 1;
  if (tmp&0x01) retval = retval | 0x8000;
  return retval;
}

void p16c_begin_prog (unsigned char cfg_bit)
{
  isp_send_8_msb(0xE0);
  _delay_ms(3);  
  if (cfg_bit!=0) _delay_ms(3); 
}

unsigned int p16c_get_ID (void)
{
  p16c_set_pc(0x8006);
  return p16c_read_data_nvm(1);
}
  
void p16c_isp_write_pgm (unsigned int * data, unsigned long addr, unsigned char n)
{
unsigned char i;
//_delay_us(3*ISP_CLK_DELAY);
p16c_set_pc(addr);
for (i=0;i<n;i++)  
  p16c_load_nvm(data[i],1);  
p16c_set_pc(addr);
p16c_begin_prog(0);
}

void p16c_isp_read_pgm (unsigned int * data, unsigned long addr, unsigned char n)
{
unsigned char i;
//unsigned int tmp1,tmp2;
//_delay_us(3*ISP_CLK_DELAY);
p16c_set_pc(addr);
for (i=0;i<n;i++)  
  data[i] = p16c_read_data_nvm(1);
}

void p16c_isp_write_cfg (unsigned int data, unsigned long addr)
{
//unsigned char i;
//_delay_us(3*ISP_CLK_DELAY);
p16c_set_pc(addr);
p16c_load_nvm(data,0);  
p16c_begin_prog(1);
}

void p18q_isp_write_pgm (unsigned int * data, unsigned long addr, unsigned char n)
{
unsigned char i;
//_delay_us(3*ISP_CLK_DELAY);
p16c_set_pc(addr);
for (i=0;i<n;i++)
  {  
  isp_send_8_msb(0xE0);  
  isp_send_24_msb(data[i]);  
  _delay_us(65);
  }
}

void p18q_isp_write_cfg (unsigned int data, unsigned long addr)
{
    //unsigned char i;
    //_delay_us(3*ISP_CLK_DELAY);
    p16c_set_pc(addr);
    isp_send_8_msb(0xE0);  
    isp_send_24_msb(data);  
    _delay_us(65); 
}


void usart_tx_b(uint8_t data)
{
    UART1_DR = data;
    while (!UART1_SR_TC);
} 

void usart_tx_s(uint8_t * data)
{
    while (*data!=0) 
        usart_tx_b(*data++);
} 


uint8_t usart_rx_rdy(void)
{
    //if (UCSR0A & _BV(RXC0))
    if (UART1_SR_RXNE == 1)
      return 1;
    else
      return 0;
}

uint8_t usart_rx_b(void)
{
    return (uint8_t) UART1_DR;
} 


void usart_tx_hexa_8 (uint8_t value)
{
    //uint8_t temp;
    //temp = value;
    usart_tx_b('0');
    usart_tx_b('x');
    usart_tx_hexa_8b(value);
    usart_tx_b(' ');
}

void usart_tx_hexa_8b (uint8_t value)
{
    uint8_t temp;
    temp = value;
    temp = ((temp>>4)&0x0F);
    if (temp<10) temp = temp + '0';
    else temp = temp + 'A'- 10;
    usart_tx_b(temp);
    temp = value;
    temp = ((temp>>0)&0x0F);
    if (temp<10) temp = temp + '0';
    else temp = temp + 'A' - 10;
    usart_tx_b(temp);
}


void usart_tx_hexa_16 (uint16_t value)
{
    usart_tx_b('0');
    usart_tx_b('x');
    usart_tx_hexa_8b((value>>8)&0xFF);
    usart_tx_hexa_8b(value&0xFF);
    usart_tx_b(' ');
}
