/* Remote control v2.5A01 
 * adapted for Arduino GUI, "landing" procedure
 * and autonomous mode for the controller with IR sensor
 * Created: February 21, 2018 10:18:32 AM (v2.4)
 * Modified: October 17, 2018 16:16:16
 * 
 * 
 * Original file:
 * TX_v2_4.c 
 * Created: September 10, 2014 9:20:41 AM
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <util/delay.h>

// Change this number to your team ID only here:
#define ID 0xF9

static volatile uint8_t msg[6]={0x55, ID, 0, 0, 0, 0}; 

//**************************************************************************************
//**************************************************************************************
//************************ start of LANDING procedure **********************************
//**************************************************************************************
// This function will be executed once you press SW5 button. 
// All controls will be disabled and only your code will have full control over the aircraft.
//**************************************************************************************
void LANDING(){
// see the write-up on what all these msg[] mean  
// here is some code that turns servo every 2 seconds, turns on both ON/OFF channels and turns PWM channels 0&1 for 1 second each at D=~39%
  msg[2]=255;
  msg[3]=127;
  msg[4]=0;
  msg[5]=0;
  _delay_ms(2000); 
  
  msg[3]=0;
  PORTD^=(1<<PD5); 
  _delay_ms(2000); 
  
  msg[3]=127;
  msg[4]=100; 
  PORTD^=(1<<PD5);  
  _delay_ms(2000); 
  
  msg[3]=255;
  msg[4]=0;
  msg[5]=100;
  PORTD^=(1<<PD5);   
  _delay_ms(2000); 
  
  msg[3]=127;
  msg[5]=0;  
  msg[2]=0; 
  PORTD^=(1<<PD5);   
  _delay_ms(2000);   
  
  msg[3]=0;
  PORTD^=(1<<PD5);   
  _delay_ms(2000); 
  
  msg[3]=127;
  PORTD^=(1<<PD5);   
  _delay_ms(2000);   
  
  msg[3]=255;
  PORTD^=(1<<PD5);    
	
  }  // end of LANDING()
//**************************************************************************************
//************************ end of LANDING procedure ************************************
//**************************************************************************************



//**************************************************************************************
// ***************** DO NOT MODIFY ANYTHING BELOW THIS LINE!!! *************************
//**************************************************************************************
#define BAUD 2400UL
#define UBRR ((F_CPU)/((BAUD)*(16UL))-(1UL))
#define ADC_sample_max 4
#define Vmin 80
static volatile uint8_t checksum, msg_char=1, ADC_sample, ADC_val, PWRG=1; 
static volatile uint16_t ADC_acc; 
ISR(__vector_default) { 
}
ISR (USART_TX_vect) {
  if (msg_char==6) {
    UDR0=checksum; 
    msg_char=0;
    checksum=0;
    return;
  }
  UDR0=msg[msg_char]; 
  checksum=checksum+msg[msg_char];
  msg_char++; 
}
ISR (ADC_vect){
  if (ADC_sample==0) { 
    ADC_acc=0;
    ADC_sample++;
    return; 
  }
  if (ADC_sample<=ADC_sample_max){ 
    ADC_acc=ADC_acc+ADCH;
    ADC_sample++;
    return;
  }
  if (ADC_sample>ADC_sample_max){
    ADC_val=(uint8_t)(ADC_acc/ADC_sample_max); 
    ADC_sample=0;
    switch (ADMUX&7) {  
      case 0: { 
        ADMUX|=(1<<MUX0);  
        msg[4]=ADC_val;
        return; 
      }
      case 1: { 
        ADMUX&=~(1<<MUX0);
        ADMUX|=(1<<MUX1);  
        msg[5]=ADC_val;
        return;
      }
      case 2: { 
        ADMUX|=(1<<MUX2);  
        msg[3]=ADC_val;
        return; 
      }
      case 6: { 
        ADMUX&=~(6<<MUX0); 
        if (ADC_val<Vmin) PWRG=0;
        return; 
      }
      default: { 
        ADMUX&=~(1<<MUX0);
        ADMUX|=(3<<MUX1);       
      }
    }
  }
}
inline void SHUTDOWN(){
  msg[2]=0;
  msg[3]=0;
  msg[4]=127;
  msg[5]=0;
  while (!msg_char);
  while (msg_char<6);
  cli(); 
  UCSR0B&=~((1<<TXCIE0)|(1<<TXEN0)); 
  ADCSRA&=~(1<<ADSC);
  ADCSRA&=~((1<<ADEN)|(1<<ADIE));  
  EIMSK&=~((1<<INT0)|(1<<INT1));
  PORTB|=((1<<PB2)|(1<<PB1)); 
  PORTD|=((1<<PD5)|(1<<PD6));
  PORTC&=~((1<<PC3)|(1<<PC4)); 
  for (uint8_t i=0; i<10; i++){
    for (uint16_t ii=0; ii<64000; ii++){
      ADC_acc=ii;
    }
    PORTB^=(1<<PB2);
  }
  PORTB|=(1<<PB2);
  PORTD&=~(1<<PD5);    
  sleep_enable ();
  sleep_cpu ();    
  }
ISR (INT1_vect){ 
  sei();
  SHUTDOWN();  
}
ISR (INT0_vect){ 
  sei();
  SHUTDOWN();  
}
void button_chk(volatile uint8_t *button_port, uint8_t button_bit, uint8_t msg_mask, volatile uint8_t *LED_port, uint8_t LED_bit) {
    if (!((*button_port)&(1<<button_bit))) { 
      debounce();      
      if (!(*button_port&(1<<button_bit))) 
      {
        while (!(*button_port&(1<<button_bit))); 
        debounce();
        if (*button_port&(1<<button_bit)) 
          {
          msg[2]^=msg_mask;
          if (msg[2]&msg_mask) *LED_port&=~(1<<LED_bit); 
            else *LED_port|=(1<<LED_bit); 
          }
      }
    }   
  }
inline void debounce() {
      TIFR1|=1; 
      TCNT1=0;
      TCCR1B|=(1<<CS11);
      while (!(TIFR1&1));
      TCCR1B&=~(1<<CS11);
  }
int main() {
  cli();
  DDRB |=(1<<PB2)|(1<<PB1); 
  PORTB|=((1<<PB2)|(1<<PB1)|(1<<PB0)); 
  DDRC |=(1<<PC3)|(1<<PC4); 
  PORTC&=~((1<<PC3)|(1<<PC4));
  PORTC|=((1<<PC3)|(1<<PC4)); 
  DDRD |=((1<<PD1)|(1<<PD5)|(1<<PD6));
  PORTD|=(1<<PD2)|(1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6); 
  UCSR0B=(1<<TXCIE0)|(1<<TXEN0);
  UCSR0C=(1<<UPM01)|(1<<UPM00)|(3<<UCSZ00); 
  UBRR0H = (uint8_t)((UBRR)>>8); 
  UBRR0L = (uint8_t)(UBRR);
  ADMUX|=((1<<ADLAR)|(1<<REFS0)); 
  ADCSRA|=((1<<ADEN)|(1<<ADIE));  
  ADCSRA|=(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);
  ADCSRA|=(1<<ADATE); 
  ADMUX|=(3<<MUX1);
  ADCSRA|=(1<<ADSC); 
  TCCR1B|=(1<<WGM12);
  OCR1A=0xFFFF;
  sei(); 
  UDR0=msg[0]; 
  while (msg_char<6);
  while(PWRG&&(PINB&(1<<PB2))) {
    debounce();
    button_chk(&PINB, PB0, 0b01000100, &PORTD, PD5);
    button_chk(&PIND, PD4, 0b10001000, &PORTD, PD6);
    button_chk(&PIND, PD2, 0b00010001, &PORTB, PB1);
    button_chk(&PIND, PD3, 0b00100010, &PORTB, PB2);   
      } 
  if (!PWRG) SHUTDOWN();
    else {
         PORTB|=((1<<PB2)|(1<<PB1)); 
         PORTD|=((1<<PD5)|(1<<PD6));
         PORTB&=~(1<<PB2);
         ADCSRA&=~(1<<ADSC);
         ADCSRA&=~((1<<ADEN)|(1<<ADIE));
         EIFR|=(1<<INTF1);                      
         EICRA|=(1<<ISC11);
         EIMSK|=(1<<INT1);          
         LANDING();
         SHUTDOWN();
      }
}
