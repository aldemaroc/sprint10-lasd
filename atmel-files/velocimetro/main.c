/*
 * velocimetro.c
 *
 * Created: 12/12/2021 19:59:34
 * Author : Aldemaro Campos - 120110926
 
     _______
    |bip bip|    _______
           \|   //   ||\\
          _____//____||_\\___     -  -  _    -
          |) _ ELÉTRICO _   (\    - _  _  - - _
          |_/.\________/.\___|   _     - _   -
            \_/        \_/          -        -

 
 */ 

#define F_CPU 16000000UL // Velocidade da CPU
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "SSD1306/SSD1306.h"
#include "SSD1306/Font5x8.h"

void USART_Init(unsigned int ubrr){			
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_Transmit(char data[]){
		int i;
		for (i = 0; data[i] != 0; i++) {
				while(!(UCSR0A & (1<<UDRE0)));
				UDR0 = data[i];
		}
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = 13;
}

unsigned char USART_Receive(void){
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

// Definição das variáveis necessárias
uint8_t flag_5ms = 0, flag_50ms = 0, flag_500ms = 0, flag_500ms2 = 0,flag_1s = 0,estado_anterior_timer=0, estado_farol_timer =0, flag_250ms = 0, estado_anterior_timer2 = 0,buzzer = 2;
uint16_t Velocidade_carro_kmH = 999, RPM_motor = 9999, Diametro_pneu_cm = 65;
uint32_t tempo_ms = 0, tempo_borda_subida, tempo_delta, distancia_cm;
float Distancia_hodometro_km = 9;

uint32_t acelerador_ADC,bateria_ADC,temperatura_ADC;

// Funções a serem utilizadas
void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo);
void acelerador(uint8_t *flag_disparo, uint32_t velocidade, uint32_t acelerador_ADC2);
void anima_LCD(uint16_t velocidade_carro, uint16_t diametro_pneu_cm, uint16_t rpm_motor, float distancia_hodometro_km, uint8_t *flag_disparo, float distancia_proximidade, float temperaturabat, float batt, uint8_t *estado_anterior, uint8_t *buzzerr);
void leitura_sensores_ADC(uint8_t *flag_disparo);

// Timer de 1ms
ISR(TIMER0_COMPA_vect){
	tempo_ms++;
	if((tempo_ms % 5)==0) // Contador para 5ms
	flag_5ms = 1;
	if((tempo_ms % 50)==0) // Contador para 50ms
	flag_50ms=1;
	if((tempo_ms % 250)==0) // Contador para 50ms
	flag_250ms=1;
	if((tempo_ms % 500)==0){ // Contador para 0,5s
	flag_500ms=1;
	flag_500ms2=1;
	}
	if((tempo_ms % 1000)==0) // Contador para 1s
	flag_1s=1;
}

ISR(USART_RX_vect){
	char recebido;
	recebido = UDR0;
	
	if(recebido=='d'){ // Retornar temperatura máxima da bateria
		
		unsigned char out_string[8];

		// Transforma as variáveis em char
		sprintf(out_string, "%u", eeprom_read_word(30));
		
		USART_Transmit("Temperatura Maxima Registrada:");
		USART_Transmit(out_string);
		}
		if(recebido=='l'){ // Limpa temperatura máxima da bateria
			USART_Transmit("Limpando temperatura maxima...");
			eeprom_write_word(30 , 1);
			USART_Transmit("Feito!");
		}
		if(recebido=='o'){ // Limpa temperatura máxima da bateria
			unsigned char out_string[8];

			// Transforma as variáveis em char
			sprintf(out_string, "%u", eeprom_read_word(40));
			
			USART_Transmit("Odo atual:");
			USART_Transmit(out_string);
		}

}

ISR(TIMER1_CAPT_vect){
	if(TCCR1B & (1<<ICES1))
		tempo_borda_subida = ICR1;
	else{
		tempo_delta = (ICR1 - tempo_borda_subida)*16; // delta de tempo entre sinais
		distancia_cm = tempo_delta/58.14; // distância em cm calculada de forma primitiva
		if(distancia_cm>700 || distancia_cm<0){
			distancia_cm=0;
		}
	}
	TCCR1B ^= (1<<ICES1);
}

// Interrupt para contagem da rotação da roda
ISR(INT0_vect){
	static uint8_t cont_5voltas = 0;
	static uint32_t tempo_ms_anterior = 0;
	uint16_t delta_t_ms=0;
	
	if(cont_5voltas == 5){ // Conta a cada 5 voltas
		delta_t_ms = tempo_ms - tempo_ms_anterior;
		RPM_motor = 300000/(delta_t_ms);
		Velocidade_carro_kmH = ((uint32_t)Diametro_pneu_cm*565)/delta_t_ms;
		tempo_ms_anterior = tempo_ms;
		cont_5voltas = 0;
	}
	cont_5voltas++;
	
	Distancia_hodometro_km += ((float)Diametro_pneu_cm*3.1415)/100000; // Incrementa o hodometro
}

ISR(PCINT2_vect){ //Verifica pressionamento dos botões
	if((PIND&0b00010000)==0){ //PD4 para incrementar
		if(Diametro_pneu_cm < 200)
			Diametro_pneu_cm++;
			eeprom_write_word(10,Diametro_pneu_cm); // Escrevemos os dados atualizados na eeprom
	}
	if((PIND&0b00100000)==0){ //PD5 para decrementar
		if(Diametro_pneu_cm > 1)
			Diametro_pneu_cm--;
			eeprom_write_word(10,Diametro_pneu_cm);// Escrevemos os dados atualizados na eeprom
	}
}

int main(void){
		// Inicializa os GPIO
		DDRB |= 0b00000000; // Define PB1->PB6 como saída e PB0 como entrada
		DDRC &= 0b00110000; // Define PC1/2/3/4/5 como saída e PD 0/6/7 como entrada
		DDRD &= 0b11000011; // Define PD0/1/5/6/7 como saída e PD 2/3/4 como entrada
		
		// Configuração dos Pullup
		PORTD = 0b00111100; // Habilitado em PD
		PORTB = 0b01110000; // Habilitado em PB
		
		//Configuração das Ext. Interrupts
		EICRA	= 0b00000010; // Interrupt INT0 na descida
		EIMSK	= 0b00000001; // Habilita INT0
		PCICR	= 0b00000100; // Habilita na porta 2
		PCMSK2	= 0b00110000; // Define pinos para interrupt
		
		//Configuração do Timer0
		TCCR0A	= 0b00000010; // TC0 no modo CTC
		TCCR0B	= 0b00000011; // PS em 64 para TC0
		OCR0A	= 249; // TC0 contar até 250-1
		TIMSK0	= 0b00000010; // Usa OCR0A como comparação
		
		// Configuração de PWM
		TCCR2A	= 0b10100011; // PWN não invertido no pino OC2B
		TCCR2B	= 0b00000011; // Habilita TC0, PS em 64
		OCR2B	= 0; // Controle inicial de PWN que será modificado pelo potenciômetro
		
		// Configuração do ADC		
		ADMUX	= 0b01000000; // Usa o canal 0 como referência (VCC=5V)
		ADCSRA	= 0b11100111; // Habilita o AD, defina modo CC, define PS em 128
		ADCSRB	= 0b00000000; // Modo CC
		DIDR0	= 0b00000000; // Desativa entrada digital
		
		// Configura o timer pra leitura da distância até juntar as tintas
		TCCR1B = (1<<ICES1)|(1<<CS12);
		TIMSK1 = 1<<ICIE1;
		
		// Interrupções globais
		sei();
		
		
		// Carrega os valores da memória
				
		// Confere se já há algo na memória
		if (eeprom_read_word((uint16_t*)10) == UINT16_MAX){
			// se não, insere um valor padrãoread_float
			eeprom_write_word((uint16_t*)10 , 65);
		}else{
			// se sim, lemos esse valor para mostrar no display
			Diametro_pneu_cm = eeprom_read_word((uint16_t*)10);
		}
		
		// Confere se já há algo na memória
		if (eeprom_read_word(40) == UINT16_MAX){
			// se não, insere um valor padrão
			eeprom_write_word(40 , 11);
		}else{
			// se sim, lemos esse valor para mostrar no display
			Distancia_hodometro_km = eeprom_read_word(40);
		}
		
		// Confere se já há algo na memória
		if (eeprom_read_word(30) == UINT16_MAX){
			// se não, insere um valor padrão
			eeprom_write_word(30 , 2);
			}else{
			// se sim, lemos esse valor para mostrar no display
			temperatura_ADC = eeprom_read_word(30);
		}
		
		// Inicia USART
		USART_Init(MYUBRR);
		
		// Inicia Display
		GLCD_Setup();
		GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
		GLCD_InvertScreen();
		GLCD_Clear();
		
		while(1){
			// Roda funções de atualização
			//anima_velocidade(Velocidade_carro_kmH, &flag_5ms);
			DispararTimers(&flag_1s, &flag_500ms2, &flag_250ms, &estado_anterior_timer, &estado_farol_timer, &estado_anterior_timer2, &buzzer);
			acelerador(&flag_5ms, Velocidade_carro_kmH, acelerador_ADC);
			anima_LCD(Velocidade_carro_kmH, Diametro_pneu_cm, RPM_motor, Distancia_hodometro_km, &flag_500ms, distancia_cm, temperatura_ADC, bateria_ADC, estado_anterior_timer, &buzzer);
			leitura_sensores_ADC(&flag_50ms);
			
			// Se o valor registrado na eeprom é diferente, atualiza
			if (eeprom_read_word(40) != Distancia_hodometro_km){
				eeprom_write_word(40,Distancia_hodometro_km);
			}		
		}
}

void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo){
	static int8_t cont_dig = 0;
		
	if(*flag_disparo){ // Verifica se é hora de atualizar o display
		switch(cont_dig){ // Casos para cada dígito
			case 0:
				PORTB &= 0b00000001;
				PORTB |= 0b00001111;
				PORTB |= (((velocidade_carro/1  )%10) & 0b00011110);
				break;
			case 1:
				PORTB &= 0b00000001;
				PORTB |= 0b11000000;
				PORTB |= (((velocidade_carro/10 )%10) & 0b00011110);
				break;
			case 2:
				PORTB &= 0b00000001;
				PORTB |= 0b00110000;
				PORTB |= (((velocidade_carro/100)%10) & 0b00011110);
				cont_dig = -1;
				break;
		}
		cont_dig++; // Incrementa o dígito a ser escrito
		*flag_disparo = 0; // Reinicia a flag de disparo
	}
}

void DispararTimers(uint8_t *flag_disparo_1s, uint8_t *flag_disparo_500m, uint8_t *flag_disparo_250m, uint8_t *estado_anterior, uint8_t *estado_farol,uint8_t *estado_anterior2, uint8_t *estado_buzzer){
	static int8_t status_anterior;
	if(*flag_disparo_500m){ // Timer de 500ms para luzes e buzzer
		if((PINB&0b00010000)==0){ // Verifica a porta
			if(*estado_anterior==0){ // Verifica se o estado é inverso
				PORTB |= 0b00000010; // Realiza a ação
			}else if(*estado_anterior==1){ // Verifica se é hora de reverter
				PORTB &= 0b11111101; // Reverte a ção
			}
		}else if((PINB&0b00010000)!=0 && *estado_anterior == 1){ // Condição final se o botão não estiver pressionado
			PORTB &= 0b11111101;
		}
		
		// Realiza as mesmas condições acima para outra porta
		if((PINB&0b00100000)==0){
			if(*estado_anterior==0){
				PORTB |= 0b10000000;
			}else if(*estado_anterior==1){
				PORTB &= 0b01111111;
			}
		}else if((PINB&0b00100000)!=0 && *estado_anterior == 1){
			PORTB &= 0b01111111;
		}
		
		if(*estado_buzzer==1){ // Verifica estado do buzzer: 1 para 500ms, 2 para 250ms e 0 para desligado
			if(*estado_anterior==0){
				PORTC |= 0b00001000;
				}else if(*estado_anterior==1){
				PORTC &= 0b11110111;
			}
			}else if((*estado_buzzer==1 && *estado_anterior == 1) || estado_buzzer==0){
			PORTC &= 0b11110111;
		}
		
			if(*estado_anterior == 0){ // Reseta as condições
				*estado_anterior = 1;
				}else{
				*estado_anterior = 0;
			}
		*flag_disparo_500m = 0; // Reseta nossa flag
	}
	
	if(*flag_disparo_250m){ // Flag para buzzer de 250ms
		if(*estado_buzzer==2){
			if(*estado_anterior2==0){
				PORTC |= 0b00001000;
				}else if(*estado_anterior2==1){
				PORTC &= 0b11110111;
			}
			}else if((*estado_buzzer==2 && *estado_anterior2 == 1) || estado_buzzer==0){
				PORTC &= 0b11110111;
		}
		if(*estado_anterior2 == 0){
			*estado_anterior2 = 1;
			}else{
			*estado_anterior2 = 0;
		}
		*flag_disparo_250m = 0;
	}
	if((PINB&0b01000000)==0 && *estado_farol == 0){ // Confere estado do farol
		PORTB |= 0b00000100; // Se ligado
		*estado_farol = 1;
	}else if((PINB&0b01000000)!=0 && *estado_farol == 1){
		PORTB &= 0b11111011; // Se desligado
		*estado_farol = 0;
	}
	
}

void leitura_sensores_ADC(uint8_t *flag_disparo){
	
	static uint8_t cont_canal = 0;
	if (*flag_disparo){
		switch(cont_canal){
			case 0:
				acelerador_ADC = ADC;
				ADMUX	= 0b01000001;
				break;
			case 1:
				temperatura_ADC = (ADC-511)*(150.0/114.0);
				ADMUX	= 0b01000010;
				
				// Se o valor registrado na eeprom é diferente, atualiza
				if (eeprom_read_word(30) < temperatura_ADC){
					eeprom_write_word(30,temperatura_ADC);
				}

				break;
			case 2:
				bateria_ADC = (uint32_t)((ADC*2597/(1023-ADC)-259)/100)/(324/100)-8;
				ADMUX	= 0b01000000;
				break;
		}
		if(cont_canal<2){
			cont_canal++;
		}else{
			cont_canal = 0;
		}
		
		*flag_disparo = 0;		
	}
}


void acelerador(uint8_t *flag_disparo, uint32_t velocidade, uint32_t acelerador_ADC2){
	if(*flag_disparo){
		
		uint32_t ADCp;
		ADCp = acelerador_ADC2*(100.0/1022.0);  // Calcula o percentual de aceleração
		
		/*
		          _____ ______ _      ______ _____            _____   ____  _____
		    /\   / ____|  ____| |    |  ____|  __ \     /\   |  __ \ / __ \|  __ \
		   /  \ | |    | |__  | |    | |__  | |__) |   /  \  | |  | | |  | | |__) |
		  / /\ \| |    |  __| | |    |  __| |  _  /   / /\ \ | |  | | |  | |  _  /
		 / ____ \ |____| |____| |____| |____| | \ \  / ____ \| |__| | |__| | | \ \
		/_/    \_\_____|______|______|______|_|  \_\/_/    \_\_____/ \____/|_|  \_\
				  
		Condições para uso do acelerador:
		
		Distância maior que 3m
		OU
		Distância menor que 3m e velocidade menor que 20km/h
		OU
		Aceleração menor que 10%
		
		Caso contrário a aceleração será limitada em 10%.
		Dessa forma ao detectar uma possível colisão o acelerador fica limitado a 10% ou menos (idealmente o motorista deve pisar no freio...)
				
		*/
		if(distancia_cm > 300 || (distancia_cm < 300 && (uint32_t)velocidade < 20) || ADCp < 10) // Análise das condições
			OCR2B	= (ADCp/100.0)*255.0; // Atualiza o valor do dutycycle do PWM de acordo com o percentual de aceleração
		else{
			OCR2B	= (0.1)*255.0; // Limita a aceleração em 10%
		}
		*flag_disparo = 0;
	}
}

void anima_LCD(uint16_t velocidade_carro, uint16_t diametro_pneu_cm, uint16_t rpm_motor, float distancia_hodometro_km, uint8_t *flag_disparo, float distancia_proximidade, float temperaturabat, float batt, uint8_t *estado_anterior, uint8_t *buzzerr){
	if(*flag_disparo){
		
		unsigned char diametro_pneu_cm_string[4];
		unsigned char rpm_motor_string[6];
		unsigned char distancia_hodometro_km_string[8];
		unsigned char velocidade_string[8];
		unsigned char distancia_proximidade_string[8];
		unsigned char temperaturabat_string[8];
		unsigned char batt_string[8];

		// Transforma as variáveis em char
		sprintf(diametro_pneu_cm_string, "%u", diametro_pneu_cm);
		sprintf(rpm_motor_string, "%u", rpm_motor);
		sprintf(distancia_hodometro_km_string, "%u", (uint16_t)distancia_hodometro_km);
		sprintf(velocidade_string, "%u", velocidade_carro);
		sprintf(distancia_proximidade_string, "%u", (uint16_t)distancia_proximidade);
		sprintf(temperaturabat_string, "%u", (uint16_t)temperaturabat);
		sprintf(batt_string, "%u", (uint16_t)batt);
		
		GLCD_Clear();

		if ((PIND &	(1<<6)) && (PIND &	(1<<7)) ){
			
			GLCD_FillRectangle(34,0,94,15,GLCD_Black);
			if(distancia_proximidade<10){
				GLCD_GotoXY(48, 30);
				GLCD_PrintString("PARAR!");
				*buzzerr= 0;
				PORTC |= 0b00001000;
			}			
			if(distancia_proximidade>=10)
			GLCD_DrawLine(34,20,94,20,GLCD_Black);
			if(distancia_proximidade>=25){
				GLCD_DrawLine(29,25,99,25,GLCD_Black);
				*buzzerr= 2;
			}
			if(distancia_proximidade>=50){
				GLCD_DrawLine(24,30,104,30,GLCD_Black);
				*buzzerr= 1;
			}
			if(distancia_proximidade>=100){
				GLCD_DrawLine(19,35,109,35,GLCD_Black);
				*buzzerr= 1;
			}
			if(distancia_proximidade>=150){
				GLCD_DrawLine(14,40,114,40,GLCD_Black);
				*buzzerr= 0;
				PORTC &= 0b11110111;
			}
			
			GLCD_GotoXY(1, 55);
			GLCD_PrintString("Dist: ");
			GLCD_PrintString(distancia_proximidade_string);
			GLCD_PrintString("cm");
			
		}else{
			*buzzerr= 0;
			PORTC &= 0b11110111;
			GLCD_GotoXY(1, 1);
			GLCD_PrintString("Vel: ");
			GLCD_PrintString(velocidade_string);
			GLCD_PrintString("Km/h");
		
			GLCD_GotoXY(1, 13);
			GLCD_PrintString("Diam: ");
			GLCD_PrintString(diametro_pneu_cm_string);
			GLCD_PrintString("cm");
		
			GLCD_GotoXY(1, 25);
			GLCD_PrintString("RPM: ");
			GLCD_PrintString(rpm_motor_string);
		
			GLCD_GotoXY(1, 37);
			GLCD_PrintString("Dist: ");
			GLCD_PrintString(distancia_proximidade_string);
			GLCD_PrintString("cm");
		
			GLCD_GotoXY(1, 49);
			GLCD_PrintString("Odo: ");
			GLCD_PrintString(distancia_hodometro_km_string);
			GLCD_PrintString("Km");
			if((PINB&0b01000000)==0){
				GLCD_PrintString("  Farol");
			}
		
			if((PINB&0b00100000)==0 && *estado_anterior == 1){
				GLCD_GotoXY(98, 37);
				GLCD_PrintString(">");
			}
			if((PINB&0b00010000)==0 && *estado_anterior == 1){
				GLCD_GotoXY(90, 37);
				GLCD_PrintString("<");
			}
		
			GLCD_GotoXY(100, 1);
			GLCD_PrintString(temperaturabat_string);
			GLCD_PrintString("C");
		
			GLCD_GotoXY(90, 13);
			GLCD_PrintString(batt_string);
			GLCD_PrintString("%");
		
			GLCD_GotoXY(118, 49);
		
			// Consulta o modo de direção atual
			if ((PIND &	(1<<7)) && (!(PIND &	(1<<6)))){
				GLCD_PrintString("D");
			}
			if ((PIND &	(1<<6)) && (!(PIND &	(1<<7))) ){
				GLCD_PrintString("P");
			}
			if ((PIND &	(1<<6)) && (PIND &	(1<<7)) ){
				GLCD_PrintString("R");
			}
		}
		
		GLCD_Render();
		*flag_disparo = 0; // Redefine flag de disparo
		
	}
}