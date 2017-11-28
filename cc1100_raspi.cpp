#include "cc1100_raspi.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>

#include <getopt.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#define PACKAGE    "CC1100 SW"
#define VERSION_SW "0.9.6"
#define BUZZ_SIZE 1024

#define INTERVAL_1S_TIMER 1000
#define CLEAR(x) memset(x,'\0',60)
#define CLEAR2(x) memset(x,'\0',255)

//--------------------------[Global CC1100 variables]--------------------------
uint8_t Tx_fifo[FIFOBUFFER], Rx_fifo[FIFOBUFFER];
uint8_t My_addr, Tx_addr, Rx_addr, Pktlen, pktlen, Lqi, Rssi;
uint8_t rx_addr,sender,lqi;
int8_t rssi_dbm;

int cc1100_freq_select, cc1100_mode_select, cc1100_channel_select;

uint32_t prev_millis_1s_timer = 0;

uint8_t cc1100_debug = 0;								//set CC1100 lib in no-debug output mode
uint8_t tx_retries = 1;
uint8_t rx_demo_addr = 3;
int interval = 1000;
char buff[BUZZ_SIZE];

CC1100 cc1100;

//-------------------------- [End] --------------------------
void print_help(int exval) {
	printf("%s,%s by CW\r\n", PACKAGE, VERSION_SW);
	printf("%s [-h] [-V] [-a My_Addr] [-r RxDemo_Addr] [-i Msg_Interval] [-t tx_retries] [-c channel] [-f frequency]\r\n", PACKAGE);
	printf("          [-m modulation]\n\r\n\r");
	printf("  -h              			print this help and exit\r\n");
	printf("  -V              			print version and exit\r\n\r\n");
	printf("  -v              			set verbose flag\r\n");
	printf("  -a my address [1-255] 		set my address\r\n\r\n");
	printf("  -r rx address [1-255] 	  	set RxDemo receiver address\r\n\r\n");
	printf("  -i interval ms[1-6000] 	  	sets message interval timing\r\n\r\n");
	printf("  -t tx_retries [0-255] 	  	sets message send retries\r\n\r\n");
	printf("  -c channel 	[1-255] 		set transmit channel\r\n");
	printf("  -f frequency  [315,434,868,915]  	set ISM band\r\n\r\n");
	printf("  -m modulation [100,250,500] 		set modulation\r\n\r\n");

	exit(exval);
}


//|============================ Main ============================|
int main(int argc, char *argv[]) {
	//------------- command line option parser -------------------
	int opt;
	// no arguments given
	if(argc == 1) {
		fprintf(stderr, "This program needs arguments....\n\r\n\r");
		print_help(1);
	}

	while((opt = getopt(argc, argv, "hVva:r:i:t:c:f:m:")) != -1) {
		switch(opt) {
		case 'h':
			print_help(0);
			break;
		case 'V':
			printf("%s %s\n\n", PACKAGE, VERSION_SW);
			exit(0);
			break;
		case 'v':
			printf("%s: Verbose option is set `%c'\n", PACKAGE, optopt);
			cc1100_debug = 1;
			break;
		case 'a':
			My_addr = atoi (optarg);
			break;
		case 'r':
			rx_demo_addr = atoi (optarg);
			break;
		case 'i':
			interval = atoi (optarg);
			break;
		case 't':
			tx_retries = atoi (optarg);
			break;
		case 'c':
			cc1100_channel_select = atoi (optarg);
			break;
		case 'f':
			cc1100_freq_select = atoi (optarg);
			switch(cc1100_freq_select){
				case 315:
					cc1100_freq_select = 1;
					break;
				case 434:
					cc1100_freq_select = 2;
					break;
				case 868:
					cc1100_freq_select = 3;
					break;
				case 915:
					cc1100_freq_select = 4;
					break;
				}
			break;
			case 'm':
				cc1100_mode_select = atoi (optarg);

				switch(cc1100_mode_select){
				case 1:
					cc1100_mode_select = 1;
					break;
				case 38:
					cc1100_mode_select = 2;
					break;

				case 100:
					cc1100_mode_select = 3;
					break;
				case 250:
					cc1100_mode_select = 4;
					break;
				case 500:
					cc1100_mode_select = 5;
					break;
				}
				break;
				case ':':
					fprintf(stderr, "%s: Error - Option `%c' needs a value\n\n", PACKAGE, optopt);
					print_help(1);
					break;
				case '?':
					fprintf(stderr, "%s: Error - No such option: `%c'\n\n", PACKAGE, optopt);
					print_help(1);
		}
	}

	// print all remaining options
	for(; optind < argc; optind++)
		printf("argument: %s\n", argv[optind]);

	//------------- welcome message ------------------------
	printf("Raspberry CC1101 SPI Library test\n");

	//------------- hardware setup ------------------------
	
	wiringPiSetup();			//setup wiringPi library

	cc1100.begin(My_addr);			//setup cc1000 RF IC
	cc1100.sidle();
	//cc1100.set_mode(0x04);                //set modulation mode 1 = GFSK_1_2_kb; 2 = GFSK_38_4_kb; 3 = GFSK_100_kb; 4 = MSK_250_kb; 5 = MSK_500_kb; 6 = OOK_4_8_kb
	//cc1100.set_ISM(0x03);                 //set ISM Band 1=315MHz; 2=433MHz; 3=868MHz; 4=915MHz
	//cc1100.set_channel(0x01);             //set channel
	cc1100.set_output_power_level(0);       //set PA level
	//cc1100.set_myaddr(0x05);
	cc1100.receive();

	cc1100.show_main_settings();             //shows setting debug messages to UART
	cc1100.show_register_settings();

	//TX Variables
	int lstring, lapdu, cu, mod, splitString, totalCounter=0, mainCounter;
	char mystr[60];
	char *apdu;

	//RX Variables
	char str[256];
	int dataCounter, stringCounter = 0;
	CLEAR2(str);
	FILE *filep; 

	Rx_addr = rx_demo_addr;                     //receiver address
	//------------------------- RX Main Loop ------------------------
	printf("Antes del for");
	while (1) {
		//printf("Entro al for");
		delay(1);                            //delay to reduce system load
		if (cc1100.packet_available())		 //checks if a packed is available
		{
			//printf("hay datos recividos");
			cc1100.get_payload(Rx_fifo, pktlen, rx_addr, sender, rssi_dbm, lqi); //stores the payload data to Rx_fifo
			if(pktlen>0){
				printf("Packet lenght: %d", pktlen);
				for (dataCounter = 3; dataCounter<=pktlen; dataCounter++){ 
					if (Rx_fifo[dataCounter] == 'H'){
						printf("\nRX_FIFO: %c",Rx_fifo[dataCounter]);
						printf("\nGuardando: %s\n",str);
						filep = fopen("rapdu.txt", "w");
						fprintf(filep, str);
						fclose(filep);
						stringCounter = 0;
						CLEAR2(str);
						break;
					}

					if (isalpha(Rx_fifo[dataCounter]) || isdigit(Rx_fifo[dataCounter])) {
						str[stringCounter] = Rx_fifo[dataCounter];
					}
					stringCounter += 1;
				}
			}
		}
		else {
			//------------------------- TX Main Loop ------------------------
			FILE * file;
			file = fopen("sapdu.txt", "r");
			if (file){ //Hay datos para enviar por SDR?
		    	fgets(buff, BUZZ_SIZE, file); //Copia os datos al buffer para eliminar archivo
		    	printf("\nString read: %s\n", buff);
		    	fclose(file);
  				remove("sapdu.txt");
   				if(remove("sapdu.txt") == 0) {
      				printf("File deleted successfully");
   				} else {
      				printf("Error: unable to delete the file");
   				}
				apdu = buff; //apdu es el source principal para el envio de datos
				//Ejemplo:
                //apdu = 6F2D840E325041592E5359532E4444463031A51BBF0C1861164F07A0000000031010500B56697361204372656469749000H = 99 length
				//6F2D840E325041592E5359532E4444463031A51BBF0C1861164F07A0000 = 59 chunk  
				printf("\nApdu:%s", apdu);
				lstring = 59; //The payload has to be 60 maximum
				lapdu = strlen(apdu);
				if (lapdu >0){
					cu = lapdu / lstring; // 1 = 59
					mod = lapdu % lstring; // 40
					for (mainCounter=0; mainCounter<=cu; mainCounter++) //Numero de ciclos que se necesitan para enviar los paquetes en chunks de 60
					{
						Pktlen = 4; // Se reservan 3 bytes: 1 rx, 1 tx, 1 lenght, + no olvidar el [0] 
						if (mainCounter < cu){ // 0 < 1
							memcpy(mystr, &apdu[totalCounter], lstring); //Copia a mystr de apdu[0] hasta 59
							totalCounter+=lstring; // contador que lleva control de lo copiado
							mystr[totalCounter] = '\0';
							Pktlen += lstring; // agrega en len el tamano del chunk = 63
							printf("\nMy string1 es: %s\n", &mystr);
						}else{
							memcpy(mystr, &apdu[totalCounter], mod);
							Pktlen += mod;
							mystr[totalCounter] = '\0';
							totalCounter = 0;
							printf("\nMy string es: %c\n", mystr);
						}
			    		for(splitString = 3; splitString < Pktlen; splitString++){
			    			Tx_fifo[splitString] = (uint8_t)(mystr[splitString-3]);
			    		}
						//int t = strlen(mystr);
						while (1){
					    		uint8_t res = cc1100.sent_packet(My_addr, Rx_addr, Tx_fifo, Pktlen, tx_retries);
					    		if(res == 1)    //sents package over air. ACK is received via GPIO polling
					    		{
			              printf("transmitted");// tx_timestamp: %ums \r\n\r\n", time_stamp);
			              break;
					    		}
						}
				    	CLEAR(mystr);
					}
				}
			}
		}
	}
    return 0;
}
//-------------------- END OF MAIN --------------------------------
