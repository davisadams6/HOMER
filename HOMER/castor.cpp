#include <iostream>
#include <string.h>
#include <bitset>

#include <termios.h>

#include <fcntl.h>

#include <libserial/SerialPort.h>
#include <cstdlib>
#include <fstream>
#include <unistd.h>

#include "motor_command.h"
#include "castor.h"

#define CASTOR_BAUDRATE 115200
#define CASTOR_PORT_1 "/dev/ttyUSB0"
#define CASTOR_PORT_2 "/dev/ttyUSB1"
#define CASTOR_PORT_3 "/dev/ttyUSB2"

const char *castor_ports[] = {CASTOR_PORT_1,CASTOR_PORT_2,CASTOR_PORT_3};

//castor_params castors[3];
//data32 pval;
using namespace LibSerial ;

int init_castors(SerialPort castor[3],int *castor_fd) {
	

	for(int i=0;i<3;i++){
		castor[i].Open(castor_ports[i]);
		if (!castor[i].IsOpen()) {
			printf("Castor %d did not open correctly!", i+1);
			return EXIT_FAILURE;
		}
		
		castor[i].SetBaudRate(BaudRate::BAUD_115200);
		castor[i].SetCharacterSize(CharacterSize::CHAR_SIZE_8);
		castor[i].SetStopBits(StopBits::STOP_BITS_1);
		castor[i].SetParity(Parity::PARITY_NONE);
		castor[i].SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
		castor_fd[i] = castor[i].GetFileDescriptor();
	}

}

