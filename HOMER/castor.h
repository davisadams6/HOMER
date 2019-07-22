#ifndef _CASTOR_H_
#define _CASTOR_H_

#include <libserial/SerialPort.h>
#include <cstdlib>
#include <fstream>
#include <unistd.h>

using namespace LibSerial ;

typedef struct {
	unsigned char id;
	int fd;
} castor_params;

int init_castors(SerialPort castor[3],int *castor_fd);
void cleanup_castor();

#endif /* _CASTOR_H_ */
