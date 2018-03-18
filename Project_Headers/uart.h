#ifndef UART_H_
#define UART_H_

void initUART2(void);
void transmitByte(uint8_t byte);
void receiveByte(uint8_t *byte);
char getReceived(void);
int isReceived(void);
int isReadyToTransmit(void);
void transmitBytes(char bytes[], uint8_t length);
void printt(char buffer[]);


#endif /* UART_H_ */
