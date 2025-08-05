/*
 * vt100_moshe.c
 *
 *  Created on: Aug 5, 2025
 *      Author: moshe
 */


void vt100_process(void){
	uint8_t rx;

	while(1){
		HAL_UART_Receive(&huart2, &rx, 1, 1000);
		if(rx != 0x1B){
			palPutChar(rx); // Print regular character
			continue;
		}

		HAL_UART_Receive(&huart2, &rx, 1, 1000); // Read next character
		switch(rx){
			case '[': // CSI sequence
				// Read more characters until we reach a command character
				char csi_buf[16];
				uint8_t csi_len = 0;
				while(1){
					HAL_UART_Receive(&huart2, &rx, 1, 1000);
					if(rx >= 0x40 && rx <= 0x7E || csi_len >= sizeof(csi_buf)-1){
						csi_buf[csi_len++] = rx;
						csi_buf[csi_len] = '\0';
						palHandleCSI(csi_buf, csi_len); // Handle the CSI command
						break;
					} else {
						csi_buf[csi_len++] = rx;
					}
				}
				break;
			case ']':
			default:
				palPutChar(rx); // Print regular character
				break;
		}
	}
}
