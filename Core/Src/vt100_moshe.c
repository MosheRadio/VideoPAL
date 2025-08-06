/*


#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart2;  // UART handle for receiving VT100 data

// Helper to parse a CSI (Control Sequence Introducer) sequence
static void parse_csi(void) {
    uint8_t rx;
    char    csi_buf[16];
    uint8_t csi_len = 0;
    int     p1 = 0, p2 = 0;
    char   *token;

    // Read bytes until we encounter the final character or buffer is nearly full
    while (1) {
        HAL_UART_Receive(&huart2, &rx, 1, 1000);
        // Final byte is in the range 0x40 to 0x7E
        if ((rx >= 0x40 && rx <= 0x7E) || (csi_len >= sizeof(csi_buf) - 1)) {
            csi_buf[csi_len++] = (char)rx;
            csi_buf[csi_len]   = '\0';
            break;
        }
        // Store parameter byte (digits or ';')
        csi_buf[csi_len++] = (char)rx;
    }

    // Split parameters at ';'
    token = strtok(csi_buf, ";");
    if (token) {
        p1 = atoi(token);
        token = strtok(NULL, ";");
        if (token) {
            p2 = atoi(token);
        }
    }

    // The final command character is the last in the buffer
    rx = (uint8_t)csi_buf[csi_len - 1];

    // Execute action based on the command
    switch (rx) {
        case 'H':  // Cursor position: row=p1, col=p2
            pal_set_cursor((uint8_t)p1, (uint8_t)p2);
            break;

        case 'J':  // Erase in display: p1=0 (below),1 (above),2 (entire)
            pal_clear_screen((uint8_t)p1);
            break;

         case 'A': // Cursor up by p1 lines
             pal_cursor_up(p1);
             break;
         case 'm': // Graphic rendition
             pal_set_style(p1, p2);
             break;

        default:
            // Unknown CSI command: ignore or handle as needed
            break;
    }
}

void pal_set_style(int p1, int p2) {
	switch(p1) {
		case 0: // Reset all attributes
			pal_reset_style();
			break;
		case 1: // Bold
			pal_set_bold(1);
			break;
		case 22: // Normal intensity
			pal_set_bold(0);
			break;
		case 30 ... 37: // Foreground color
			pal_set_foreground_color(p1 - 30);
			break;
		case 40 ... 47: // Background color
			pal_set_background_color(p1 - 40);
			break;
		default:
			// Unsupported style, ignore
			break;
	}

}

// Main VT100 processing loop
void vt100_process(void) {
    uint8_t rx;

    while (1) {
        // 1) Receive one byte via UART (polling)
        HAL_UART_Receive(&huart2, &rx, 1, 1000);

        // 2) If it's not ESC, treat as normal character and print
        if (rx != 0x1B) {
            palPutChar(rx);
            continue;
        }

        // 3) Received ESC, read next byte to identify sequence type
        HAL_UART_Receive(&huart2, &rx, 1, 1000);

        switch (rx) {
            case '[':
                // CSI sequence: parse parameters and execute
                parse_csi();
                break;

            case ']':
                // OSC (Operating System Command): skip until BEL or ESC '\\'
                // (Implementation can be added if OSC support is needed)
                break;

            // Add more escape sequences as needed:
            // case 'O': // SS3 (Function keys F1-F4)
            // case 'c': // Reset terminal

            default:

                palPutChar(rx);
                break;
        }
    }
}






#include "vt100.h"
#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef huart2;


static uint8_t rx_byte;


void uart_vt100_start(void)
{
    vt100_init();

    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
         Feed the received byte into the VT100 parser
        vt100_feed(rx_byte);

        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

*/
