#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include "hoverboard-proxy.h"

static bool const print_debug     {false};
static bool const print_debug_max {false};

static char const * filename {"/dev/ttyAMA1"};

// Setpoint and feedback data format for client-server communication (PoD)
struct setpoint_and_feedback_data
{
    parameters_control_instruction_format control;
    parameters_control_acknowledge_format feedback;
};

u8 sendSteerIdentifier = {0};

//----------------------------------------------------------------------------
// Calculate CRC
//----------------------------------------------------------------------------
u16 CalcCRC(u8 *ptr, int count)
{
  u16  crc;
  u8 i;
  crc = 0;
  while (--count >= 0)
  {
    crc = crc ^ (u16) *ptr++ << 8;
    i = 8;
    do
    {
      if (crc & 0x8000)
      {
        crc = crc << 1 ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    } while(--i);
  }
  return (crc);
}

void hexDump(const char *desc, void *addr, int len)
{
    int i;
    unsigned char buff[17];
    unsigned char *pc = (unsigned char*)addr;

    // Output description if given.
    if (desc != NULL)
        printf ("%s:\n", desc);

    // Process every byte in the data.
    for (i = 0; i < len; i++) {
        // Multiple of 16 means new line (with line offset).

        if ((i % 16) == 0) {
            // Just don't print ASCII for the zeroth line.
            if (i != 0)
                printf("  %s\n", buff);

            // Output the offset.
            printf("  %04x ", i);
        }

        // Now the hex code for the specific character.
        printf(" %02x", pc[i]);

        // And store a printable ASCII character for later.
        if ((pc[i] < 0x20) || (pc[i] > 0x7e)) {
            buff[i % 16] = '.';
        } else {
            buff[i % 16] = pc[i];
        }

        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly 16 characters.
    while ((i % 16) != 0) {
        printf("   ");
        i++;
    }

    // And print the final ASCII bit.
    printf("  %s\n", buff);
}

u16 big2little16(u8 *buffer, int offset)
{
    union
    {
        s16 v;
        char b[2];
    } dst;

    dst.b[1] = buffer[offset + 0];
    dst.b[0] = buffer[offset + 1];
    return dst.v;
}

u32 big2little32(u8 *buffer, int offset)
{
    union
    {
        s32 v;
        char b[8];
    } dst;

    dst.b[7] = 0;
    dst.b[6] = 0;
    dst.b[5] = 0;
    dst.b[4] = 0;
    dst.b[3] = buffer[offset + 0];
    dst.b[2] = buffer[offset + 1];
    dst.b[1] = buffer[offset + 2];
    dst.b[0] = buffer[offset + 3];
    return dst.v;
}

// Task handling communication with Hoverboard
// - parameter (input/output) : the client/server shared-memory buffer
void hoverboard_protocol(setpoint_and_feedback_data * control_block)
{
    // Check control block once
    if(control_block==NULL) exit(EXIT_FAILURE);
    
    // Initialize all control/speed and steer to 0
    control_block->control.leftSpeed = 0;
    control_block->control.rightSpeed = 0;

    // reference : https://www.pololu.com/docs/0J73/15.5

    // Open serial device
    int fd = open(
        filename,           // UART device
        O_RDWR | O_NOCTTY   // Read-Write access + Not the terminal of this process
    );
    if (fd < 0)
    {
        printf("%s: failed to open UART device\n", __func__);
        exit(EXIT_FAILURE);
    }

    // Flush away any bytes previously read or written.
    int result = tcflush(fd, TCIOFLUSH);
    if (result)
    {
        printf("%s: failed to flush\r\n", __func__);
        close(fd);
        exit(EXIT_FAILURE);
    }

    // Get the current configuration of the serial port.
    struct termios options;
    result = tcgetattr(fd, &options);
    if (result)
    {
        printf("%s: failed to allocate UART TTY instance\n", __func__);
        close(fd);
        exit(EXIT_FAILURE);
    }

    // note : Les fonctions termios ??tablissent une interface g??n??rale 
    //        sous forme de terminal, permettant de contr??ler les ports 
    //        de communication asynchrone.  

    // Turn off any options that might interfere with our ability to send and
    // receive raw binary bytes.
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF); // IGNPAR ?
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Set up timeouts: Calls to read() will return as soon as there is
    // at least one byte available or when 100 ms has passed.
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;

    // Set baud rate
    cfsetospeed(&options, B115200);

    // Apply options
    result = tcsetattr(fd, TCSANOW, &options);
    if (result)
    {
        printf("%s: failed to set attributes\r\n", __func__);
        close(fd);
        exit(EXIT_FAILURE);
    }

    // control-loop
    for (;;)
    {
        // heart-beat
        if (print_debug_max)
        {
            printf(".");
        }

        // Flush away any bytes previously read or written.
        int result = tcflush(fd, TCIOFLUSH);
        if (result)
        {
            printf("%s: failed to flush\r\n", __func__);
            close(fd);
            exit(EXIT_FAILURE);
        }

        /*
         * Encode a CONTROL frame
         */

        size_t const tx_buffer_size {11};

        // Build the frame
        u8 tx_buffer[tx_buffer_size]
        {
            0x2F,               // header '/'
        };

	int direction = -1;
	tx_buffer[1] = ((control_block->control.leftSpeed * direction) >> 8) & 0xFF;
	tx_buffer[2] = (control_block->control.leftSpeed * direction) & 0xFF;
	tx_buffer[3] = ((control_block->control.rightSpeed * direction) >> 8) & 0xFF;
	tx_buffer[4] = (control_block->control.rightSpeed * direction) & 0xFF;
	tx_buffer[5] = sendSteerIdentifier;
	switch(sendSteerIdentifier)
        {
	    case PID_P:
		tx_buffer[6] = (control_block->control.pid_p >> 8) & 0xFF;
                tx_buffer[7] = control_block->control.pid_p & 0xFF;
		break;
	    case PID_I:
		tx_buffer[6] = (control_block->control.pid_i >> 8) & 0xFF;
                tx_buffer[7] = control_block->control.pid_i & 0xFF;
		break;
	    case PID_D:
		tx_buffer[6] = (control_block->control.pid_d >> 8) & 0xFF;
                tx_buffer[7] = control_block->control.pid_d & 0xFF;
		break;
	    case LED_S:
		tx_buffer[6] = (control_block->control.led_l >> 8) & 0xFF;
                tx_buffer[7] = control_block->control.led_l & 0xFF;
		break;
	    case LED_M:
		tx_buffer[6] = (control_block->control.led_r >> 8) & 0xFF;
                tx_buffer[7] = control_block->control.led_r & 0xFF;
		break;
	    case BACK_LED_S:
		tx_buffer[6] = (control_block->control.back_led_l >> 8) & 0xFF;
                tx_buffer[7] = control_block->control.back_led_l & 0xFF;
		break;
	    case BACK_LED_M:
		tx_buffer[6] = (control_block->control.back_led_r >> 8) & 0xFF;
                tx_buffer[7] = control_block->control.back_led_r & 0xFF;
		break;
	    case BUZZER:
		tx_buffer[6] = (control_block->control.buzzer >> 8) & 0xFF;
                tx_buffer[7] = control_block->control.buzzer & 0xFF;
		break;
              default:
                  break;
	}
	sendSteerIdentifier++;
	if (sendSteerIdentifier > 7)
	    sendSteerIdentifier = 0;

        // Checksum
	u16 crc = CalcCRC(tx_buffer, tx_buffer_size - 3);
	tx_buffer[8] = (crc >> 8) & 0xFF;
	tx_buffer[9] = crc & 0xFF;

	// Stop byte
        tx_buffer[10] = 0x0A;

        if (print_debug)
        {
	    hexDump("tx_buffer", (void *)tx_buffer, 11);
        }

        // Send
        result = write(fd, (char *)tx_buffer, tx_buffer_size);
        if(result != (ssize_t)tx_buffer_size)
        {
            printf("failed to write to port");
            close(fd);
            exit(EXIT_FAILURE);
        }
        if (print_debug_max)
        {
    	    printf("uart writen:%d\n",result);
    	}

        /*
        result = tcdrain(fd);
        if(result)
        {
            printf("failed to drain port");
            close(fd);
            exit(EXIT_FAILURE);
        }
        */

        // Read buffer
    	// If we do not get data within 1 second we assume Hoverboard stopped sending and we send again
        size_t const rx_buffer_size { 128 };
        u8 rx_buffer[rx_buffer_size] {0};
    	size_t const expected_lenght {15}; 
        size_t received_length {0};
        size_t bytes_to_read {1};
    	while(received_length<expected_lenght)
        {
	    bytes_to_read = received_length>0 ? expected_lenght-received_length : 1;
	    
            ssize_t read_length = read(
                fd,
                (char*)(rx_buffer+received_length),
                expected_lenght-received_length
            );
            if(read_length<0)
            {
                printf("failed to read from port");
                close(fd);
                exit(EXIT_FAILURE);
            }
	    // start reading at beginning of frame
	    if(rx_buffer[0]!=0x2F)
	    {
                if (print_debug_max)
		{
		    printf("ignoring 0x%.1X\n", rx_buffer[0]);
		}
		continue;
	    }
            if (print_debug_max)
            {
                printf("uart read:%lu, waiting for %lu/%lu\n",read_length, received_length,expected_lenght);
            }
            if(read_length==0)
            {
              // time out
              break;
            }
    	    received_length += read_length;
    	}
        if (print_debug_max)
        {
            printf("uart read: %lu/%lu\n",received_length,expected_lenght);
        }

        // not enough data received ?
    	if(received_length<expected_lenght) continue;

        /*
         * Decode a CONTROL ACK frame
         */

        if (print_debug)
        {
	    hexDump("rx_buffer", (void *)rx_buffer, 24);
        }

        bool const rx_header_check {
                    (rx_buffer[0]==0x2F)
                &&  (rx_buffer[expected_lenght-1]==0x0A)
        };
        if(!rx_header_check)
        {
            // log
            if (print_debug)
    	    {
                printf("RX frame error : header invalid!\n");
    	    }
            // next
            continue;
        }

	crc = CalcCRC(rx_buffer, expected_lenght - 3);
	if ( rx_buffer[expected_lenght - 3] != ((crc >> 8) & 0xFF) ||
                rx_buffer[expected_lenght - 2] != (crc & 0xFF))
        {
            // log
            if (print_debug)
    	    {
                printf("RX cecksum error : [received: 0x%.1X 0x%.1X,expected: 0x%.1X 0x%.1X!\n",
		       	rx_buffer[expected_lenght - 3],
		       	rx_buffer[expected_lenght - 2],
		       	(crc >> 8) & 0xFF,
		       	crc & 0xFF);
    	    }
            // next
            continue;
        }

        // decode parameters
        u8 recSteerIdentifier;
        control_block->feedback.responseId += 1;
        control_block->feedback.speedMaster = big2little32(rx_buffer, 1);
        control_block->feedback.speedSlave = big2little32(rx_buffer, 5);
	control_block->feedback.speedMaster *= -1;
	recSteerIdentifier = rx_buffer[9];
	switch(recSteerIdentifier)
       	{
              case BAT_U:
                  control_block->feedback.battery = big2little16(rx_buffer, 10);
                  break;
              case MOT_S_I:
                  control_block->feedback.currentSlave = big2little16(rx_buffer, 10);
                  break;
              case MOT_M_I:
                  control_block->feedback.currentMaster = big2little16(rx_buffer, 10);
                  break;
              case MOT_S_D:
                  control_block->feedback.debugSlave = big2little16(rx_buffer, 10);
                  break;
              case MOT_M_D:
                  control_block->feedback.debugMaster = big2little16(rx_buffer, 10);
                  break;
              default:
                  break;
	}

        // log
        if (print_debug)
        {
	    hexDump("control_block", (void *)control_block, sizeof(parameters_control_instruction_format) + sizeof(parameters_control_acknowledge_format));
            printf("responseId: %hu\n", control_block->feedback.responseId);
            printf("encM: %lu\n", control_block->feedback.speedMaster);
            printf("encS: %lu\n", control_block->feedback.speedSlave);
            printf("battery: %u\n", control_block->feedback.battery);
            printf("currentMaster: %u\n", control_block->feedback.currentMaster);
            printf("debugMaster: %u\n", control_block->feedback.debugMaster);
            printf("currentSlave: %u\n", control_block->feedback.currentSlave);
            printf("debugSlave: %u\n", control_block->feedback.debugSlave);
	}


    }
}

int main(int argc, char *argv[])
{
    // allocate a shared-memory buffer for setpoint and feedback data exchange between clients and server
    void * control_block = mmap(NULL, sizeof(setpoint_and_feedback_data), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    memset(control_block, 0, sizeof(setpoint_and_feedback_data));

    /* start UART protocol with Hoverboard */
    int pid = fork();
    if (pid == 0)
    {
	   hoverboard_protocol(reinterpret_cast<setpoint_and_feedback_data*>(control_block));
    }

    /* Create local socket. */

    int connection_socket = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (connection_socket == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    /*
     * For portability clear the whole structure, since some
     * implementations have additional (nonstandard) fields in
     * the structure.
     */

    struct sockaddr_un name;
    memset(&name, 0, sizeof(name));

    /* Bind socket to socket name. */

    name.sun_family = AF_UNIX;
    strncpy(name.sun_path, SOCKET_NAME, sizeof(name.sun_path) - 1);
    int ret = unlink(SOCKET_NAME);
    if (ret == -1 && errno != ENOENT) {
        printf("unlink %s", strerror(errno));
        perror("unlink");
        exit(EXIT_FAILURE);
    }
    ret = bind(connection_socket, (const struct sockaddr *) &name,
               sizeof(name));
    if (ret == -1) {
        perror("bind");
        exit(EXIT_FAILURE);
    }

    /*
     * Prepare for accepting connections. The backlog size is set
     * to 20. So while one request is being processed other requests
     * can be waiting.
     */

    ret = listen(connection_socket, 20);
    if (ret == -1) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    /* This is the main loop for handling connections. */

    int data_socket {0};
    u8 r_buffer[38];
    u8 s_buffer[38];
    for (;;) {

        /* Wait for incoming connection. */

        data_socket = accept(connection_socket, NULL, NULL);
        if (data_socket == -1) {
            perror("accept");
            exit(EXIT_FAILURE);
        }

	pid = fork();
        if (pid == 0) {

            for (;;) {

                /* Wait for next data packet. */

                ret = read(data_socket, r_buffer, sizeof(r_buffer));
                if (ret == -1) {
                    /* client terminated? */
                    exit(EXIT_SUCCESS);
                }

                /* Handle commands. */
		int offset;
                offset = 0;
                if(r_buffer[1] == INST_SETSPEED && r_buffer[0] == 6) {
                    memcpy((char*)control_block + offset, &r_buffer[2], 4);
                    s_buffer[0]= 2;
                    s_buffer[1]= INST_SETSPEED;
                }

                if(r_buffer[1] == INST_GETCB && r_buffer[0] == 2) {
                    s_buffer[0]= 2 + sizeof(parameters_control_instruction_format) + sizeof(parameters_control_acknowledge_format);
                    s_buffer[1]= INST_GETCB;
                    memcpy(&s_buffer[2], (char*)control_block + offset, sizeof(parameters_control_instruction_format) + sizeof(parameters_control_acknowledge_format));
                }

                offset += 2 * sizeof(s16);
                if(r_buffer[1] == INST_SETPID && r_buffer[0] == 8) {
                    memcpy((char*)control_block + offset, &r_buffer[2], 6);
                    s_buffer[0]= 2;
                    s_buffer[1]= INST_SETPID;
                }

                offset += 3 * sizeof(s16);
                if(r_buffer[1] == INST_SETLED && r_buffer[0] == 6) {
                    memcpy((char*)control_block + offset, &r_buffer[2], 4);
                    s_buffer[0]= 2;
                    s_buffer[1]= INST_SETLED;
                }

                offset += 2 * sizeof(s16);
                if(r_buffer[1] == INST_SETBACK_LED && r_buffer[0] == 6) {
                    memcpy((char*)control_block + offset, &r_buffer[2], 4);
                    s_buffer[0]= 2;
                    s_buffer[1]= INST_SETBACK_LED;
                }

                offset += 2 * sizeof(s16);
                if(r_buffer[1] == INST_SETBUZZER && r_buffer[0] == 4) {
                    memcpy((char*)control_block + offset, &r_buffer[2], 2);
                    s_buffer[0]= 2;
                    s_buffer[1]= INST_SETBUZZER;
                }

                offset += sizeof(s16);
		offset += 2; // skip responseId
                if(r_buffer[1] == INST_GETSPEED && r_buffer[0] == 2) {
                    s_buffer[0]= 2 + 16;
                    s_buffer[1]= INST_GETSPEED;
                    memcpy(&s_buffer[2], (char*)control_block + offset, 16);
                }

		offset += 16;
                if(r_buffer[1] == INST_GETBATT && r_buffer[0] == 2) {
                    s_buffer[0]= 2 + sizeof(s16);
                    s_buffer[1]= INST_GETBATT;
                    memcpy(&s_buffer[2], (char*)control_block + offset, sizeof(s16));
                }

		offset += sizeof(s16);
                if(r_buffer[1] == INST_GETCURR && r_buffer[0] == 2) {
                    s_buffer[0]= 2 + 2*sizeof(s16);
                    s_buffer[1]= INST_GETCURR;
                    memcpy(&s_buffer[2], (char*)control_block + offset, sizeof(16));
                    memcpy(&s_buffer[4], (char*)control_block + offset + 2*sizeof(s16), sizeof(s16));
                }

		offset += sizeof(s16);
                if(r_buffer[1] == INST_GETDEBUG && r_buffer[0] == 2) {
                    s_buffer[0]= 2 + 2*sizeof(s16);
                    s_buffer[1]= INST_GETDEBUG;
                    memcpy(&s_buffer[2], (char*)control_block + offset, sizeof(16));
                    memcpy(&s_buffer[4], (char*)control_block + offset + 2*sizeof(s16), sizeof(s16));
                }

                /* Send result. */

                ret = write(data_socket, s_buffer, s_buffer[0]);
                if (ret == -1) {
                    /* client terminated? */
                    exit(EXIT_SUCCESS);
                }

            }
        }
        else {
            close(data_socket);
            continue;
        }
    }

    close(connection_socket);

    /* Unlink the socket. */

    unlink(SOCKET_NAME);

    exit(EXIT_SUCCESS);
}
