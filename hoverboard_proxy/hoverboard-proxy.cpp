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

    // note : Les fonctions termios établissent une interface générale 
    //        sous forme de terminal, permettant de contrôler les ports 
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
    cfsetospeed(&options, B19200);

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

        // Compute the size of the payload (parameters length + 2)
        size_t const tx_payload_length { sizeof(parameters_control_instruction_format) + 2 };

        // Compute the size of the frame
        size_t const tx_buffer_size { tx_payload_length + 2 };

        // Build the frame
        u8 tx_buffer[tx_buffer_size]
        {
            0x2F,               // header '/'
        };

	tx_buffer[1] = (control_block->control.leftSpeed >> 8) & 0xFF;
	tx_buffer[2] = control_block->control.leftSpeed & 0xFF;
	tx_buffer[3] = (control_block->control.rightSpeed >> 8) & 0xFF;
	tx_buffer[4] = control_block->control.rightSpeed & 0xFF;

        // Checksum
	u16 crc = CalcCRC(tx_buffer, tx_buffer_size - 3);
	tx_buffer[5] = (crc >> 8) & 0xFF;
	tx_buffer[6] = crc & 0xFF;

	// Stop byte
        tx_buffer[7] = 0x0A;

        if (print_debug)
        {
            printf("0x%.1X\n", tx_buffer[0]);
            printf("0x%.1X\n", tx_buffer[1]);
            printf("0x%.1X\n", tx_buffer[2]);
            printf("0x%.1X\n", tx_buffer[3]);
            printf("0x%.1X\n", tx_buffer[4]);
            printf("0x%.1X\n", tx_buffer[5]);
            printf("0x%.1X\n", tx_buffer[6]);
            printf("0x%.1X\n", tx_buffer[7]);
            printf("----------\n");
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
    	size_t const expected_lenght {4 + sizeof(parameters_control_acknowledge_format)}; // '/<data><crc>\n'
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
            printf("0x%.1X\n", rx_buffer[0]);
            printf("0x%.1X\n", rx_buffer[1]);
            printf("0x%.1X\n", rx_buffer[2]);
            printf("0x%.1X\n", rx_buffer[3]);
            printf("0x%.1X\n", rx_buffer[4]);
            printf("0x%.1X\n", rx_buffer[5]);
            printf("0x%.1X\n", rx_buffer[6]);
            printf("0x%.1X\n", rx_buffer[7]);
            printf("0x%.1X\n", rx_buffer[8]);
            printf("0x%.1X\n", rx_buffer[9]);
            printf("0x%.1X\n", rx_buffer[10]);
            printf("0x%.1X\n", rx_buffer[11]);
            printf("0x%.1X\n", rx_buffer[12]);
            printf("0x%.1X\n", rx_buffer[13]);
            printf("0x%.1X\n", rx_buffer[14]);
            printf("0x%.1X\n", rx_buffer[15]);
            printf("0x%.1X\n", rx_buffer[16]);
            printf("0x%.1X\n", rx_buffer[17]);
            printf("0x%.1X\n", rx_buffer[18]);
            printf("0x%.1X\n", rx_buffer[19]);
            printf("0x%.1X\n", rx_buffer[20]);
            printf("0x%.1X\n", rx_buffer[21]);
            printf("0x%.1X\n", rx_buffer[22]);
            printf("0x%.1X\n", rx_buffer[23]);
            printf("----------\n");
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
        memcpy(&control_block->feedback,rx_buffer+1,sizeof(parameters_control_acknowledge_format));

        // log
        if (print_debug)
        {
            printf("battery: %.2f\n", control_block->feedback.battery);
            printf("currentMaster: %.2f\n", control_block->feedback.currentMaster);
            printf("speedMaster: %.2f\n", control_block->feedback.speedMaster);
            printf("currentSlave: %.2f\n", control_block->feedback.currentSlave);
            printf("speedSlave: %.2f\n", control_block->feedback.speedSlave);
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
                    memcpy((char*)control_block + offset, &r_buffer[2], sizeof(parameters_control_instruction_format));
                    s_buffer[0]= 2;
                    s_buffer[1]= INST_SETSPEED;
                }

		offset = sizeof(parameters_control_instruction_format);
                if(r_buffer[1] == INST_GETBATT && r_buffer[0] == 2) {
                    s_buffer[0]= 2 + sizeof(float);
                    s_buffer[1]= INST_GETBATT;
                    memcpy(&s_buffer[2], (char*)control_block + offset, sizeof(float));
                }

		offset += 4;
                if(r_buffer[1] == INST_GETCURR && r_buffer[0] == 2) {
                    s_buffer[0]= 2 + 2*sizeof(float);
                    s_buffer[1]= INST_GETCURR;
                    memcpy(&s_buffer[2], (char*)control_block + offset, sizeof(float));
                    memcpy(&s_buffer[6], (char*)control_block + offset +6, sizeof(float));
                }

		offset += 4;
                if(r_buffer[1] == INST_GETSPEED && r_buffer[0] == 2) {
                    s_buffer[0]= 2 + 2*sizeof(float);
                    s_buffer[1]= INST_GETSPEED;
                    memcpy(&s_buffer[2], (char*)control_block + offset, sizeof(float));
                    memcpy(&s_buffer[6], (char*)control_block + offset +6, sizeof(float));
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
