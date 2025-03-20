#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/time.h>

#define ODO_LASER_PORT 24919  // Robot server port
#define CMD_PORT 31001  // Port for motion commands
#define SERVER_IP "192.38.66.89"  // Replace with your server's IP
#define LOG_DURATION 5  // Time in seconds to log data
#define INTERVAL 500000  // 0.5 seconds in microseconds

// Function to get current time in seconds with microsecond precision
double get_time_seconds() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + (tv.tv_usec / 1e6);
}

// Function to set up a client connection
int setup_client(int port) {
    int client_fd;
    struct sockaddr_in serv_addr;

    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        perror("Invalid address / Address not supported");
        close(client_fd);
        return -1;
    }

    if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection Failed");
        close(client_fd);
        return -1;
    }

    printf("Connected to server on port %d\n", port);
    return client_fd;
}

// Function to send a command
void send_command(int client_fd, const char* command) {
    send(client_fd, command, strlen(command), 0);
    usleep(INTERVAL);
}

// Main function
int main() {
    int odo_laser_fd = setup_client(ODO_LASER_PORT);
    if (odo_laser_fd == -1) return -1;

    int cmd_fd = setup_client(CMD_PORT);
    if (cmd_fd == -1) {
        close(odo_laser_fd);
        return -1;
    }

    // --- Step 1: Store Start Time and Start Logging ---
    struct timeval log_start_time;
    gettimeofday(&log_start_time, NULL);
    printf("Log started at: %ld.%06ld seconds\n", log_start_time.tv_sec, log_start_time.tv_usec);

    // Store start time in the robot
    send_command(cmd_fd, "var $start_time = $time\n");
    usleep(INTERVAL);  // Small delay to ensure execution

    // Start odometry logging
    printf("Starting odometry logging...\n");
    send_command(cmd_fd, "log \"$odox\" \"$odoy\" \"$odoth\" \"$odovelocity\" \"$ododist\" \"$time - $start_time\"\n");
    sleep(1);  // Ensure logging has started

    // --- Step 2: Send Step Response Command ---
    struct timeval command_time;
    gettimeofday(&command_time, NULL);
    printf("Motor command sent at: %ld.%06ld seconds\n", command_time.tv_sec, command_time.tv_usec);

    send_command(cmd_fd, "motorcmds 0.5 0.5\n");  // Apply step response

    // --- Step 3: Wait for the logging period ---
    sleep(LOG_DURATION);

    // --- Step 4: Stop Logging ---
    printf("Stopping logging...\n");
    send_command(cmd_fd, "stop\n");

    // Cleanup
    shutdown(cmd_fd, SHUT_RDWR);
    close(cmd_fd);
    printf("Motion command connection closed.\n");

    close(odo_laser_fd);
    printf("Odometry & laser scanner connection closed.\n");

    return 0;
}
