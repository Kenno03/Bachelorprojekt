#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <math.h>  // Required for isnan()

#define ODO_LASER_PORT 24919  // Port for odometry & laser scanner
#define CMD_PORT 31001  // Port for motion commands
#define SERVER_IP "192.38.66.89" // SMR9 server
#define INTERVAL 500000  // 0.5 seconds in microseconds
#define STOP_TIME 2000000  // 2 seconds in microseconds
#define MOVEMENT_THRESHOLD 0.005  // Minimum movement difference to consider stopping

// Function to parse odometry data from XML
void parse_odoPose(char *response, float *x, float *y, float *h) {
    const char *pos_start = strstr(response, "<pose name=\"newest\"");
    if (pos_start) {
        sscanf(pos_start, "<pose name=\"newest\" x=\"%f\" y=\"%f\" h=\"%f\"", x, y, h);
    } else {
        *x = 0;
        *y = 0;
        *h = 0;
    }
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

// Function to send a command and read response
void send_command(int client_fd, const char* command) {
    char buffer[1024] = {0};

    printf("Sending command: %s", command);
    send(client_fd, command, strlen(command), 0);
    usleep(INTERVAL);  

    int valread = read(client_fd, buffer, sizeof(buffer) - 1);
    if (valread > 0) {
        buffer[valread] = '\0';
        printf("Server response: %s\n", buffer);
    }
}

// Function to read odometry and laser scan data
void read_sensors(int odo_laser_fd, float *x, float *y, float *h) {
    char buffer[4096] = {0};

    send(odo_laser_fd, "odoPose pose\n", 14, 0);
    usleep(INTERVAL);

    int valread = read(odo_laser_fd, buffer, sizeof(buffer) - 1);
    if (valread > 0) {
        buffer[valread] = '\0';
        parse_odoPose(buffer, x, y, h);
        if (isnan(*y) || isnan(*h)) {  
            *x = 0;
            *y = 0;
            *h = 0;
        }
        printf("Odometry Data: X = %.3f, Y = %.3f, Theta = %.3f\n", *x, *y, *h);
    } else {
        printf("Error reading odometry data\n");
    }

    send(odo_laser_fd, "zoneobst\n", 9, 0);
    usleep(INTERVAL);

    valread = read(odo_laser_fd, buffer, sizeof(buffer) - 1);
    if (valread > 0) {
        buffer[valread] = '\0';
        printf("Laser Data Received: %s\n", buffer);
    } else {
        printf("Error reading laser data\n");
    }
}

// Function to wait until robot stops moving
void wait_until_stopped(int odo_laser_fd) {
    printf("Waiting for robot to stop...\n");

    float prev_x = 0, prev_y = 0, prev_h = 0;
    float x, y, h;
    int stable_count = 0;

    read_sensors(odo_laser_fd, &prev_x, &prev_y, &prev_h);

    while (stable_count < (STOP_TIME / INTERVAL)) {  
        read_sensors(odo_laser_fd, &x, &y, &h);

        if (fabs(x - prev_x) < MOVEMENT_THRESHOLD &&
            fabs(y - prev_y) < MOVEMENT_THRESHOLD &&
            fabs(h - prev_h) < MOVEMENT_THRESHOLD) {
            stable_count++;
        } else {
            stable_count = 0;
        }

        prev_x = x;
        prev_y = y;
        prev_h = h;
        usleep(INTERVAL);
    }

    printf("Robot has stopped moving.\n");
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

    char* startCommand = "laser scanpush cmd='zoneobst'\n";
    send(odo_laser_fd, startCommand, strlen(startCommand), 0);
    printf("Zoneobst scan started\n");
    sleep(1);

    // --- Drive in a Square Path ---
    char* square_path_commands[] = {
        "fwd 1\n",  // Move forward 1m
        "turn 90\n", // Turn right 90 degrees
        "fwd 1\n",
        "turn 90\n",
        "fwd 1\n",
        "turn 90\n",
        "fwd 1\n",
        "turn 90\n"
    };
    int num_commands = sizeof(square_path_commands) / sizeof(square_path_commands[0]);

    for (int i = 0; i < num_commands; i++) {
        float x, y, h;
        read_sensors(odo_laser_fd, &x, &y, &h);
        send_command(cmd_fd, square_path_commands[i]);
    }

    wait_until_stopped(odo_laser_fd);

    shutdown(cmd_fd, SHUT_RDWR);
    close(cmd_fd);
    printf("Motion command connection closed.\n");

    close(odo_laser_fd);
    printf("Odometry & laser scanner connection closed.\n");

    return 0;
}
