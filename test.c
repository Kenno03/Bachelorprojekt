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
#define TARGET_DISTANCE 0.2  // Target distance in meters
#define MAX_SPEED 0.5  // Maximum wheel speed
#define SAMPLE_TIME 0.1 //Sample Time

// PID gains
float kp = 4.18792;
float ki = 0.66314;
float kd = 3.31572;

// PID variables
float integral = 0;
float previous_error = 0;

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

float read_laser_distance(int odo_laser_fd) {
    char buffer[4096] = {0};

    send(odo_laser_fd, "zoneobst\n", 9, 0);
    usleep(INTERVAL);

    int valread = read(odo_laser_fd, buffer, sizeof(buffer) - 1);
    if (valread > 0) {
        buffer[valread] = '\0';
        printf("Laser Data Received: %s\n", buffer);  // Debugging output

        float distances[9]; // Array to hold l0-l8 values
        int matched = sscanf(buffer, "<laser l0=\"%f\" l1=\"%f\" l2=\"%f\" l3=\"%f\" l4=\"%f\" l5=\"%f\" l6=\"%f\" l7=\"%f\" l8=\"%f\"",
                             &distances[0], &distances[1], &distances[2], &distances[3], &distances[4],
                             &distances[5], &distances[6], &distances[7], &distances[8]);

        if (matched == 9) {  // Ensure all values were extracted correctly
            printf("Extracted Front Distance (l4): %.2f m\n", distances[4]); // l4 is front
            return distances[4];
        } else {
            printf("Error: Could not extract front-facing distance from laser data.\n");
        }
    }
    return -1; // Return -1 if reading failed
}


void pid_controller(int cmd_fd, float front_distance){

    float error = front_distance - TARGET_DISTANCE;
    integral += error *SAMPLE_TIME;
    float derivative = (error - previous_error)/SAMPLE_TIME;

    // Calculate speed correction
    float correction = (kp * error) + (ki * integral) + (kd * derivative);

      // Set a minimum movement threshold
      float MIN_SPEED = 0.05;  // Adjust as needed
      if (fabs(correction) < MIN_SPEED && fabs(error) > 0.01) {
          correction = (correction > 0) ? MIN_SPEED : -MIN_SPEED;
      }

    // Ensure speed limits
    if (correction > MAX_SPEED) correction = MAX_SPEED;
    if (correction <= 0) correction = 0;

    // Convert correction to differential drive
    float vl = correction;
    float vr = correction;

    //Send command
    char command[50];
    snprintf(command, sizeof(command), "motorcmds %.2f %.2f\n", vl, vr);
    send_command(cmd_fd, command);
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

    // Control loop
    while (1) {
        float front_distance = read_laser_distance(odo_laser_fd);
        if (front_distance > 0) {
            printf("Front distance: %.2f m\n", front_distance);
            pid_controller(cmd_fd, front_distance);
        } else {
            printf("Failed to read laser data, stopping robot.\n");
            send_command(cmd_fd, "motorcmds 0 0\n");
        }
        usleep(INTERVAL);
    }

    wait_until_stopped(odo_laser_fd);

    shutdown(cmd_fd, SHUT_RDWR);
    close(cmd_fd);
    printf("Motion command connection closed.\n");

    close(odo_laser_fd);
    printf("Odometry & laser scanner connection closed.\n");

    return 0;
}
