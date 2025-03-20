#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

#define ODO_LASER_PORT 24919  // Robot server port
#define CMD_PORT 31001  // Port for motion commands
#define SERVER_IP "192.38.66.89"  // Replace with your server's IP
#define LOG_DURATION 20  // Time in seconds to log data
#define INTERVAL 100000  // 0.1 seconds in microseconds
#define MAX_VELOCITY 0.5 // Max velocity
#define LOG_SIZE 2000  // Maximum log entries
#define DESIRED_DISTANCE 0.2  // Desired distance

float e[3] = {0, 0, 0};
float u[3] = {0, 0, 0};

// Buffers to log control values
float u_log[LOG_SIZE][3];
float e_log[LOG_SIZE][3];
int log_index = 0;

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

// Read laser distance
float read_laser_distance(int odo_laser_fd) {
    char buffer[4096] = {0};
    send(odo_laser_fd, "zoneobst\n", 9, 0);

    int valread = read(odo_laser_fd, buffer, sizeof(buffer) - 1);
    if (valread > 0) {
        buffer[valread] = '\0';

        float l3 = 0.0, l4 = 0.0, l5 = 0.0;  // Ensure variables are initialized
        float average_distance = 0.0;
        
        int matched = sscanf(buffer, "<laser %*[^l] l3=\"%f\" l4=\"%f\" l5=\"%f\"", &l3, &l4, &l5);
        if (matched == 3) {
            average_distance = (l3 + l4 + l5) / 3.0; 
            if (average_distance < 0) {
                return 0.01;
            } else if (average_distance > 4) {
                return 4.0;
            } else {
                return average_distance;
            }
        }

        // Alternative extraction
        char *l3_ptr = strstr(buffer, "l3=\"");
        char *l4_ptr = strstr(buffer, "l4=\"");
        char *l5_ptr = strstr(buffer, "l5=\"");

        if (l3_ptr && l4_ptr && l5_ptr) {
            l3 = atof(l3_ptr + 4);
            l4 = atof(l4_ptr + 4);
            l5 = atof(l5_ptr + 4);
            return (l3 + l4 + l5) / 3.0;
        }
    }
    return -1;
}


// Control function
float control(float ref, float measurement) {

    e[0] = measurement - ref ;

    // Corrected control equation
    u[0] = (1.9433 * u[1]) - (0.9454 * u[2]) + (0.0010 * e[1]) + (0.0010 * e[2]);

    // Velocity clamping
    if (u[0] > MAX_VELOCITY) {
        u[0] = MAX_VELOCITY;
    } else if (u[0] < 0) {
        u[0] = 0;
    }

    // Store values in log buffer
    if (log_index < LOG_SIZE) {
        for (int i = 0; i < 3; i++) {
            u_log[log_index][i] = u[i];
            e_log[log_index][i] = e[i];
        }
        log_index++;
    }

    // Shift old values for next iteration
    for (int i = 2; i > 0; i--) {
        e[i] = e[i - 1];
        u[i] = u[i - 1];
    }

    return u[0];
}

void log_data_to_file(float distance_log[]) {
    FILE *log_file = fopen("control_log.txt", "w");
    if (!log_file) {
        perror("Failed to open log file");
        return;
    }

    // Updated header to include "Distance"
    fprintf(log_file, "Iteration,Distance,u[0],u[1],u[2],e[0],e[1],e[2]\n");

    for (int i = 0; i < log_index; i++) {
        fprintf(log_file, "%d,%.5f,", i, distance_log[i]); // Log iteration number and distance

        // Log control values (u[0], u[1], u[2])
        for (int j = 0; j < 3; j++) {
            fprintf(log_file, "%.5f,", u_log[i][j]);
        }

        // Log error values (e[0], e[1], e[2])
        for (int j = 0; j < 3; j++) {
            fprintf(log_file, "%.5f", e_log[i][j]);
            if (j < 2) fprintf(log_file, ",");  // Avoid trailing comma
        }

        fprintf(log_file, "\n");
    }

    fclose(log_file);
    printf("Control data logged to 'control_log.txt'.\n");
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

    send_command(odo_laser_fd, "laser scanpush cmd='zoneobst'\n");
    printf("Zoneobst scan started\n");
    sleep(1);

    float distance_log[LOG_SIZE];  // Add this line at the top of main()

    int elapsed_time = 0;

while (elapsed_time < LOG_DURATION * 1000000) {
    float distance = read_laser_distance(odo_laser_fd);
    
    if (distance < 0) {
        printf("Error: Invalid laser reading!\n");
        continue;  // Skip iteration if invalid distance
    }

    printf("Distance Read: %.3f m\n", distance);

    float velocity_command = control(DESIRED_DISTANCE, distance);

    printf("Error: %.3f m | Control Output: %.3f m/s\n", DESIRED_DISTANCE - distance, velocity_command);

    distance_log[log_index] = distance;  // Store distance in log

    char motor_cmd[50];
    snprintf(motor_cmd, sizeof(motor_cmd), "motorcmds %.2f %.2f\n", velocity_command, velocity_command);
    send_command(cmd_fd, motor_cmd);

    usleep(INTERVAL);
    elapsed_time += INTERVAL;
}


    send_command(cmd_fd, "motorcmds 0 0\n");
    printf("Stopping robot...\n");

    shutdown(cmd_fd, SHUT_RDWR);
    close(cmd_fd);
    printf("Motion command connection closed.\n");

    close(odo_laser_fd);
    printf("Odometry & laser scanner connection closed.\n");

    log_data_to_file(distance_log); // Write log data to file

    return 0;
}
