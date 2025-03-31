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
#define LOG_DURATION 15  // Time in seconds to log data
#define INTERVAL 100000  // 0.1 seconds in microseconds
#define MAX_VELOCITY 0.2 // Max velocity
#define LOG_SIZE 2000  // Maximum log entries
#define DESIRED_DISTANCE 0.3  // Desired distance 30cm

float e[3] = {0.0, 0.0, 0.0};
float u[3] = {0.0, 0.0, 0.0};

// Buffers to log control values
float u_log[LOG_SIZE][3];
float e_log[LOG_SIZE][3];
float time_log[LOG_SIZE];  // Store time in seconds
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
// Read laser distance
float read_laser_distance(int odo_laser_fd) {
    char buffer[4096] = {0};
    send(odo_laser_fd, "zoneobst\n", 9, 0);

    int valread = read(odo_laser_fd, buffer, sizeof(buffer) - 1);
    if (valread > 0) {
        buffer[valread] = '\0';

        char *l4_ptr = strstr(buffer, "l4=\"");
        if (l4_ptr) {
            float l4 = atof(l4_ptr + 4);

            //Check that the value is in valid range
            if (l4 >= 0.1 && l4 <= 4.0) {
                return l4;
            }
        }
    }
    // Fallback if nothing valid is found
    return -1;
}


// Control function
float control(float ref, float measurement) {
    // Define coefficient arrays
    double b[] = {  23.0548,  -32.0138,   11.0703 }; // Numerator coefficients
    double a[] = {   1.0000,   -0.3182,   -0.6818 };  // Denominator coefficients

    // Compute current error
    e[0] = measurement - ref;

    // Compute *unclamped* control signal
    u[0] =  u[0] = (b[0] * e[0]) + (b[1] * e[1]) + (b[2] * e[2]) -(a[1] * u[1]) - (a[2] * u[2]);



    // Then clamp output
    if (u[0] > MAX_VELOCITY) {
        u[0] = MAX_VELOCITY;
    } else if (u[0] < 0) {
        u[0] = 0;
    }


    // Log current values
    if (log_index < LOG_SIZE) {
        for (int i = 0; i < 3; i++) {
            u_log[log_index][i] = u[i];
            e_log[log_index][i] = e[i];
        }
        time_log[log_index] = log_index * 0.1;  // 100ms intervals
        log_index++;
    }

    // Shift history *after* clamping
    for (int i = 2; i > 0; i--) {
        u[i] = u[i - 1];
        e[i] = e[i - 1];
    }

    return u[0];
}


void log_data_to_file(float distance_log[]) {
    FILE *log_file = fopen("control_log.txt", "w");
    if (!log_file) {
        perror("Failed to open log file");
        return;
    }

    // Updated header to include time
    fprintf(log_file, "Iteration,Time (s),Distance,u[0],u[1],u[2],e[0],e[1],e[2]\n");

    for (int i = 0; i < log_index; i++) {
        fprintf(log_file, "%d,%.6f,%.5f,", i, time_log[i], distance_log[i]);

        // Log control values
        for (int j = 0; j < 3; j++) {
            fprintf(log_file, "%.5f,", u_log[i][j]);
        }

        // Log error values
        for (int j = 0; j < 3; j++) {
            fprintf(log_file, "%.5f", e_log[i][j]);
            if (j < 2) fprintf(log_file, ",");
        }

        fprintf(log_file, "\n");
    }

    fclose(log_file);
    printf("Control data logged to 'control_log.txt'.\n");
}



// Main function
int main() {
    
    float distance_log[LOG_SIZE];  //Log data
    int elapsed_time = 0; //Time tracking

    //Socket Client setup
    int odo_laser_fd = setup_client(ODO_LASER_PORT);
    if (odo_laser_fd == -1) return -1;

    int cmd_fd = setup_client(CMD_PORT);
    if (cmd_fd == -1) {
        close(odo_laser_fd);
        return -1;
    }

    //Call Zoneobst on ULMSSERVER
    send_command(odo_laser_fd, "laser scanpush cmd='zoneobst'\n");
    printf("Zoneobst scan started\n");
    sleep(1);

    //Loop function
    while (elapsed_time < LOG_DURATION * 1000000) {
        float distance = read_laser_distance(odo_laser_fd);
    
        //printf("Distance Read: %.3f m\n", distance);
        float velocity_command = control(DESIRED_DISTANCE, distance);
    
        //printf("Error: %.3f m | Control Output: %.3f m/s\n", DESIRED_DISTANCE - distance, velocity_command);
        
        distance_log[log_index] = distance;
        time_log[log_index] = elapsed_time / 1e6;  // Convert microseconds to seconds
        char motor_cmd[50];
        snprintf(motor_cmd, sizeof(motor_cmd), "motorcmds %.2f %.2f\n", velocity_command, velocity_command);
        send_command(cmd_fd, motor_cmd);
    
        usleep(INTERVAL);
        elapsed_time += INTERVAL;
    }

    //Exit statements
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
