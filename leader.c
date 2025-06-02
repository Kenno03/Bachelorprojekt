// LiDAR pose logger with MCP command execution and stop functionality
// Author: Markus Kenno Hansen

#include <arpa/inet.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#define SERVER_IP "192.38.66.91"
#define LIDAR_PORT 24919
#define CMD_PORT 31004
#define BUFFER_SIZE 8192

volatile bool keep_running = true;
double velocity_cmd_global = 0.0;

double extract_attr(const char* line, const char* key) {
    char* start = strstr(line, key);
    if (!start) return 0.0;
    start = strchr(start, '"');
    if (!start) return 0.0;
    return atof(start + 1);
}

void* stop_listener(void* arg) {
    char input[100];
    while (fgets(input, sizeof(input), stdin)) {
        if (strncmp(input, "stop", 4) == 0) {
            printf("[STOP] Manual stop triggered\n");
            keep_running = false;
            break;
        }
    }
    return NULL;
}

void* lidar_logger_thread(void* arg) {
    printf("[LOG] Starting LiDAR logger thread...\n");

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        pthread_exit(NULL);
    }

    struct sockaddr_in serv_addr = {0};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(LIDAR_PORT);
    inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr);

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection to ulmsserver failed");
        close(sock);
        pthread_exit(NULL);
    }

    FILE* log = fopen("pose_log.csv", "w");
    if (!log) {
        perror("Failed to open log file");
        close(sock);
        pthread_exit(NULL);
    }

    fprintf(log, "time_unix,x,y,heading,velocity_cmd\n");
    char buffer[BUFFER_SIZE];

    while (keep_running) {
        const char* cmd = "scanget\n";
        send(sock, cmd, strlen(cmd), 0);
        usleep(150000);

        int bytes = read(sock, buffer, BUFFER_SIZE - 1);
        if (bytes <= 0) break;
        buffer[bytes] = '\0';

        double tod = extract_attr(buffer, "tod=");

        // Extract velocity from odometry log line
        char* vel_line = strstr(buffer, "$odovelocity ");
        if (vel_line) {
            velocity_cmd_global = atof(vel_line + strlen("$odovelocity "));
        }

        char* pose_line = strstr(buffer, "<pose name=\"robot\"");
        if (pose_line) {
            double x = extract_attr(pose_line, "x=");
            double y = extract_attr(pose_line, "y=");
            double h = extract_attr(pose_line, "h=");
            double v = velocity_cmd_global;
            fprintf(log, "%.6f,%.3f,%.3f,%.3f,%.3f\n", tod, x, y, h, v);
            fflush(log);
        }
    }

    fclose(log);
    close(sock);
    printf("[LOG] LiDAR logger stopped.\n");
    pthread_exit(NULL);
}

void send_command(int sock, const char* cmd) {
    char buffer[1024] = {0};
    printf("[MCP] Sending: %s", cmd);
    send(sock, cmd, strlen(cmd), 0);
    usleep(500000);
    read(sock, buffer, sizeof(buffer) - 1);
}

bool is_still_moving(int sock) {
    char buffer[1024] = {0};
    const char* cmd = "vel\n";
    send(sock, cmd, strlen(cmd), 0);
    usleep(100000);
    int n = read(sock, buffer, sizeof(buffer) - 1);
    if (n <= 0) return true;

    buffer[n] = '\0';
    char* vstr = strstr(buffer, "$odovelocity ");
    if (!vstr) return true;

    double v = atof(vstr + strlen("$odovelocity "));
    return v > 0.01;
}

void* command_thread(void* arg) {
    printf("[MCP] Starting MCP command thread...\n");

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        pthread_exit(NULL);
    }

    struct sockaddr_in serv_addr = {0};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(CMD_PORT);
    inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr);

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection to CMD server failed");
        close(sock);
        pthread_exit(NULL);
    }

    const char* commands[] = {
        "ignoreobstacles\n",
        "log \"$odox\" \"$odoy\" \"$odoth\" \"$odovelocity\" \"$time\"\n",
        "drive @v0.3 :($drivendist > 1.2)\n",
        "turnr 0.5 90\n",
        "drive @v0.3 :($drivendist > 3)\n",
        "turnr 0.5 -180\n",
        "drive @v0.4 :($drivendist > 1)\n",
        "drive @v0.3 :($drivendist > 0.5)\n",
        "turnr 0.5 90\n",
        "turnr 0.5 -180\n",
        "turnr 0.5 180\n",
        "drive @v0.3 :($drivendist > 0.5)\n",
        "turnr 0.5 90\n",
        "drive @v0.2 :($drivendist > 2)\n",
        "turnr 0.3 90\n",
        "drive @v0.2 :($drivendist > 1)\n",
        "drive @v0.4 :($drivendist > 1.5)\n",
        "stop\n"
    };

    int num_cmds = sizeof(commands) / sizeof(commands[0]);
    for (int i = 0; i < num_cmds && keep_running; ++i) {
        send_command(sock, commands[i]);
    }

    printf("[MCP] Commands finished. Monitoring for stop...\n");

    int still_counter = 0;
    while (keep_running && still_counter < 3) {
        if (!is_still_moving(sock)) {
            still_counter++;
            printf("[WAIT] Robot is stationary (%d/3)\n", still_counter);
        } else {
            still_counter = 0;
            printf("[WAIT] Robot still moving...\n");
        }
        sleep(1);
    }

    if (still_counter >= 3) {
        printf("[DONE] Robot stopped. Exiting program.\n");
        keep_running = false;
    }

    close(sock);
    pthread_exit(NULL);
}

int main() {
    printf("[MAIN] Program started. Connecting...\n");

    pthread_t input_thread, logger_thread, mcp_thread;
    pthread_create(&input_thread, NULL, stop_listener, NULL);
    pthread_create(&logger_thread, NULL, lidar_logger_thread, NULL);
    pthread_create(&mcp_thread, NULL, command_thread, NULL);

    pthread_join(input_thread, NULL);
    pthread_join(logger_thread, NULL);
    pthread_join(mcp_thread, NULL);

    printf("[MAIN] Program finished.\n");
    return 0;
}
    
