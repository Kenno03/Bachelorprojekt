#include <arpa/inet.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/time.h>

#define ODO_LASER_PORT 24919
#define SERVER_IP "192.38.66.89"
#define MAX_POINTS 512
#define ANGLE_RESOLUTION 0.36f
#define START_ANGLE_DEG -120.0f
#define M_PI 3.14159265358979323846

float scan_log[MAX_POINTS];

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
        perror("Invalid address");
        close(client_fd);
        return -1;
    }

    if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection failed");
        close(client_fd);
        return -1;
    }

    printf("Connected to port %d\n", port);
    return client_fd;
}

void send_command(int client_fd, const char* command) {
    if (send(client_fd, command, strlen(command), 0) < 0) {
        perror("Send command failed");
    }
    usleep(100000);
}

char* extract_bin_data(char* buffer) {
    char* start = strstr(buffer, "<bin");
    if (!start) return NULL;

    start = strchr(start, '>');
    if (!start) return NULL;
    start++;

    char* end = strstr(start, "</bin>");
    if (!end) return NULL;

    int len = end - start;
    char* hex_data = malloc(len + 1);
    strncpy(hex_data, start, len);
    hex_data[len] = '\0';
    return hex_data;
}

void parse_lidar_bin(char *hex_data, float *distances, int count) {
    for (int i = 0; i < count; i++) {
        int index = i * 4;
        char hex_pair[5];
        hex_pair[0] = hex_data[index + 2];
        hex_pair[1] = hex_data[index + 3];
        hex_pair[2] = hex_data[index + 0];
        hex_pair[3] = hex_data[index + 1];
        hex_pair[4] = '\0';

        int raw_mm = (int)strtol(hex_pair, NULL, 16);
        distances[i] = raw_mm * 0.001f;
    }
}

void save_xy_csv(const char* filename, float distances[]) {
    FILE* fp = fopen(filename, "w");
    if (!fp) {
        perror("Failed to open XY CSV file");
        return;
    }

    fprintf(fp, "x,y\n");
    for (int i = 0; i < MAX_POINTS; i++) {
        float r = distances[i];
        if (r <= 0.01f || r > 4.0f) continue;

        float angle_deg = START_ANGLE_DEG + i * ANGLE_RESOLUTION;
        float angle_rad = angle_deg * M_PI / 180.0f;
        float x = r * cosf(angle_rad);
        float y = r * sinf(angle_rad);
        fprintf(fp, "%.3f,%.3f\n", x, y);
    }

    fclose(fp);
    printf("Saved XY to %s\n", filename);
}

void save_polar_csv(const char* filename, float distances[]) {
    FILE* fp = fopen(filename, "w");
    if (!fp) {
        perror("Failed to open polar CSV file");
        return;
    }

    fprintf(fp, "angle_deg,distance_m\n");
    for (int i = 0; i < MAX_POINTS; i++) {
        float r = distances[i];
        if (r <= 0.01f || r > 4.0f) continue;

        float angle_deg = START_ANGLE_DEG + i * ANGLE_RESOLUTION;
        fprintf(fp, "%.2f,%.3f\n", angle_deg, r);
    }

    fclose(fp);
    printf("Saved polar to %s\n", filename);
}

int main() {
    int laser_fd = setup_client(ODO_LASER_PORT);
    if (laser_fd == -1) return -1;

    send_command(laser_fd, "scanget\n");

    char buffer[16384] = {0};
    int total_read = 0;

    fd_set fds;
    struct timeval timeout;
    FD_ZERO(&fds);
    FD_SET(laser_fd, &fds);
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;

    int ready = select(laser_fd + 1, &fds, NULL, NULL, &timeout);
    if (ready <= 0) {
        fprintf(stderr, "Timeout waiting for scan\n");
        close(laser_fd);
        return -1;
    }

    while (total_read < sizeof(buffer) - 1) {
        int valread = read(laser_fd, buffer + total_read, sizeof(buffer) - total_read - 1);
        if (valread <= 0) break;
        total_read += valread;
        buffer[total_read] = '\0';
        if (strstr(buffer, "</scanget>")) break;
    }

    if (total_read == 0) {
        fprintf(stderr, "Empty scan data\n");
        close(laser_fd);
        return -1;
    }

    char* hex_data = extract_bin_data(buffer);
    if (hex_data) {
        parse_lidar_bin(hex_data, scan_log, MAX_POINTS);
        free(hex_data);

        save_xy_csv("lidar_xy.csv", scan_log);
        save_polar_csv("lidar_polar.csv", scan_log);
    } else {
        fprintf(stderr, "Failed to extract scan\n");
    }

    close(laser_fd);
    return 0;
}
