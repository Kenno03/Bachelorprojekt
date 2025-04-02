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
#define ROTATION_OFFSET 120.0f
#define M_PI 3.14159265358979323846
#define COMBINED_CSV_FILE "lidar_combined.csv"

float scan_log[MAX_POINTS];

typedef struct {
    float angle_sum;
    float min_distance;
    int count;
    float centroid_angle;
    float centroid_distance;
} Cluster;

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

    if (connect(client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection failed");
        close(client_fd);
        return -1;
    }

    printf("Connected to port %d\n", port);
    return client_fd;
}

void send_command(int client_fd, const char *command) {
    if (send(client_fd, command, strlen(command), 0) < 0) {
        perror("Send command failed");
    }
    usleep(100000);
}

char *extract_bin_data(char *buffer) {
    char *start = strstr(buffer, "<bin");
    if (!start) return NULL;
    start = strchr(start, '>');
    if (!start) return NULL;
    start++;
    char *end = strstr(start, "</bin>");
    if (!end) return NULL;
    int len = end - start;
    char *hex_data = malloc(len + 1);
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

int find_clusters(float *distances, Cluster *clusters, int max_clusters) {
    const float min_distance = 0.15f;
    const float max_distance = 3.0f;

    int cluster_count = 0;
    int in_cluster = 0;
    Cluster current = {0};

    for (int i = 0; i <= MAX_POINTS; i++) {
        float r = (i < MAX_POINTS) ? distances[i] : 0.0f;
        int is_valid = (r > min_distance && r <= max_distance);

        if (!is_valid || i == MAX_POINTS) {
            if (in_cluster && current.count >= 3 && cluster_count < max_clusters) {
                current.centroid_angle = current.angle_sum / current.count;
                current.centroid_distance = current.min_distance;
                clusters[cluster_count++] = current;
            }
            in_cluster = 0;
            memset(&current, 0, sizeof(Cluster));
        } else {
            float angle_deg = START_ANGLE_DEG + i * ANGLE_RESOLUTION + ROTATION_OFFSET;
            if (!in_cluster) {
                in_cluster = 1;
                current.min_distance = r;
                current.angle_sum = 0;
                current.count = 0;
            }
            if (r < current.min_distance) current.min_distance = r;
            current.angle_sum += angle_deg;
            current.count++;
        }
    }
    return cluster_count;
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

    char *hex_data = extract_bin_data(buffer);
    if (hex_data) {
        parse_lidar_bin(hex_data, scan_log, MAX_POINTS);
        free(hex_data);

        FILE *fp = fopen(COMBINED_CSV_FILE, "w");
        if (!fp) {
            perror("Failed to open combined CSV file");
            close(laser_fd);
            return -1;
        }

        fprintf(fp, "angle_deg,distance_m,is_centroid\n");

        // Log raw LiDAR points
        for (int i = 0; i < MAX_POINTS; i++) {
            float r = scan_log[i];
            if (r <= 0.1f || r > 4.0f) continue;
            float angle_deg = START_ANGLE_DEG + i * ANGLE_RESOLUTION + ROTATION_OFFSET;
            fprintf(fp, "%.2f,%.3f,0\n", angle_deg, r);
        }

        // Detect and log cluster centroids
        Cluster clusters[32];
        int num_clusters = find_clusters(scan_log, clusters, 32);

        Cluster best = {0};
        float best_score = 9999.0f;

        for (int i = 0; i < num_clusters; i++) {
            float angle_error = fabsf(clusters[i].centroid_angle - 90.0f);
            float distance = clusters[i].centroid_distance;
            float score = angle_error + distance;

            fprintf(fp, "%.2f,%.3f,1\n", clusters[i].centroid_angle, clusters[i].centroid_distance);

            if (score < best_score) {
                best_score = score;
                best = clusters[i];
            }
        }

        if (best.count > 0) {
            fprintf(fp, "%.2f,%.3f,2\n", best.centroid_angle, best.centroid_distance);
            printf("Best cluster: angle = %.2f°, distance = %.2f m\n", best.centroid_angle, best.centroid_distance);
        }

        fclose(fp);
        printf("Saved combined data to %s with %d centroids.\n", COMBINED_CSV_FILE, num_clusters);
    } else {
        fprintf(stderr, "Failed to extract scan\n");
    }

    close(laser_fd);
    return 0;
}
