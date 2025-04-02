#include <arpa/inet.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

// --- Configuration constants ---
#define ODO_LASER_PORT 24919
#define SERVER_IP "192.38.66.89"
#define MAX_POINTS 512
#define ANGLE_RESOLUTION 0.36f
#define START_ANGLE_DEG -120.0f
#define ROTATION_OFFSET 120.0f
#define M_PI 3.14159265358979323846

#define MAX_CLUSTERS 32
#define MAX_CENTROID_HISTORY 1000
#define LOG_DURATION_SEC 15
#define TRAJ_LOG_FILE "trajectory_log.csv"

// --- Structs to hold cluster and centroid info ---
typedef struct {
    float angle_sum;          // Sum of angles in cluster (for averaging)
    float min_distance;       // Closest point in cluster
    int count;                // Number of points in cluster
    float centroid_angle;     // Average angle (centroid)
    float centroid_distance;  // Minimum distance (used as centroid distance)
} Cluster;

typedef struct {
    float angle_deg;  // Centroid angle in degrees
    float distance_m; // Centroid distance in meters
    float x;          // Cartesian X coordinate
    float y;          // Cartesian Y coordinate
} CentroidLog;

float scan_log[MAX_POINTS];
CentroidLog trajectory[MAX_CENTROID_HISTORY];
int traj_count = 0;  // Counter for logged centroids

// --- Setup TCP client socket connection to laser server ---
int setup_client(int port) {
    int client_fd;
    struct sockaddr_in serv_addr;

    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr);

    if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection failed");
        close(client_fd);
        return -1;
    }

    printf("Connected to port %d\n", port);
    return client_fd;
}

// --- Send a command over socket ---
void send_command(int client_fd, const char* command) {
    send(client_fd, command, strlen(command), 0);
    usleep(100000);  // Wait 100 ms for stability
}

// --- Extract raw hex <bin> data from XML ---
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

// --- Parse hex string into float distances ---
void parse_lidar_bin(char *hex_data, float *distances, int count) {
    for (int i = 0; i < count; i++) {
        int index = i * 4;  // 4 hex chars per reading
        char hex_pair[5];  // Temporary string to hold 4 hex chars + null

        // Reorder bytes: LSB format
        hex_pair[0] = hex_data[index + 2];
        hex_pair[1] = hex_data[index + 3];
        hex_pair[2] = hex_data[index + 0];
        hex_pair[3] = hex_data[index + 1];
        hex_pair[4] = '\0';

        int raw_mm = (int)strtol(hex_pair, NULL, 16);  // Convert to integer in mm
        distances[i] = raw_mm * 0.001f;                // Convert to meters
    }
}

// --- Group consecutive valid points into clusters ---
int find_clusters(float *distances, Cluster *clusters, int max_clusters) {
    const float min_distance = 0.15f;
    const float max_distance = 3.0f;
    int cluster_count = 0;
    int in_cluster = 0;
    Cluster current = {0};  // Temporary storage for building cluster

    for (int i = 0; i <= MAX_POINTS; i++) {
        float r = (i < MAX_POINTS) ? distances[i] : 0.0f;
        int is_valid = (r > min_distance && r <= max_distance);

        if (!is_valid || i == MAX_POINTS) {
            // If we were in a cluster and it meets min size, store it
            if (in_cluster && current.count >= 3 && cluster_count < max_clusters) {
                current.centroid_angle = current.angle_sum / current.count;
                current.centroid_distance = current.min_distance;
                clusters[cluster_count++] = current;
            }
            in_cluster = 0;
            memset(&current, 0, sizeof(Cluster));  // Reset temp cluster
        } else {
            // Compute angle for current index
            float angle_deg = START_ANGLE_DEG + i * ANGLE_RESOLUTION + ROTATION_OFFSET;

            // If entering new cluster
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

// --- Log all recorded centroids to CSV ---
void log_trajectory(const char* filename) {
    FILE* fp = fopen(filename, "w");
    if (!fp) {
        perror("Failed to open trajectory file");
        return;
    }
    fprintf(fp, "angle_deg,distance_m,x,y\n");
    for (int i = 0; i < traj_count; i++) {
        fprintf(fp, "%.2f,%.3f,%.3f,%.3f\n", trajectory[i].angle_deg, trajectory[i].distance_m,
                trajectory[i].x, trajectory[i].y);
    }
    fclose(fp);
    printf("Saved trajectory log to %s (%d entries).\n", filename, traj_count);
}

// --- Main execution loop ---
int main() {
    int sock = setup_client(ODO_LASER_PORT);
    if (sock == -1) return -1;

    send_command(sock, "scanpush cmd='scanget'\n");  // Start continuous scanning
    printf("Continuous LiDAR scan started...\n");

    char buffer[32768] = {0};  // Raw buffer for data from socket
    int buffer_len = 0;

    time_t start = time(NULL);

    // Main loop for fixed duration
    while (difftime(time(NULL), start) < LOG_DURATION_SEC) {
        int bytes = read(sock, buffer + buffer_len, sizeof(buffer) - buffer_len - 1);
        if (bytes <= 0) {
            usleep(10000);  // No data? Wait and retry
            continue;
        }

        buffer_len += bytes;
        buffer[buffer_len] = '\0';

        // Process all <bin>...</bin> blocks in buffer
        while (1) {
            char* bin_start = strstr(buffer, "<bin");
            char* bin_end = strstr(buffer, "</bin>");
            if (!(bin_start && bin_end && bin_end > bin_start)) break;

            int block_len = bin_end - bin_start + strlen("</bin>");
            char bin_block[16384];
            strncpy(bin_block, bin_start, block_len);
            bin_block[block_len] = '\0';

            // Shift buffer left to discard processed block
            size_t shift_offset = (bin_end + strlen("</bin>")) - buffer;
            buffer_len -= shift_offset;
            memmove(buffer, buffer + shift_offset, buffer_len);
            buffer[buffer_len] = '\0';

            // Extract and decode scan
            char* hex_data = extract_bin_data(bin_block);
            if (!hex_data) {
                printf("Failed to extract <bin> section from scan data.\n");
                continue;
            }

            parse_lidar_bin(hex_data, scan_log, MAX_POINTS);
            free(hex_data);
            printf("Parsed scan into distances.\n");

            Cluster clusters[MAX_CLUSTERS];
            int n = find_clusters(scan_log, clusters, MAX_CLUSTERS);
            printf("Found %d clusters in this scan.\n", n);

            // --- Pick best cluster: smallest distance + closest to 90° ---
            float best_score = 9999.0f;
            int best_idx = -1;
            for (int i = 0; i < n; i++) {
                float angle_error = fabs(clusters[i].centroid_angle - 90.0f);
                float score = clusters[i].centroid_distance + 0.5f * angle_error;
                if (score < best_score) {
                    best_score = score;
                    best_idx = i;
                }
            }

            // --- Log centroid of best cluster ---
            if (best_idx >= 0 && traj_count < MAX_CENTROID_HISTORY) {
                float angle_rad = clusters[best_idx].centroid_angle * M_PI / 180.0f;
                trajectory[traj_count].angle_deg = clusters[best_idx].centroid_angle;
                trajectory[traj_count].distance_m = clusters[best_idx].centroid_distance;
                trajectory[traj_count].x = cosf(angle_rad) * clusters[best_idx].centroid_distance;
                trajectory[traj_count].y = sinf(angle_rad) * clusters[best_idx].centroid_distance;
                printf("Logged centroid %d: angle=%.2f°, distance=%.2f m\n",
                       traj_count, trajectory[traj_count].angle_deg, trajectory[traj_count].distance_m);
                traj_count++;
            } else {
                printf("No valid centroid to log in this scan.\n");
            }
        }
    }

    close(sock);
    log_trajectory(TRAJ_LOG_FILE);
    return 0;
}
