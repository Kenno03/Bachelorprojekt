// --- Merged Program: LiDAR Centroid Tracking + Velocity Control + Curvature Steering ---
// Author: Markus Kenno Hansen
// Description: Tracks a leading robot using LiDAR centroid clustering,
// calculates curvature based on angle deviation, and uses PID control
// to adjust velocity while steering with differential wheel speeds.

#include <arpa/inet.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

// --- Configuration ---
#define ODO_LASER_PORT 24919                  // Port for LiDAR data
#define CMD_PORT 31001                        // Port for sending motor commands
#define SERVER_IP "192.38.66.91"              // IP of the robot system
#define MAX_POINTS 512                        // Max number of LiDAR scan points
#define ANGLE_RESOLUTION 0.36f               // Angular resolution per point
#define START_ANGLE_DEG -120.0f              // Starting angle of LiDAR scan
#define ROTATION_OFFSET 120.0f               // Offset to align front to 90°
#define M_PI 3.14159265358979323846
#define MAX_CLUSTERS 32
#define MAX_CENTROID_HISTORY 1000
#define LOG_DURATION 30                      // How long the program should run (seconds)
#define INTERVAL 100000                      // Delay between loop iterations (100ms)
#define MAX_VELOCITY 0.2                     // Cap velocity output
#define DESIRED_DISTANCE 0.3f                // Target distance to maintain
#define SEARCH_ANGLE_MARGIN 20.0f            // +/- angle window to accept clusters
#define WHEELBASE 0.23f                      // Distance between robot wheels
#define DEADZONE_MIN 87.0f                   // Do not update angle if within 87–93°
#define DEADZONE_MAX 93.0f
#define LOOKAHEAD_DIST 0.3f                  // Lookahead used for curvature calc
#define MAX_CENTROID_JUMP 0.3f               // Max change in distance between centroids

// --- Logging ---
#define LOG_SIZE 2000
float e[3] = {0}, u[3] = {0};                // PID error and control memory
float u_log[LOG_SIZE][3];                   // PID control log
float e_log[LOG_SIZE][3];                   // PID error log
float time_log[LOG_SIZE];                   // Timestamps
float distance_log[LOG_SIZE];               // Distance to centroid
float angle_log[LOG_SIZE];                  // Angle of centroid
float v_l_log[LOG_SIZE];                    // Left motor command
float v_r_log[LOG_SIZE];                    // Right motor command
int log_index = 0;                           // Log entry index

// --- Data structs ---
typedef struct {
    float angle_sum, total_distance;
    int count;
    float centroid_angle, centroid_distance;
} Cluster;

typedef struct {
    float angle_deg, distance_m, x, y;
} CentroidLog;

CentroidLog trajectory[MAX_CENTROID_HISTORY];
int traj_count = 0;
float scan_log[MAX_POINTS];

// --- Setup TCP client connection ---
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

// --- Send command over socket ---
void send_command(int client_fd, const char* command) {
    send(client_fd, command, strlen(command), 0);
    usleep(100000); // Wait for server to process
}

// --- Extract binary LiDAR payload from XML-like format ---
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

// --- Convert LiDAR hex data to float distances ---
void parse_lidar_bin(char *hex_data, float *distances, int count) {
    for (int i = 0; i < count; i++) {
        int index = i * 4;
        char hex_pair[5] = {
            hex_data[index + 2], hex_data[index + 3],
            hex_data[index + 0], hex_data[index + 1], '\0'
        };
        distances[i] = strtol(hex_pair, NULL, 16) * 0.001f;
    }
}

// --- Cluster consecutive valid LiDAR points ---
int find_clusters(float *distances, Cluster *clusters, int max_clusters) {
    const float min_distance = 0.15f, max_distance = 3.0f;
    int cluster_count = 0, in_cluster = 0;
    Cluster current = {0};

    for (int i = 0; i <= MAX_POINTS; i++) {
        float r = (i < MAX_POINTS) ? distances[i] : 0.0f;
        int is_valid = (r > min_distance && r <= max_distance);

        if (!is_valid || i == MAX_POINTS) {
            if (in_cluster && current.count >= 3 && cluster_count < max_clusters) {
                current.centroid_angle = current.angle_sum / current.count;
                current.centroid_distance = current.total_distance / current.count;
                clusters[cluster_count++] = current;
            }
            in_cluster = 0;
            memset(&current, 0, sizeof(Cluster));
        } else {
            float angle = START_ANGLE_DEG + i * ANGLE_RESOLUTION + ROTATION_OFFSET;
            if (!in_cluster) {
                in_cluster = 1;
                current.total_distance = 0;
                current.angle_sum = 0;
                current.count = 0;
            }
            current.total_distance += r;
            current.angle_sum += angle;
            current.count++;
        }
    }

    return cluster_count;
}

// --- Discrete PID controller (distance) ---
float control(float ref, float measurement) {
    double b[] = {23.0548, -32.0138, 11.0703};
    double a[] = {1.0, -0.3182, -0.6818};

    e[0] = measurement - ref;
    u[0] = b[0]*e[0] + b[1]*e[1] + b[2]*e[2] - a[1]*u[1] - a[2]*u[2];

    // Saturate output
    if (u[0] > MAX_VELOCITY) u[0] = MAX_VELOCITY;
    else if (u[0] < 0) u[0] = 0;

    // Log values
    if (log_index < LOG_SIZE) {
        for (int i = 0; i < 3; i++) {
            u_log[log_index][i] = u[i];
            e_log[log_index][i] = e[i];
        }
        time_log[log_index] = log_index * 0.1f;
    }

    // Shift buffers
    for (int i = 2; i > 0; i--) {
        u[i] = u[i-1];
        e[i] = e[i-1];
    }

    return u[0];
}

// --- Save trajectory log to CSV ---
void log_trajectory(const char* filename) {
    FILE* fp = fopen(filename, "w");
    if (!fp) { perror("Failed to open log file"); return; }
    fprintf(fp, "angle_deg,distance_m,x,y\n");
    for (int i = 0; i < traj_count; i++) {
        fprintf(fp, "%.2f,%.3f,%.3f,%.3f\n", trajectory[i].angle_deg,
                trajectory[i].distance_m, trajectory[i].x, trajectory[i].y);
    }
    fclose(fp);
}

// --- Save PID and control signal log to CSV ---
void log_control(const char* filename) {
    FILE* fp = fopen(filename, "w");
    if (!fp) { perror("Failed to open control log"); return; }
    fprintf(fp, "Time,Distance,Angle,v_l,v_r,u[0],u[1],u[2],e[0],e[1],e[2]\n");
    for (int i = 0; i < log_index; i++) {
        fprintf(fp, "%.2f,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            time_log[i], distance_log[i], angle_log[i],
            v_l_log[i], v_r_log[i],
            u_log[i][0], u_log[i][1], u_log[i][2],
            e_log[i][0], e_log[i][1], e_log[i][2]);
    }
    fclose(fp);
}

// --- MAIN PROGRAM ---
int main() {
    int laser_fd = setup_client(ODO_LASER_PORT);
    int cmd_fd = setup_client(CMD_PORT);
    if (laser_fd == -1 || cmd_fd == -1) return -1;

    send_command(laser_fd, "scanpush cmd='scanget'\n");
    printf("Continuous LiDAR scan started.\n");

    char buffer[32768] = {0};
    int buffer_len = 0;
    float search_angle = 90.0f;
    time_t start = time(NULL);

    while (difftime(time(NULL), start) < LOG_DURATION) {
        int bytes = read(laser_fd, buffer + buffer_len, sizeof(buffer) - buffer_len - 1);
        if (bytes <= 0) { usleep(10000); continue; }
        buffer_len += bytes;
        buffer[buffer_len] = '\0';

        while (1) {
            char* bin_start = strstr(buffer, "<bin");
            char* bin_end = strstr(buffer, "</bin>");
            if (!(bin_start && bin_end && bin_end > bin_start)) break;

            int block_len = bin_end - bin_start + strlen("</bin>");
            char bin_block[16384];
            strncpy(bin_block, bin_start, block_len);
            bin_block[block_len] = '\0';

            size_t shift = (bin_end + strlen("</bin>")) - buffer;
            buffer_len -= shift;
            memmove(buffer, buffer + shift, buffer_len);
            buffer[buffer_len] = '\0';

            char* hex_data = extract_bin_data(bin_block);
            if (!hex_data) continue;
            parse_lidar_bin(hex_data, scan_log, MAX_POINTS);
            free(hex_data);

            Cluster clusters[MAX_CLUSTERS];
            int n = find_clusters(scan_log, clusters, MAX_CLUSTERS);

            float best_score = 9999.0f;
            int best_idx = -1;
            for (int i = 0; i < n; i++) {
                float angle = clusters[i].centroid_angle;
                if (fabs(angle - search_angle) > SEARCH_ANGLE_MARGIN) continue;
                float score = clusters[i].centroid_distance + 0.5f * fabs(angle - search_angle);
                if (score < best_score) { best_score = score; best_idx = i; }
            }

            if (best_idx >= 0 && traj_count < MAX_CENTROID_HISTORY) {
                float angle_deg = clusters[best_idx].centroid_angle;
                float angle_rad = angle_deg * M_PI / 180.0f;
                float dist = clusters[best_idx].centroid_distance;

                // Reject centroid jump
                if (traj_count > 0) {
                    float last_dist = trajectory[traj_count - 1].distance_m;
                    if (fabsf(dist - last_dist) > MAX_CENTROID_JUMP) continue;
                }

                float velocity_cmd = control(DESIRED_DISTANCE, dist);
                distance_log[log_index] = dist;

                // --- Curvature-based steering ---
                float angle_error = angle_deg - 90.0f;
                float angle_rad_err = angle_error * M_PI / 180.0f;
                float kappa = 2.0f * sinf(angle_rad_err) / LOOKAHEAD_DIST;

                float v_l = velocity_cmd * (2.0f - kappa * WHEELBASE) / 2.0f;
                float v_r = velocity_cmd * (2.0f + kappa * WHEELBASE) / 2.0f;

                char cmd[64];
                snprintf(cmd, sizeof(cmd), "motorcmds %.2f %.2f\n", v_l, v_r);
                send_command(cmd_fd, cmd);

                if (log_index < LOG_SIZE) {
                    angle_log[log_index] = angle_deg;
                    v_l_log[log_index] = v_l;
                    v_r_log[log_index] = v_r;
                }

                float x = cosf(angle_rad) * dist;
                float y = sinf(angle_rad) * dist;
                trajectory[traj_count] = (CentroidLog){ angle_deg, dist, x, y };

                if (angle_deg < DEADZONE_MIN || angle_deg > DEADZONE_MAX) {
                    search_angle = angle_deg;
                }

                traj_count++;
                log_index++;
            }
        }
    }

    send_command(cmd_fd, "motorcmds 0 0\n");
    shutdown(cmd_fd, SHUT_RDWR);
    close(cmd_fd);
    close(laser_fd);
    printf("Logging Trajectory\n");
    log_trajectory("trajectory_log.csv");
    printf("Logging Control\n");
    log_control("control_log.csv");
    return 0;
}
