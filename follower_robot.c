// --- Improved LiDAR Follower Program (Stabilized Steering + Safety) ---
// Author: Markus Kenno Hansen
// Updates: Emergency stop, smoother turning, angle deadzone, curvature filtering

#include <arpa/inet.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#define ODO_LASER_PORT 24919
#define CMD_PORT 31001
#define SERVER_IP "192.38.66.95"
#define MAX_POINTS 512
#define ANGLE_RESOLUTION 0.36f
#define START_ANGLE_DEG -120.0f
#define ROTATION_OFFSET 120.0f
#define M_PI 3.14159265358979323846
#define MAX_CLUSTERS 32
#define MAX_CENTROID_HISTORY 1000
#define INTERVAL 100000
#define MAX_VELOCITY 0.2
#define DESIRED_DISTANCE 0.3f
#define SEARCH_ANGLE_MARGIN 25.0f
#define WHEELBASE 0.23f
#define DEADZONE_WIDTH 3.0f
#define LOOKAHEAD_DIST 0.3f
#define MAX_CENTROID_JUMP 0.3f
#define ROBOT_WIDTH 0.28f
#define LOG_SIZE 2000
#define MAX_ANGLE_JUMP 20.0f
#define CENTROID_DELAY 5

volatile int stop_requested = 0;

float e[3] = {0}, u[3] = {0};
float u_log[LOG_SIZE][3];
float e_log[LOG_SIZE][3];
float time_log[LOG_SIZE];
float distance_log[LOG_SIZE];
float angle_log[LOG_SIZE];
float v_l_log[LOG_SIZE];
float v_r_log[LOG_SIZE];
int log_index = 0;

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

void* input_thread(void* arg) {
    char command[64];
    while (1) {
        fgets(command, sizeof(command), stdin);
        if (strncmp(command, "stop", 4) == 0) {
            stop_requested = 1;
            break;
        }
    }
    return NULL;
}

int setup_client(int port) {
    int client_fd;
    struct sockaddr_in serv_addr;
    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) return -1;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr);
    if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) return -1;
    return client_fd;
}

void send_command(int fd, const char* cmd) {
    send(fd, cmd, strlen(cmd), 0);
    usleep(INTERVAL);
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
        char hex_pair[5] = {
            hex_data[index + 2], hex_data[index + 3],
            hex_data[index + 0], hex_data[index + 1], '\0'
        };
        distances[i] = strtol(hex_pair, NULL, 16) * 0.001f;
    }
}

int find_clusters(float *distances, Cluster *clusters, int max_clusters) {
    const float min_distance = 0.15f, max_distance = 3.0f;
    int cluster_count = 0, in_cluster = 0;
    Cluster current = {0};
    for (int i = 0; i <= MAX_POINTS; i++) {
        float r = (i < MAX_POINTS) ? distances[i] : 0.0f;
        int valid = (r > min_distance && r <= max_distance);
        if (!valid || i == MAX_POINTS) {
            if (in_cluster && current.count >= 3 && cluster_count < max_clusters) {
                current.centroid_angle = current.angle_sum / current.count;
                current.centroid_distance = current.total_distance / current.count;
                clusters[cluster_count++] = current;
            }
            in_cluster = 0;
            memset(&current, 0, sizeof(Cluster));
        } else {
            float a = START_ANGLE_DEG + i * ANGLE_RESOLUTION + ROTATION_OFFSET;
            if (!in_cluster) in_cluster = 1;
            current.total_distance += r;
            current.angle_sum += a;
            current.count++;
        }
    }
    return cluster_count;
}

float control(float ref, float meas) {
    double b[] = {23.0548, -32.0138, 11.0703};
    double a[] = {1.0, -0.3182, -0.6818};
    e[0] = meas - ref;
    u[0] = 0.4 * (b[0]*e[0] + b[1]*e[1] + b[2]*e[2] - a[1]*u[1] - a[2]*u[2]);
    if (u[0] > MAX_VELOCITY) u[0] = MAX_VELOCITY;
    if (u[0] < 0) u[0] = 0;
    for (int i = 2; i > 0; i--) {
        e[i] = e[i-1];
        u[i] = u[i-1];
    }
    return u[0];
}

void log_full(const char* filename) {
    FILE* fp = fopen(filename, "w");
    if (!fp) return;
    fprintf(fp, "Time,Distance,Angle,X,Y,v_l,v_r,u0,u1,u2,e0,e1,e2\n");
    for (int i = 0; i < traj_count && i < log_index; i++) {
        fprintf(fp, "%.2f,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            time_log[i], distance_log[i], angle_log[i],
            trajectory[i].x, trajectory[i].y,
            v_l_log[i], v_r_log[i],
            u_log[i][0], u_log[i][1], u_log[i][2],
            e_log[i][0], e_log[i][1], e_log[i][2]);
    }
    fclose(fp);
}

int main() {
    struct timeval start_time;
    gettimeofday(&start_time, NULL);

    pthread_t thread_id;
    pthread_create(&thread_id, NULL, input_thread, NULL);

    int laser_fd = setup_client(ODO_LASER_PORT);
    int cmd_fd = setup_client(CMD_PORT);
    if (laser_fd == -1 || cmd_fd == -1) return -1;

    printf("Servers Started...");

    send_command(laser_fd, "scanpush cmd='scanget'\n");
    send_command(cmd_fd, "log \"$odox\" \"$odoy\" \"$odovelocity\" \"$time\"\n");

    char buffer[32768] = {0};
    int buffer_len = 0;
    float search_angle = 90.0f;
    static float kappa_filtered = 0;
    static float last_valid_angle_deg = 90.0f;
    static float last_valid_distance = DESIRED_DISTANCE;
    static int frames_without_centroid = 0;

    while (!stop_requested) {
        int bytes = read(laser_fd, buffer + buffer_len, sizeof(buffer) - buffer_len - 1);
        if (bytes <= 0) { usleep(10000); continue; }
        buffer_len += bytes;
        buffer[buffer_len] = '\0';

        while (!stop_requested) {
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

            int best_idx = -1;
            float best_score = 9999.0f;
            for (int i = 0; i < n; i++) {
                float a = clusters[i].centroid_angle;
                float d = clusters[i].centroid_distance;
                float expected_points = 2.0f * atanf((ROBOT_WIDTH/2.0f)/d) / (ANGLE_RESOLUTION*M_PI/180.0f);
                float size_penalty = fabsf(clusters[i].count - expected_points);
                float score = d + 0.2f * fabs(a - search_angle) + 0.2f * size_penalty;
                if (fabs(a - search_angle) <= SEARCH_ANGLE_MARGIN && score < best_score) {
                    best_score = score;
                    best_idx = i;
                }
            }

            float angle_deg, dist;
            if (best_idx >= 0) {
                frames_without_centroid = 0;
                angle_deg = clusters[best_idx].centroid_angle;
                dist = clusters[best_idx].centroid_distance;
                last_valid_angle_deg = angle_deg;
                last_valid_distance = dist;
            } else if (frames_without_centroid < 5) {
                frames_without_centroid++;
                angle_deg = last_valid_angle_deg;
                dist = last_valid_distance;
            } else {
                continue;
            }

            if (traj_count > 0) {
                float last_dist = trajectory[traj_count - 1].distance_m;
                float last_angle = trajectory[traj_count - 1].angle_deg;
                if (fabsf(dist - last_dist) > MAX_CENTROID_JUMP) continue;
                if (fabsf(angle_deg - last_angle) > MAX_ANGLE_JUMP) continue;
            }

            if (dist < 0.18f) {
                send_command(cmd_fd, "motorcmds 0 0\n");
                printf("Emergency stop: too close (%.2f m)\n", dist);
                continue;
            }

            float velocity_cmd = control(DESIRED_DISTANCE, dist);

            struct timeval current_time;
            gettimeofday(&current_time, NULL);
            float elapsed_time = (current_time.tv_sec - start_time.tv_sec) +
                                 (current_time.tv_usec - start_time.tv_usec) / 1e6f;

            if (log_index < LOG_SIZE) {
                time_log[log_index] = elapsed_time;
                distance_log[log_index] = dist;
                angle_log[log_index] = angle_deg;
            }

            float angle_error = angle_deg - 90.0f;
            float angle_err_rad = angle_error * M_PI / 180.0f;
            float kappa_raw = 2.0f * sinf(angle_err_rad) / LOOKAHEAD_DIST;

            if (fabsf(angle_error) > 10.0f)
                kappa_filtered = kappa_raw;
            else
                kappa_filtered = 0.7f * kappa_filtered + 0.3f * kappa_raw;

            float v_l, v_r;
            if (fabsf(angle_error) < 2.5f) {
                v_l = v_r = velocity_cmd;
            } else {
                v_l = velocity_cmd * (1.0f - kappa_filtered * WHEELBASE / 2.0f);
                v_r = velocity_cmd * (1.0f + kappa_filtered * WHEELBASE / 2.0f);
            }

            char cmd[64];
            snprintf(cmd, sizeof(cmd), "motorcmds %.2f %.2f\n", v_l, v_r);
            send_command(cmd_fd, cmd);

            float angle_rad = angle_deg * M_PI / 180.0f;
            float x = cosf(angle_rad) * dist;
            float y = sinf(angle_rad) * dist;
            trajectory[traj_count++] = (CentroidLog){ angle_deg, dist, x, y };

            if (log_index < LOG_SIZE) {
                v_l_log[log_index] = v_l;
                v_r_log[log_index] = v_r;
            }

            float angle_offset = fabsf(angle_deg - 90.0f);
            if (angle_offset > 10.0f)
                search_angle = 0.7f * search_angle + 0.3f * angle_deg;
            else
                search_angle = 0.9f * search_angle + 0.1f * angle_deg;

            log_index++;
        }
    }

    send_command(cmd_fd, "motorcmds 0 0\n");
    shutdown(cmd_fd, SHUT_RDWR);
    close(cmd_fd);
    close(laser_fd);

    log_full("full_log.csv");
    return 0;
}
