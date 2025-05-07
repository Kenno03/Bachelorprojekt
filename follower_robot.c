// --- Improved LiDAR Follower Program with Direct Pose Extraction, Geometric Offset Biasing, Fixed Angle Reference, and Centroid Delay ---
// Author: Markus Kenno Hansen (fully merged and uncut)

#include <arpa/inet.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/time.h>

#define ODO_LASER_PORT 24919
#define CMD_PORT 31001
#define SERVER_IP "192.38.66.95" //smr15
#define MAX_POINTS 512
#define ANGLE_RESOLUTION 0.36f
#define START_ANGLE_DEG -120.0f
#define ROTATION_OFFSET 120.0f
#define M_PI 3.14159265358979323846
#define MAX_CLUSTERS 32
#define MAX_CENTROID_HISTORY 1000
#define INTERVAL 100000
#define MAX_VELOCITY 0.2
#define DESIRED_DISTANCE 0.2f
#define SEARCH_ANGLE_MARGIN 45.0f
#define WHEELBASE 0.26f
#define LOOKAHEAD_DIST 0.8f
#define ROBOT_WIDTH 0.28f
#define LOG_SIZE 2000
#define TRAJ_DELAY 5

volatile int stop_requested = 0;

float e[3] = {0}, u[3] = {0};
float u_log[LOG_SIZE][3], e_log[LOG_SIZE][3];
float time_log[LOG_SIZE], distance_log[LOG_SIZE], angle_log[LOG_SIZE];
float x_log[LOG_SIZE], y_log[LOG_SIZE];
float v_l_log[LOG_SIZE], v_r_log[LOG_SIZE];
int cluster_size_log[LOG_SIZE];
int log_index = 0;

typedef struct {
    float angle_sum, total_distance;
    int count;
    float centroid_angle, centroid_distance;
    float min_distance;
    float min_angle;
} Cluster;

typedef struct {
    float angle_deg, distance_m;
    float x_global, y_global;
} CentroidLog;

CentroidLog trajectory[MAX_CENTROID_HISTORY];
int traj_count = 0;
float scan_log[MAX_POINTS];

float x_global = 0.0f, y_global = 0.0f, theta_global = 0.0f;
float x_offset = 0.0f, y_offset = 0.0f, theta_offset = 0.0f;
int offset_initialized = 0;
int centroid_lost_warning_printed = 0;

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

char* extract_tag_value(const char* buf, const char* tag, const char* attr, char* out) {
    char find[64];
    sprintf(find, "<%s ", tag);
    char* start = strstr(buf, find);
    if (!start) return NULL;
    start = strstr(start, attr);
    if (!start) return NULL;
    start = strchr(start, '\"') + 1;
    char* end = strchr(start, '\"');
    strncpy(out, start, end-start);
    out[end-start] = '\0';
    return out;
}

char* extract_bin_data(char* buffer) {
    char* start = strstr(buffer, "<bin");
    if (!start) return NULL;
    start = strchr(start, '>') + 1;
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
        int idx = i * 4;
        char hex_pair[5] = { hex_data[idx+2], hex_data[idx+3], hex_data[idx], hex_data[idx+1], '\0' };
        distances[i] = strtol(hex_pair, NULL, 16) * 0.001f;
    }
    free(hex_data);
}

int find_clusters(float *distances, Cluster *clusters, int max_clusters) {
    const float min_distance = 0.15f, max_distance = 1.0f;
    int cluster_count = 0, in_cluster = 0;
    Cluster current = {0};
    current.min_distance = 9999.0f;

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
            current.min_distance = 9999.0f;
        } else {
            float a = START_ANGLE_DEG + i * ANGLE_RESOLUTION + ROTATION_OFFSET - 90.0f;
            if (!in_cluster) in_cluster = 1;
            current.total_distance += r;
            current.angle_sum += a;
            current.count++;
            if (r < current.min_distance) {
                current.min_distance = r;
                current.min_angle = a;
            }
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
    fprintf(fp, "Time,Angle,Distance,Robot_X,Robot_Y,Robot_Heading_deg,Centroid_X,Centroid_Y,ClusterSize\n");
    for (int i = 0; i < traj_count && i < log_index; i++) {
        fprintf(fp, "%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%d\n",
            time_log[i], angle_log[i], distance_log[i],
            x_log[i], y_log[i], theta_global * 180.0f / M_PI,
            trajectory[i].x_global, trajectory[i].y_global,
            cluster_size_log[i]);
    }
    fclose(fp);
}


void log_control(const char* filename) {
    FILE* fp = fopen(filename, "w");
    if (!fp) return;
    fprintf(fp, "Time,v_l,v_r,u0,u1,u2,e0,e1,e2\n");
    for (int i = 0; i < log_index; i++) {
        fprintf(fp, "%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            time_log[i],
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

    printf("Servers Started...\n");
    send_command(laser_fd, "scanpush cmd='scanget'\n");

    char buffer[32768] = {0};
    int buffer_len = 0;
    float search_angle = 0.0f;
    float kappa_filtered = 0;
    float last_valid_angle_deg = 0.0f;
    float last_valid_distance = DESIRED_DISTANCE;
    int frames_without_centroid = 0;

    while (!stop_requested) {
        int bytes = read(laser_fd, buffer + buffer_len, sizeof(buffer) - buffer_len - 1);
        if (bytes <= 0) { usleep(10000); continue; }
        buffer_len += bytes;
        buffer[buffer_len] = '\0';

        char* bin_start = strstr(buffer, "<bin");
        char* bin_end = strstr(buffer, "</bin>");
        if (!(bin_start && bin_end && bin_end > bin_start)) continue;
    

        // Extract global robot pose
        char pose_x[32], pose_y[32], pose_h[32];
        extract_tag_value(buffer, "pose", "x", pose_x);
        extract_tag_value(buffer, "pose", "y", pose_y);
        extract_tag_value(buffer, "pose", "h", pose_h);
        
        x_global = atof(pose_x);
        y_global = atof(pose_y);
        theta_global = atof(pose_h);

        
        if (!offset_initialized) {
            x_offset = x_global;
            y_offset = y_global;
            theta_offset = theta_global;
            offset_initialized = 1;
        }
        
        x_global -= x_offset;
        y_global -= y_offset;
        theta_global -= theta_offset;
        
        //printf("[POSE] x=%.3f y=%.3f h=%.3f deg\n", x_global, y_global, theta_global * 180.0f / M_PI);
        
        

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

        Cluster clusters[MAX_CLUSTERS];
        int n = find_clusters(scan_log, clusters, MAX_CLUSTERS);

        int best_idx = -1;
        float best_score = 9999.0f;
        for (int i = 0; i < n; i++) {
            if (clusters[i].count < 20 || clusters[i].count > 140) continue;
            float a = clusters[i].centroid_angle;
            float d = clusters[i].centroid_distance;
            float expected_points = 2.0f * atanf((ROBOT_WIDTH / 2.0f) / d) / (ANGLE_RESOLUTION * M_PI / 180.0f);
            float size_penalty = fabsf(clusters[i].count - expected_points);
            float score = d + 0.2f * fabs(a - search_angle) + 0.2f * size_penalty;
            if (fabs(a - search_angle) <= SEARCH_ANGLE_MARGIN && score < best_score) {
                best_score = score;
                best_idx = i;
            }
        }

        float angle_deg = 0.0f, dist = DESIRED_DISTANCE;
        int selected_cluster_size = -1;
        if (best_idx >= 0) {
            frames_without_centroid = 0;
            centroid_lost_warning_printed = 0;

            float angle_c = clusters[best_idx].centroid_angle * M_PI / 180.0f;
            float angle_p = clusters[best_idx].min_angle * M_PI / 180.0f;
            float dist_c = clusters[best_idx].centroid_distance;
            float dist_p = clusters[best_idx].min_distance;

            float xc = cosf(angle_c) * dist_c;
            float yc = sinf(angle_c) * dist_c;
            float xp = cosf(angle_p) * dist_p;
            float yp = sinf(angle_p) * dist_p;

            float dx = xc - xp;
            float dy = yc - yp;
            float len = sqrtf(dx * dx + dy * dy);
            if (len == 0) len = 0.001f;

            float x_shift = xp + dx;
            float y_shift = yp + dy;

            dist = sqrtf(x_shift * x_shift + y_shift * y_shift);
            angle_deg = atan2f(y_shift, x_shift) * 180.0f / M_PI;

            selected_cluster_size = clusters[best_idx].count;
            last_valid_angle_deg = angle_deg;
            last_valid_distance = dist;
        } else if (frames_without_centroid < 5) {
            frames_without_centroid++;
            angle_deg = last_valid_angle_deg;
            dist = last_valid_distance;
        } else {
            if (!centroid_lost_warning_printed) {
                printf("Centroid lost for more than 5 frames.\n");
                centroid_lost_warning_printed = 1;
            }
            continue;
        }

        float angle_rad = angle_deg * M_PI / 180.0f;
        float x_local = cosf(angle_rad) * dist;
        float y_local = sinf(angle_rad) * dist;
        float xg = x_global + cosf(theta_global) * x_local - sinf(theta_global) * y_local;
        float yg = y_global + sinf(theta_global) * x_local + cosf(theta_global) * y_local;

        trajectory[traj_count++] = (CentroidLog){ angle_deg, dist, xg, yg };

        int critical = 0;
        for (int i = 0; i < MAX_POINTS; i++) {
            float angle = START_ANGLE_DEG + i * ANGLE_RESOLUTION + ROTATION_OFFSET - 90.0f;
            if (fabs(angle) < 10.0f && scan_log[i] > 0.05f && scan_log[i] < 0.18f) {
                critical = 1;
                break;
            }
        }
        if (critical) {
            send_command(cmd_fd, "motorcmds 0 0\n");
            printf("Emergency stop: obstacle detected ahead within 18 cm\n");
            continue;
        }

        if (traj_count < TRAJ_DELAY) continue;
        int delayed_index = traj_count - TRAJ_DELAY;
        angle_deg = trajectory[delayed_index].angle_deg;
        dist = trajectory[delayed_index].distance_m;

        float velocity_cmd = control(DESIRED_DISTANCE, dist);

        struct timeval current_time;
        gettimeofday(&current_time, NULL);
        float elapsed_time = (current_time.tv_sec - start_time.tv_sec) +
                             (current_time.tv_usec - start_time.tv_usec) / 1e6f;

        float dt = INTERVAL / 1e6f;
        float v = velocity_cmd;

        float angle_error = angle_deg;
        float angle_err_rad = angle_error * M_PI / 180.0f;
        float kappa_raw = 2.0f * sinf(angle_err_rad) / LOOKAHEAD_DIST;
        kappa_filtered = (fabsf(angle_error) > 10.0f) ?
                         kappa_raw : 0.7f * kappa_filtered + 0.3f * kappa_raw;

        float v_l = v * (1.0f - kappa_filtered * WHEELBASE / 2.0f);
        float v_r = v * (1.0f + kappa_filtered * WHEELBASE / 2.0f);

        char cmd[64];
        snprintf(cmd, sizeof(cmd), "motorcmds %.2f %.2f\n", v_l, v_r);
        send_command(cmd_fd, cmd);

        if (log_index < LOG_SIZE) {
            time_log[log_index] = elapsed_time;
            distance_log[log_index] = dist;
            angle_log[log_index] = angle_deg;
            v_l_log[log_index] = v_l;
            v_r_log[log_index] = v_r;
            u_log[log_index][0] = u[0];
            u_log[log_index][1] = u[1];
            u_log[log_index][2] = u[2];
            e_log[log_index][0] = e[0];
            e_log[log_index][1] = e[1];
            e_log[log_index][2] = e[2];
            x_log[log_index] = x_global;
            y_log[log_index] = y_global;
            cluster_size_log[log_index] = selected_cluster_size;
            log_index++;
        }

        float angle_offset = fabsf(angle_deg);
        search_angle = (angle_offset > 10.0f) ?
                       0.7f * search_angle + 0.3f * angle_deg :
                       0.9f * search_angle + 0.1f * angle_deg;
    }

    send_command(cmd_fd, "motorcmds 0 0\n");
    shutdown(cmd_fd, SHUT_RDWR);
    close(cmd_fd);
    close(laser_fd);
    log_full("full_log15.csv");
    log_control("control_log.csv");

    return 0;
}
