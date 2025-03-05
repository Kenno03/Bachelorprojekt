#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define PORT 24919  // Robot server port
#define SERVER_IP "192.38.66.89"  // Replace with your server's IP

// Function to parse odometry data from XML
void parse_odoPose(char *response, float *x, float *y, float *h) {
    sscanf(response, "<pose name=\"newest\" x=\"%f\" y=\"%f\" h=\"%f\"", x, y, h);
}

int main() {
    int client_fd;
    struct sockaddr_in serv_addr;
    char buffer[4096] = {0};

    // Create socket
    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("Socket creation error\n");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        printf("Invalid address / Address not supported\n");
        return -1;
    }

    // Connect to the server
    if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("Connection Failed\n");
        return -1;
    }

    printf("Connected to odometry and laser scanner server\n");

    // --- Step 1: Start Zoneobst Scanning ---
    char* startCommand = "laser scanpush cmd='zoneobst'\n";
    send(client_fd, startCommand, strlen(startCommand), 0);
    printf("Zoneobst scan started\n");

    sleep(1);  // Allow time for the scanner to start sending data

    // --- Step 2: Request Data Multiple Times ---
    for (int i = 0; i < 5; i++) {
        // Request odometry data
        send(client_fd, "odoPose pose\n", 14, 0);
        sleep(1);

        int valread = read(client_fd, buffer, sizeof(buffer) - 1);
        if (valread < 0) {
            printf("Error reading odometry data\n");
            close(client_fd);
            return -1;
        }

        buffer[valread] = '\0';  // Null-terminate response
        float x, y, h;
        parse_odoPose(buffer, &x, &y, &h);
        printf("Odometry Data: X = %.3f, Y = %.3f, Theta = %.3f\n", x, y, h);

        // Request laser data
        send(client_fd, "zoneobst\n", 9, 0);
        sleep(1);

        valread = read(client_fd, buffer, sizeof(buffer) - 1);
        if (valread < 0) {
            printf("Error reading laser data\n");
            close(client_fd);
            return -1;
        }

        buffer[valread] = '\0';  // Null-terminate response
        printf("Laser Data Received: %s\n", buffer);
    }

    // Close connection
    close(client_fd);
    return 0;
}
