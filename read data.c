#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define PORT 24919  // Laser scanner server port
#define SERVER_IP "192.38.66.89"  // Replace with your server's IP

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

    // Convert IP address
    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        printf("Invalid address / Address not supported\n");
        return -1;
    }

    // Connect to the laser scanner server
    if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("Connection Failed\n");
        return -1;
    }

    printf("Connected to laser scanner server\n");

    // Send command to start scanning
    char* startCommand = "laser scanpush cmd='zoneobst'\n";
    send(client_fd, startCommand, strlen(startCommand), 0);
    printf("Zoneobst scan started\n");

    sleep(1);  // Allow time for the scanner to start sending data

    // Request data multiple times
    for (int i = 0; i < 5; i++) {
        send(client_fd, "zoneobst\n", 9, 0);
        sleep(1);

        int valread = read(client_fd, buffer, sizeof(buffer) - 1);
        if (valread < 0) {
            printf("Error reading from server\n");
            close(client_fd);
            return -1;
        }

        buffer[valread] = '\0';  // Null-terminate response
        printf("Received: %s\n", buffer);
    }

    // Close connection
    close(client_fd);
    return 0;
}
