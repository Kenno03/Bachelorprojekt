#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#define PORT 31001
#define SERVER_IP "192.38.66.89" //SMR9

// Function to set up and connect the client
int setup_client() {
    int client_fd;
    struct sockaddr_in serv_addr;

    // Create socket
    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        return -1;
    }

    // Setup server address struct
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        perror("Invalid address / Address not supported");
        close(client_fd);
        return -1;
    }

    // Connect to the server
    if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection Failed");
        close(client_fd);
        return -1;
    }

    printf("Connected to server!\n");
    return client_fd;  // Return the socket descriptor
}





// Function to send a command
void send_command(int client_fd, const char* command) {
    char buffer[1024] = {0};

    printf("Sending command: %s", command);
    send(client_fd, command, strlen(command), 0);
    sleep(1);  // Delay between commands (adjust as needed)

    // Read server response (if any)
    int valread = read(client_fd, buffer, sizeof(buffer) - 1);
    if (valread > 0) {
        buffer[valread] = '\0';
        printf("Server response: %s\n", buffer);
    }
}

// Main function
int main() {
    // Setup the client
    int client_fd = setup_client();
    if (client_fd == -1) return -1; // Exit if connection failed

    // List of commands to send
    char* commands[] = {
        "fwd 1\n",
        "turn 45\n",
        "fwd 2\n",
        "turn -45\n",
        "fwd 1\n",
    };
    int num_commands = sizeof(commands) / sizeof(commands[0]);

    // Send commands in a loop
    for (int i = 0; i < num_commands; i++) {
        send_command(client_fd, commands[i]);
    }

    // Close connection after all commands are sent
    shutdown(client_fd, SHUT_RDWR);
    close(client_fd);
    printf("Connection closed.\n");

    return 0;
}
