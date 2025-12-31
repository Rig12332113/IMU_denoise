#include <iostream>

#include <stdio.h>
#include <netdb.h> 
#include <netinet/in.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <sys/types.h> 
#include <unistd.h>
#include <nlohmann/json.hpp>
#include "IMUsample.hpp"

#define MAX 1024
#define PORT 8888
#define SA struct sockaddr 

bool parse_one_quat_accg(const std::string& line, IMUsample& out) {
    using nlohmann::json;

    json j;
    try {
        j = json::parse(line);

        if (!j.contains("t") || !j.contains("quat") || !j.contains("acc_g")) {
            return false;
        }

        const auto& qj = j["quat"];
        const auto& aj = j["acc_g"];

        if (!qj.is_array() || qj.size() != 4) return false;
        if (!aj.is_array() || aj.size() != 3) return false;

        double q[4], a[3];
        for (int i = 0; i < 4; ++i) q[i] = qj[i].get<double>();
        for (int i = 0; i < 3; ++i) a[i] = aj[i].get<double>();
        out.setTimestamp(j["t"].get<double>());
        out.setQuat(q);
        out.setAccG(a);
        return true;
    }catch(...){
        return false;
    }
}

void process(int connfd)
{
    std::string data(MAX, '\0');
    std::string accum;
    accum.reserve(8 * MAX);

    bool printed_debug_line = false;

    while (true) {
        ssize_t byteCount = ::read(connfd, &data[0], data.size());
        if (byteCount == 0) {
            std::cout << "Client disconnected.\n";
            break;
        } else if (byteCount < 0) {
            std::perror("read");
            break;
        }

        accum.append(data.data(), static_cast<size_t>(byteCount));

        // Extract complete lines (newline-delimited JSON)
        while (true) {
            size_t pos = accum.find('\n');
            if (pos == std::string::npos) break;

            std::string line = accum.substr(0, pos);
            accum.erase(0, pos + 1);

            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;

            IMUsample sample;
            if (parse_one_quat_accg(line, sample)) {
                std::cout << sample;
            }
        }
    }
}
  
int main() 
{ 
    int sockfd, connfd; 
    socklen_t len;
    struct sockaddr_in servaddr, cli; 
  
    // socket create and verification 
    sockfd = socket(AF_INET, SOCK_STREAM, 0); 
    if (sockfd == -1) { 
        printf("socket creation failed...\n"); 
        exit(0); 
    } 
    else
        printf("Socket successfully created..\n"); 
    bzero(&servaddr, sizeof(servaddr)); 
  
    // assign IP, PORT 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY); 
    servaddr.sin_port = htons(PORT); 
  
    // Binding newly created socket to given IP and verification 
    if ((bind(sockfd, (SA*)&servaddr, sizeof(servaddr))) != 0) { 
        printf("socket bind failed...\n"); 
        exit(0); 
    } 
    else
        printf("Socket successfully binded..\n"); 
  
    // Now server is ready to listen and verification 
    if ((listen(sockfd, 5)) != 0) { 
        printf("Listen failed...\n"); 
        exit(0); 
    } 
    else
        printf("Server listening..\n"); 
    len = sizeof(cli); 
  
    // Accept the data packet from client and verification 
    connfd = accept(sockfd, (SA*)&cli, &len); 
    if (connfd < 0) { 
        printf("server accept failed...\n"); 
        exit(0); 
    } 
    else
        printf("server accept the client...\n"); 
  
    process(connfd); 
  
    // After chatting close the socket 
    close(sockfd); 
}