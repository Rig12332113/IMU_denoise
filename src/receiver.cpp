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

#define MAX 1024
#define PORT 8888
#define SA struct sockaddr 
  
struct IMUsample{
    double timestamp{};
    double quat[4]{};
    double acc_g[3]{};
};

static void printSample(const IMUsample& s) {
    std::cout << "Parsed IMUsample:\n";
    std::cout << "  t    = " << s.timestamp << "\n";
    std::cout << "  quat = ["
              << s.quat[0] << ", " << s.quat[1] << ", " << s.quat[2] << ", " << s.quat[3] << "]\n";
    std::cout << "  acc_g= ["
              << s.acc_g[0] << ", " << s.acc_g[1] << ", " << s.acc_g[2] << "]\n";
}

bool parse_one_quat_accg(const std::string& line, IMUsample& out) {
    using nlohmann::json;

    json j;
    try {
        j = json::parse(line);
    } catch (...) {
        return false;
    }

    if (!j.contains("t") || !j.contains("quat") || !j.contains("acc_g")) {
        return false;
    }

    const auto& q = j["quat"];
    const auto& a = j["acc_g"];
    if (!q.is_array() || q.size() != 4) return false;
    if (!a.is_array() || a.size() != 3) return false;

    try {
        out.timestamp = j["t"].get<double>();
        for (int i = 0; i < 4; ++i) out.quat[i] = q[i].get<double>();
        for (int i = 0; i < 3; ++i) out.acc_g[i] = a[i].get<double>();
    } catch (...) {
        return false;
    }
    return true;
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
                printSample(sample);
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