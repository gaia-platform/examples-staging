#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <cassert>
#include <cstring>

#include <iostream>

#include <netinet/in.h>
#include <sys/socket.h>

#include <gaia/logger.hpp>

#include "gaia/db/db.hpp"
#include "json.hpp"

#include "slam_sim.hpp"

// Network interface between simulator and 'satellite' process that
//  serves as an endpoint. 
// It's trivial to create a python process that's an HTTP server using
//  something like flask but it's lot more difficult to do so in c/c++.
//  The satellite process is a python flask app that receives HTTP
//  traffic and relays it over a socket to the comm thread, which in
//  turn sends data back to the satellite for it tor put back in the
//  HTTP pipeline.

using namespace std;

static int g_sockfd = -1;

namespace slam_sim
{

using nlohmann::json;

void thread_shutdown_callback(int sig)
{
    g_quit = 1;
    shutdown(g_sockfd, SHUT_RD);
}

// Wrapper around read() that reads in the full buffer, or fails trying.
static int read_stream(int connfd, char* buf, size_t count)
{
    size_t tot_bytes_read = 0;
    while (tot_bytes_read < count)
    {
        size_t bytes_left = (size_t) (count - tot_bytes_read);
        ssize_t n_bytes = read(connfd, &buf[tot_bytes_read], bytes_left);
        if (n_bytes <= 0)
        {
            perror("Error reading inbound message");
            return -1;
        }
        tot_bytes_read += (size_t) n_bytes;
    }
    return tot_bytes_read;
}


// Wrapper around read() that reads in the full buffer, or fails trying.
static int write_stream(int connfd, const char* buf, size_t count)
{
    size_t tot_bytes_written = 0;
    while (tot_bytes_written < count)
    {
        size_t bytes_left = (size_t) (count - tot_bytes_written);
        ssize_t n_bytes = write(connfd, &buf[tot_bytes_written], bytes_left);
        if (n_bytes <= 0)
        {
            perror("Error writing outbound message");
            return -1;
        }
        tot_bytes_written += (size_t) n_bytes;
    }
    return tot_bytes_written;
}


void process_json(char json_body[], string& response)
{
    const json& packet = json::parse(json_body);
    string type = packet["type"];
    if (type == "position") {
        printf("Recieved position request\n");
        std::cout << json_body << std::endl;
        double x = packet["x"];
        double y = packet["y"];
        double heading = packet["heading"];
        set_position(x, y, heading);
        calculate_sensors(response);
//std::cout << "Response: " << response << std::endl;
    }
    else if (type == "motion") 
    {
        printf("Recieved motion request\n");
        std::cout << json_body << std::endl;
        double dist = packet["distance"];
        double turn_angle = packet["turn_angle"];
        move_robot(dist, turn_angle);
        calculate_sensors(response);
//std::cout << "Response: " << response << std::endl;
    }
    else if (type == "get-map") 
    {
        printf("Recieved map request\n");
        std::cout << json_body << std::endl;
        get_map(response);
//std::cout << "Response: " << response << std::endl;
    }
}


void* comm_thread_main(void* unused)
{
    (void) unused;
    // create and initialize socket.
    g_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    int option = 1;
    setsockopt(g_sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
    if (g_sockfd < 0)
    {
        perror("Failed to create socket");
        exit(-1);
    }
    // bind
    sockaddr_in sockaddr;
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = INADDR_ANY;
    sockaddr.sin_port = htons(SIM_PORT);
    size_t addrlen = sizeof(sockaddr);
    if (bind(g_sockfd, (struct sockaddr*)&sockaddr, sizeof(sockaddr)) < 0)
    {
        gaia_log::app().error("Failed to bind port {}: {}", SIM_PORT,
                strerror(errno));
        exit(-1);
    }
    // Prepare socket for incoming data.
    if (listen(g_sockfd, 2) < 0)
    {
        perror("Failed to listen");
        exit(-1);
    }
    // Initialize Gaia DB access for this thread.
    gaia::db::begin_session();
    ////////////////////////////////////////////
    // Accept connections while program is still running.
    int32_t errs = 0;
    char json_body[JSON_BUFFER_LEN];
    string response;
    while (g_quit == 0)
    {
        int connfd = accept(g_sockfd, (struct sockaddr*) &sockaddr,
                (socklen_t*) &addrlen);
        if (connfd < 0)
        {
            perror("Failed to accept connection");
            if (errs++ > 2)
            {
                gaia_log::app().error("Multiple sequential failures "
                        "-- bailing out");
                exit(-1);
            }
            continue;
        }
        errs = 0;
        // Connection established -- read incoming message.
        // Message is composed of 16 char header string, storing
        //  packet size, and then json packet.
        char header[32] = { 0 };
        memset(header, 0, sizeof header);
        if (read_stream(connfd, header, PACKET_HEADER_LEN) < 0)
        {
            errs++;
            continue;
        }
        // Read body.
        errno = 0;
        size_t size = (size_t) strtol(header, NULL, 10);
        // Inbound packets should be relatively small. If they're too 
        //  big assume it's an error
        assert(size < sizeof json_body - 1);
        memset(json_body, 0, sizeof json_body);
        if (read_stream(connfd, json_body, size) < 0)
        {
            errs++;
            continue;
        }
        json_body[size] = 0;    // NULL terminate string.
        ////////////////////////////////////////////////////////////////
        // Packet has arrived. Parse json and handle request.
        process_json(json_body, response);
        // Send response to calling process.
        sprintf(header, "%16lu", response.size());
        write_stream(connfd, header, PACKET_HEADER_LEN);
        write_stream(connfd, response.c_str(), response.size());
        // All done. Clean up and close socket connection.
        close(connfd);
    }
    if (g_sockfd >= 0)
    {
        close(g_sockfd);
    }
    gaia::db::end_session();
    return NULL;
}

} // namespace slam_sim

