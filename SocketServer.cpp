

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <set>
#include <mutex>

// Alias for WebSocket++ server
typedef websocketpp::server<websocketpp::config::asio> server;

// Global variables
std::vector<float> temperature_data(5, 0.0);
std::vector<float> pressure_data(5, 0.0);
std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> connections;
std::mutex connections_mutex;
server ws_server; // Use the global WebSocket server instance

// Callback function to receive temperature data
void temperatureCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> guard(connections_mutex);

    for (size_t i = 0; i < msg->data.size(); i++) {
        temperature_data[i] = msg->data[i];
    }

    std::string message = "Received temperature data: [";
    for (size_t i = 0; i < temperature_data.size(); i++) {
        message += std::to_string(temperature_data[i]);
        if (i != temperature_data.size() - 1) {
            message += ", ";
        }
    }
    message += "]";

    std::cout << message << std::endl;

    // Broadcast the message to all connected WebSocket clients
    for (auto it : connections) {
        try {
            ws_server.send(it, message, websocketpp::frame::opcode::text);
        } catch (websocketpp::exception const & e) {
            std::cerr << "Error broadcasting message: " << e.what() << std::endl;
        }
    }
}
void pressureCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> guard(connections_mutex);

    for (size_t i = 0; i < msg->data.size(); i++) {
        pressure_data[i] = msg->data[i];
    }
    std::string message = "Received pressure data: [";
    for (size_t i = 0; i < pressure_data.size(); i++) {
        message += std::to_string(pressure_data[i]);
        if (i != pressure_data.size() - 1) {
            message += ", ";
        }
    }
    message += "]";

    std::cout << message << std::endl;

    // Broadcast the message to all connected WebSocket clients
    for (auto it : connections) {
        try {
            ws_server.send(it, message, websocketpp::frame::opcode::text);
        } catch (websocketpp::exception const & e) {
            std::cerr << "Error broadcasting message: " << e.what() << std::endl;
        }
    }
}

// Function to publish temperature data in a separate thread
void publishTemperatureData(ros::Publisher& temp_pub) {
    ros::Rate loop_rate(1); // Publish at 1 Hz
    while (ros::ok()) {
        std_msgs::Float32MultiArray temp_msg;
        temp_msg.data.resize(5);
        for (size_t i = 0; i < 5; i++) {
            temp_msg.data[i] = static_cast<float>(std::rand() % 400) / 10.0; // Generate values between 0.0 and 39.9
        }

        temp_pub.publish(temp_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// WebSocket++ event handlers
void on_open(server* s, websocketpp::connection_hdl hdl) {
    std::lock_guard<std::mutex> guard(connections_mutex);
    connections.insert(hdl);
    std::cout << "Client connected." << std::endl;
}

void on_close(server* s, websocketpp::connection_hdl hdl) {
    std::lock_guard<std::mutex> guard(connections_mutex);
    connections.erase(hdl);
    std::cout << "Client disconnected." << std::endl;
}

void on_fail(server* s, websocketpp::connection_hdl hdl) {
    std::cerr << "WebSocket connection failed." << std::endl;
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "socket_server_node");
    ros::NodeHandle nh;

    // Set up the temperature data publisher
    ros::Publisher temp_pub = nh.advertise<std_msgs::Float32MultiArray>("temperature_data", 10);

    // Set up the temperature data subscriber
    ros::Subscriber temp_sub = nh.subscribe("temperature_data", 10, temperatureCallback);

    ros::Subscriber pressure_sub = nh.subscribe("pressure_data", 10, pressureCallback);

    // Seed random number generator
    std::srand(std::time(0));

    // Start a separate thread for publishing temperature data
    std::thread publisher_thread(publishTemperatureData, std::ref(temp_pub));

    // WebSocket++ server setup
    ws_server.set_open_handler(bind(&on_open, &ws_server, ::_1));
    ws_server.set_close_handler(bind(&on_close, &ws_server, ::_1));
    ws_server.set_fail_handler(bind(&on_fail, &ws_server, ::_1));

    ws_server.init_asio();
    ws_server.listen(boost::asio::ip::tcp::v4(), 9002);  // Explicitly bind to IPv4
    ws_server.start_accept();

    // Run the WebSocket++ server in a separate thread
    std::thread ws_thread([]() {
        try {
            ws_server.run();
        } catch (const std::exception& e) {
            std::cerr << "WebSocket server error: " << e.what() << std::endl;
        }
    });

    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    // Sample HTTP response to send to the client
    const char *http_response_template = 
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/plain\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Access-Control-Allow-Methods: POST, OPTIONS\r\n"
        "Access-Control-Allow-Headers: Content-Type\r\n"
        "Content-Length: %d\r\n"
        "\r\n"
        "Command received: %s\n";

    const char *options_response = 
        "HTTP/1.1 204 No Content\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Access-Control-Allow-Methods: POST, OPTIONS\r\n"
        "Access-Control-Allow-Headers: Content-Type\r\n"
        "\r\n";

    // Sample HTTP response to redirect to the video stream
    const char *video_redirect_template = 
    "HTTP/1.1 302 Found\r\n"
    "Location: http://192.168.0.109:8081/stream?topic=/usb_cam/image_raw\r\n"
    "Content-Length: 0\r\n"
    "\r\n";


    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        std::cerr << "Socket failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        std::cerr << "setsockopt" << std::endl;
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8080);

    // Bind the socket to the network address and port
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Listen for incoming connections
    if (listen(server_fd, 3) < 0) {
        std::cerr << "Listen failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    while (ros::ok()) {
        // Accept an incoming connection
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            std::cerr << "Accept failed" << std::endl;
            exit(EXIT_FAILURE);
        }

        // Read the client's request
        read(new_socket, buffer, 1024);
        std::cout << "Received request:\n" << buffer << std::endl;

        // Check for OPTIONS request (CORS preflight)
        if (strncmp(buffer, "OPTIONS", 7) == 0) {
            send(new_socket, options_response, strlen(options_response), 0);
            close(new_socket);
            continue; // Wait for another connection
        }
        
        if (strncmp(buffer, "GET /video", 10) == 0 || strncmp(buffer, "HEAD /video", 11) == 0) {
    // If the request is for /video, redirect to the webcam stream
            const char *video_redirect_template = 
            "HTTP/1.1 302 Found\r\n"
            "Location: http://192.168.0.109:8081/stream?topic=/usb_cam/image_raw\r\n"
            "Content-Length: 0\r\n"
            "\r\n";

            send(new_socket, video_redirect_template, strlen(video_redirect_template), 0);
            close(new_socket);
            continue; // Wait for another connection
        }


        // Process POST request for direction commands
        if (strncmp(buffer, "POST", 4) == 0) {
            char *json_body = strstr(buffer, "\r\n\r\n");
            if (json_body) {
                json_body += 4; // Skip the "\r\n\r\n"

                // Parse JSON body to extract the direction value
                const char *direction_key = "\"direction\":\"";
                char *direction_start = strstr(json_body, direction_key);
                if (direction_start) {
                    direction_start += strlen(direction_key); // Move past "direction":"
                    char *direction_end = strchr(direction_start, '"'); // Find the closing quote
                    if (direction_end) {
                        std::string direction(direction_start, direction_end - direction_start);
                        std::string temp_data_str = "Temperature data: [";
                        for (size_t i = 0; i < temperature_data.size(); i++) {
                            temp_data_str += std::to_string(temperature_data[i]);
                            if (i != temperature_data.size() - 1) {
                                temp_data_str += ", ";
                            }
                        }
                        temp_data_str += "]";
                        std::string pressure_data_str = "Pressure data: [";
                        for (size_t i = 0; i < pressure_data.size(); i++) {
                            pressure_data_str += std::to_string(pressure_data[i]);
                            if (i != pressure_data.size() - 1) {
                                pressure_data_str += ", ";
                            }
                        }
                        pressure_data_str += "]";

                        // Create the HTTP response with temperature data
                        char response[1024];
                        snprintf(response, sizeof(response), http_response_template, (int)(strlen(direction.c_str()) + 1), direction.c_str());

                        send(new_socket, response, strlen(response), 0);
                        std::cout << "Sent response: " << response << std::endl;
                    }
                }
            }
        }

        // Close the socket
        close(new_socket);
    }

    // Ensure all threads complete before exiting
    publisher_thread.join();
    ws_thread.join();

    return 0;
}
