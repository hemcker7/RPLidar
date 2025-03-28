/*
 *  SLAMTEC LIDAR
 *  Visual Data Logger for RPLidar S2
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <fstream>
#include <ctime>
#include <vector>
#include <cmath>  // Add this for cos and sin functions
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;

// Add shader source code
const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    uniform mat4 projection;
    void main() {
        gl_Position = projection * vec4(aPos.x, aPos.y, 0.0, 1.0);
        gl_PointSize = 5.0;
    }
)";

const char* fragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;
    uniform vec3 color;
    void main() {
        FragColor = vec4(color, 1.0);
    }
)";

// OpenGL variables
GLFWwindow* window = nullptr;
GLuint shaderProgram;
GLuint VBO, VAO;
GLuint circleVBO, circleVAO;
std::vector<float> pointData;
std::vector<float> circleData;
const int MAX_POINTS = 1800;
bool shouldClose = false;

// LIDAR data variables
ILidarDriver* drv = nullptr;
sl_lidar_response_measurement_node_hq_t nodes[8192];
size_t count = 0;
float last_angle = 0;
const int BARCOUNT = 360;
int points_per_degree[BARCOUNT] = {0};
const int MAX_POINTS_PER_DEGREE = 5;
int scan_count = 0;
bool skip_next = false;

// Function declarations
void print_usage(int argc, const char * argv[]);
bool checkSLAMTECLIDARHealth(ILidarDriver * drv);
void ctrlc(int);
void initOpenGL();
void renderFrame();
void cleanupOpenGL();

// Add shader compilation function
GLuint compileShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        fprintf(stderr, "Shader compilation failed: %s\n", infoLog);
        exit(-1);
    }
    return shader;
}

// Modified OpenGL initialization
void initOpenGL() {
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        exit(-1);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(1200, 1200, "LIDAR Visual Logger", NULL, NULL);
    if (!window) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        exit(-1);
    }

    glfwMakeContextCurrent(window);

    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        exit(-1);
    }

    // Create and compile shaders
    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);
    
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    GLint success;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        fprintf(stderr, "Shader program linking failed: %s\n", infoLog);
        exit(-1);
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Create buffers for points
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    
    // Create buffers for circles
    glGenVertexArrays(1, &circleVAO);
    glGenBuffers(1, &circleVBO);

    // Generate circle data
    for (int r = 1000; r <= 4000; r += 1000) {
        for (int i = 0; i <= 360; i++) {
            float angle = i * 3.14159f / 180.0f;
            circleData.push_back(r * cos(angle));
            circleData.push_back(r * sin(angle));
        }
    }

    // Setup circle buffer
    glBindVertexArray(circleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, circleVBO);
    glBufferData(GL_ARRAY_BUFFER, circleData.size() * sizeof(float), circleData.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

// Modified render frame function
void renderFrame() {
    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(shaderProgram);

    // Set up projection matrix
    float projection[16] = {
        2.0f/8000.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 2.0f/8000.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, projection);

    // Draw circles
    GLint colorLoc = glGetUniformLocation(shaderProgram, "color");
    glUniform3f(colorLoc, 0.2f, 0.2f, 0.2f);
    glBindVertexArray(circleVAO);
    for (int i = 0; i < 4; i++) {
        glDrawArrays(GL_LINE_STRIP, i * 361, 361);
    }

    // Draw points
    if (!pointData.empty()) {
        glUniform3f(colorLoc, 1.0f, 0.0f, 0.0f);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, pointData.size() * sizeof(float), pointData.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glDrawArrays(GL_POINTS, 0, pointData.size() / 2);
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
}

// Modified cleanup function
void cleanupOpenGL() {
    if (window) {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteVertexArrays(1, &circleVAO);
        glDeleteBuffers(1, &circleVBO);
        glDeleteProgram(shaderProgram);
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

void print_usage(int argc, const char * argv[])
{
    printf("Usage:\n"
           " For serial channel\n %s --channel --serial <com port> [baudrate] [output_file]\n"
           " The baudrate used by different models is as follows:\n"
           "  A1(115200),A2M7(256000),A2M8(115200),A2M12(256000),"
           "A3(256000),S1(256000),S2(1000000),S3(1000000)\n"
           " For udp channel\n %s --channel --udp <ipaddr> [port NO.] [output_file]\n"
           " The T1 default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
           , argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) {
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            return false;
        } else {
            return true;
        }
    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

void ctrlc(int)
{
    shouldClose = true;
}

int main(int argc, const char * argv[]) {
    const char * opt_is_channel = NULL; 
    const char * opt_channel = NULL;
    const char * opt_channel_param_first = NULL;
    const char * output_file = NULL;
    sl_u32         opt_channel_param_second = 0;
    sl_u32         baudrateArray[2] = {115200, 256000};
    sl_result     op_result;
    int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;
    bool useArgcBaudrate = false;
    IChannel* _channel = NULL;
    time_t now = 0;
    char filename[100] = {0};
    time_t current_time = 0;
    std::ofstream outFile;
    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    printf("RPLidar S2 Visual Data Logger\n"
           "Version: %s\n", SL_LIDAR_SDK_VERSION);

    if (argc < 4) {
        print_usage(argc, argv);
        return -1;
    }

    opt_is_channel = argv[1];
    opt_channel = argv[2];
    opt_channel_param_first = argv[3];
    if (argc > 4) {
        opt_channel_param_second = strtoul(argv[4], NULL, 10);
    }
    if (argc > 5) {
        output_file = argv[5];
    } else {
        time(&now);
        struct tm *tm_info = localtime(&now);
        strftime(filename, sizeof(filename), "../../lidar_data_%Y%m%d_%H%M%S.csv", tm_info);
        output_file = filename;
    }

    if(strcmp(opt_is_channel, "--channel")==0) {
        if(strcmp(opt_channel, "-s")==0||strcmp(opt_channel, "--serial")==0) {
            useArgcBaudrate = true;
        }
        else if(strcmp(opt_channel, "-u")==0||strcmp(opt_channel, "--udp")==0) {
            opt_channel_type = CHANNEL_TYPE_UDP;
        }
        else {
            print_usage(argc, argv);
            return -1;
        }
    }
    else {
        print_usage(argc, argv);
        return -1;
    }

    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
        if (!opt_channel_param_first) {
#ifdef _WIN32
            opt_channel_param_first = "\\\\.\\com3";
#elif __APPLE__
            opt_channel_param_first = "/dev/tty.SLAB_USBtoUART";
#else
            opt_channel_param_first = "/dev/ttyUSB0";
#endif
        }
    }

    // Initialize OpenGL
    initOpenGL();

    // create the driver instance
    drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        cleanupOpenGL();
        exit(-2);
    }

    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
        if(useArgcBaudrate) {
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK((drv)->connect(_channel))) {
                op_result = drv->getDeviceInfo(devinfo);
                if (SL_IS_OK(op_result)) {
                    connectSuccess = true;
                }
                else {
                    delete drv;
                    drv = NULL;
                }
            }
        }
        else {
            size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
            for(size_t i = 0; i < baudRateArraySize; ++i) {
                _channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK((drv)->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo);
                    if (SL_IS_OK(op_result)) {
                        connectSuccess = true;
                        break;
                    }
                    else {
                        delete drv;
                        drv = NULL;
                    }
                }
            }
        }
    }
    else if(opt_channel_type == CHANNEL_TYPE_UDP) {
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK((drv)->connect(_channel))) {
            op_result = drv->getDeviceInfo(devinfo);
            if (SL_IS_OK(op_result)) {
                connectSuccess = true;
            }
            else {
                delete drv;
                drv = NULL;
            }
        }
    }

    if (!connectSuccess) {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
            (fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_channel_param_first)):(fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n"
                , opt_channel_param_first));
        cleanupOpenGL();
        goto on_finished;
    }

    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    if (!checkSLAMTECLIDARHealth(drv)) {
        cleanupOpenGL();
        goto on_finished;
    }

    signal(SIGINT, ctrlc);

    drv->setMotorSpeed();
    drv->startScan(0,1);

    printf("Successfully started scan. Saving data to %s\n", output_file);
    outFile.open(output_file);
    if (!outFile.is_open()) {
        fprintf(stderr, "Error, cannot open output file %s.\n", output_file);
        cleanupOpenGL();
        goto on_finished;
    }

    outFile << "timestamp,angle,distance,quality,scan_number\n";

    while (!shouldClose && !glfwWindowShouldClose(window)) {
        count = _countof(nodes);
        op_result = drv->grabScanDataHq(nodes, count, 0);
        
        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            scan_count++;
            
            // Clear point data for new scan
            pointData.clear();
            
            for (int pos = 0; pos < (int)count ; ++pos) {
                float current_angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                float current_distance = nodes[pos].dist_mm_q2/4.0f;
                
                if (current_distance > 0 && !skip_next) {
                    int degree = (int)current_angle;
                    if (degree >= 0 && degree < BARCOUNT) {
                        if (points_per_degree[degree] < MAX_POINTS_PER_DEGREE) {
                            time(&current_time);
                            outFile << current_time << "," 
                                   << current_angle << ","
                                   << current_distance << ","
                                   << (int)nodes[pos].quality << ","
                                   << scan_count << "\n";
                            
                            // Convert polar coordinates to Cartesian for OpenGL
                            float rad = current_angle * 3.14159f / 180.0f;
                            pointData.push_back(current_distance * cos(rad));
                            pointData.push_back(current_distance * sin(rad));
                            
                            points_per_degree[degree]++;
                        }
                    }
                }
                skip_next = !skip_next;
                
                if (current_angle < last_angle) {
                    memset(points_per_degree, 0, sizeof(points_per_degree));
                }
                last_angle = current_angle;
            }
            
            // Render the frame
            renderFrame();
            
            printf("Scan #%d - Collected %d data points\n", scan_count, (int)count);
        }
        
        delay(10);  // Small delay to prevent CPU overload
    }

    drv->stop();
    outFile.close();
    printf("Scan stopped. Data saved to %s\n", output_file);

on_finished:
    cleanupOpenGL();
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
} 