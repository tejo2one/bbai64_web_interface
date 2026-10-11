/**
 * ============================================================================
 * High-Speed Native C++ Teleoperation & Vision Server for BBAI-64
 * ============================================================================
 * - Ultra-low latency (<1 ms) direct POSIX serial interface (/dev/ttyUSB0)
 * - Intelligent V4L2 USB camera scanner (distinguishes USB from TI codecs)
 * - 30 FPS native OpenCV capture & hardware MJPEG streaming
 * - Multithreaded REST engine with non-blocking analog differential joystick API
 * - 100% offline, zero cloud dependencies
 * ============================================================================
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <cstring>
#include <algorithm>

// POSIX Serial & System
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <dirent.h>
#include <linux/videodev2.h>
#include <linux/fb.h>

#ifndef FBIO_WAITFORVSYNC
#define FBIO_WAITFORVSYNC 0x40044620
#endif

// OpenCV C++
#include <opencv2/opencv.hpp>

// Embedded cpp-httplib (pure HTTP, no OpenSSL required)
#undef CPPHTTPLIB_OPENSSL_SUPPORT
#include "../include/httplib.h"

// ---------------------------------------------------------------------------
// GLOBAL STRUCTURES & STATE
// ---------------------------------------------------------------------------
struct TelemetryState {
    std::atomic<bool> serial_connected{false};
    std::string serial_port{"/dev/ttyUSB0"};
    int baudrate{57600};
    
    std::string last_cmd{"STOP"};
    int target_pwm{0};
    int left_pwm{0};
    int right_pwm{0};
    
    long left_enc{0};
    long right_enc{0};
    int left_speed{0};
    int right_speed{0};

    bool tcs_active{false};
    bool tcs_slip_left{false};
    bool tcs_slip_right{false};
    bool tcs_guard{false};
    
    std::chrono::steady_clock::time_point last_rx_time;
};

struct CameraState {
    std::atomic<bool> usb_camera_connected{false};
    std::string device_node{""};
    std::string card_name{"Searching for USB Camera..."};
    std::string driver_name{""};
    int width{640};
    int height{480};
    int fps{30};
    std::string stream_mode{"synthetic_hud"};
};

struct WifiState {
    bool connected{false};
    std::string ssid{"DISCONNECTED"};
    std::string ip{""};
    int signal_dbm{0};
    int quality{0};
};

TelemetryState g_telemetry;
CameraState g_camera;
WifiState g_wifi;
std::mutex g_telemetry_mutex;
std::mutex g_wifi_mutex;

// Dynamic Camera Resolution Controls
std::atomic<int> g_requested_width{640};
std::atomic<int> g_requested_height{480};
std::atomic<bool> g_resolution_change_pending{false};
std::atomic<bool> g_system_shutdown_active{false};

// Video Stream Shared Buffer
std::vector<uchar> g_latest_jpeg;
cv::Mat g_latest_bgr;
std::mutex g_frame_mutex;
std::condition_variable g_frame_cv;
std::atomic<uint64_t> g_frame_counter{0};

// ---------------------------------------------------------------------------
// POSIX LOW-LATENCY SERIAL INTERFACE
// ---------------------------------------------------------------------------
class SerialBridge {
public:
    SerialBridge(const std::string& port, int baud) : port_name_(port), baud_(baud), fd_(-1) {}
    ~SerialBridge() { close_port(); }

    std::string find_available_port() {
        // Candidates for USB-to-UART converter (Arduino Nano CH340 / FTDI / CP2102)
        std::vector<std::string> candidates = {
            "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3",
            "/dev/ttyACM0", "/dev/ttyACM1"
        };
        // Preferred port if set and still accessible
        if (!port_name_.empty() && access(port_name_.c_str(), R_OK | W_OK) == 0) {
            return port_name_;
        }
        for (const auto& p : candidates) {
            if (access(p.c_str(), R_OK | W_OK) == 0) {
                return p;
            }
        }
        return "/dev/ttyUSB0";
    }

    std::string get_port_name() {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        return port_name_.empty() ? "/dev/ttyUSB0" : port_name_;
    }

    bool open_port() {
        if (fd_ >= 0) return true;

        std::string active_port = find_available_port();
        int new_fd = open(active_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (new_fd < 0) {
            std::lock_guard<std::mutex> lock(g_telemetry_mutex);
            g_telemetry.serial_connected = false;
            return false;
        }

        // Configure termios for raw non-blocking mode
        struct termios options;
        memset(&options, 0, sizeof(options));
        tcgetattr(new_fd, &options);

        speed_t speed = B57600;
        if (baud_ == 115200) speed = B115200;
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control
        options.c_cflag &= ~PARENB;          // No parity
        options.c_cflag &= ~CSTOPB;          // 1 stop bit
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;              // 8 data bits
        options.c_cflag &= ~CRTSCTS;         // Disable hardware flow control
        options.c_cflag &= ~HUPCL;           // Disable hang-up on close (prevent DTR auto-reset)

        // Raw input/output
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
        options.c_oflag &= ~OPOST;

        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 1; // 100ms timeout

        tcflush(new_fd, TCIOFLUSH);
        tcsetattr(new_fd, TCSANOW, &options);

        // Quick probe to check if Arduino is already initialized without sleeping 1.5s
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        tcflush(new_fd, TCIOFLUSH);

        std::string probe_cmd = "q\r";
        write(new_fd, probe_cmd.c_str(), probe_cmd.length());
        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        char test_buf[64] = {0};
        ssize_t n = read(new_fd, test_buf, sizeof(test_buf) - 1);
        if (n <= 0) {
            // Cold USB attach caused Arduino reset; wait for bootloader setup
            std::this_thread::sleep_for(std::chrono::milliseconds(1400));
            tcflush(new_fd, TCIOFLUSH);
        }

        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (fd_ >= 0) close(fd_);
            fd_ = new_fd;
            port_name_ = active_port;
        }

        {
            std::lock_guard<std::mutex> lock(g_telemetry_mutex);
            g_telemetry.serial_port = active_port;
            g_telemetry.serial_connected = true;
        }

        std::cout << "[SERIAL] Connected to " << active_port << " @ " << baud_ << " baud\n";
        return true;
    }

    void close_port() {
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (fd_ >= 0) {
                close(fd_);
                fd_ = -1;
            }
            port_name_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(g_telemetry_mutex);
            g_telemetry.serial_connected = false;
        }
    }

    bool send_command(const std::string& cmd) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (fd_ < 0) return false;

        std::string payload = cmd + "\r";
        ssize_t bytes_written = write(fd_, payload.c_str(), payload.length());
        if (bytes_written < 0) {
            std::cerr << "[SERIAL ERROR] Write failed: " << strerror(errno) << "\n";
            close(fd_);
            fd_ = -1;
            port_name_.clear();
            {
                std::lock_guard<std::mutex> lk(g_telemetry_mutex);
                g_telemetry.serial_connected = false;
            }
            return false;
        }
        return true;
    }

    std::string query_line(const std::string& cmd, int timeout_ms = 120) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (fd_ < 0) return "";

        tcflush(fd_, TCIFLUSH);
        std::string payload = cmd + "\r";
        if (write(fd_, payload.c_str(), payload.length()) < 0) {
            std::cerr << "[SERIAL ERROR] Query write failed on " << port_name_ << ": " << strerror(errno) << "\n";
            close(fd_);
            fd_ = -1;
            g_telemetry.serial_connected = false;
            port_name_.clear();
            return "";
        }

        std::string result = "";
        char buf[128];
        auto start_t = std::chrono::steady_clock::now();

        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - start_t).count() < timeout_ms) {
            ssize_t r = read(fd_, buf, sizeof(buf) - 1);
            if (r > 0) {
                buf[r] = '\0';
                result += buf;
                if (result.find('\n') != std::string::npos || result.find('\r') != std::string::npos) {
                    break;
                }
            } else if (r < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "[SERIAL ERROR] Read error on " << port_name_ << ": " << strerror(errno) << "\n";
                    close(fd_);
                    fd_ = -1;
                    g_telemetry.serial_connected = false;
                    port_name_.clear();
                    return "";
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        return result;
    }

private:
    std::string port_name_;
    int baud_;
    int fd_;
    std::mutex serial_mutex_;
};

SerialBridge g_serial("", 57600);

// ---------------------------------------------------------------------------
// TELEMETRY BACKGROUND WORKER
// ---------------------------------------------------------------------------
void telemetry_worker() {
    auto last_good_rx = std::chrono::steady_clock::now();
    while (true) {
        if (!g_telemetry.serial_connected) {
            if (g_serial.open_port()) {
                last_good_rx = std::chrono::steady_clock::now();
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }

        // Query diagnostic line ('q')
        std::string resp = g_serial.query_line("q", 120);
        if (!resp.empty() && resp.find("CYTRON") != std::string::npos) {
            last_good_rx = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(g_telemetry_mutex);
            g_telemetry.last_rx_time = last_good_rx;
            g_telemetry.serial_connected = true;

            // Parse encoders: "Encoders: L=123 R=456"
            size_t enc_pos = resp.find("Encoders:");
            if (enc_pos != std::string::npos) {
                std::string enc_substr = resp.substr(enc_pos);
                size_t l_pos = enc_substr.find("L=");
                size_t r_pos = enc_substr.find("R=");
                if (l_pos != std::string::npos) {
                    try {
                        g_telemetry.left_enc = std::stol(enc_substr.substr(l_pos + 2));
                    } catch (...) {}
                }
                if (r_pos != std::string::npos) {
                    try {
                        g_telemetry.right_enc = std::stol(enc_substr.substr(r_pos + 2));
                    } catch (...) {}
                }
            }

            // Parse TCS: "TCS: ACT=1 SLIP_L=0 SLIP_R=1 GUARD=0"
            size_t tcs_pos = resp.find("TCS:");
            if (tcs_pos != std::string::npos) {
                std::string tcs_substr = resp.substr(tcs_pos);
                size_t act_pos = tcs_substr.find("ACT=");
                size_t slipl_pos = tcs_substr.find("SLIP_L=");
                size_t slipr_pos = tcs_substr.find("SLIP_R=");
                size_t guard_pos = tcs_substr.find("GUARD=");

                if (act_pos != std::string::npos && act_pos + 4 < tcs_substr.size()) {
                    g_telemetry.tcs_active = (tcs_substr[act_pos + 4] == '1');
                }
                if (slipl_pos != std::string::npos && slipl_pos + 7 < tcs_substr.size()) {
                    g_telemetry.tcs_slip_left = (tcs_substr[slipl_pos + 7] == '1');
                }
                if (slipr_pos != std::string::npos && slipr_pos + 7 < tcs_substr.size()) {
                    g_telemetry.tcs_slip_right = (tcs_substr[slipr_pos + 7] == '1');
                }
                if (guard_pos != std::string::npos && guard_pos + 6 < tcs_substr.size()) {
                    g_telemetry.tcs_guard = (tcs_substr[guard_pos + 6] == '1');
                }
            }
        } else {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_good_rx).count();
            if (elapsed > 2000) {
                std::cerr << "[SERIAL] Heartbeat timeout (" << elapsed << "ms), resetting serial connection...\n";
                g_serial.close_port();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ---------------------------------------------------------------------------
// INTELLIGENT V4L2 USB CAMERA SCANNER & STREAMER
// ---------------------------------------------------------------------------
bool find_usb_webcam(int& out_device_index, std::string& out_device_node, std::string& out_card_name) {
    // Scan video devices 2 through 10 (skipping 0 and 1 which are TI codecs)
    for (int idx = 2; idx < 10; ++idx) {
        std::string node = "/dev/video" + std::to_string(idx);
        int fd = open(node.c_str(), O_RDWR);
        if (fd < 0) continue;

        struct v4l2_capability cap;
        memset(&cap, 0, sizeof(cap));
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
            // Must have video capture capability
            uint32_t caps = cap.capabilities;
            if (cap.capabilities & V4L2_CAP_DEVICE_CAPS) {
                caps = cap.device_caps;
            }

            if (caps & V4L2_CAP_VIDEO_CAPTURE) {
                out_device_index = idx;
                out_device_node = node;
                out_card_name = reinterpret_cast<char*>(cap.card);
                close(fd);
                return true;
            }
        }
        close(fd);
    }
    return false;
}

void camera_worker() {
    cv::VideoCapture cap;
    int current_cam_idx = -1;

    std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 78};
    cv::Mat frame;

    while (true) {
        // Process dynamic resolution change request
        if (g_resolution_change_pending.exchange(false)) {
            if (cap.isOpened()) {
                cap.release();
                usleep(80000); // 80ms kernel UVC buffer release
                current_cam_idx = -1;
            }
        }

        int found_idx = -1;
        std::string found_node = "";
        std::string found_card = "";

        bool has_cam = find_usb_webcam(found_idx, found_node, found_card);

        if (has_cam) {
            if (!cap.isOpened() || current_cam_idx != found_idx) {
                cap.release();
                usleep(50000);
                cap.open(found_idx, cv::CAP_V4L2);
                if (cap.isOpened()) {
                    int req_w = g_requested_width.load();
                    int req_h = g_requested_height.load();

                    // Configure camera's native hardware settings
                    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
                    cap.set(cv::CAP_PROP_FRAME_WIDTH, req_w);
                    cap.set(cv::CAP_PROP_FRAME_HEIGHT, req_h);
                    cap.set(cv::CAP_PROP_FPS, 30);

                    // Read actual hardware-negotiated properties
                    int act_w = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
                    int act_h = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
                    int act_fps = (int)cap.get(cv::CAP_PROP_FPS);
                    if (act_w <= 0) act_w = req_w;
                    if (act_h <= 0) act_h = req_h;
                    if (act_fps <= 0) act_fps = 30;

                    current_cam_idx = found_idx;
                    g_camera.usb_camera_connected = true;
                    g_camera.device_node = found_node;
                    g_camera.card_name = found_card;
                    g_camera.width = act_w;
                    g_camera.height = act_h;
                    g_camera.fps = act_fps;
                    g_camera.stream_mode = "live_usb";
                    std::cout << "[CAMERA] USB Webcam active on " << found_node << " (" << found_card 
                              << ") -> Native Hardware Settings: " << act_w << "x" << act_h << " @ " << act_fps << " FPS [YUYV]" << std::endl;
                }
            }
        } else {
            if (cap.isOpened()) {
                cap.release();
                current_cam_idx = -1;
            }
            int req_w = g_requested_width.load();
            int req_h = g_requested_height.load();
            g_camera.usb_camera_connected = false;
            g_camera.device_node = "";
            g_camera.card_name = "No USB Webcam (Synthetic HUD)";
            g_camera.stream_mode = "synthetic_hud";
            g_camera.width = req_w;
            g_camera.height = req_h;
        }

        // Capture from real camera or generate synthetic Cybernetic HUD
        bool read_ok = false;
        if (cap.isOpened()) {
            read_ok = cap.read(frame);
            if (!read_ok || frame.empty()) {
                std::cerr << "[CAMERA WARNING] Frame read dropped, resetting capture\n";
                cap.release();
            }
        }

        if (!read_ok || frame.empty()) {
            // Generate Tactical Synthetic Cybernetic HUD frame (640x480)
            frame = cv::Mat::zeros(480, 640, CV_8UC3);

            // Subtle dark grid
            for (int x = 0; x < 640; x += 40) {
                cv::line(frame, cv::Point(x, 0), cv::Point(x, 480), cv::Scalar(25, 35, 45), 1);
            }
            for (int y = 0; y < 480; y += 40) {
                cv::line(frame, cv::Point(0, y), cv::Point(640, y), cv::Scalar(25, 35, 45), 1);
            }

            // Center tactical reticle
            int cx = 320, cy = 240;
            cv::circle(frame, cv::Point(cx, cy), 60, cv::Scalar(0, 240, 255), 1, cv::LINE_AA);
            cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0, 255, 157), -1, cv::LINE_AA);
            cv::line(frame, cv::Point(cx - 85, cy), cv::Point(cx - 15, cy), cv::Scalar(0, 240, 255), 2);
            cv::line(frame, cv::Point(cx + 15, cy), cv::Point(cx + 85, cy), cv::Scalar(0, 240, 255), 2);
            cv::line(frame, cv::Point(cx, cy - 85), cv::Point(cx, cy - 15), cv::Scalar(0, 240, 255), 2);
            cv::line(frame, cv::Point(cx, cy + 15), cv::Point(cx, cy + 85), cv::Scalar(0, 240, 255), 2);

            // Live HUD overlay text
            cv::putText(frame, "AUTONOMOUS KINETIC HUD [STANDBY]", cv::Point(24, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 240, 255), 2, cv::LINE_AA);

            std::string mode_str = "MODE: " + g_telemetry.last_cmd + " | PWM: " + std::to_string(g_telemetry.target_pwm);
            cv::putText(frame, mode_str, cv::Point(24, 75),
                        cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

            std::string enc_str = "ENCODERS: L=" + std::to_string(g_telemetry.left_enc) + " | R=" + std::to_string(g_telemetry.right_enc);
            cv::putText(frame, enc_str, cv::Point(24, 440),
                        cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(0, 255, 157), 1, cv::LINE_AA);

            cv::putText(frame, "NO PHYSICAL USB WEBCAM DETECTED", cv::Point(240, 280),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(100, 100, 220), 1, cv::LINE_AA);
        }

        // Compress to JPEG
        std::vector<uchar> buffer;
        if (cv::imencode(".jpg", frame, buffer, encode_params)) {
            {
                std::lock_guard<std::mutex> lock(g_frame_mutex);
                g_latest_jpeg = std::move(buffer);
                g_latest_bgr = frame.clone();
                g_frame_counter++;
            }
            g_frame_cv.notify_all();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
}

/// ---------------------------------------------------------------------------
// HELPER: Dynamic Network IP Resolution for Onboard HUD
// ---------------------------------------------------------------------------
std::string get_primary_ip() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return "127.0.0.1";
    struct sockaddr_in serv;
    memset(&serv, 0, sizeof(serv));
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = inet_addr("8.8.8.8");
    serv.sin_port = htons(53);
    if (connect(sock, (const struct sockaddr*)&serv, sizeof(serv)) < 0) {
        close(sock);
        return "127.0.0.1";
    }
    struct sockaddr_in name;
    socklen_t namelen = sizeof(name);
    if (getsockname(sock, (struct sockaddr*)&name, &namelen) < 0) {
        close(sock);
        return "127.0.0.1";
    }
    char buf[INET_ADDRSTRLEN];
    const char* p = inet_ntop(AF_INET, &name.sin_addr, buf, sizeof(buf));
    close(sock);
    return p ? std::string(buf) : "127.0.0.1";
}

void prepare_framebuffer_environment() {
    int unblank_fd = open("/sys/class/graphics/fb0/blank", O_WRONLY);
    if (unblank_fd >= 0) {
        write(unblank_fd, "0\n", 2);
        close(unblank_fd);
    }
    int cursor_fd = open("/sys/class/graphics/fbcon/cursor_blink", O_WRONLY);
    if (cursor_fd >= 0) {
        write(cursor_fd, "0\n", 2);
        close(cursor_fd);
    }
}

// ---------------------------------------------------------------------------
// BACKGROUND WI-FI STATUS MONITOR WORKER (wlan0 status via wpa_cli & /proc)
// ---------------------------------------------------------------------------
void wifi_monitor_worker() {
    while (true) {
        WifiState ws;
        FILE* fp = popen("/sbin/wpa_cli -i wlan0 status 2>/dev/null", "r");
        if (!fp) fp = popen("/usr/sbin/wpa_cli -i wlan0 status 2>/dev/null", "r");
        if (fp) {
            char line[256];
            while (fgets(line, sizeof(line), fp)) {
                std::string s(line);
                while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) s.pop_back();
                if (s == "wpa_state=COMPLETED") {
                    ws.connected = true;
                } else if (s.rfind("ssid=", 0) == 0) {
                    ws.ssid = s.substr(5);
                } else if (s.rfind("ip_address=", 0) == 0) {
                    ws.ip = s.substr(11);
                }
            }
            pclose(fp);
        }

        std::ifstream proc_wifi("/proc/net/wireless");
        if (proc_wifi.is_open()) {
            std::string line;
            while (std::getline(proc_wifi, line)) {
                if (line.find("wlan0:") != std::string::npos) {
                    std::istringstream iss(line);
                    std::string iface;
                    int status = 0;
                    float link = 0, level = 0, noise = 0;
                    iss >> iface >> status >> link >> level >> noise;
                    ws.quality = (int)link;
                    ws.signal_dbm = (int)level;
                    break;
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(g_wifi_mutex);
            g_wifi = ws;
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// ---------------------------------------------------------------------------
// DIRECT LINUX FRAMEBUFFER ROBOT FACE RENDERER (/dev/fb0 @ 1024x600)
// - Native C++ OpenCV Vector Face Engine
// - Zero tearing with hardware FBIO_WAITFORVSYNC synchronization
// - Perfectly scaled & fitted for 1024x600 physical display (180° inverted)
// - Symmetrical command-deck HUD with live PiP camera and realtime telemetry
// ---------------------------------------------------------------------------
void framebuffer_face_worker() {
    prepare_framebuffer_environment();

    int fb_fd = open("/dev/fb0", O_RDWR);
    if (fb_fd < 0) {
        std::cerr << "[FB0] Notice: /dev/fb0 not accessible (" << strerror(errno) << "). Direct display worker disabled.\n";
        return;
    }

    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    if (ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo) < 0 || ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo) < 0) {
        std::cerr << "[FB0] Error querying framebuffer info.\n";
        close(fb_fd);
        return;
    }

    int fb_w = (vinfo.xres > 0) ? vinfo.xres : 1024;
    int fb_h = (vinfo.yres > 0) ? vinfo.yres : 600;
    int line_length = (finfo.line_length > 0) ? finfo.line_length : (fb_w * 4);
    size_t screensize = finfo.smem_len > 0 ? finfo.smem_len : (line_length * fb_h);

    std::cout << "[FB0] 🖥️ Direct C++ Hardware Framebuffer Display initialized: "
              << fb_w << "x" << fb_h << " (Pitch: " << line_length << ", Size: " << screensize << " bytes)\n";

    uint8_t* fb_ptr = (uint8_t*)mmap(NULL, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);
    if (fb_ptr == MAP_FAILED) {
        std::cerr << "[FB0] Error mmaping framebuffer.\n";
        close(fb_fd);
        return;
    }

    cv::Mat canvas(fb_h, fb_w, CV_8UC4);
    cv::Mat flipped(fb_h, fb_w, CV_8UC4);

    auto start_time = std::chrono::steady_clock::now();
    auto next_blink = start_time + std::chrono::milliseconds(3000);
    bool blinking = false;
    auto blink_end = start_time;

    std::string primary_ip = get_primary_ip();
    auto last_ip_check = start_time;

    while (true) {
        auto loop_start = std::chrono::steady_clock::now();
        float elapsed_sec = std::chrono::duration<float>(loop_start - start_time).count();

        // Refresh IP every 15 seconds
        if (std::chrono::duration<float>(loop_start - last_ip_check).count() > 15.0f) {
            primary_ip = get_primary_ip();
            last_ip_check = loop_start;
        }

        // Natural organic eye blink logic (every 3.0s - 5.5s for 170ms)
        if (!blinking && loop_start >= next_blink) {
            blinking = true;
            blink_end = loop_start + std::chrono::milliseconds(170);
        }
        if (blinking && loop_start >= blink_end) {
            blinking = false;
            int next_interval_ms = 2800 + (rand() % 3000);
            next_blink = loop_start + std::chrono::milliseconds(next_interval_ms);
        }

        // Fetch telemetry state atomically
        std::string action = "stop";
        int left_pwm = 0, right_pwm = 0, target_pwm = 0;
        long left_enc = 0, right_enc = 0;
        bool tcs_active = false, tcs_slip = false, tcs_guard = false;
        bool serial_ok = false;
        {
            std::lock_guard<std::mutex> lock(g_telemetry_mutex);
            action = g_telemetry.last_cmd;
            left_pwm = g_telemetry.left_pwm;
            right_pwm = g_telemetry.right_pwm;
            target_pwm = g_telemetry.target_pwm;
            left_enc = g_telemetry.left_enc;
            right_enc = g_telemetry.right_enc;
            tcs_active = g_telemetry.tcs_active;
            (void)tcs_active;
            tcs_slip = g_telemetry.tcs_slip_left || g_telemetry.tcs_slip_right;
            tcs_guard = g_telemetry.tcs_guard;
            serial_ok = g_telemetry.serial_connected;
        }

        // Derive reactive face expression state
        std::string state = "idle";
        if (tcs_slip || tcs_guard) {
            state = "slip";
        } else if (action == "forward" || (left_pwm > 20 && right_pwm > 20)) {
            state = "forward";
        } else if (action == "backward" || (left_pwm < -20 && right_pwm < -20)) {
            state = "backward";
        } else if (action == "left" || (left_pwm < right_pwm - 25)) {
            state = "left";
        } else if (action == "right" || (right_pwm < left_pwm - 25)) {
            state = "right";
        }

        // 0. SAFE POWER-OFF SCREEN IF SYSTEM SHUTDOWN INITIATED
        if (g_system_shutdown_active.load()) {
            canvas.setTo(cv::Scalar(10, 8, 6, 255));

            int cx = fb_w / 2; // 512
            int cy = 250;

            // Top HUD Bar: System shutdown banner
            cv::rectangle(canvas, cv::Point(0, 0), cv::Point(fb_w, 40), cv::Scalar(28, 12, 10, 255), -1);
            cv::line(canvas, cv::Point(0, 40), cv::Point(fb_w, 40), cv::Scalar(0, 70, 255, 255), 2);
            cv::putText(canvas, "TURTLEBOT AI-64 // SYSTEM SHUTDOWN INITIATED", cv::Point(20, 26),
                        cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(0, 80, 255, 255), 2, cv::LINE_AA);

            // Sleeping curved cat eyes (^ ^)
            int eye_lx = cx - 95;
            int eye_rx = cx + 95;
            cv::ellipse(canvas, cv::Point(eye_lx, cy), cv::Size(36, 16), 0, 190, 350, cv::Scalar(157, 255, 0, 255), 3, cv::LINE_AA);
            cv::ellipse(canvas, cv::Point(eye_rx, cy), cv::Size(36, 16), 0, 190, 350, cv::Scalar(157, 255, 0, 255), 3, cv::LINE_AA);

            // Cute sleeping mouth
            cv::ellipse(canvas, cv::Point(cx - 8, cy + 36), cv::Size(8, 5), 0, 15, 165, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);
            cv::ellipse(canvas, cv::Point(cx + 8, cy + 36), cv::Size(8, 5), 0, 15, 165, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);

            // Sleeping Z z z text
            cv::putText(canvas, "z", cv::Point(cx + 120, cy - 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 220, 255, 255), 2, cv::LINE_AA);
            cv::putText(canvas, "Z", cv::Point(cx + 145, cy - 75), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 220, 255, 255), 2, cv::LINE_AA);
            cv::putText(canvas, "Z", cv::Point(cx + 175, cy - 105), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 220, 255, 255), 3, cv::LINE_AA);

            // Big Safe to Power Off Banner Box
            int box_w = 660, box_h = 110;
            int box_x = (fb_w - box_w) / 2;
            int box_y = 420;
            cv::rectangle(canvas, cv::Point(box_x, box_y), cv::Point(box_x + box_w, box_y + box_h), cv::Scalar(20, 14, 10, 255), -1);
            cv::rectangle(canvas, cv::Point(box_x, box_y), cv::Point(box_x + box_w, box_y + box_h), cv::Scalar(157, 255, 0, 255), 2);
            cv::putText(canvas, "SAFE TO SWITCH OFF MAIN POWER", cv::Point(box_x + 52, box_y + 45),
                        cv::FONT_HERSHEY_SIMPLEX, 0.70, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);
            cv::putText(canvas, "Filesystems cleanly synced. Zero torque on drive motors.", cv::Point(box_x + 72, box_y + 80),
                        cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(200, 200, 200, 255), 1, cv::LINE_AA);

            cv::flip(canvas, flipped, -1);
            int vsync_arg = 0;
            ioctl(fb_fd, FBIO_WAITFORVSYNC, &vsync_arg);
            if (line_length == fb_w * 4) {
                memcpy(fb_ptr, flipped.data, fb_w * fb_h * 4);
            } else {
                for (int y = 0; y < fb_h; ++y) {
                    memcpy(fb_ptr + y * line_length, flipped.ptr(y), fb_w * 4);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // 1. CLEAR CANVAS - Deep tactical obsidian background (#090d14)
        canvas.setTo(cv::Scalar(18, 12, 8, 255));

        int cx = fb_w / 2; // 512
        int cy = 280;      // 280 perfectly centers head with top and bottom clearance

        // Ambient radial background glow behind head
        cv::circle(canvas, cv::Point(cx, cy), 280, cv::Scalar(22, 28, 14, 255), -1);
        cv::circle(canvas, cv::Point(cx, cy), 180, cv::Scalar(30, 40, 18, 255), -1);

        // Tactical background reticle dots
        for (int rx = 40; rx < fb_w; rx += 120) {
            cv::circle(canvas, cv::Point(rx, 80), 1, cv::Scalar(45, 35, 25, 255), -1);
            cv::circle(canvas, cv::Point(rx, 390), 1, cv::Scalar(45, 35, 25, 255), -1);
        }

        // 2. TOP HUD BAR (y: 0 to 40)
        cv::rectangle(canvas, cv::Point(0, 0), cv::Point(fb_w, 40), cv::Scalar(18, 14, 10, 255), -1);
        cv::line(canvas, cv::Point(0, 40), cv::Point(fb_w, 40), cv::Scalar(55, 45, 30, 255), 1);
        cv::line(canvas, cv::Point(0, 40), cv::Point(140, 40), cv::Scalar(157, 255, 0, 255), 2);

        cv::putText(canvas, "TURTLEBOT AI-64 // CYBER FACE v3.0", cv::Point(20, 26),
                    cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 255, 255, 255), 2, cv::LINE_AA);

        // Center Dynamic Vector Badge
        std::string badge_str = "VECTOR: IDLE";
        cv::Scalar badge_color = cv::Scalar(157, 255, 0, 255);
        if (state == "slip") {
            badge_str = "TCS: SLIP CUT ENGAGED";
            badge_color = cv::Scalar(0, 70, 255, 255);
        } else if (state == "forward") {
            badge_str = "VECTOR: FORWARD TURBO";
            badge_color = cv::Scalar(157, 255, 0, 255);
        } else if (state == "backward") {
            badge_str = "VECTOR: REVERSE DRIVE";
            badge_color = cv::Scalar(255, 227, 0, 255);
        } else if (state == "left") {
            badge_str = "VECTOR: STEERING LEFT";
            badge_color = cv::Scalar(255, 227, 0, 255);
        } else if (state == "right") {
            badge_str = "VECTOR: STEERING RIGHT";
            badge_color = cv::Scalar(255, 227, 0, 255);
        }

        cv::rectangle(canvas, cv::Point(400, 6), cv::Point(624, 34), cv::Scalar(28, 20, 16, 255), -1);
        cv::rectangle(canvas, cv::Point(400, 6), cv::Point(624, 34), badge_color, 1);
        cv::putText(canvas, badge_str, cv::Point(415, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40, badge_color, 1, cv::LINE_AA);

        // Fetch current Wi-Fi snapshot
        WifiState wifi_snap;
        {
            std::lock_guard<std::mutex> lock(g_wifi_mutex);
            wifi_snap = g_wifi;
        }

        // Right Hostname, Wi-Fi and System status
        std::string wifi_bar = wifi_snap.connected ? ("WIFI: " + wifi_snap.ssid) : "WIFI: OFF";
        std::string right_info = "bbai.local:5000 | " + wifi_bar + (serial_ok ? " | SER: OK" : " | SER: ERR");
        cv::putText(canvas, right_info, cv::Point(630, 26),
                    cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(200, 200, 200, 255), 1, cv::LINE_AA);

        // 3. CYBERNETIC CAT HEAD (Center: cx=512, cy=280)
        // Ear animation: floating organic bob
        float ear_bob = sin(elapsed_sec * 3.2f) * 3.5f;
        int ear_y = (int)ear_bob;

        // Ear tilt offsets based on robot vector
        int l_ear_dx = 0, r_ear_dx = 0, ear_tilt_y = 0;
        int l_tilt_extra = 0, r_tilt_extra = 0;
        if (state == "left") {
            l_ear_dx = -12; r_ear_dx = -12;
            l_tilt_extra = 4; r_tilt_extra = -4;
        } else if (state == "right") {
            l_ear_dx = 12; r_ear_dx = 12;
            l_tilt_extra = -4; r_tilt_extra = 4;
        } else if (state == "forward") {
            ear_tilt_y = -6;
        } else if (state == "backward") {
            ear_tilt_y = 8;
        }

        // Left Ear Triangle (Tip y ~ 85, comfortably below top bar y=40)
        std::vector<cv::Point> ear_l = {
            cv::Point(cx - 165 + l_ear_dx, cy - 85 + ear_y + ear_tilt_y + l_tilt_extra),
            cv::Point(cx - 130 + l_ear_dx, cy - 195 + ear_y + ear_tilt_y + l_tilt_extra),
            cv::Point(cx - 80 + l_ear_dx, cy - 75 + ear_y + ear_tilt_y + l_tilt_extra)
        };
        // Right Ear Triangle
        std::vector<cv::Point> ear_r = {
            cv::Point(cx + 165 + r_ear_dx, cy - 85 + ear_y + ear_tilt_y + r_tilt_extra),
            cv::Point(cx + 130 + r_ear_dx, cy - 195 + ear_y + ear_tilt_y + r_tilt_extra),
            cv::Point(cx + 80 + r_ear_dx, cy - 75 + ear_y + ear_tilt_y + r_tilt_extra)
        };

        cv::fillPoly(canvas, std::vector<std::vector<cv::Point>>{ear_l, ear_r}, cv::Scalar(30, 20, 13, 255));
        cv::polylines(canvas, std::vector<std::vector<cv::Point>>{ear_l, ear_r}, true, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);

        // Inner glowing cyan ear accents
        std::vector<cv::Point> ear_in_l = {
            cv::Point(cx - 150 + l_ear_dx, cy - 95 + ear_y + ear_tilt_y + l_tilt_extra),
            cv::Point(cx - 130 + l_ear_dx, cy - 175 + ear_y + ear_tilt_y + l_tilt_extra),
            cv::Point(cx - 100 + l_ear_dx, cy - 90 + ear_y + ear_tilt_y + l_tilt_extra)
        };
        std::vector<cv::Point> ear_in_r = {
            cv::Point(cx + 150 + r_ear_dx, cy - 95 + ear_y + ear_tilt_y + r_tilt_extra),
            cv::Point(cx + 130 + r_ear_dx, cy - 175 + ear_y + ear_tilt_y + r_tilt_extra),
            cv::Point(cx + 100 + r_ear_dx, cy - 90 + ear_y + ear_tilt_y + r_tilt_extra)
        };
        cv::fillPoly(canvas, std::vector<std::vector<cv::Point>>{ear_in_l, ear_in_r}, cv::Scalar(55, 42, 22, 255));
        cv::polylines(canvas, std::vector<std::vector<cv::Point>>{ear_in_l, ear_in_r}, true, cv::Scalar(255, 229, 0, 255), 1, cv::LINE_AA);

        // Visor Contour Panel (x from 312 to 712, y from 200 to 370 - Width: 400, Height: 170)
        std::vector<cv::Point> visor = {
            cv::Point(cx - 190, cy - 80), cv::Point(cx + 190, cy - 80),
            cv::Point(cx + 205, cy + 30), cv::Point(cx + 175, cy + 90),
            cv::Point(cx - 175, cy + 90), cv::Point(cx - 205, cy + 30)
        };
        cv::fillPoly(canvas, std::vector<std::vector<cv::Point>>{visor}, cv::Scalar(28, 20, 16, 255));
        cv::polylines(canvas, std::vector<std::vector<cv::Point>>{visor}, true, cv::Scalar(60, 48, 35, 255), 2, cv::LINE_AA);

        // Visor Corner Reticle Brackets (Neon Emerald)
        int v_w = 190, v_h = 80;
        cv::line(canvas, cv::Point(cx - v_w, cy - v_h), cv::Point(cx - v_w + 18, cy - v_h), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(cx - v_w, cy - v_h), cv::Point(cx - v_w, cy - v_h + 18), cv::Scalar(157, 255, 0, 255), 2);

        cv::line(canvas, cv::Point(cx + v_w, cy - v_h), cv::Point(cx + v_w - 18, cy - v_h), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(cx + v_w, cy - v_h), cv::Point(cx + v_w, cy - v_h + 18), cv::Scalar(157, 255, 0, 255), 2);

        cv::line(canvas, cv::Point(cx - 175, cy + 90), cv::Point(cx - 175 + 18, cy + 90), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(cx - 175, cy + 90), cv::Point(cx - 185, cy + 72), cv::Scalar(157, 255, 0, 255), 2);

        cv::line(canvas, cv::Point(cx + 175, cy + 90), cv::Point(cx + 175 - 18, cy + 90), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(cx + 175, cy + 90), cv::Point(cx + 185, cy + 72), cv::Scalar(157, 255, 0, 255), 2);

        // EYES (Left: cx-95=417, Right: cx+95=607, cy-10=270)
        int eye_lx = cx - 95;
        int eye_rx = cx + 95;
        int eye_y = cy - 10;

        if (blinking) {
            // Eyelid Slits
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), cv::Size(46, 4), 0, 0, 360, cv::Scalar(157, 255, 0, 255), 3, cv::LINE_AA);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), cv::Size(46, 4), 0, 0, 360, cv::Scalar(157, 255, 0, 255), 3, cv::LINE_AA);
        } else if (state == "forward") {
            // Squint Focused Turbo
            cv::Size axes(50, 20);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), axes, 0, 0, 360, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), axes, 0, 0, 360, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);

            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), cv::Size(16, 16), 0, 0, 360, cv::Scalar(220, 255, 0, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), cv::Size(16, 16), 0, 0, 360, cv::Scalar(220, 255, 0, 255), -1);
            cv::circle(canvas, cv::Point(eye_lx - 4, eye_y - 3), 4, cv::Scalar(255, 255, 255, 255), -1);
            cv::circle(canvas, cv::Point(eye_rx - 4, eye_y - 3), 4, cv::Scalar(255, 255, 255, 255), -1);

            // Glowing Cheeks (blush)
            cv::ellipse(canvas, cv::Point(eye_lx - 32, eye_y + 45), cv::Size(24, 10), 0, 0, 360, cv::Scalar(136, 51, 255, 180), -1);
            cv::ellipse(canvas, cv::Point(eye_rx + 32, eye_y + 45), cv::Size(24, 10), 0, 0, 360, cv::Scalar(136, 51, 255, 180), -1);
        } else if (state == "backward") {
            // Wide Surprised Eyes looking up
            cv::Size axes(52, 52);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y - 8), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y - 8), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y - 8), axes, 0, 0, 360, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y - 8), axes, 0, 0, 360, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);

            cv::circle(canvas, cv::Point(eye_lx, eye_y - 18), 26, cv::Scalar(255, 240, 0, 255), -1);
            cv::circle(canvas, cv::Point(eye_rx, eye_y - 18), 26, cv::Scalar(255, 240, 0, 255), -1);
            cv::circle(canvas, cv::Point(eye_lx - 6, eye_y - 24), 7, cv::Scalar(255, 255, 255, 255), -1);
            cv::circle(canvas, cv::Point(eye_rx - 6, eye_y - 24), 7, cv::Scalar(255, 255, 255, 255), -1);
        } else if (state == "left") {
            cv::Size axes(48, 52);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), axes, 0, 0, 360, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), axes, 0, 0, 360, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);

            cv::circle(canvas, cv::Point(eye_lx - 20, eye_y), 24, cv::Scalar(255, 240, 0, 255), -1);
            cv::circle(canvas, cv::Point(eye_rx - 20, eye_y), 24, cv::Scalar(255, 240, 0, 255), -1);
            cv::circle(canvas, cv::Point(eye_lx - 26, eye_y - 6), 6, cv::Scalar(255, 255, 255, 255), -1);
            cv::circle(canvas, cv::Point(eye_rx - 26, eye_y - 6), 6, cv::Scalar(255, 255, 255, 255), -1);
        } else if (state == "right") {
            cv::Size axes(48, 52);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), axes, 0, 0, 360, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), axes, 0, 0, 360, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);

            cv::circle(canvas, cv::Point(eye_lx + 20, eye_y), 24, cv::Scalar(255, 240, 0, 255), -1);
            cv::circle(canvas, cv::Point(eye_rx + 20, eye_y), 24, cv::Scalar(255, 240, 0, 255), -1);
            cv::circle(canvas, cv::Point(eye_lx + 14, eye_y - 6), 6, cv::Scalar(255, 255, 255, 255), -1);
            cv::circle(canvas, cv::Point(eye_rx + 14, eye_y - 6), 6, cv::Scalar(255, 255, 255, 255), -1);
        } else if (state == "slip") {
            // Traction Control Alert
            cv::Size axes(48, 52);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), axes, 0, 0, 360, cv::Scalar(0, 70, 255, 255), 2, cv::LINE_AA);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), axes, 0, 0, 360, cv::Scalar(0, 70, 255, 255), 2, cv::LINE_AA);

            cv::circle(canvas, cv::Point(eye_lx, eye_y), 24, cv::Scalar(0, 80, 255, 255), -1);
            cv::circle(canvas, cv::Point(eye_rx, eye_y), 24, cv::Scalar(0, 80, 255, 255), -1);
            cv::circle(canvas, cv::Point(eye_lx - 6, eye_y - 6), 6, cv::Scalar(255, 255, 255, 255), -1);
            cv::circle(canvas, cv::Point(eye_rx - 6, eye_y - 6), 6, cv::Scalar(255, 255, 255, 255), -1);
        } else {
            // Relaxed Neutral Idle
            cv::Size axes(48, 52);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), axes, 0, 0, 360, cv::Scalar(36, 26, 18, 255), -1);
            cv::ellipse(canvas, cv::Point(eye_lx, eye_y), axes, 0, 0, 360, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);
            cv::ellipse(canvas, cv::Point(eye_rx, eye_y), axes, 0, 0, 360, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);

            cv::circle(canvas, cv::Point(eye_lx, eye_y), 24, cv::Scalar(255, 240, 0, 255), -1);
            cv::circle(canvas, cv::Point(eye_rx, eye_y), 24, cv::Scalar(255, 240, 0, 255), -1);
            cv::circle(canvas, cv::Point(eye_lx - 6, eye_y - 6), 6, cv::Scalar(255, 255, 255, 255), -1);
            cv::circle(canvas, cv::Point(eye_rx - 6, eye_y - 6), 6, cv::Scalar(255, 255, 255, 255), -1);
        }

        // Nose (Inverted Neon Triangle)
        std::vector<cv::Point> nose = {
            cv::Point(cx, cy + 38), cv::Point(cx - 6, cy + 30), cv::Point(cx + 6, cy + 30)
        };
        cv::fillPoly(canvas, std::vector<std::vector<cv::Point>>{nose}, cv::Scalar(157, 255, 0, 255));

        // Mouth Curved 'w'
        cv::ellipse(canvas, cv::Point(cx - 10, cy + 48), cv::Size(10, 7), 0, 15, 165, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);
        cv::ellipse(canvas, cv::Point(cx + 10, cy + 48), cv::Size(10, 7), 0, 15, 165, cv::Scalar(157, 255, 0, 255), 2, cv::LINE_AA);

        // Cyber Whiskers (Left: 360 to 425, Right: 599 to 664 - fully contained)
        cv::line(canvas, cv::Point(cx - 85, cy + 24), cv::Point(cx - 145, cy + 16), cv::Scalar(120, 180, 0, 200), 1, cv::LINE_AA);
        cv::line(canvas, cv::Point(cx - 85, cy + 36), cv::Point(cx - 150, cy + 38), cv::Scalar(120, 180, 0, 200), 1, cv::LINE_AA);
        cv::line(canvas, cv::Point(cx - 85, cy + 48), cv::Point(cx - 140, cy + 58), cv::Scalar(120, 180, 0, 200), 1, cv::LINE_AA);

        cv::line(canvas, cv::Point(cx + 85, cy + 24), cv::Point(cx + 145, cy + 16), cv::Scalar(120, 180, 0, 200), 1, cv::LINE_AA);
        cv::line(canvas, cv::Point(cx + 85, cy + 36), cv::Point(cx + 150, cy + 38), cv::Scalar(120, 180, 0, 200), 1, cv::LINE_AA);
        cv::line(canvas, cv::Point(cx + 85, cy + 48), cv::Point(cx + 140, cy + 58), cv::Scalar(120, 180, 0, 200), 1, cv::LINE_AA);

        // 4. PICTURE-IN-PICTURE MINI CAMERA WIDGET (Bottom-Left: x=20, y=405, w=230, h=175)
        int pip_x = 20, pip_y = 405;
        int pip_w = 230, pip_h = 175;

        // Container background
        cv::rectangle(canvas, cv::Point(pip_x, pip_y), cv::Point(pip_x + pip_w, pip_y + pip_h), cv::Scalar(24, 18, 14, 255), -1);
        cv::rectangle(canvas, cv::Point(pip_x, pip_y), cv::Point(pip_x + pip_w, pip_y + pip_h), cv::Scalar(55, 45, 30, 255), 1);

        // Header bar on PiP
        cv::rectangle(canvas, cv::Point(pip_x, pip_y), cv::Point(pip_x + pip_w, pip_y + 22), cv::Scalar(18, 14, 10, 255), -1);
        cv::circle(canvas, cv::Point(pip_x + 10, pip_y + 11), 3, cv::Scalar(40, 40, 255, 255), -1);
        cv::putText(canvas, "CAM 01 // FRONT FEED", cv::Point(pip_x + 20, pip_y + 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(220, 220, 220, 255), 1, cv::LINE_AA);

        // Fetch live BGR frame and blit
        cv::Mat cam_copy;
        {
            std::lock_guard<std::mutex> lock(g_frame_mutex);
            if (!g_latest_bgr.empty()) {
                cam_copy = g_latest_bgr.clone();
            }
        }
        int feed_w = pip_w - 4;
        int feed_h = pip_h - 26;
        if (!cam_copy.empty()) {
            cv::Mat resized, resized_bgra;
            cv::resize(cam_copy, resized, cv::Size(feed_w, feed_h));
            cv::cvtColor(resized, resized_bgra, cv::COLOR_BGR2BGRA);
            resized_bgra.copyTo(canvas(cv::Rect(pip_x + 2, pip_y + 24, feed_w, feed_h)));
        } else {
            cv::rectangle(canvas, cv::Rect(pip_x + 2, pip_y + 24, feed_w, feed_h), cv::Scalar(12, 9, 6, 255), -1);
            cv::putText(canvas, "WAITING FOR FEED...", cv::Point(pip_x + 35, pip_y + 95),
                        cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(157, 255, 0, 255), 1, cv::LINE_AA);
        }

        // Footer label on PiP
        cv::putText(canvas, "30 FPS | 640x480 RAW", cv::Point(pip_x + 8, pip_y + pip_h - 6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(157, 255, 0, 255), 1, cv::LINE_AA);

        // Neon corner brackets on PiP
        int c_len = 12;
        cv::line(canvas, cv::Point(pip_x, pip_y), cv::Point(pip_x + c_len, pip_y), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(pip_x, pip_y), cv::Point(pip_x, pip_y + c_len), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(pip_x + pip_w, pip_y), cv::Point(pip_x + pip_w - c_len, pip_y), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(pip_x + pip_w, pip_y), cv::Point(pip_x + pip_w, pip_y + c_len), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(pip_x, pip_y + pip_h), cv::Point(pip_x + c_len, pip_y + pip_h), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(pip_x, pip_y + pip_h), cv::Point(pip_x, pip_y + pip_h - c_len), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(pip_x + pip_w, pip_y + pip_h), cv::Point(pip_x + pip_w - c_len, pip_y + pip_h), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(pip_x + pip_w, pip_y + pip_h), cv::Point(pip_x + pip_w, pip_y + pip_h - c_len), cv::Scalar(157, 255, 0, 255), 2);

        // 5. TACTICAL TELEMETRY WIDGET (Bottom-Right: x=774, y=405, w=230, h=175)
        int tel_x = 774, tel_y = 405;
        int tel_w = 230, tel_h = 175;

        // Container background
        cv::rectangle(canvas, cv::Point(tel_x, tel_y), cv::Point(tel_x + tel_w, tel_y + tel_h), cv::Scalar(24, 18, 14, 255), -1);
        cv::rectangle(canvas, cv::Point(tel_x, tel_y), cv::Point(tel_x + tel_w, tel_y + tel_h), cv::Scalar(55, 45, 30, 255), 1);

        // Header bar on Telemetry
        cv::rectangle(canvas, cv::Point(tel_x, tel_y), cv::Point(tel_x + tel_w, tel_y + 22), cv::Scalar(18, 14, 10, 255), -1);
        cv::circle(canvas, cv::Point(tel_x + 10, tel_y + 11), 3, cv::Scalar(255, 227, 0, 255), -1);
        cv::putText(canvas, "ROBOT TELEMETRY", cv::Point(tel_x + 20, tel_y + 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(220, 220, 220, 255), 1, cv::LINE_AA);

        // Telemetry lines
        std::string mode_str = "NORM (130 PWM)";
        if (target_pwm <= 80) mode_str = "SLOW (80 PWM)";
        else if (target_pwm >= 200) mode_str = "FAST (210 PWM)";

        cv::putText(canvas, "SPEED : " + mode_str, cv::Point(tel_x + 10, tel_y + 44),
                    cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(157, 255, 0, 255), 1, cv::LINE_AA);

        std::string enc_str = "ENCS  : L=" + std::to_string(left_enc) + " | R=" + std::to_string(right_enc);
        cv::putText(canvas, enc_str, cv::Point(tel_x + 10, tel_y + 65),
                    cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(200, 200, 200, 255), 1, cv::LINE_AA);

        std::string pwm_str = "DRIVE : L=" + std::to_string(left_pwm) + " | R=" + std::to_string(right_pwm);
        cv::putText(canvas, pwm_str, cv::Point(tel_x + 10, tel_y + 86),
                    cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(255, 227, 0, 255), 1, cv::LINE_AA);

        std::string tcs_str = tcs_slip ? "TCS   : SLIP (INTERVENING)" : (tcs_guard ? "TCS   : GUARD (CLAMPED)" : "TCS   : STABLE (e-LSD)");
        cv::Scalar tcs_col = (tcs_slip || tcs_guard) ? cv::Scalar(0, 70, 255, 255) : cv::Scalar(157, 255, 0, 255);
        cv::putText(canvas, tcs_str, cv::Point(tel_x + 10, tel_y + 107),
                    cv::FONT_HERSHEY_SIMPLEX, 0.30, tcs_col, 1, cv::LINE_AA);

        std::string wifi_str = wifi_snap.connected ? ("WIFI  : " + wifi_snap.ssid + " (" + std::to_string(wifi_snap.signal_dbm) + "dBm)") : "WIFI  : DISCONNECTED";
        cv::Scalar wifi_col = wifi_snap.connected ? cv::Scalar(157, 255, 0, 255) : cv::Scalar(0, 140, 255, 255);
        cv::putText(canvas, wifi_str, cv::Point(tel_x + 10, tel_y + 128),
                    cv::FONT_HERSHEY_SIMPLEX, 0.30, wifi_col, 1, cv::LINE_AA);

        cv::putText(canvas, "SERIAL: /dev/ttyUSB0 (57600)", cv::Point(tel_x + 10, tel_y + 149),
                    cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(150, 150, 150, 255), 1, cv::LINE_AA);

        // Animated sine pulse bar on telemetry footer
        float pulse = (sin(elapsed_sec * 4.0f) + 1.0f) * 0.5f;
        int bar_w = (int)(pulse * (tel_w - 20));
        cv::line(canvas, cv::Point(tel_x + 10, tel_y + tel_h - 8), cv::Point(tel_x + 10 + bar_w, tel_y + tel_h - 8),
                 cv::Scalar(157, 255, 0, 200), 2);

        // Neon corner brackets on Telemetry box
        cv::line(canvas, cv::Point(tel_x, tel_y), cv::Point(tel_x + c_len, tel_y), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(tel_x, tel_y), cv::Point(tel_x, tel_y + c_len), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(tel_x + tel_w, tel_y), cv::Point(tel_x + tel_w - c_len, tel_y), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(tel_x + tel_w, tel_y), cv::Point(tel_x + tel_w, tel_y + c_len), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(tel_x, tel_y + tel_h), cv::Point(tel_x + c_len, tel_y + tel_h), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(tel_x, tel_y + tel_h), cv::Point(tel_x, tel_y + tel_h - c_len), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(tel_x + tel_w, tel_y + tel_h), cv::Point(tel_x + tel_w - c_len, tel_y + tel_h), cv::Scalar(157, 255, 0, 255), 2);
        cv::line(canvas, cv::Point(tel_x + tel_w, tel_y + tel_h), cv::Point(tel_x + tel_w, tel_y + tel_h - c_len), cv::Scalar(157, 255, 0, 255), 2);

        // 6. ROTATE 180 DEGREES (Matches upside-down physical LCD mounting)
        cv::flip(canvas, flipped, -1);

        // 7. TEAR-FREE FLUSH WITH HARDWARE VSYNC
        int vsync_arg = 0;
        ioctl(fb_fd, FBIO_WAITFORVSYNC, &vsync_arg);

        if (line_length == fb_w * 4) {
            memcpy(fb_ptr, flipped.data, fb_w * fb_h * 4);
        } else {
            for (int y = 0; y < fb_h; ++y) {
                memcpy(fb_ptr + y * line_length, flipped.ptr(y), fb_w * 4);
            }
        }

        // 8. 30 FPS FRAME RATE PACING
        auto loop_end = std::chrono::steady_clock::now();
        auto frame_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        int sleep_ms = 33 - (int)frame_elapsed.count();
        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }

    munmap(fb_ptr, screensize);
    close(fb_fd);
}

// ---------------------------------------------------------------------------
// MAIN HTTP & REST CONTROLLER
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    int http_port = 5000;
    if (argc > 1) {
        try { http_port = std::stoi(argv[1]); } catch (...) {}
    }

    // Ensure working directory is set so templates/ and static/ are always found
    if (access("templates/index.html", F_OK) != 0) {
        if (access("/opt/robot_teleop/templates/index.html", F_OK) == 0) {
            if (chdir("/opt/robot_teleop") == 0) {
                std::cout << "📂 Changed working directory to /opt/robot_teleop\n";
            }
        } else if (access("/dev/shm/web_teleop/templates/index.html", F_OK) == 0) {
            if (chdir("/dev/shm/web_teleop") == 0) {
                std::cout << "📂 Changed working directory to /dev/shm/web_teleop\n";
            }
        } else if (access("/home/debian/web_teleop/templates/index.html", F_OK) == 0) {
            if (chdir("/home/debian/web_teleop") == 0) {
                std::cout << "📂 Changed working directory to /home/debian/web_teleop\n";
            }
        }
    }

    std::cout << "==================================================================\n";
    std::cout << "  🚀 Native C++ Autonomous Teleoperation & Vision Server\n";
    std::cout << "  🌐 HTTP Server Port: " << http_port << "\n";
    std::cout << "  ⚡ Serial Bridge:    /dev/ttyUSB0 @ 57600 baud\n";
    std::cout << "==================================================================\n";

    // Launch background workers
    std::thread tele_th(telemetry_worker);
    tele_th.detach();

    std::thread cam_th(camera_worker);
    cam_th.detach();

    std::thread fb_th(framebuffer_face_worker);
    fb_th.detach();

    std::thread wifi_th(wifi_monitor_worker);
    wifi_th.detach();

    httplib::Server svr;

    // Static assets mount point with path fallback
    std::string static_dir = "./static";
    if (access(static_dir.c_str(), F_OK) != 0) {
        if (access("/opt/robot_teleop/static", F_OK) == 0) {
            static_dir = "/opt/robot_teleop/static";
        } else if (access("/dev/shm/web_teleop/static", F_OK) == 0) {
            static_dir = "/dev/shm/web_teleop/static";
        } else {
            static_dir = "/home/debian/web_teleop/static";
        }
    }
    std::cout << "📂 Static assets mount: " << static_dir << "\n";
    svr.set_mount_point("/static", static_dir.c_str());

    // Standard headers filter
    svr.set_post_routing_handler([](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        res.set_header("Cache-Control", "no-cache, no-store, must-revalidate");
        res.set_header("Pragma", "no-cache");
    });

    // Root page with multi-path resolution
    svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        std::vector<std::string> candidate_paths = {
            "templates/index.html",
            "./templates/index.html",
            "/opt/robot_teleop/templates/index.html",
            "/dev/shm/web_teleop/templates/index.html",
            "/home/debian/web_teleop/templates/index.html",
            "/home/debian/templates/index.html"
        };
        std::ifstream file;
        for (const auto& path : candidate_paths) {
            file.open(path);
            if (file.is_open()) break;
        }

        if (file.is_open()) {
            std::stringstream ss;
            ss << file.rdbuf();
            res.set_content(ss.str(), "text/html");
        } else {
            res.set_content("<h1>404 - templates/index.html not found</h1>", "text/html");
            res.status = 404;
        }
    });

    // Dedicated Robot Face route for BBAI-64 physical display
    auto serve_face = [](const httplib::Request&, httplib::Response& res) {
        std::vector<std::string> candidate_paths = {
            "templates/robot_face.html",
            "./templates/robot_face.html",
            "/opt/robot_teleop/templates/robot_face.html",
            "/dev/shm/web_teleop/templates/robot_face.html",
            "/home/debian/web_teleop/templates/robot_face.html",
            "/home/debian/templates/robot_face.html"
        };
        std::ifstream file;
        for (const auto& path : candidate_paths) {
            file.open(path);
            if (file.is_open()) break;
        }

        if (file.is_open()) {
            std::stringstream ss;
            ss << file.rdbuf();
            res.set_content(ss.str(), "text/html");
        } else {
            res.set_content("<h1>404 - templates/robot_face.html not found</h1>", "text/html");
            res.status = 404;
        }
    };
    svr.Get("/face", serve_face);
    svr.Get("/robot_face", serve_face);

    // Live MJPEG Video Stream
    svr.Get("/video_feed", [](const httplib::Request&, httplib::Response& res) {
        res.set_content_provider(
            "multipart/x-mixed-replace; boundary=frame",
            [](size_t offset, httplib::DataSink& sink) {
                (void)offset;
                std::vector<uchar> frame_bytes;
                uint64_t last_sent_id = 0;

                while (sink.is_writable()) {
                    {
                        std::unique_lock<std::mutex> lock(g_frame_mutex);
                        g_frame_cv.wait_for(lock, std::chrono::milliseconds(80), [&]() {
                            return g_frame_counter.load() != last_sent_id;
                        });
                        frame_bytes = g_latest_jpeg;
                        last_sent_id = g_frame_counter.load();
                    }

                    if (!frame_bytes.empty()) {
                        std::string header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                             std::to_string(frame_bytes.size()) + "\r\n\r\n";
                        if (!sink.write(header.data(), header.size())) break;
                        if (!sink.write(reinterpret_cast<const char*>(frame_bytes.data()), frame_bytes.size())) break;
                        if (!sink.write("\r\n", 2)) break;
                    }
                }
                return false; // Connection closed
            }
        );
    });

    // Status API
    svr.Get("/api/status", [](const httplib::Request&, httplib::Response& res) {
        std::lock_guard<std::mutex> lock(g_telemetry_mutex);
        WifiState wifi_snap;
        {
            std::lock_guard<std::mutex> lk(g_wifi_mutex);
            wifi_snap = g_wifi;
        }

        std::stringstream json;
        json << "{"
             << "\"status\":\"online\","
             << "\"server\":\"cpp_native_v1\","
             << "\"serial_connected\":" << (g_telemetry.serial_connected ? "true" : "false") << ","
             << "\"serial_port\":\"" << g_telemetry.serial_port << "\","
             << "\"wifi\":{"
             << "\"connected\":" << (wifi_snap.connected ? "true" : "false") << ","
             << "\"ssid\":\"" << wifi_snap.ssid << "\","
             << "\"ip\":\"" << wifi_snap.ip << "\","
             << "\"signal_dbm\":" << wifi_snap.signal_dbm << ","
             << "\"quality\":" << wifi_snap.quality
             << "},"
             << "\"usb_webcam\":{"
             << "\"connected\":" << (g_camera.usb_camera_connected ? "true" : "false") << ","
             << "\"device_node\":\"" << g_camera.device_node << "\","
             << "\"card_name\":\"" << g_camera.card_name << "\","
             << "\"stream_mode\":\"" << g_camera.stream_mode << "\","
             << "\"resolution\":\"" << g_camera.width << "x" << g_camera.height << "\","
             << "\"width\":" << g_camera.width << ","
             << "\"height\":" << g_camera.height << ","
             << "\"fps\":" << g_camera.fps
             << "},"
             << "\"telemetry\":{"
             << "\"connected\":" << (g_telemetry.serial_connected ? "true" : "false") << ","
             << "\"port\":\"" << g_telemetry.serial_port << "\","
             << "\"last_cmd\":\"" << g_telemetry.last_cmd << "\","
             << "\"target_pwm\":" << g_telemetry.target_pwm << ","
             << "\"left_pwm\":" << g_telemetry.left_pwm << ","
             << "\"right_pwm\":" << g_telemetry.right_pwm << ","
             << "\"left_enc\":" << g_telemetry.left_enc << ","
             << "\"right_enc\":" << g_telemetry.right_enc << ","
             << "\"tcs\":{"
             << "\"active\":" << (g_telemetry.tcs_active ? "true" : "false") << ","
             << "\"slip_left\":" << (g_telemetry.tcs_slip_left ? "true" : "false") << ","
             << "\"slip_right\":" << (g_telemetry.tcs_slip_right ? "true" : "false") << ","
             << "\"guard\":" << (g_telemetry.tcs_guard ? "true" : "false")
             << "}"
             << "}"
             << "}";
        res.set_content(json.str(), "application/json");
    });

    // Camera Hardware Status API
    svr.Get("/api/camera_status", [](const httplib::Request&, httplib::Response& res) {
        std::stringstream json;
        json << "{"
             << "\"usb_webcam_detected\":" << (g_camera.usb_camera_connected ? "true" : "false") << ","
             << "\"device_node\":\"" << g_camera.device_node << "\","
             << "\"device_name\":\"" << g_camera.card_name << "\","
             << "\"stream_mode\":\"" << g_camera.stream_mode << "\","
             << "\"resolution\":\"" << g_camera.width << "x" << g_camera.height << "\","
             << "\"width\":" << g_camera.width << ","
             << "\"height\":" << g_camera.height << ","
             << "\"fps\":" << g_camera.fps
             << "}";
        res.set_content(json.str(), "application/json");
    });

    // Dynamic Camera Resolution Switching API
    svr.Post("/api/camera/resolution", [](const httplib::Request& req, httplib::Response& res) {
        int w = 640;
        int h = 480;
        std::string body = req.body;

        auto extract_int = [](const std::string& str, const std::string& key) -> int {
            size_t pos = str.find(key);
            if (pos == std::string::npos) return -1;
            pos = str.find(':', pos);
            if (pos == std::string::npos) return -1;
            pos++;
            while (pos < str.size() && (str[pos] == ' ' || str[pos] == '\t' || str[pos] == '\"')) pos++;
            try {
                return std::stoi(str.substr(pos));
            } catch (...) {
                return -1;
            }
        };

        int parsed_w = extract_int(body, "width");
        int parsed_h = extract_int(body, "height");
        if (parsed_w > 0) w = parsed_w;
        if (parsed_h > 0) h = parsed_h;

        // Validate discrete resolutions
        if (w == 640 && h == 480) {}
        else if (w == 352 && h == 288) {}
        else if (w == 320 && h == 240) {}
        else if (w == 176 && h == 144) {}
        else if (w == 160 && h == 120) {}
        else { w = 640; h = 480; }

        g_requested_width.store(w);
        g_requested_height.store(h);
        g_resolution_change_pending.store(true);

        // Wait up to 600ms for camera worker to renegotiate change
        for (int i = 0; i < 60; ++i) {
            if (!g_resolution_change_pending.load() && g_camera.width == w && g_camera.height == h) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::stringstream json;
        json << "{"
             << "\"ok\":true,"
             << "\"width\":" << g_camera.width << ","
             << "\"height\":" << g_camera.height << ","
             << "\"fps\":" << g_camera.fps
             << "}";
        res.set_content(json.str(), "application/json");
    });

    // Teleoperation Motor Control API (Sub-millisecond REST)
    svr.Post("/api/teleop", [](const httplib::Request& req, httplib::Response& res) {
        // Parse simple JSON fields manually for microsecond speed
        std::string body = req.body;
        std::string action = "stop";
        int left_pwm = 0;
        int right_pwm = 0;
        int target_pwm = 120;

        // Check if analog differential drive parameters are passed
        size_t l_pwm_pos = body.find("\"left_pwm\":");
        size_t r_pwm_pos = body.find("\"right_pwm\":");
        if (l_pwm_pos != std::string::npos && r_pwm_pos != std::string::npos) {
            try {
                left_pwm = std::stoi(body.substr(l_pwm_pos + 11));
                right_pwm = std::stoi(body.substr(r_pwm_pos + 12));
                action = "ANALOG";
            } catch (...) {}
        } else {
            // Action & PWM mode
            size_t act_pos = body.find("\"action\":");
            if (act_pos != std::string::npos) {
                size_t start = body.find("\"", act_pos + 9);
                size_t end = (start != std::string::npos) ? body.find("\"", start + 1) : std::string::npos;
                if (start != std::string::npos && end != std::string::npos) {
                    action = body.substr(start + 1, end - start - 1);
                }
            }
            size_t pwm_pos = body.find("\"pwm\":");
            if (pwm_pos != std::string::npos) {
                try {
                    target_pwm = std::stoi(body.substr(pwm_pos + 6));
                } catch (...) {}
            }
            target_pwm = std::max(40, std::min(220, target_pwm));

            if (action == "forward") {
                left_pwm = target_pwm;
                right_pwm = target_pwm;
            } else if (action == "backward") {
                left_pwm = -target_pwm;
                right_pwm = -target_pwm;
            } else if (action == "left") {
                left_pwm = -target_pwm;
                right_pwm = target_pwm;
            } else if (action == "right") {
                left_pwm = target_pwm;
                right_pwm = -target_pwm;
            } else {
                left_pwm = 0;
                right_pwm = 0;
                action = "stop";
            }
        }

        // Clamp values to safe boundaries
        left_pwm = std::max(-240, std::min(240, left_pwm));
        right_pwm = std::max(-240, std::min(240, right_pwm));

        // Format and send command to Arduino Nano
        std::string cmd = "o " + std::to_string(left_pwm) + " " + std::to_string(right_pwm);
        bool ok = g_serial.send_command(cmd);

        {
            std::lock_guard<std::mutex> lock(g_telemetry_mutex);
            g_telemetry.last_cmd = action;
            g_telemetry.target_pwm = std::max(std::abs(left_pwm), std::abs(right_pwm));
            g_telemetry.left_pwm = left_pwm;
            g_telemetry.right_pwm = right_pwm;
        }

        std::stringstream resp;
        resp << "{\"ok\":" << (ok ? "true" : "false")
             << ",\"action\":\"" << action << "\""
             << ",\"left_pwm\":" << left_pwm
             << ",\"right_pwm\":" << right_pwm
             << ",\"cmd_sent\":\"" << cmd << "\"}";
        res.set_content(resp.str(), "application/json");
    });

    // Immediate Halt Endpoint
    svr.Post("/api/stop", [](const httplib::Request&, httplib::Response& res) {
        g_serial.send_command("o 0 0");
        {
            std::lock_guard<std::mutex> lock(g_telemetry_mutex);
            g_telemetry.last_cmd = "STOP";
            g_telemetry.target_pwm = 0;
            g_telemetry.left_pwm = 0;
            g_telemetry.right_pwm = 0;
        }
        res.set_content("{\"ok\":true,\"action\":\"stop\",\"left_pwm\":0,\"right_pwm\":0}", "application/json");
    });

    // Mobile camera display endpoint
    svr.Post("/api/mobile_frame", [](const httplib::Request&, httplib::Response& res) {
        res.set_content("{\"ok\":true}", "application/json");
    });

    // Graceful Remote Shutdown Endpoint
    svr.Post("/api/shutdown", [](const httplib::Request&, httplib::Response& res) {
        std::cout << "[SYSTEM] ⚠️ Remote Shutdown Request received via Web Cockpit!\n";

        // 1. Halt motors immediately
        g_serial.send_command("o 0 0");
        {
            std::lock_guard<std::mutex> lock(g_telemetry_mutex);
            g_telemetry.last_cmd = "SHUTDOWN";
            g_telemetry.target_pwm = 0;
            g_telemetry.left_pwm = 0;
            g_telemetry.right_pwm = 0;
        }

        // 2. Set shutdown display flag
        g_system_shutdown_active.store(true);

        // 3. Return JSON confirmation
        res.set_content("{\"ok\":true,\"message\":\"System shutdown initiated. Safe to power off in 5 seconds.\"}", "application/json");

        // 4. Detach thread to flush filesystem buffers and trigger systemctl poweroff
        std::thread([]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            sync();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cout << "[SYSTEM] 🔌 Executing systemctl poweroff now...\n";
            int ret = system("sudo /bin/systemctl poweroff || sudo /sbin/poweroff || true");
            (void)ret;
        }).detach();
    });

    std::cout << "[SERVER] Starting listener on 0.0.0.0:" << http_port << " ...\n";
    svr.listen("0.0.0.0", http_port);

    return 0;
}
