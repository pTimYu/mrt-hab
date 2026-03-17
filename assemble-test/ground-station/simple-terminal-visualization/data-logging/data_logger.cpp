#include "data_logger.h"
#include <ctime>
#include <iomanip>
#include <sstream>
#include <iostream>

DataLogger::DataLogger()
    : m_line_count(0)
{}

DataLogger::~DataLogger() {
    close();
}

bool DataLogger::open() {
    // Generate filename: HAB_YYYYMMDD_HHMMSS.tsv
    std::time_t now = std::time(nullptr);
    std::tm* lt = std::localtime(&now);

    std::ostringstream oss;
    oss << "HAB_"
        << (lt->tm_year + 1900)
        << std::setfill('0') << std::setw(2) << (lt->tm_mon + 1)
        << std::setfill('0') << std::setw(2) << lt->tm_mday
        << "_"
        << std::setfill('0') << std::setw(2) << lt->tm_hour
        << std::setfill('0') << std::setw(2) << lt->tm_min
        << std::setfill('0') << std::setw(2) << lt->tm_sec
        << ".tsv";

    m_filename = oss.str();
    m_file.open(m_filename, std::ios::out | std::ios::app);

    if (!m_file.is_open()) {
        std::cerr << "DataLogger: failed to open " << m_filename << std::endl;
        return false;
    }

    // Write TSV header
    m_file << "Time\tLatitude\tLongitude\tSpeed_GPS\tHeading_GPS\t"
           << "Altitude\tVoltage\tRSSI\tSNR\t"
           << "Accel_X\tAccel_Y\tAccel_Z"
           << "\n";

    m_line_count = 0;
    return true;
}

bool DataLogger::log(const DataSet& data) {
    if (!m_file.is_open()) return false;

    // Format time as HH:MM:SS string for readability
    int hh = data.time / 10000;
    int mm = (data.time / 100) % 100;
    int ss = data.time % 100;

    char time_str[16];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hh, mm, ss);

    // Write one tab-separated row
    // Using snprintf for precise float formatting
    char line[512];
    snprintf(line, sizeof(line),
        "%s\t%.5f\t%.5f\t%.1f\t%d\t%.1f\t%.2f\t%d\t%d\t%.2f\t%.2f\t%.2f",
        time_str,
        data.latitude,
        data.longitude,
        data.speed_gps,
        data.heading_gps,
        data.altitude,
        data.voltage,
        data.rssi,
        data.snr,
        data.accel_x,
        data.accel_y,
        data.accel_z
    );

    m_file << line << "\n";
    m_line_count++;

    // Periodic flush — every FLUSH_EVERY lines
    if (m_line_count >= FLUSH_EVERY) {
        m_file.flush();
        m_line_count = 0;
    }

    return true;
}

void DataLogger::close() {
    if (m_file.is_open()) {
        m_file.flush();
        m_file.close();
        m_line_count = 0;
    }
}
