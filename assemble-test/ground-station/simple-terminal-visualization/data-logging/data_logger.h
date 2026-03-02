#pragma once

#include "../data-receive/data_receive.h"
#include <fstream>
#include <string>

// ── DataLogger ───────────────────────────────────────────────────────────────
//
// Opens a timestamped .tsv file once, writes a header, then appends one
// tab-separated row per received packet.  Flushes periodically (every N
// lines) to balance data safety with disk friendliness.
//
// Usage:
//   DataLogger logger;
//   logger.open();                   // creates HAB_YYYYMMDD_HHMMSS.tsv
//   logger.log(data);               // call after each successful receive
//   logger.close();                  // call on shutdown (or destructor)
//
// Pandas import:
//   df = pd.read_csv("HAB_20250503_143022.tsv", sep="\t")
// ─────────────────────────────────────────────────────────────────────────────

class DataLogger {
public:
    DataLogger();
    ~DataLogger();

    // Open a new timestamped log file.  Returns true on success.
    bool open();

    // Append one row of data.  Returns true on success.
    bool log(const DataSet& data);

    // Flush and close the file.
    void close();

    // Returns the filename that was opened (empty if not open).
    const std::string& filename() const { return m_filename; }

private:
    std::ofstream m_file;
    std::string   m_filename;
    int           m_line_count;       // lines since last flush
    static const int FLUSH_EVERY = 10;  // flush every N lines
};
