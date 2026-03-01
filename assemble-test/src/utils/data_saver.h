#pragma once

#include "data_extractor.h"

// Initialize SD card and open the log file.
// Returns true on success, false on failure.
bool data_saver_init();

// Write one row of DataSet to the log file.
// Call this every loop during ASCENT and DESCENT.
// Returns true on success.
bool data_saver_write(const DataSet& data);

// Force-flush buffered data to SD card.
// Call this periodically (e.g. every 5s) or on state changes.
void data_saver_flush();

// Close the file cleanly (call on landing).
void data_saver_close();
