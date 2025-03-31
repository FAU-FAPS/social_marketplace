#ifndef SOCIAL_MARKETPLACE_COMMON_TABLE_FUNCTIONS_HPP
#define SOCIAL_MARKETPLACE_COMMON_TABLE_FUNCTIONS_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <stdexcept>

namespace social_marketplace_common {

struct Table {
    std::vector<std::string> headers;
    std::vector<std::unordered_map<std::string, std::string>> rows;
};

// Function prototypes for table management
Table readCSV(const std::string& filename);

// Reset table headers with optional combined headers
void resetTableWithNewHeaders(Table& table, const std::vector<std::string>& newHeaders);
void resetTableWithNewHeaders(Table& table, const std::vector<std::string>& individualHeaders, const std::vector<std::string>& vectorHeaders);

// Adding rows
void addRowFromVector(Table& table, const std::vector<std::string>& values);
void addRowFromDoubleVector(Table& table, const std::vector<double>& values);
void addRowFromMixedStringDoubleVectors(Table& table, const std::vector<std::string>& stringValues, const std::vector<double>& doubleValues);
void addRowFromStringVector(Table& table, const std::vector<std::string>& values);
void updateLastRowIndex(std::unordered_map<std::string, size_t>& rowIndexMap, const std::string& agent_name, int row_index);

// Printing
void printTable(const Table& table, int rowIndex = -1);

// Data retrieval
std::vector<std::string> getHeadersAsStringArray(const Table& table);
std::vector<double> getRowAsDoubleArray(const Table& table, size_t rowIndex);
std::vector<std::string> getRowAsStringArray(const Table& table, size_t rowIndex);
int getRowCount(const Table& table);
int getLastRowIndex(const std::unordered_map<std::string, size_t>& rowIndexMap, const std::string& agent_name);
std::vector<double> getRowValuesFromStartColumn(const Table& table, size_t rowIndex, size_t startColumn);

// Search and validation
int findRowByValue(const Table& table, const std::string& searchValue);
int findRowByValueInColumn(const Table& table, const std::string& searchValue, int columnIndex);
bool areFirstColumnElementsContained(const Table& table1, const Table& table2);

// Table Comparison and Transformation
Table compareAndTransformTables(const Table& table1, const Table& table2, double factor);

}

#endif
