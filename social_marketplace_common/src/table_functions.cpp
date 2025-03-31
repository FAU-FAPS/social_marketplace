#include "social_marketplace_common/table_functions.hpp"
#include <unordered_set>

namespace social_marketplace_common {

Table readCSV(const std::string& filename) {
    Table table;
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Could not open file " + filename);
    }

    std::string line;
    bool isHeader = true;

    while (std::getline(file, line)) {
        // Remove inline comments starting with #
        size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }

        // Trim whitespace
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        if (line.empty()) {
            continue;  // Skip empty lines
        }

        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> rowValues;

        while (std::getline(ss, cell, ',')) {
            // Trim whitespace from each cell
            cell.erase(0, cell.find_first_not_of(" \t"));
            cell.erase(cell.find_last_not_of(" \t") + 1);
            rowValues.push_back(cell);
        }

        if (isHeader) {
            table.headers = rowValues;
            isHeader = false;
        } else {
            std::unordered_map<std::string, std::string> rowMap;
            for (size_t i = 0; i < rowValues.size(); ++i) {
                if (i < table.headers.size()) {
                    rowMap[table.headers[i]] = rowValues[i];
                }
            }
            table.rows.push_back(rowMap);
        }
    }

    file.close();

    if (table.rows.empty()) {
        throw std::runtime_error("CSV file '" + filename + "' is empty or contains no valid data.");
    }

    return table;
}

void resetTableWithNewHeaders(Table& table, const std::vector<std::string>& newHeaders) {
    table.rows.clear();
    table.headers = newHeaders;
}

void resetTableWithNewHeaders(Table& table, const std::vector<std::string>& individualHeaders, const std::vector<std::string>& vectorHeaders) {
    table.rows.clear(); 

    table.headers.clear();
    table.headers.insert(table.headers.end(), individualHeaders.begin(), individualHeaders.end());
    table.headers.insert(table.headers.end(), vectorHeaders.begin(), vectorHeaders.end());
}

void addRowFromVector(Table& table, const std::vector<std::string>& values) {
    if (values.size() != table.headers.size()) {
        throw std::invalid_argument("Number of values does not match number of headers");
    }

    std::unordered_map<std::string, std::string> newRow;
    for (size_t i = 0; i < table.headers.size(); ++i) {
        newRow[table.headers[i]] = values[i];
    }

    table.rows.push_back(newRow);
}

void addRowFromDoubleVector(Table& table, const std::vector<double>& values) {
    if (values.size() != table.headers.size()) {
        throw std::invalid_argument("Number of values does not match number of headers");
    }

    std::unordered_map<std::string, std::string> newRow;
    for (size_t i = 0; i < table.headers.size(); ++i) {
        newRow[table.headers[i]] = std::to_string(values[i]);
    }

    table.rows.push_back(newRow);
}

void addRowFromStringVector(Table& table, const std::vector<std::string>& values) {
    if (values.size() != table.headers.size()) {
        throw std::invalid_argument("Number of values does not match number of headers");
    }

    std::unordered_map<std::string, std::string> newRow;
    for (size_t i = 0; i < table.headers.size(); ++i) {
        newRow[table.headers[i]] = values[i];
    }

    table.rows.push_back(newRow);
}

void addRowFromMixedStringDoubleVectors(Table& table, const std::vector<std::string>& stringValues, const std::vector<double>& doubleValues) {
    if (stringValues.size() + doubleValues.size() != table.headers.size()) {
        throw std::invalid_argument("Number of values does not match number of headers");
    }

    std::unordered_map<std::string, std::string> newRow;
    size_t i = 0;
    for (; i < stringValues.size(); ++i) {
        newRow[table.headers[i]] = stringValues[i];
    }
    for (size_t j = 0; j < doubleValues.size(); ++j, ++i) {
        newRow[table.headers[i]] = std::to_string(doubleValues[j]);
    }

    table.rows.push_back(newRow);
}

void printTable(const Table& table, int rowIndex) {
    // Print headers
    for (const auto& header : table.headers) {
        std::cout << header << "\t";
    }
    std::cout << std::endl;

    // If a specific row index is provided, only print that row
    if (rowIndex >= 0 && rowIndex < static_cast<int>(table.rows.size())) {
        const auto& row = table.rows[rowIndex];
        for (const auto& header : table.headers) {
            auto it = row.find(header);
            if (it != row.end()) {
                std::cout << it->second << "\t";
            } else {
                std::cout << "N/A\t"; // Fallback for missing values
            }
        }
        std::cout << std::endl;
    } 
    // Otherwise, print the entire table
    else {
        for (const auto& row : table.rows) {
            for (const auto& header : table.headers) {
                auto it = row.find(header);
                if (it != row.end()) {
                    std::cout << it->second << "\t";
                } else {
                    std::cout << "N/A\t"; // Fallback for missing values
                }
            }
            std::cout << std::endl;
        }
    }
}

std::vector<std::string> getHeadersAsStringArray(const Table& table) {
    return table.headers;
}

// Retrieve a specific row as an array of doubles
std::vector<double> getRowAsDoubleArray(const Table& table, size_t rowIndex) {
    if (rowIndex >= table.rows.size()) {
        throw std::out_of_range("Row index out of range");
    }

    const auto& row = table.rows[rowIndex];
    std::vector<double> doubleArray;

    for (const auto& header : table.headers) {
        auto it = row.find(header);
        if (it != row.end()) {
            try {
                double value = std::stod(it->second); // Attempt to convert the value to double
                doubleArray.push_back(value);
            } catch (const std::invalid_argument&) {
                doubleArray.push_back(0.0); // Default value for non-convertible values
            } catch (const std::out_of_range&) {
                doubleArray.push_back(0.0); // Default value for numbers outside the valid range
            }
        } else {
            doubleArray.push_back(0.0); // Default value for missing values
        }
    }

    return doubleArray;
}

// Retrieve a specific row as an array of strings
std::vector<std::string> getRowAsStringArray(const Table& table, size_t rowIndex) {
    if (rowIndex >= table.rows.size()) {
        throw std::out_of_range("Row index out of range");
    }

    const auto& row = table.rows[rowIndex];
    std::vector<std::string> result;

    for (const auto& header : table.headers) {
        auto it = row.find(header);
        if (it != row.end()) {
            result.push_back(it->second);
        } else {
            result.push_back("");
        }
    }

    return result;
}

// Get the total row count of the table
int getRowCount(const Table& table) {
    return table.rows.size();
}

// Retrieve row values starting from a specific column index
std::vector<double> getRowValuesFromStartColumn(const Table& table, size_t rowIndex, size_t startColumn) {
    if (rowIndex >= table.rows.size()) {
        throw std::out_of_range("Row index out of range");
    }
    if (startColumn >= table.headers.size()) {
        throw std::out_of_range("Start column index out of range");
    }

    const auto& row = table.rows[rowIndex];
    std::vector<double> result;

    for (size_t colIndex = startColumn; colIndex < table.headers.size(); ++colIndex) {
        const std::string& header = table.headers[colIndex];
        auto it = row.find(header);
        if (it != row.end()) {
            try {
                result.push_back(std::stod(it->second));
            } catch (const std::invalid_argument&) {
                result.push_back(0.0);
            } catch (const std::out_of_range&) {
                result.push_back(0.0);
            }
        } else {
            result.push_back(0.0);
        }
    }

    return result;
}

int getLastRowIndex(const std::unordered_map<std::string, size_t>& rowIndexMap, const std::string& agent_name) {
    auto it = rowIndexMap.find(agent_name);
    return (it != rowIndexMap.end()) ? it->second : 0;  // Default to 0 if not found
}

void updateLastRowIndex(std::unordered_map<std::string, size_t>& rowIndexMap, const std::string& agent_name, int row_index) {
    rowIndexMap[agent_name] = row_index;
}

// Find row by specific value in the entire table
int findRowByValue(const Table& table, const std::string& searchValue) {
    if (table.rows.empty()) {
        return -1;
    }

    for (size_t rowIndex = 0; rowIndex < table.rows.size(); ++rowIndex) {
        const auto& row = table.rows[rowIndex];

        for (const auto& [key, value] : row) {
            if (value.find(searchValue) != std::string::npos) {
                return static_cast<int>(rowIndex);
            }
        }
    }
    return -1;
}

// Find row by specific value in a particular column
int findRowByValueInColumn(const Table& table, const std::string& searchValue, int columnIndex) {
    if (table.headers.empty()) {
        throw std::invalid_argument("Table has no headers");
    }
    if (columnIndex < 0 || static_cast<size_t>(columnIndex) >= table.headers.size()) {
        throw std::out_of_range("Column index out of range");
    }

    const std::string& column = table.headers[columnIndex];

    for (size_t rowIndex = 0; rowIndex < table.rows.size(); ++rowIndex) {
        const auto& row = table.rows[rowIndex];

        auto it = row.find(column);
        if (it != row.end() && it->second == searchValue) {
            return static_cast<int>(rowIndex);
        }
    }

    return -1;
}


Table compareAndTransformTables(const Table& table1, const Table& table2, double factor) {
    if (table1.headers != table2.headers) {
        throw std::invalid_argument("Both tables must have the same headers");
    }
    if (table1.rows.size() != table2.rows.size()) {
        throw std::invalid_argument("Both tables must have the same number of rows");
    }

    Table resultTable;
    resultTable.headers = table1.headers;

    for (size_t rowIndex = 0; rowIndex < table1.rows.size(); ++rowIndex) {
        std::unordered_map<std::string, std::string> newRow;

        for (const auto& header : table1.headers) {
            auto it1 = table1.rows[rowIndex].find(header);
            auto it2 = table2.rows[rowIndex].find(header);

            if (it1 == table1.rows[rowIndex].end() || it2 == table2.rows[rowIndex].end()) {
                throw std::runtime_error("Both tables must have values for all headers");
            }

            try {
                double value1 = std::stod(it1->second);
                double value2 = std::stod(it2->second);

                double difference;
                if (value1 < value2) {
                    difference = (value2 - value1) * factor;
                    value2 -= difference;
                } else {
                    difference = (value1 - value2) * factor;
                    value2 += difference;
                }

                newRow[header] = std::to_string(value2);
            } catch (const std::invalid_argument&) {
                throw std::runtime_error("Invalid number format in table values");
            }
        }

        resultTable.rows.push_back(newRow);
    }

    return resultTable;
}


// Check if all elements of the first column of table1 are contained in table2
bool areFirstColumnElementsContained(const Table& table1, const Table& table2) {
    if (table1.headers.empty() || table2.headers.empty()) {
        throw std::invalid_argument("One or both tables have no headers");
    }

    const std::string& column1 = table1.headers[0];
    const std::string& column2 = table2.headers[0];

    std::unordered_set<std::string> table2ColumnSet;
    for (const auto& row : table2.rows) {
        auto it = row.find(column2);
        if (it != row.end()) {
            table2ColumnSet.insert(it->second);
        }
    }

    for (const auto& row : table1.rows) {
        auto it = row.find(column1);
        if (it != row.end() && table2ColumnSet.find(it->second) == table2ColumnSet.end()) {
            return false;
        }
    }

    return true;
}

}