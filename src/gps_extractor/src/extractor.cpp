#include <iostream>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <regex>
#include <cstring>
#include <cmath>
#include <sqlite3.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/get_message_type_support_handle.hpp>

#include <novatel_oem7_msgs/msg/bestpos.hpp>

namespace fs = std::filesystem;

struct Coordinate {
  double lat;
  double lon;
};

// Haversine 거리 계산 함수 (미터 단위)
double haversine(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // 지구 반경 (m)
  double phi1 = lat1 * M_PI / 180.0;
  double phi2 = lat2 * M_PI / 180.0;
  double dphi = (lat2 - lat1) * M_PI / 180.0;
  double dlambda = (lon2 - lon1) * M_PI / 180.0;
  double a = std::sin(dphi / 2.0) * std::sin(dphi / 2.0) +
             std::cos(phi1) * std::cos(phi2) *
             std::sin(dlambda / 2.0) * std::sin(dlambda / 2.0);
  double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
  return R * c;
}

// 이동 거리가 5m 이상인 경우의 좌표를 vector에 저장하여 리턴.
std::vector<Coordinate> process_db_file(const std::string & db_path) {
  std::vector<Coordinate> coords;

  sqlite3 *db = nullptr;
  sqlite3_stmt *stmt = nullptr;
  if (sqlite3_open(db_path.c_str(), &db) != SQLITE_OK) {
    std::cerr << "SQLite 데이터베이스 열기 실패(" << db_path << "): " 
              << sqlite3_errmsg(db) << std::endl;
    return coords;
  }

  std::string topic_name = "/sensing/gnss/bestpos";

  // topics 테이블에서 해당 토픽의 ID 조회
  const char* query_topic_id = "SELECT id FROM topics WHERE name=?";
  if (sqlite3_prepare_v2(db, query_topic_id, -1, &stmt, nullptr) != SQLITE_OK) {
    std::cerr << "SQL 준비 실패 (topics): " << sqlite3_errmsg(db) << std::endl;
    sqlite3_close(db);
    return coords;
  }
  sqlite3_bind_text(stmt, 1, topic_name.c_str(), -1, SQLITE_STATIC);

  int topic_id = -1;
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    topic_id = sqlite3_column_int(stmt, 0);
  } else {
    std::cerr << "토픽 '" << topic_name << "'을 찾을 수 없습니다." << std::endl;
    sqlite3_finalize(stmt);
    sqlite3_close(db);
    return coords;
  }
  sqlite3_finalize(stmt);

  // messages 테이블에서 해당 topic_id의 메시지 데이터 조회
  const char* query_messages = "SELECT data FROM messages WHERE topic_id=?";
  if (sqlite3_prepare_v2(db, query_messages, -1, &stmt, nullptr) != SQLITE_OK) {
    std::cerr << "SQL 준비 실패 (messages): " << sqlite3_errmsg(db) << std::endl;
    sqlite3_close(db);
    return coords;
  }
  sqlite3_bind_int(stmt, 1, topic_id);

  // ROS 2 BESTPOS 메시지 역직렬화용 serializer 생성
  rclcpp::Serialization<novatel_oem7_msgs::msg::BESTPOS> serializer;

  // 이전 좌표 저장 (첫 메시지 여부 판별)
  double prev_lat = NAN, prev_lon = NAN;

  // 각 메시지 행 순회
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const void *data = sqlite3_column_blob(stmt, 0);
    int data_size = sqlite3_column_bytes(stmt, 0);

    rclcpp::SerializedMessage serialized_msg;
    serialized_msg.reserve(data_size);
    std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, data, data_size);
    serialized_msg.get_rcl_serialized_message().buffer_length = data_size;

    novatel_oem7_msgs::msg::BESTPOS msg;
    serializer.deserialize_message(&serialized_msg, &msg);

    double latitude = msg.lat;
    double longitude = msg.lon;

    if (std::isnan(prev_lat) || std::isnan(prev_lon)) {
      // 첫 메시지인 경우 바로 저장
      coords.push_back({latitude, longitude});
      prev_lat = latitude;
      prev_lon = longitude;
    } else {
      double distance = haversine(prev_lat, prev_lon, latitude, longitude);
      if (distance >= 5.0) {  
        coords.push_back({latitude, longitude});
        prev_lat = latitude;
        prev_lon = longitude;
      }
    }
  }

  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return coords;
}

// 파일 이름에서 숫자 부분 추출 
int extract_number(const fs::path & path) {
  std::string filename = path.stem().string(); 
  std::regex re(".*_(\\d+)");
  std::smatch match;
  if (std::regex_match(filename, match, re)) {
    return std::stoi(match[1]);
  }
  return -1;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <folder_path>" << std::endl;
        return 1;
    }

    std::string folder_path = argv[1];
    std::string yaml_path = folder_path + "/gps.yaml";

    // 폴더 내 .db3 파일 목록 수집
    std::vector<fs::directory_entry> db_files;
    for (const auto &entry : fs::directory_iterator(folder_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".db3") {
            db_files.push_back(entry);
        }
    }

    // 파일 이름 숫자 기준 오름차순 정렬
    std::sort(db_files.begin(), db_files.end(),
              [](const fs::directory_entry &a, const fs::directory_entry &b) {
                  return extract_number(a.path()) < extract_number(b.path());
              });

    std::ofstream yaml_out(yaml_path);
    if (!yaml_out.is_open()) {
        std::cerr << "YAML 파일을 열 수 없습니다: " << yaml_path << std::endl;
        return 1;
    }

    // 각 파일 처리 및 YAML로 결과 저장
    int file_num = 0;
    yaml_out << std::fixed << std::setprecision(15);
    yaml_out << "traver_path:" << std::endl;
    for (const auto &entry : db_files) {
        std::string file_path = entry.path().string();
        auto coords = process_db_file(file_path);
        yaml_out << "  " << file_num << std::endl;
        for (const auto &coord : coords) {
            yaml_out << "      - lat: " << coord.lat << std::endl;
            yaml_out << "        lon: " << coord.lon << std::endl;
        }
        file_num++;
    }

    yaml_out.close();
    rclcpp::shutdown();
    return 0;
}
