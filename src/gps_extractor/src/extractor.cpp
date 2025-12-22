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
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/get_message_type_support_handle.hpp>

#include "tf2_msgs/msg/tf_message.hpp"

namespace fs = std::filesystem;

struct Coordinate {
  double x;
  double y;
  int32_t ts_sec; // 20251103, timestamp
  uint32_t ts_nsec; // 20251103, timestamp
};

static void utm_to_wgs84(double easting, double northing, int zone, bool northern,
                         double &lat_deg, double &lon_deg)
{
  // WGS84 타원체 상수
  const double a = 6378137.0;                  // 장반경
  const double f = 1.0 / 298.257223563;        // 편평률
  const double b = a * (1 - f);
  const double e2 = (a*a - b*b) / (a*a);       // 1차 이심률^2
  const double ep2 = (a*a - b*b) / (b*b);      // 2차 이심률^2
  const double k0 = 0.9996;

  // 위경도 단위 변환
  auto rad2deg = [](double r){ return r * 180.0 / M_PI; };

  // 중심경도 (zone 중앙 자오선)
  double lon0_deg = -183.0 + 6.0 * zone;
  double lon0 = lon0_deg * M_PI / 180.0;

  // False Easting/Northing 보정
  double x = easting - 500000.0;
  double y = northing;
  if (!northern) y -= 10000000.0;

  // M (자오선호장)
  double M = y / k0;

  // mu (footprint latitude 근사)
  const double e1 = (1.0 - std::sqrt(1.0 - e2)) / (1.0 + std::sqrt(1.0 - e2));
  double mu = M / (a * (1.0 - e2/4.0 - 3.0*e2*e2/64.0 - 5.0*e2*e2*e2/256.0));

  // phi1 (footprint latitude) 급수 전개
  double J1 = (3.0*e1/2.0 - 27.0*std::pow(e1,3)/32.0);
  double J2 = (21.0*e1*e1/16.0 - 55.0*std::pow(e1,4)/32.0);
  double J3 = (151.0*std::pow(e1,3)/96.0);
  double J4 = (1097.0*std::pow(e1,4)/512.0);
  double phi1 = mu + J1*std::sin(2.0*mu) + J2*std::sin(4.0*mu) + J3*std::sin(6.0*mu) + J4*std::sin(8.0*mu);

  // 보조량
  double sin_phi1 = std::sin(phi1);
  double cos_phi1 = std::cos(phi1);
  double tan_phi1 = std::tan(phi1);

  double N1 = a / std::sqrt(1.0 - e2 * sin_phi1 * sin_phi1);
  double R1 = a * (1.0 - e2) / std::pow(1.0 - e2 * sin_phi1 * sin_phi1, 1.5);
  double T1 = tan_phi1 * tan_phi1;
  double C1 = ep2 * cos_phi1 * cos_phi1;
  double D  = x / (N1 * k0);

  // 위도 (rad)
  double lat = phi1
    - (N1 * tan_phi1 / R1) *
      ( D*D/2.0
      - (5.0 + 3.0*T1 + 10.0*C1 - 4.0*C1*C1 - 9.0*ep2) * std::pow(D,4) / 24.0
      + (61.0 + 90.0*T1 + 298.0*C1 + 45.0*T1*T1 - 252.0*ep2 - 3.0*C1*C1) * std::pow(D,6) / 720.0 );

  // 경도 (rad)
  double lon = lon0
    + ( D
      - (1.0 + 2.0*T1 + C1) * std::pow(D,3) / 6.0
      + (5.0 - 2.0*C1 + 28.0*T1 - 3.0*C1*C1 + 8.0*ep2 + 24.0*T1*T1) * std::pow(D,5) / 120.0 ) / cos_phi1;

  lat_deg = rad2deg(lat);
  lon_deg = rad2deg(lon);
}

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

  std::string topic_name = "/tf";

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
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;

  // 각 메시지 행 순회
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const void *data = sqlite3_column_blob(stmt, 0);
    int data_size = sqlite3_column_bytes(stmt, 0);

    rclcpp::SerializedMessage serialized_msg;
    serialized_msg.reserve(data_size);
    std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, data, data_size);
    serialized_msg.get_rcl_serialized_message().buffer_length = data_size;

    tf2_msgs::msg::TFMessage msg;
    serializer.deserialize_message(&serialized_msg, &msg);
    
    for (const auto &tf : msg.transforms)
    {
        if (tf.header.frame_id == "map" && tf.child_frame_id == "base_link") {
            Coordinate c{tf.transform.translation.x, tf.transform.translation.y, tf.header.stamp.sec, tf.header.stamp.nanosec}; // 20251103, timestamp
            coords.push_back(c);
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
  std::regex re("(\\d+)");
  std::sregex_iterator it(filename.begin(), filename.end(), re);
  std::sregex_iterator end;
  int last_num = -1;
  for (; it != end; ++it) {
    last_num = std::stoi((*it)[1].str());
  }
  return last_num;
}
struct FileWithKey {
  fs::directory_entry entry;
  int key;
  std::string name;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <folder_path>" << std::endl;
        return 1;
    }

    std::string folder_path = argv[1];
    std::string yaml_path = folder_path + "/gps.yaml";

    // 폴더 내 .db3 파일 목록 수집
    std::vector<FileWithKey> files;
    for (const auto &entry : fs::directory_iterator(folder_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".db3") {
            int num = extract_number(entry.path());
            int key = (num >= 0) ? num : std::numeric_limits<int>::max();
            files.push_back({entry, key, entry.path().stem().string()});
        }
    }

    // 파일 이름 숫자 기준 오름차순 정렬
    std::sort(files.begin(), files.end(),
              [](const FileWithKey &a, const FileWithKey &b) {
                  if (a.key != b.key) return a.key < b.key;
                  return a.name < b.name;
              });

    std::ofstream yaml_out(yaml_path);
    if (!yaml_out.is_open()) {
        std::cerr << "YAML 파일을 열 수 없습니다: " << yaml_path << std::endl;
        return 1;
    }

    // UTM 기준좌표
    const double std_e = 445600.0; // 128.399072
    const double std_n = 3945100.0; // 35.648235
    const int zone = 52;
    const bool northern = true;

    // 각 파일 처리 및 YAML로 결과 저장
    int file_num = 0;
    yaml_out << std::fixed << std::setprecision(15);
    yaml_out << "traver_path:" << std::endl;
    for (const auto &f : files) {
        std::string file_path = f.entry.path().string();
        auto coords = process_db_file(file_path);
        yaml_out << "  " << file_num << std::endl;
        for (const auto &coord : coords) {
            // UTM 실제좌표
            double easting  = std_e + coord.x;
            double northing = std_n + coord.y;
            // UTM -> WGS84
            double lat=0.0, lon=0.0;
            utm_to_wgs84(easting, northing, zone, northern, lat, lon);
            // 20251103, timestamp
            unsigned long timestamp = static_cast<unsigned long>(coord.ts_sec) * 1000UL + static_cast<unsigned long>(coord.ts_nsec) / 1000000UL;

            yaml_out << "      - lat: " << lat << std::endl;
            yaml_out << "        lon: " << lon << std::endl;
            yaml_out << "        timestamp: " << timestamp << std::endl;
        }
        file_num++;
    }

    yaml_out.close();
    rclcpp::shutdown();
    return 0;
}
