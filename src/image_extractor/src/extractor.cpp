#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/opencv.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <filesystem>

namespace fs = std::filesystem;

void process_bag_file(const std::string &bag_path,
                        const std::string &target_topic,
                        double start_offset_sec,     
                        double capture_duration_sec,  
                        std::vector<cv::Mat> &frames,
                        bool &thumbnail_saved,
                        const std::string &output_directory) {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "sqlite3";
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    try {
        reader.open(storage_options, converter_options);
    } catch (const std::exception &e) {
        std::cerr << "Failed to open bag file: " << bag_path 
                  << " Error: " << e.what() << std::endl;
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("image_extractor"), "Opened bag file: %s", bag_path.c_str());

    int fps = 30;
    int skip_frames = static_cast<int>(start_offset_sec * fps);
    int frames_to_capture = static_cast<int>(capture_duration_sec * fps);

    int topic_frame_counter = 0;        
    int local_frames_extracted = 0;       

    while (reader.has_next()) {
        auto bag_message = reader.read_next();

        if (bag_message->topic_name == target_topic) {
            if (topic_frame_counter < skip_frames) {
                topic_frame_counter++;
                continue;
            }

            // 메시지 deserialize 후 cv::Mat으로 디코딩
            rclcpp::SerializedMessage serialized_msg(*(bag_message->serialized_data));
            rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
            sensor_msgs::msg::CompressedImage compressed_msg;
            serializer.deserialize_message(&serialized_msg, &compressed_msg);

            cv::Mat image = cv::imdecode(compressed_msg.data, cv::IMREAD_COLOR);
            if (image.empty()) {
                RCLCPP_WARN(rclcpp::get_logger("image_extractor"), "Failed to decode image.");
                topic_frame_counter++;
                continue;
            }

            // 썸네일 저장 
            if (!thumbnail_saved) {
                std::string thumbnail_path = output_directory + "/thumbnail.jpg";
                cv::imwrite(thumbnail_path, image);
                RCLCPP_INFO(rclcpp::get_logger("image_extractor"), "Saved thumbnail image: %s", thumbnail_path.c_str());
                thumbnail_saved = true;
            }

            frames.push_back(image);
            local_frames_extracted++; 
            topic_frame_counter++;

            if (local_frames_extracted >= frames_to_capture) {
                RCLCPP_INFO(rclcpp::get_logger("image_extractor"), "Reached %d frames for bag file %s, stopping processing...", frames_to_capture, bag_path.c_str());
                break;
            }
        }
    }

    if (local_frames_extracted == 0) {
        RCLCPP_WARN(rclcpp::get_logger("image_extractor"), "No images processed from bag file: %s on topic: %s", bag_path.c_str(), target_topic.c_str());
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 7) {
        std::cerr << "Usage: ros2 run image_extractor extractor <bag1> <duration1> <topic1> <bag2> <duration2> <topic2> <output_directory>" << std::endl;
        return 1;
    }

    std::string bag1_path = argv[1];
    double duration1 = std::stod(argv[2]);
    std::string topic1 = argv[3];

    std::string bag2_path = argv[4];
    double duration2 = std::stod(argv[5]);
    std::string topic2 = argv[6];

    std::string output_directory = argv[7];

    if (!fs::exists(output_directory)) {
        try {
            fs::create_directories(output_directory);
        } catch (const std::exception &e) {
            std::cerr << "Failed to create output directory: " << e.what() << std::endl;
            return 1;
        }
    }

    std::vector<cv::Mat> frames;
    bool thumbnail_saved = false;  
    double start_offset_sec = 15.0;

    process_bag_file(bag1_path, topic1, start_offset_sec, duration1, frames, thumbnail_saved, output_directory);
    process_bag_file(bag2_path, topic2, 0.0, duration2, frames, thumbnail_saved, output_directory);

    if (frames.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("image_extractor"), "No frames extracted from the bags.");
        rclcpp::shutdown();
        return 1;
    }

    // 실제 캡처된 프레임 수
    int total_frames = frames.size();
    // 동영상 길이는 duration1 + duration2로 고정
    double total_duration = duration1 + duration2;

    // 프레임 당 시간을 계산 (예: duration1 + duration2에 해당하는 전체 프레임 시간)
    double fps = total_frames / total_duration;

    // 각 프레임의 크기
    int frame_width = frames[0].cols;
    int frame_height = frames[0].rows;

    // 최종 비디오 파일 경로
    std::string video_output_path = output_directory + "/video_clip.mp4";

    cv::VideoWriter video_writer;
    video_writer.open(video_output_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, cv::Size(frame_width, frame_height));

    if (!video_writer.isOpened()) {
        RCLCPP_ERROR(rclcpp::get_logger("image_extractor"), "Failed to open video writer!");
        rclcpp::shutdown();
        return 1;
    }

    // 모든 프레임을 비디오로 저장
    for (const auto &frame : frames) {
        video_writer.write(frame);
    }

    video_writer.release();

    RCLCPP_INFO(rclcpp::get_logger("image_extractor"), "Video saved: %s", video_output_path.c_str());

    rclcpp::shutdown();
    return 0;
}
