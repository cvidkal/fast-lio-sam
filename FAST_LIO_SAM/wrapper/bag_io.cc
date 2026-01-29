//
// Created by xiang on 23-12-14.
//

#include "bag_io.h"

#include <glog/logging.h>
#include <filesystem>

#ifdef ROS1
#include <rosbag/view.h>
#else
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#endif

void RosbagIO::Go(int sleep_usec) {
#ifdef ROS1
    rosbag::Bag bag;
    LOG(INFO) << "Opening bag file: " << bag_file_;
    bag.open(bag_file_, rosbag::bagmode::Read);

    rosbag::View view(bag);
    LOG(INFO) << "Bag file opened, total messages: " << view.size();
    for (rosbag::MessageInstance const m : view) {
        auto iter = process_func_.find(m.getTopic());
        if (iter != process_func_.end()) {
            iter->second(m);
        }

        if (sleep_usec > 0) {
            usleep(sleep_usec);
        }
    }

    bag.close();
    LOG(INFO) << "bag " << bag_file_ << " finished.";
#else
    std::filesystem::path p(bag_file_);
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    rosbag2_cpp::ConverterOptions cv_options{"cdr", "cdr"};
    reader.open({bag_file_, "sqlite3"}, cv_options);

    while (reader.has_next()) {
        auto msg = reader.read_next();
        auto iter = process_func_.find(msg->topic_name);
        if (iter != process_func_.end()) {
            iter->second(msg);
        }

        if (sleep_usec > 0) {
            usleep(sleep_usec);
        }
    }

    LOG(INFO) << "bag " << bag_file_ << " finished.";
#endif
}