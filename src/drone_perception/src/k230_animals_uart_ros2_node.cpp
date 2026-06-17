#include "drone_perception/k230_animals_uart_ros2_node.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>

#include <fcntl.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

namespace
{
using Json = nlohmann::json;
}

K230AnimalsUartRos2Node::K230AnimalsUartRos2Node() :
	Node("k230_animals_uart_ros2_node")
{
	_serial_port = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
	_baudrate = declare_parameter<int>("baudrate", 460800);
	_image_topic = declare_parameter<std::string>("image_topic", "/drone/image");
	_detect_topic = declare_parameter<std::string>("detect_topic", "/k230/animals/detect");
	_center_topic = declare_parameter<std::string>("center_topic", "/k230/animals/center");
	_heartbeat_topic = declare_parameter<std::string>("heartbeat_topic", "/k230/animals/heartbeat");
	_targets_topic = declare_parameter<std::string>("targets_topic", "/k230/animals/targets");
	_route_topic = declare_parameter<std::string>("route_topic", "/return/drone/world_group");
	_local_pose_topic = declare_parameter<std::string>("local_pose_topic", "/mavros/local_position/pose");
	_capture_ready_topic = declare_parameter<std::string>("capture_ready_topic", "/k230/animals/capture_ready");
	_record_result_topic = declare_parameter<std::string>("record_result_topic", "/k230/animals/record_result");
	_scan_point_done_topic = declare_parameter<std::string>("scan_point_done_topic", "/k230/animals/scan_point_done");

	_scan_radius_m = declare_parameter<double>("scan_radius_m", kDefaultScanRadiusM);
	_record_deadband_m = declare_parameter<double>("record_deadband_m", kDefaultRecordDeadbandM);
	_pending_capture_timeout_s = declare_parameter<double>(
		"pending_capture_timeout_s",
		kDefaultPendingCaptureTimeoutS);

	RCLCPP_INFO(get_logger(), "K230 animals UART ROS2 node started");
	RCLCPP_INFO(get_logger(), "serial_port=%s baudrate=%d", _serial_port.c_str(), _baudrate);
	RCLCPP_INFO(get_logger(), "image_topic=%s", _image_topic.c_str());
	RCLCPP_INFO(get_logger(), "detect_topic=%s", _detect_topic.c_str());
	RCLCPP_INFO(get_logger(), "center_topic=%s", _center_topic.c_str());
	RCLCPP_INFO(get_logger(), "heartbeat_topic=%s", _heartbeat_topic.c_str());
	RCLCPP_INFO(get_logger(), "targets_topic=%s", _targets_topic.c_str());
	RCLCPP_INFO(get_logger(), "record_deadband_m=%f", _record_deadband_m);
	RCLCPP_INFO(get_logger(), "pending_capture_timeout_s=%f", _pending_capture_timeout_s);


	_image_pub = create_publisher<drone_msgs::msg::BarcodeCapture>(
		_image_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
	_detect_pub = create_publisher<std_msgs::msg::String>(
		_detect_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
	_center_pub = create_publisher<drone_msgs::msg::K230AnimalCenter>(
		_center_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
	_heartbeat_pub = create_publisher<std_msgs::msg::String>(
		_heartbeat_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
	_targets_pub = create_publisher<drone_msgs::msg::K230AnimalTargets>(
		_targets_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
	_record_result_pub = create_publisher<drone_msgs::msg::K230RecordResult>(
		_record_result_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

	_route_points_sub = create_subscription<drone_msgs::msg::WorldGroup>(
		_route_topic,
		rclcpp::QoS(10).best_effort(),
		std::bind(&K230AnimalsUartRos2Node::handleRoutePoints, this, std::placeholders::_1));

	_local_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
		_local_pose_topic,
		rclcpp::QoS(10).best_effort(),
		std::bind(&K230AnimalsUartRos2Node::handleLocalPose, this, std::placeholders::_1));
	_capture_ready_sub = create_subscription<drone_msgs::msg::K230CaptureReady>(
		_capture_ready_topic,
		rclcpp::QoS(10).reliable(),
		std::bind(&K230AnimalsUartRos2Node::handleCaptureReady, this, std::placeholders::_1));
	_scan_point_done_sub = create_subscription<drone_msgs::msg::K230ScanPointDone>(
		_scan_point_done_topic,
		rclcpp::QoS(10).reliable(),
		std::bind(&K230AnimalsUartRos2Node::handleScanPointDone, this, std::placeholders::_1));

	openSerial();

	_read_timer = create_wall_timer(
		std::chrono::milliseconds(5),
		std::bind(&K230AnimalsUartRos2Node::readSerialOnce, this));

	_heartbeat_timer = create_wall_timer(
		std::chrono::seconds(2),
		std::bind(&K230AnimalsUartRos2Node::publishHeartbeat, this));
}

K230AnimalsUartRos2Node::~K230AnimalsUartRos2Node()
{
	if (_serial_fd >= 0) {
		close(_serial_fd);
		_serial_fd = -1;
	}
}

speed_t K230AnimalsUartRos2Node::baudrateToSpeed(int baudrate)
{
	switch (baudrate) {
	case 115200:
		return B115200;
	case 230400:
		return B230400;
	case 460800:
		return B460800;
	case 921600:
		return B921600;
	default:
		throw std::runtime_error("Unsupported baudrate");
	}
}

uint32_t K230AnimalsUartRos2Node::readU32Be(const uint8_t *p)
{
	return (static_cast<uint32_t>(p[0]) << 24) |
	       (static_cast<uint32_t>(p[1]) << 16) |
	       (static_cast<uint32_t>(p[2]) << 8) |
	       static_cast<uint32_t>(p[3]);
}

uint16_t K230AnimalsUartRos2Node::readU16Be(const uint8_t *p)
{
	return (static_cast<uint16_t>(p[0]) << 8) |
	       static_cast<uint16_t>(p[1]);
}

uint16_t K230AnimalsUartRos2Node::crc16Ccitt(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < len; ++i) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;

		for (int bit = 0; bit < 8; ++bit) {
			if (crc & 0x8000) {
				crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
			} else {
				crc = static_cast<uint16_t>(crc << 1);
			}
		}
	}

	return crc;
}

bool K230AnimalsUartRos2Node::isLikelyJpeg(const std::vector<uint8_t> &payload)
{
	if (payload.size() < 4) {
		return false;
	}

	return payload[0] == 0xFF &&
	       payload[1] == 0xD8 &&
	       payload[payload.size() - 2] == 0xFF &&
	       payload[payload.size() - 1] == 0xD9;
}

bool K230AnimalsUartRos2Node::jsonBoolFieldTrue(const std::string &json_text, const std::string &field)
{
	const std::string key = "\"" + field + "\"";
	const size_t key_pos = json_text.find(key);

	if (key_pos == std::string::npos) {
		return false;
	}

	const size_t colon_pos = json_text.find(':', key_pos + key.size());

	if (colon_pos == std::string::npos) {
		return false;
	}

	size_t value_pos = colon_pos + 1;

	while (value_pos < json_text.size() && json_text[value_pos] == ' ') {
		++value_pos;
	}

	return json_text.compare(value_pos, 4, "true") == 0 ||
	       json_text.compare(value_pos, 1, "1") == 0;
}

std::string K230AnimalsUartRos2Node::jsonStringField(const std::string &json_text, const std::string &field)
{
	const std::string key = "\"" + field + "\"";
	const size_t key_pos = json_text.find(key);

	if (key_pos == std::string::npos) {
		return "";
	}

	const size_t colon_pos = json_text.find(':', key_pos + key.size());

	if (colon_pos == std::string::npos) {
		return "";
	}

	size_t quote_pos = json_text.find('"', colon_pos + 1);

	if (quote_pos == std::string::npos) {
		return "";
	}

	const size_t end_quote_pos = json_text.find('"', quote_pos + 1);

	if (end_quote_pos == std::string::npos || end_quote_pos <= quote_pos + 1) {
		return "";
	}

	return json_text.substr(quote_pos + 1, end_quote_pos - quote_pos - 1);
}

bool K230AnimalsUartRos2Node::jsonNumberField(
	const std::string &json_text, const std::string &field, double &value)
{
	const std::string key = "\"" + field + "\"";
	const size_t key_pos = json_text.find(key);

	if (key_pos == std::string::npos) {
		return false;
	}

	const size_t colon_pos = json_text.find(':', key_pos + key.size());

	if (colon_pos == std::string::npos) {
		return false;
	}

	size_t value_pos = colon_pos + 1;

	while (value_pos < json_text.size() && json_text[value_pos] == ' ') {
		++value_pos;
	}

	char *end_ptr = nullptr;
	const double parsed = std::strtod(json_text.c_str() + value_pos, &end_ptr);

	if (end_ptr == json_text.c_str() + value_pos) {
		return false;
	}

	value = parsed;
	return true;
}

void K230AnimalsUartRos2Node::openSerial()
{
	_serial_fd = open(_serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_serial_fd < 0) {
		throw std::runtime_error("open serial failed: " + _serial_port + " error=" + std::strerror(errno));
	}

	termios tty{};

	if (tcgetattr(_serial_fd, &tty) != 0) {
		throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
	}

	cfmakeraw(&tty);

	const speed_t speed = baudrateToSpeed(_baudrate);
	cfsetispeed(&tty, speed);
	cfsetospeed(&tty, speed);

	tty.c_cflag |= CLOCAL | CREAD;
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;

	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

	if (tcsetattr(_serial_fd, TCSANOW, &tty) != 0) {
		throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
	}

	tcflush(_serial_fd, TCIOFLUSH);

	RCLCPP_INFO(get_logger(), "serial opened successfully fd=%d", _serial_fd);
}

std::string K230AnimalsUartRos2Node::makeImageBarcode(uint32_t) const
{
	return _last_label.empty() ? "动物" : _last_label;
}

void K230AnimalsUartRos2Node::publishImagePacket(uint32_t seq, const std::vector<uint8_t> &payload)
{
	if (!isLikelyJpeg(payload)) {
		RCLCPP_WARN(
			get_logger(),
			"skip non-jpeg image packet seq=%u bytes=%zu",
			seq,
			payload.size());
		return;
	}

	drone_msgs::msg::BarcodeCapture msg;
	msg.stamp = now();
	msg.barcode = makeImageBarcode(seq);
	msg.image_format = "jpeg";
	msg.image_data = payload;

	_image_pub->publish(msg);

	RCLCPP_INFO(
		get_logger(),
		"published image barcode=%s bytes=%zu",
		msg.barcode.c_str(),
		msg.image_data.size());
}

void K230AnimalsUartRos2Node::publishDetectionPacket(uint32_t seq, const std::string &json_text)
{
	std_msgs::msg::String detect_msg;
	detect_msg.data = json_text;
	_detect_pub->publish(detect_msg);

	if (!jsonBoolFieldTrue(json_text, "valid")) {
		return;
	}

	double norm_x = 0.0;
	double norm_y = 0.0;
	double score = 0.0;
	double cx = 0.0;
	double cy = 0.0;
	double err_x = 0.0;
	double err_y = 0.0;
	double stable_frames = 0.0;
	double count = 0.0;

	if (!jsonNumberField(json_text, "norm_x", norm_x) ||
	    !jsonNumberField(json_text, "norm_y", norm_y) ||
	    !jsonNumberField(json_text, "cx", cx) ||
	    !jsonNumberField(json_text, "cy", cy) ||
	    !jsonNumberField(json_text, "err_x", err_x) ||
	    !jsonNumberField(json_text, "err_y", err_y)) {
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			*get_clock(),
			1000,
			"valid detection missing center fields: %s",
			json_text.c_str());
		return;
	}

	jsonNumberField(json_text, "score", score);
	jsonNumberField(json_text, "stable_frames", stable_frames);
	jsonNumberField(json_text, "count", count);

	drone_msgs::msg::K230AnimalCenter center_msg;
	center_msg.stamp = now();
	center_msg.seq = seq;
	center_msg.valid = true;
	center_msg.confirmed = jsonBoolFieldTrue(json_text, "confirmed");
	center_msg.stable_frames = static_cast<int32_t>(stable_frames);
	center_msg.count = static_cast<uint32_t>(count);
	center_msg.label = jsonStringField(json_text, "label");
	center_msg.score = score;
	center_msg.cx = static_cast<int32_t>(cx);
	center_msg.cy = static_cast<int32_t>(cy);
	center_msg.err_x = static_cast<int32_t>(err_x);
	center_msg.err_y = static_cast<int32_t>(err_y);
	center_msg.norm_x = norm_x;
	center_msg.norm_y = norm_y;

	_center_pub->publish(center_msg);
}

void K230AnimalsUartRos2Node::publishTargetsPacket(uint32_t seq, const std::string &json_text)
{
	try {
		const Json payload = Json::parse(json_text);

		if (!payload.contains("targets") || !payload["targets"].is_array()) {
			RCLCPP_WARN(get_logger(), "targets packet missing targets array: %s", json_text.c_str());
			return;
		}

		drone_msgs::msg::K230AnimalTargets msg;
		msg.stamp = now();
		msg.frame_seq = payload.value("frame_seq", seq);

		const int32_t active_scan_point_offset = findActiveScanPointOffset();

		if (active_scan_point_offset < 0) {
			RCLCPP_INFO_THROTTLE(
				get_logger(),
				*get_clock(),
				1000,
				"skip targets: not inside active scan point");
			return;
		}

		const auto &scan_point = _scan_points[static_cast<size_t>(active_scan_point_offset)];
		msg.scan_point_index = scan_point.index;
		msg.scan_point_x = scan_point.x;
		msg.scan_point_y = scan_point.y;

		for (const auto &target_json : payload["targets"]) {
			if (!target_json.is_object()) {
				continue;
			}

			drone_msgs::msg::K230AnimalTarget target_msg;
			target_msg.label = target_json.value("label", "");
			target_msg.label_instance_id = target_json.value("label_instance_id", 0U);
			target_msg.score = target_json.value("score", 0.0);
			target_msg.confirmed = target_json.value("confirmed", false);
			target_msg.stable_frames = target_json.value("stable_frames", 0);

			target_msg.cx = target_json.value("cx", -1);
			target_msg.cy = target_json.value("cy", -1);
			target_msg.err_x = target_json.value("err_x", 0);
			target_msg.err_y = target_json.value("err_y", 0);
			target_msg.norm_x = target_json.value("norm_x", 0.0);
			target_msg.norm_y = target_json.value("norm_y", 0.0);

			target_msg.x1 = target_json.value("x1", 0);
			target_msg.y1 = target_json.value("y1", 0);
			target_msg.x2 = target_json.value("x2", 0);
			target_msg.y2 = target_json.value("y2", 0);
			target_msg.bbox_w = target_json.value("bbox_w", 0);
			target_msg.bbox_h = target_json.value("bbox_h", 0);
			target_msg.bbox_area = target_json.value("bbox_area", 0);

			msg.targets.push_back(target_msg);
		}

		const uint32_t parsed_target_count = static_cast<uint32_t>(msg.targets.size());
		msg.target_count = payload.value("target_count", parsed_target_count);

		if (msg.target_count != parsed_target_count) {
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				*get_clock(),
				1000,
				"targets count mismatch frame_seq=%u reported=%u parsed=%u",
				msg.frame_seq,
				msg.target_count,
				parsed_target_count);
		}

		_targets_pub->publish(msg);

		RCLCPP_INFO_THROTTLE(
			get_logger(),
			*get_clock(),
			1000,
			"published targets frame_seq=%u count=%u",
			msg.frame_seq,
			msg.target_count);
	} catch (const Json::exception &e) {
		RCLCPP_WARN(get_logger(), "failed to parse targets json: %s", e.what());
	}
}

void K230AnimalsUartRos2Node::handlePacket(uint8_t type, uint32_t seq, const std::vector<uint8_t> &payload)
{
	if (type == 1) {
		const std::string json_text(payload.begin(), payload.end());
		const std::string label = jsonStringField(json_text, "label");

		if (jsonBoolFieldTrue(json_text, "valid") && !label.empty()) {
			_last_label = label;
		}

		publishDetectionPacket(seq, json_text);

		RCLCPP_INFO_THROTTLE(
			get_logger(),
			*get_clock(),
			1000,
			"DETECTION packet seq=%u bytes=%zu json=%s",
			seq,
			payload.size(),
			json_text.c_str());
		return;
	}

	if (type == 2) {
		RCLCPP_INFO(
			get_logger(),
			"JPEG packet seq=%u bytes=%zu",
			seq,
			payload.size());
		publishImagePacket(seq, payload);
		return;
	}

	if (type == 3) {
		const std::string json_text(payload.begin(), payload.end());
		publishTargetsPacket(seq, json_text);
		return;
	}

	if (type == kPacketTypeRecordResult) {
		const std::string json_text(payload.begin(), payload.end());
		publishRecordResultFromPacket(seq, json_text);
		return;
	}

	RCLCPP_WARN(
		get_logger(),
		"unknown packet type=%u seq=%u bytes=%zu",
		type,
		seq,
		payload.size());
}

void K230AnimalsUartRos2Node::parsePackets()
{
	const std::array<uint8_t, 4> magic{'K', '2', '3', '0'};

	while (true) {
		if (_rx_buffer.size() < kHeaderSize + kCrcSize) {
			return;
		}

		auto magic_it = std::search(
			_rx_buffer.begin(),
			_rx_buffer.end(),
			magic.begin(),
			magic.end());

		if (magic_it == _rx_buffer.end()) {
			if (_rx_buffer.size() > magic.size() - 1) {
				_rx_buffer.erase(_rx_buffer.begin(), _rx_buffer.end() - (magic.size() - 1));
			}

			return;
		}

		if (magic_it != _rx_buffer.begin()) {
			_rx_buffer.erase(_rx_buffer.begin(), magic_it);
		}

		if (_rx_buffer.size() < kHeaderSize + kCrcSize) {
			return;
		}

		const uint8_t type = _rx_buffer[4];
		const uint32_t seq = readU32Be(&_rx_buffer[5]);
		const uint32_t payload_len = readU32Be(&_rx_buffer[9]);

		if (payload_len > kMaxPayloadSize) {
			RCLCPP_WARN(get_logger(), "payload too large=%u, dropping one byte", payload_len);
			_rx_buffer.erase(_rx_buffer.begin());
			continue;
		}

		const size_t packet_len = kHeaderSize + static_cast<size_t>(payload_len) + kCrcSize;

		if (_rx_buffer.size() < packet_len) {
			return;
		}

		const uint16_t rx_crc = readU16Be(&_rx_buffer[kHeaderSize + payload_len]);
		const uint16_t calc_crc = crc16Ccitt(_rx_buffer.data(), kHeaderSize + payload_len);

		if (rx_crc != calc_crc) {
			RCLCPP_WARN(
				get_logger(),
				"crc mismatch type=%u seq=%u len=%u rx=0x%04x calc=0x%04x",
				type,
				seq,
				payload_len,
				rx_crc,
				calc_crc);
			_rx_buffer.erase(_rx_buffer.begin());
			continue;
		}

		std::vector<uint8_t> payload(
			_rx_buffer.begin() + kHeaderSize,
			_rx_buffer.begin() + kHeaderSize + payload_len);

		handlePacket(type, seq, payload);
		_rx_buffer.erase(_rx_buffer.begin(), _rx_buffer.begin() + packet_len);
	}
}

void K230AnimalsUartRos2Node::readSerialOnce()
{
	if (_serial_fd < 0) {
		return;
	}

	std::array<uint8_t, 4096> temp{};
	size_t tick_bytes = 0;

	while (true) {
		const ssize_t n = read(_serial_fd, temp.data(), temp.size());

		if (n > 0) {
			_rx_buffer.insert(_rx_buffer.end(), temp.begin(), temp.begin() + n);
			tick_bytes += static_cast<size_t>(n);
			_total_rx_bytes += static_cast<size_t>(n);
			continue;
		}

		if (n == 0) {
			break;
		}

		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			break;
		}

		RCLCPP_ERROR(
			get_logger(),
			"serial read failed: %s",
			std::strerror(errno));
		break;
	}

	if (tick_bytes > 0) {
		parsePackets();

		RCLCPP_INFO_THROTTLE(
			get_logger(),
			*get_clock(),
			1000,
			"uart rx tick_bytes=%zu buffer=%zu total=%zu",
			tick_bytes,
			_rx_buffer.size(),
			_total_rx_bytes);
	}

	if (_rx_buffer.size() > 4 * 1024 * 1024) {
		RCLCPP_WARN(get_logger(), "rx_buffer too large, clearing");
		_rx_buffer.clear();
	}
}

void K230AnimalsUartRos2Node::publishHeartbeat()
{
	std_msgs::msg::String msg;
	msg.data = "alive";
	_heartbeat_pub->publish(msg);
}

void K230AnimalsUartRos2Node::handleRoutePoints(const drone_msgs::msg::WorldGroup::SharedPtr msg)
{
	if (msg->points.empty()) {
		RCLCPP_WARN(get_logger(), "handleRoutePoints: empty points array");
		return;
	}

	_scan_points.clear();
	_scan_points.reserve(msg->points.size());

	for (size_t i = 0; i < msg->points.size(); ++i) {
		const auto &wp = msg->points[i];

		ScanPoint sp;
		sp.index = static_cast<int32_t>(i);
		sp.x = wp.x;
		sp.y = wp.y;
		sp.scanned = false;

		_scan_points.push_back(sp);
	}

	RCLCPP_INFO(get_logger(),
		"handleRoutePoints: received %zu waypoints (%.2f, %.2f) .. (%.2f, %.2f)",
		_scan_points.size(),
		_scan_points.front().x, _scan_points.front().y,
		_scan_points.back().x, _scan_points.back().y);
}

void K230AnimalsUartRos2Node::handleLocalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	_current_x = msg->pose.position.x;
	_current_y = msg->pose.position.y;
	_has_local_pose = true;

	RCLCPP_DEBUG(get_logger(), "handleLocalPose: (%.2f, %.2f)", _current_x, _current_y);
}

void K230AnimalsUartRos2Node::handleCaptureReady(const drone_msgs::msg::K230CaptureReady::SharedPtr msg)
{
	if (!msg->capture_ready) {
		return;
	}

	if (!_has_local_pose) {
		publishSkippedRecordResult(*msg, "no local pose");
		return;
	}

	const int32_t scan_point_offset = findScanPointOffsetByIndex(msg->scan_point_index);

	if (scan_point_offset < 0) {
		publishSkippedRecordResult(*msg, "unknown scan point");
		return;
	}

	const auto &scan_point = _scan_points[static_cast<size_t>(scan_point_offset)];

	if (scan_point.scanned) {
		publishSkippedRecordResult(*msg, "scan point already scanned");
		return;
	}

	const double dx = scan_point.x - _current_x;
	const double dy = scan_point.y - _current_y;
	const double scan_radius_sq = _scan_radius_m * _scan_radius_m;

	if (dx * dx + dy * dy > scan_radius_sq) {
		publishSkippedRecordResult(*msg, "not inside scan radius");
		return;
	}

	if (isCaptureTargetRecorded(*msg)) {
		publishSkippedRecordResult(*msg, "target already recorded");
		return;
	}

	clearTimedOutPendingCapture();

	if (_pending_capture.active) {
		publishSkippedRecordResult(*msg, "capture already pending");
		return;
	}

	rememberPendingCapture(*msg);

	if (!sendCaptureCommand(*msg)) {
		_pending_capture = PendingCapture{};
		publishSkippedRecordResult(*msg, "failed to send capture command");
		return;
	}

	RCLCPP_INFO(
		get_logger(),
		"capture_ready received: scan_point=%d frame_seq=%u label=%s instance_id=%u",
		msg->scan_point_index,
		msg->frame_seq,
		msg->label.c_str(),
		msg->label_instance_id);
}

void K230AnimalsUartRos2Node::handleScanPointDone(const drone_msgs::msg::K230ScanPointDone::SharedPtr msg)
{
	if (!msg->scan_point_done) {
		return;
	}

	const int32_t scan_point_offset = findScanPointOffsetByIndex(msg->scan_point_index);

	if (scan_point_offset < 0) {
		RCLCPP_WARN(
			get_logger(),
			"scan_point_done ignored: unknown scan_point=%d",
			msg->scan_point_index);
		return;
	}

	auto &scan_point = _scan_points[static_cast<size_t>(scan_point_offset)];
	scan_point.scanned = true;

	if (_pending_capture.active &&
	    _pending_capture.scan_point_index == msg->scan_point_index) {
		_pending_capture = PendingCapture{};
	}

	RCLCPP_INFO(
		get_logger(),
		"scan_point_done accepted: scan_point=%d marked scanned",
		msg->scan_point_index);
}

bool K230AnimalsUartRos2Node::isCaptureTargetRecorded(
	const drone_msgs::msg::K230CaptureReady &msg) const
{
	for (const auto &recorded_pose : _recorded_poses) {
		if (recorded_pose.scan_point_index == msg.scan_point_index &&
		    recorded_pose.label == msg.label &&
		    recorded_pose.label_instance_id == msg.label_instance_id) {
			return true;
		}
	}

	return false;
}

bool K230AnimalsUartRos2Node::pendingCaptureTimedOut()
{
	if (!_pending_capture.active || _pending_capture.request_time.nanoseconds() == 0) {
		return false;
	}

	return (now() - _pending_capture.request_time).seconds() > _pending_capture_timeout_s;
}

void K230AnimalsUartRos2Node::clearTimedOutPendingCapture()
{
	if (!pendingCaptureTimedOut()) {
		return;
	}

	const double elapsed_s = (now() - _pending_capture.request_time).seconds();

	RCLCPP_WARN(
		get_logger(),
		"clear timed out pending capture: scan_point=%d frame_seq=%u label=%s instance_id=%u elapsed=%.2f timeout=%.2f",
		_pending_capture.scan_point_index,
		_pending_capture.frame_seq,
		_pending_capture.label.c_str(),
		_pending_capture.label_instance_id,
		elapsed_s,
		_pending_capture_timeout_s);

	_pending_capture = PendingCapture{};
}

void K230AnimalsUartRos2Node::rememberPendingCapture(const drone_msgs::msg::K230CaptureReady &msg)
{
	_pending_capture.active = true;
	_pending_capture.frame_seq = msg.frame_seq;
	_pending_capture.scan_point_index = msg.scan_point_index;
	_pending_capture.label = msg.label;
	_pending_capture.label_instance_id = msg.label_instance_id;
	_pending_capture.x = _current_x;
	_pending_capture.y = _current_y;
	_pending_capture.request_time = now();
}

void K230AnimalsUartRos2Node::commitPendingCaptureIfMatched(const drone_msgs::msg::K230RecordResult &msg)
{
	if (!_pending_capture.active) {
		return;
	}

	if (msg.frame_seq != _pending_capture.frame_seq ||
	    msg.scan_point_index != _pending_capture.scan_point_index ||
	    msg.label != _pending_capture.label ||
	    msg.label_instance_id != _pending_capture.label_instance_id) {
		return;
	}

	if (msg.record_success && msg.result_state == "captured") {
		RecordedPose recorded_pose;
		recorded_pose.x = _pending_capture.x;
		recorded_pose.y = _pending_capture.y;
		recorded_pose.scan_point_index = _pending_capture.scan_point_index;
		recorded_pose.label = _pending_capture.label;
		recorded_pose.label_instance_id = _pending_capture.label_instance_id;

		_recorded_poses.push_back(recorded_pose);

		RCLCPP_INFO(
			get_logger(),
			"recorded capture pose: scan_point=%d label=%s instance_id=%u x=%.3f y=%.3f",
			recorded_pose.scan_point_index,
			recorded_pose.label.c_str(),
			recorded_pose.label_instance_id,
			recorded_pose.x,
			recorded_pose.y);
	}

	_pending_capture = PendingCapture{};
}

void K230AnimalsUartRos2Node::publishRecordResultFromPacket(uint32_t seq, const std::string &json_text)
{
	try {
		const Json payload = Json::parse(json_text);

		drone_msgs::msg::K230RecordResult msg;
		msg.stamp = now();
		msg.frame_seq = payload.value("frame_seq", seq);
		msg.scan_point_index = payload.value("scan_point_index", -1);
		msg.label = payload.value("label", "");
		msg.label_instance_id = payload.value("label_instance_id", 0U);
		msg.record_success = payload.value("record_success", false);
		msg.result_state = payload.value("result_state", "failed");
		msg.image_name = payload.value("image_name", "");

		commitPendingCaptureIfMatched(msg);
		_record_result_pub->publish(msg);

		RCLCPP_INFO(
			get_logger(),
			"published record_result: scan_point=%d frame_seq=%u label=%s instance_id=%u state=%s success=%s",
			msg.scan_point_index,
			msg.frame_seq,
			msg.label.c_str(),
			msg.label_instance_id,
			msg.result_state.c_str(),
			msg.record_success ? "true" : "false");
	} catch (const Json::exception &e) {
		RCLCPP_WARN(
			get_logger(),
			"failed to parse record_result json: %s",
			e.what());
	}
}

void K230AnimalsUartRos2Node::publishSkippedRecordResult(
	const drone_msgs::msg::K230CaptureReady &msg,
	const std::string &reason)
{
	drone_msgs::msg::K230RecordResult result_msg;
	result_msg.stamp = now();
	result_msg.frame_seq = msg.frame_seq;
	result_msg.scan_point_index = msg.scan_point_index;
	result_msg.label = msg.label;
	result_msg.label_instance_id = msg.label_instance_id;
	result_msg.record_success = false;
	result_msg.result_state = "skipped";
	result_msg.image_name = "";

	_record_result_pub->publish(result_msg);

	RCLCPP_INFO(
		get_logger(),
		"published skipped record_result: scan_point=%d frame_seq=%u label=%s instance_id=%u reason=%s",
		msg.scan_point_index,
		msg.frame_seq,
		msg.label.c_str(),
		msg.label_instance_id,
		reason.c_str());
}

bool K230AnimalsUartRos2Node::writePacket(uint8_t type, uint32_t seq, const std::string &payload)
{
	if (_serial_fd < 0) {
		RCLCPP_WARN(get_logger(), "write packet failed: serial is not open");
		return false;
	}

	const uint32_t payload_len = static_cast<uint32_t>(payload.size());

	std::vector<uint8_t> packet;
	packet.reserve(kHeaderSize + payload.size() + kCrcSize);

	packet.push_back('K');
	packet.push_back('2');
	packet.push_back('3');
	packet.push_back('0');
	packet.push_back(type);

	packet.push_back(static_cast<uint8_t>((seq >> 24) & 0xFF));
	packet.push_back(static_cast<uint8_t>((seq >> 16) & 0xFF));
	packet.push_back(static_cast<uint8_t>((seq >> 8) & 0xFF));
	packet.push_back(static_cast<uint8_t>(seq & 0xFF));

	packet.push_back(static_cast<uint8_t>((payload_len >> 24) & 0xFF));
	packet.push_back(static_cast<uint8_t>((payload_len >> 16) & 0xFF));
	packet.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
	packet.push_back(static_cast<uint8_t>(payload_len & 0xFF));

	packet.insert(packet.end(), payload.begin(), payload.end());

	const uint16_t crc = crc16Ccitt(packet.data(), packet.size());
	packet.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
	packet.push_back(static_cast<uint8_t>(crc & 0xFF));

	size_t written_total = 0;

	while (written_total < packet.size()) {
		const ssize_t written = write(
			_serial_fd,
			packet.data() + written_total,
			packet.size() - written_total);

		if (written > 0) {
			written_total += static_cast<size_t>(written);
			continue;
		}

		if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			RCLCPP_WARN(get_logger(), "write packet would block");
			return false;
		}

		RCLCPP_WARN(
			get_logger(),
			"write packet failed: %s",
			std::strerror(errno));
		return false;
	}

	return true;
}

bool K230AnimalsUartRos2Node::sendCaptureCommand(const drone_msgs::msg::K230CaptureReady &msg)
{
	Json payload;
	payload["type"] = "capture_command";
	payload["schema"] = 1;
	payload["source"] = "k230_animals_uart_ros2_node";
	payload["frame_seq"] = msg.frame_seq;
	payload["scan_point_index"] = msg.scan_point_index;
	payload["label"] = msg.label;
	payload["label_instance_id"] = msg.label_instance_id;
	payload["capture_ready"] = msg.capture_ready;

	const bool ok = writePacket(kPacketTypeCaptureCommand, msg.frame_seq, payload.dump());

	if (!ok) {
		RCLCPP_WARN(
			get_logger(),
			"failed to send capture command: scan_point=%d frame_seq=%u label=%s instance_id=%u",
			msg.scan_point_index,
			msg.frame_seq,
			msg.label.c_str(),
			msg.label_instance_id);
		return false;
	}

	RCLCPP_INFO(
		get_logger(),
		"capture command sent: scan_point=%d frame_seq=%u label=%s instance_id=%u",
		msg.scan_point_index,
		msg.frame_seq,
		msg.label.c_str(),
		msg.label_instance_id);

	return true;
}

int32_t K230AnimalsUartRos2Node::findScanPointOffsetByIndex(int32_t scan_point_index) const
{
	for (size_t i = 0; i < _scan_points.size(); ++i) {
		if (_scan_points[i].index == scan_point_index) {
			return static_cast<int32_t>(i);
		}
	}

	return -1;
}

int32_t K230AnimalsUartRos2Node::findActiveScanPointOffset()
{
	if (!_has_local_pose || _scan_points.empty()) {
		return -1;
	}

	const double radius_sq = _scan_radius_m * _scan_radius_m;

	for (size_t i = 0; i < _scan_points.size(); ++i) {
		const auto &sp = _scan_points[i];

		if (sp.scanned) {
			continue;
		}

		const double dx = sp.x - _current_x;
		const double dy = sp.y - _current_y;

		if (dx * dx + dy * dy <= radius_sq) {
			return static_cast<int32_t>(i);
		}
	}

	return -1;
}

int main(int argc, const char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<K230AnimalsUartRos2Node>());
	rclcpp::shutdown();
	return 0;
}
