#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/WaitHelpers.hh>

#include "convert.h"
#include "fts/proto.h"
#include "gate.h"
#include "mavlink_hb.h"
#include "pty.h"

namespace {

constexpr uint64_t kReadbackPeriodUs = 100000;
constexpr uint64_t kZeroPeriodUs = 10000;
constexpr uint64_t kTruthPeriodUs = 20000;
constexpr double kGnssJumpDeg = 0.02; /* about 2.2 km north */
constexpr int kMotors = 4;
constexpr uint8_t kPx4SysId = 1;
constexpr uint8_t kPx4CompId = 1;

struct Args {
	std::string link;
	std::string truth;
	std::string model = "x500_fts_0";
	std::string world = "fts_field";
	int mav_port = 14540;
};

Args parse_args(int argc, char **argv)
{
	Args a;

	for (int i = 1; i < argc; i += 2) {
		const std::string key = argv[i];
		if (i + 1 >= argc) {
			throw std::runtime_error("missing value for " + key);
		}
		const std::string value = argv[i + 1];
		if (key == "--link") {
			a.link = value;
		} else if (key == "--truth") {
			a.truth = value;
		} else if (key == "--model") {
			a.model = value;
		} else if (key == "--world") {
			a.world = value;
		} else if (key == "--mav-port") {
			a.mav_port = std::stoi(value);
		} else {
			throw std::runtime_error("unknown option " + key);
		}
	}
	if (a.link.empty() || a.truth.empty()) {
		throw std::runtime_error("usage: fts_bridge --link PTY --truth FILE [--model NAME] "
					 "[--world NAME] [--mav-port N]");
	}
	return a;
}

uint64_t stamp_of(const gz::msgs::Header &h)
{
	return stamp_to_us(h.stamp().sec(), h.stamp().nsec());
}

/* Next due time for a periodic action; restarts if the clock went back. */
bool due(uint64_t t, uint64_t &next, uint64_t period)
{
	if ((t < next) && (next - t <= period)) {
		return false;
	}
	next = t + period;
	return true;
}

class Bridge {
public:
	explicit Bridge(const Args &a) : args_(a)
	{
		fd_ = pty_open_raw(a.link);
		truth_ = std::fopen(a.truth.c_str(), "w");
		if (truth_ == nullptr) {
			throw std::runtime_error("cannot open " + a.truth + ": " + std::strerror(errno));
		}
		std::fprintf(truth_, "t_us,x,y,z,vx,vy,vz,relay_open,chute_fired,cut_motor,gnss_on,t_hb_us\n");
		udp_ = open_udp(a.mav_port);
		gated_pub_ = node_.Advertise<gz::msgs::Actuators>("/" + a.model + "/command/motor_speed_gated");
		chute_pub_ = node_.Advertise<gz::msgs::Boolean>("/fts/chute");
		const std::string world = "/world/" + a.world;
		sub(world + "/clock", &Bridge::on_clock);
		sub(world + "/pose/info", &Bridge::on_pose);
		sub("/fts/imu", &Bridge::on_imu);
		sub("/fts/navsat", &Bridge::on_navsat);
		sub("/fts/baro", &Bridge::on_baro);
		sub("/fts/fault", &Bridge::on_fault);
		sub("/fts/chute_state", &Bridge::on_chute_state);
		sub("/" + a.model + "/command/motor_speed", &Bridge::on_motor);
	}

	void start()
	{
		std::thread(&Bridge::link_reader, this).detach();
		std::thread(&Bridge::mav_reader, this).detach();
	}

	void close_truth()
	{
		std::lock_guard<std::mutex> lk(truth_mu_);
		std::fclose(truth_);
		truth_ = nullptr;
	}

private:
	template <typename M> void sub(const std::string &topic, void (Bridge::*cb)(const M &))
	{
		if (!node_.Subscribe(topic, cb, this)) {
			throw std::runtime_error("cannot subscribe to " + topic);
		}
	}

	static int open_udp(int port)
	{
		const int s = ::socket(AF_INET, SOCK_DGRAM, 0);
		if (s < 0) {
			throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
		}
		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_port = htons(static_cast<uint16_t>(port));
		addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		if (::bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
			throw std::runtime_error("bind UDP " + std::to_string(port) + ": " + std::strerror(errno));
		}
		return s;
	}

	void send(const uint8_t *buf, size_t n)
	{
		if (n == 0U) {
			throw std::logic_error("frame encoding failed");
		}
		std::lock_guard<std::mutex> lk(tx_mu_);
		size_t off = 0;
		while (off < n) {
			const ssize_t w = ::write(fd_, buf + off, n - off);
			if (w < 0) {
				if (errno == EINTR) {
					continue;
				}
				std::cerr << "fts_bridge: link write failed: " << std::strerror(errno) << "\n";
				std::exit(1);
			}
			off += static_cast<size_t>(w);
		}
	}

	void publish_motors(const std::vector<double> &v)
	{
		gz::msgs::Actuators out;
		for (double x : v) {
			out.add_velocity(x);
		}
		gated_pub_.Publish(out);
	}

	void on_clock(const gz::msgs::Clock &m)
	{
		const uint64_t t = stamp_to_us(m.sim().sec(), m.sim().nsec());
		std::lock_guard<std::mutex> lk(clock_mu_);

		t_us_ = t;
		if (due(t, next_readback_us_, kReadbackPeriodUs)) {
			fts_readback r{};
			uint8_t buf[FTS_FRAME_MAX];
			r.t_us = t;
			r.relay_open = relay_open_ ? 1U : 0U;
			r.chute_fired = chute_state_ ? 1U : 0U;
			send(buf, fts_encode_readback(&r, buf, sizeof(buf)));
		}
		if (relay_open_ && due(t, next_zero_us_, kZeroPeriodUs)) {
			publish_motors(std::vector<double>(kMotors, 0.0));
		}
	}

	void on_imu(const gz::msgs::IMU &m)
	{
		fts_imu f{};
		uint8_t buf[FTS_FRAME_MAX];

		f.t_us = stamp_of(m.header());
		f.q[0] = static_cast<float>(m.orientation().w());
		f.q[1] = static_cast<float>(m.orientation().x());
		f.q[2] = static_cast<float>(m.orientation().y());
		f.q[3] = static_cast<float>(m.orientation().z());
		f.gyro[0] = static_cast<float>(m.angular_velocity().x());
		f.gyro[1] = static_cast<float>(m.angular_velocity().y());
		f.gyro[2] = static_cast<float>(m.angular_velocity().z());
		f.accel[0] = static_cast<float>(m.linear_acceleration().x());
		f.accel[1] = static_cast<float>(m.linear_acceleration().y());
		f.accel[2] = static_cast<float>(m.linear_acceleration().z());
		send(buf, fts_encode_imu(&f, buf, sizeof(buf)));
	}

	void on_navsat(const gz::msgs::NavSat &m)
	{
		fts_gnss f{};
		uint8_t buf[FTS_FRAME_MAX];
		double lat = m.latitude_deg();

		if (!gnss_on_) {
			return;
		}
		if (gnss_jump_.exchange(false)) {
			lat += kGnssJumpDeg;
		}
		f.t_us = stamp_of(m.header());
		f.lat_e7 = deg_to_e7(lat);
		f.lon_e7 = deg_to_e7(m.longitude_deg());
		f.alt_mm = m_to_mm(m.altitude());
		f.fix = 3U;
		f.sats = 10U;
		send(buf, fts_encode_gnss(&f, buf, sizeof(buf)));
	}

	void on_baro(const gz::msgs::FluidPressure &m)
	{
		fts_baro f{};
		uint8_t buf[FTS_FRAME_MAX];

		f.t_us = stamp_of(m.header());
		f.pressure_pa = static_cast<float>(m.pressure());
		f.temp_c = 15.0f;
		send(buf, fts_encode_baro(&f, buf, sizeof(buf)));
	}

	void on_motor(const gz::msgs::Actuators &m)
	{
		const std::vector<double> in(m.velocity().begin(), m.velocity().end());
		gz::msgs::Actuators out;

		*out.mutable_header() = m.header();
		for (double v : gate_apply(in, relay_open_, cut_)) {
			out.add_velocity(v);
		}
		gated_pub_.Publish(out);
	}

	static int parse_count(const std::string &s, size_t prefix)
	{
		size_t used = 0;
		const int v = std::stoi(s.substr(prefix), &used);
		if (used != s.size() - prefix) {
			throw std::invalid_argument(s);
		}
		return v;
	}

	void on_fault(const gz::msgs::StringMsg &m)
	{
		const std::string &s = m.data();
		try {
			if (s.rfind("cut_motor:", 0) == 0) {
				const int n = parse_count(s, 10);
				if ((n < 0) || (n >= kMotors)) {
					throw std::invalid_argument(s);
				}
				cut_ = n;
			} else if (s == "gnss_off") {
				gnss_on_ = false;
			} else if (s == "gnss_on") {
				gnss_on_ = true;
			} else if (s == "gnss_jump") {
				gnss_jump_ = true;
			} else if (s.rfind("hb_drop:", 0) == 0) {
				hb_drop_ = parse_count(s, 8);
			} else {
				throw std::invalid_argument(s);
			}
		} catch (const std::exception &) {
			std::cerr << "fts_bridge: unknown fault [" << s << "]\n";
			return;
		}
		std::cerr << "fts_bridge: fault " << s << " at t_us " << t_us_.load() << "\n";
	}

	void on_chute_state(const gz::msgs::Boolean &m)
	{
		chute_state_ = m.data();
	}

	void on_pose(const gz::msgs::Pose_V &m)
	{
		const uint64_t t = stamp_of(m.header());

		for (const auto &p : m.pose()) {
			if (p.name() == args_.model) {
				write_truth(t, p.position().x(), p.position().y(), p.position().z());
				return;
			}
		}
	}

	void write_truth(uint64_t t, double x, double y, double z)
	{
		std::lock_guard<std::mutex> lk(truth_mu_);
		double vx = 0.0;
		double vy = 0.0;
		double vz = 0.0;

		if (truth_ == nullptr) {
			return;
		}
		if (have_prev_ && (t > prev_t_) && (t - prev_t_ < kTruthPeriodUs)) {
			return;
		}
		if (have_prev_ && (t > prev_t_)) {
			const double dt = static_cast<double>(t - prev_t_) / 1e6;
			vx = (x - prev_x_) / dt;
			vy = (y - prev_y_) / dt;
			vz = (z - prev_z_) / dt;
		}
		std::fprintf(truth_, "%llu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%llu\n",
			     static_cast<unsigned long long>(t), x, y, z, vx, vy, vz, relay_open_ ? 1 : 0,
			     chute_state_ ? 1 : 0, cut_.load(), gnss_on_ ? 1 : 0,
			     static_cast<unsigned long long>(t_hb_us_.load()));
		std::fflush(truth_);
		prev_t_ = t;
		prev_x_ = x;
		prev_y_ = y;
		prev_z_ = z;
		have_prev_ = true;
	}

	void on_frame(const fts_frame &f)
	{
		fts_output o;

		if (fts_decode_output(&f, &o) != 0) {
			return; /* STATUS also arrives on link 1; the bridge ignores it */
		}
		const bool relay = o.relay_open != 0U;
		if (relay && !relay_open_) {
			std::cerr << "fts_bridge: relay open at t_us " << t_us_.load() << "\n";
			relay_open_ = true;
			publish_motors(std::vector<double>(kMotors, 0.0));
		}
		relay_open_ = relay;
		if ((o.chute_fire != 0U) && !chute_cmd_) {
			gz::msgs::Boolean b;
			b.set_data(true);
			chute_pub_.Publish(b);
			chute_cmd_ = true;
			std::cerr << "fts_bridge: parachute fired at t_us " << t_us_.load() << "\n";
		}
	}

	void link_reader()
	{
		fts_parser p;
		fts_frame f;
		uint8_t buf[256];

		fts_parser_init(&p);
		for (;;) {
			const ssize_t n = ::read(fd_, buf, sizeof(buf));
			if (n < 0) {
				if (errno == EINTR) {
					continue;
				}
				std::cerr << "fts_bridge: link read failed: " << std::strerror(errno) << "\n";
				std::exit(1);
			}
			if (n == 0) {
				std::cerr << "fts_bridge: link closed\n";
				std::exit(1);
			}
			for (ssize_t i = 0; i < n; i++) {
				if (!fts_parser_feed(&p, buf[i], &f)) {
					continue;
				}
				do {
					on_frame(f);
				} while (fts_parser_next(&p, &f));
			}
		}
	}

	void mav_reader()
	{
		uint8_t buf[2048];
		uint32_t seq = 0;

		for (;;) {
			const ssize_t n = ::recv(udp_, buf, sizeof(buf), 0);
			if (n < 0) {
				if (errno == EINTR) {
					continue;
				}
				std::cerr << "fts_bridge: UDP receive failed: " << std::strerror(errno) << "\n";
				std::exit(1);
			}
			if (!mav_has_heartbeat(buf, static_cast<size_t>(n), kPx4SysId, kPx4CompId)) {
				continue;
			}
			const int drop = hb_drop_.load();
			if (drop > 0) {
				hb_drop_ = drop - 1;
				continue;
			}
			fts_ap_heartbeat h{};
			uint8_t out[FTS_FRAME_MAX];
			h.t_us = t_us_.load();
			h.seq = seq++;
			t_hb_us_ = h.t_us;
			send(out, fts_encode_ap_heartbeat(&h, out, sizeof(out)));
		}
	}

	Args args_;
	int fd_ = -1;
	int udp_ = -1;
	std::FILE *truth_ = nullptr;
	gz::transport::Node node_;
	gz::transport::Node::Publisher gated_pub_;
	gz::transport::Node::Publisher chute_pub_;
	std::mutex tx_mu_;
	std::mutex clock_mu_;
	std::mutex truth_mu_;
	std::atomic<uint64_t> t_us_{0};
	std::atomic<uint64_t> t_hb_us_{0};
	std::atomic<bool> relay_open_{false};
	std::atomic<bool> chute_cmd_{false};
	std::atomic<bool> chute_state_{false};
	std::atomic<bool> gnss_on_{true};
	std::atomic<bool> gnss_jump_{false};
	std::atomic<int> cut_{-1};
	std::atomic<int> hb_drop_{0};
	uint64_t next_readback_us_ = 0;
	uint64_t next_zero_us_ = 0;
	bool have_prev_ = false;
	uint64_t prev_t_ = 0;
	double prev_x_ = 0.0;
	double prev_y_ = 0.0;
	double prev_z_ = 0.0;
};

} // namespace

int main(int argc, char **argv)
{
	try {
		Bridge bridge(parse_args(argc, argv));
		bridge.start();
		std::cerr << "fts_bridge: running\n";
		gz::transport::waitForShutdown();
		bridge.close_truth();
	} catch (const std::exception &e) {
		std::cerr << "fts_bridge: " << e.what() << "\n";
		return 1;
	}
	/* Reader threads are blocked in read(); end the process without unwinding them. */
	std::_Exit(0);
}
