#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include <gz/msgs/boolean.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

#include "check.h"

namespace {

constexpr double kMass = 2.0;
constexpr double kGravity = 9.8;
constexpr double kCd = 1.5;
constexpr double kArea = 0.85;
constexpr double kRho = 1.225;

void sleep_ms(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace

int main()
{
	gz::sim::ServerConfig config;
	config.SetSdfFile(TEST_WORLD);
	gz::sim::TestFixture fixture(config);
	gz::sim::Link box;
	gz::sim::Link canopy;
	double box_vz = 0.0;
	double box_z = 0.0;
	double canopy_z = 0.0;

	fixture
		.OnConfigure([&](const gz::sim::Entity &world, const std::shared_ptr<const sdf::Element> &,
				 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) {
			gz::sim::World w(world);
			box = gz::sim::Link(gz::sim::Model(w.ModelByName(ecm, "box")).LinkByName(ecm, "link"));
			canopy = gz::sim::Link(
				gz::sim::Model(w.ModelByName(ecm, "canopy")).LinkByName(ecm, "link"));
			box.EnableVelocityChecks(ecm, true);
		})
		.OnPostUpdate([&](const gz::sim::UpdateInfo &, const gz::sim::EntityComponentManager &ecm) {
			const auto v = box.WorldLinearVelocity(ecm);
			const auto p = box.WorldPose(ecm);
			const auto c = canopy.WorldPose(ecm);
			if (v) {
				box_vz = v->Z();
			}
			if (p) {
				box_z = p->Pos().Z();
			}
			if (c) {
				canopy_z = c->Pos().Z();
			}
		})
		.Finalize();

	std::atomic<bool> deployed{false};
	gz::transport::Node node;
	CHECK(node.Subscribe<gz::msgs::Boolean>(
		"/test/chute_state", [&](const gz::msgs::Boolean &m) { deployed = m.data(); }));
	auto pub = node.Advertise<gz::msgs::Boolean>("/test/chute");

	/* 2 s of free fall: no drag before the command. */
	fixture.Server()->Run(true, 500, false);
	std::printf("free fall vz %.2f\n", box_vz);
	CHECK(std::fabs(box_vz + 2.0 * kGravity) < 0.5);

	for (int i = 0; (i < 50) && !pub.HasConnections(); i++) {
		sleep_ms(100);
	}
	CHECK(pub.HasConnections());
	gz::msgs::Boolean fire;
	fire.set_data(true);
	CHECK(pub.Publish(fire));
	CHECK(pub.Publish(fire)); /* a repeated command changes nothing */
	sleep_ms(200);

	/* 10 s under the canopy: close to terminal speed, canopy 2 m above. */
	fixture.Server()->Run(true, 2500, false);
	const double terminal = std::sqrt(2.0 * kMass * kGravity / (kRho * kCd * kArea));
	std::printf("canopy vz %.2f expected %.2f, canopy above box %.2f m\n", box_vz, -terminal,
		    canopy_z - box_z);
	CHECK(std::fabs(box_vz + terminal) < 0.3);
	CHECK(std::fabs(canopy_z - box_z - 2.0) < 0.3);

	for (int i = 0; (i < 20) && !deployed; i++) {
		sleep_ms(100);
	}
	CHECK(deployed);
	return CHECK_DONE();
}
