#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace fts {

/*
 * Parachute. After a true message on <topic>, applies the drag
 * 0.5 * rho * Cd * A * |v| * v against the link's velocity relative to
 * <wind>, and keeps <canopy_model> 2 m above the link. Publishes the
 * deployed state on <state_topic> on change and once per simulated second.
 */
class Parachute : public gz::sim::System,
		  public gz::sim::ISystemConfigure,
		  public gz::sim::ISystemPreUpdate {
public:
	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
	{
		const gz::sim::Model model(entity);
		const std::string link_name = sdf->Get<std::string>("link_name", "base_link").first;

		link_ = gz::sim::Link(model.LinkByName(ecm, link_name));
		if (!link_.Valid(ecm)) {
			gzerr << "Parachute: no link [" << link_name << "]\n";
			return;
		}
		link_.EnableVelocityChecks(ecm, true);
		canopy_name_ = sdf->Get<std::string>("canopy_model", "fts_canopy").first;
		cd_ = sdf->Get<double>("cd", 1.5).first;
		area_ = sdf->Get<double>("area", 0.85).first;
		rho_ = sdf->Get<double>("air_density", 1.225).first;
		wind_ = sdf->Get<gz::math::Vector3d>("wind", gz::math::Vector3d::Zero).first;
		const std::string topic = sdf->Get<std::string>("topic", "/fts/chute").first;
		const std::string state_topic =
			sdf->Get<std::string>("state_topic", "/fts/chute_state").first;
		if (!node_.Subscribe(topic, &Parachute::OnCommand, this)) {
			gzerr << "Parachute: cannot subscribe to [" << topic << "]\n";
			return;
		}
		state_pub_ = node_.Advertise<gz::msgs::Boolean>(state_topic);
		configured_ = true;
	}

	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
	{
		if (!configured_ || info.paused) {
			return;
		}
		const bool fired = fired_.load();
		const auto now = info.simTime;

		if ((fired != published_) || (now < last_pub_) || (now - last_pub_ >= std::chrono::seconds(1))) {
			gz::msgs::Boolean m;
			m.set_data(fired);
			state_pub_.Publish(m);
			published_ = fired;
			last_pub_ = now;
		}
		if (!fired) {
			return;
		}
		const auto v = link_.WorldLinearVelocity(ecm);
		const auto pose = link_.WorldPose(ecm);
		if (!v || !pose) {
			return;
		}
		const gz::math::Vector3d rel = *v - wind_;
		link_.AddWorldForce(ecm, rel * (-0.5 * rho_ * cd_ * area_ * rel.Length()));
		if (canopy_ == gz::sim::kNullEntity) {
			canopy_ = gz::sim::World(gz::sim::worldEntity(ecm)).ModelByName(ecm, canopy_name_);
		}
		if (canopy_ != gz::sim::kNullEntity) {
			gz::sim::Model(canopy_).SetWorldPoseCmd(
				ecm, gz::math::Pose3d(pose->Pos() + gz::math::Vector3d(0.0, 0.0, 2.0),
						      gz::math::Quaterniond::Identity));
		}
	}

private:
	void OnCommand(const gz::msgs::Boolean &msg)
	{
		if (msg.data()) {
			fired_ = true;
		}
	}

	gz::sim::Link link_;
	gz::sim::Entity canopy_{gz::sim::kNullEntity};
	std::string canopy_name_;
	double cd_{1.5};
	double area_{0.85};
	double rho_{1.225};
	gz::math::Vector3d wind_;
	gz::transport::Node node_;
	gz::transport::Node::Publisher state_pub_;
	std::atomic<bool> fired_{false};
	bool published_{false};
	std::chrono::steady_clock::duration last_pub_{};
	bool configured_{false};
};

} // namespace fts

GZ_ADD_PLUGIN(fts::Parachute, gz::sim::System, fts::Parachute::ISystemConfigure,
	      fts::Parachute::ISystemPreUpdate)
