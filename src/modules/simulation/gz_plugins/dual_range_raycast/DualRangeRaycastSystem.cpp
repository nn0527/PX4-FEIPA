/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/transport/Node.hh>

namespace custom
{

class DualRangeRaycastSystem final :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPostUpdate
{
public:
	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &) override
	{
		if (_sdf->HasElement("min_range")) {
			_min_range = _sdf->Get<double>("min_range");
		}

		if (_sdf->HasElement("max_range")) {
			_max_range = _sdf->Get<double>("max_range");
		}

		if (_sdf->HasElement("update_rate")) {
			_update_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
						 std::chrono::duration<double>(1.0 / _sdf->Get<double>("update_rate")));
		}

		const gz::sim::Model model(_entity);
		ConfigureChannel(_ecm, model, _sdf, 0, "front");
		ConfigureChannel(_ecm, model, _sdf, 1, "up");

		const gz::math::Vector3d front_origin = _sdf->Get<gz::math::Vector3d>("front_origin");
		const gz::math::Vector3d up_origin = _sdf->Get<gz::math::Vector3d>("up_origin");
		gz::sim::components::RaycastDataInfo data;
		data.rays.push_back({front_origin, front_origin + gz::math::Vector3d(_max_range, 0.0, 0.0)});
		data.rays.push_back({up_origin, up_origin + gz::math::Vector3d(0.0, 0.0, _max_range)});
		_ecm.CreateComponent(_entity, gz::sim::components::RaycastData(data));
		_raycast_entity = _entity;
	}

	void PostUpdate(const gz::sim::UpdateInfo &_info,
			const gz::sim::EntityComponentManager &_ecm) override
	{
		if (_info.paused || _info.simTime < _next_publish) {
			return;
		}

		_next_publish = _info.simTime + _update_period;

		const auto *raycast = _ecm.Component<gz::sim::components::RaycastData>(_raycast_entity);

		for (std::size_t index = 0; index < _channels.size(); ++index) {
			auto &channel = _channels[index];
			double distance = std::numeric_limits<double>::infinity();

			if (raycast != nullptr && raycast->Data().results.size() > index) {
				const double fraction = raycast->Data().results[index].fraction;

				if (std::isfinite(fraction) && fraction >= 0.0 && fraction < 1.0) {
					distance = fraction * _max_range;
				}
			}

			gz::msgs::LaserScan message;
			gz::msgs::Set(message.mutable_header()->mutable_stamp(), _info.simTime);
			message.set_frame(channel.frame);
			message.set_angle_min(0.0);
			message.set_angle_max(0.0);
			message.set_angle_step(0.0);
			message.set_range_min(_min_range);
			message.set_range_max(_max_range);
			message.set_count(1);
			message.set_vertical_count(1);
			message.add_ranges(distance >= _min_range ? distance : _min_range);
			message.add_intensities(std::isfinite(distance) ? 1.0 : 0.0);
			channel.publisher.Publish(message);
		}
	}

private:
	struct Channel {
		std::string frame;
		gz::transport::Node::Publisher publisher;
	};

	void ConfigureChannel(gz::sim::EntityComponentManager &_ecm,
			      const gz::sim::Model &_model,
			      const std::shared_ptr<const sdf::Element> &_sdf,
			      const std::size_t _index,
			      const std::string &_prefix)
	{
		const std::string link_element = _prefix + "_link";
		const std::string topic_element = _prefix + "_topic";
		const std::string link_name = _sdf->Get<std::string>(link_element);
		const std::string topic = _sdf->Get<std::string>(topic_element);
		const gz::sim::Entity link = _model.LinkByName(_ecm, link_name);

		if (link == gz::sim::kNullEntity) {
			gzerr << "DualRangeRaycastSystem: link [" << link_name << "] not found" << std::endl;
			return;
		}

		_channels[_index].frame = link_name;
		_channels[_index].publisher = _node.Advertise<gz::msgs::LaserScan>(topic);

		if (!_channels[_index].publisher) {
			gzerr << "DualRangeRaycastSystem: failed to advertise [" << topic << "]" << std::endl;
		}
	}

	std::array<Channel, 2> _channels{};
	gz::sim::Entity _raycast_entity{gz::sim::kNullEntity};
	gz::transport::Node _node;
	double _min_range{0.1};
	double _max_range{12.0};
	std::chrono::steady_clock::duration _update_period{std::chrono::milliseconds(33)};
	std::chrono::steady_clock::duration _next_publish{};
};

} // namespace custom

GZ_ADD_PLUGIN(
	custom::DualRangeRaycastSystem,
	gz::sim::System,
	custom::DualRangeRaycastSystem::ISystemConfigure,
	custom::DualRangeRaycastSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(custom::DualRangeRaycastSystem, "custom::DualRangeRaycastSystem")
