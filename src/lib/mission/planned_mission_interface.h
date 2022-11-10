/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file planned_mission_interface.h
 *
 */

#pragma once

#include <stdint.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_status.h>

#include "navigator/navigation.h"

class PlannedMissionInterface
{
public:
	void update();
	void getPreviousPositionItems(int32_t start_index, struct mission_item_s items[], size_t &num_found_items,
				      uint8_t max_num_items) const;
	void getNextPositionItems(int32_t start_index, struct mission_item_s items[], size_t &num_found_items,
				  uint8_t max_num_items) const;
	bool hasMissionLandStart() const;
	int goToNextItem(bool execute_jump);
	int goToPreviousItem(bool execute_jump);
	int goToItem(int32_t index, bool execute_jump);
	int goToPreviousPositionItem(bool execute_jump);
	int goToNextPositionItem(bool execute_jump);
	int goToMissionLandStart();
	int setMissionToClosestItem(double lat, double lon, float alt, float home_alt, const vehicle_status_s &vehicle_status);
	virtual void onMissionUpdate(bool has_mission_items_changed) = 0;

	void initMission();
	void resetMission();
	void resetMissionJumpCounter();
	bool item_contains_position(const mission_item_s &item) const;
private:
	int getNonJumpItem(int32_t &mission_index, mission_item_s &mission, bool execute_jump, bool write_jumps) const;
	int setMissionIndex(int32_t index);
	int writeMission(mission_s &mission);
	int readMission(mission_s &read_mission) const;
	int readMissionItem(mission_item_s &read_mission_item, size_t index) const;
	int writeMissionItem(const mission_item_s &mission_item, size_t index) const ;
	bool isMissionValid(mission_s &mission) const;
	float get_absolute_altitude_for_item(const  mission_item_s &mission_item, float home_alt) const;
	void findLandStartItem();
	bool checkMissionWaypointsChanged(const mission_s &old_mission, const mission_s &new_mission) const;
public:
	static const uint16_t invalid_index{UINT16_MAX};
private:
	static const uint16_t max_jump_iteraion{10u};
protected:
	mission_s _mission;
	mission_item_s _current_mission_item;
	uint16_t _land_start_index;
	struct {
		double lat;
		double lon;
		float alt;
	} _land_start_pos{.lat = 0.0,
				  .lon = 0.0};

private:
	uORB::Subscription _mission_sub{ORB_ID(mission)};
	uORB::Publication<mission_s> _mission_pub{ORB_ID(mission)};
};
