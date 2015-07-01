/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @author Kyle Manna <kyle@kylemanna.com>
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/time.hpp>

#include <uavcan/equipment/ctrl/Command.hpp>
#include <uavcan/equipment/derailleur/Command.hpp>
#include <uavcan/simple/Analog.hpp>
#include <uavcan/simple/Output.hpp>

class SimpleSignalOutput : uavcan::TimerBase
{
private:
	uavcan::INode 			&node;
	const uint8_t			id;
	const uint8_t			idx;
	const uint8_t			duty_night;
	const uint8_t			duty_on;
	uint8_t					last;

	bool					enabled;
	bool					flash;
	bool					night;

	uavcan::Publisher<uavcan::simple::Output> _pub_output_cmd;

public:

	SimpleSignalOutput(uavcan::INode &arg_node, uint8_t arg_id, uint8_t arg_idx, uint8_t arg_night, uint8_t on)
	: 	uavcan::TimerBase::TimerBase(arg_node),
		node(arg_node), id(arg_id), idx(arg_idx),
		duty_night(night), duty_on(on), last(0),
		enabled(false), flash(false), night(false),
		_pub_output_cmd(node)
	{
	}

	void setNight(bool newNight)
	{
		if (night == newNight)
			return;

		night = newNight;
		sendOutputCmd();
	}
	void setEnable(bool newEnable)
	{
		if (enabled == newEnable)
			return;

		enabled = newEnable;
		sendOutputCmd();
	}

	void toggleFlash() {
		if (flash) {
			stop();
		} else {
			startPeriodic(uavcan::MonotonicDuration::fromMSec(250));
		}
		flash = !flash;
		sendOutputCmd();
	}

private:
	void sendOutputCmd() {

		uint8_t val;

		uint8_t off = (night) ? duty_night : 0;

		if (flash) {
			val = (last == 0xff) ? off : 0xff;
		} else {
			val = (enabled) ? 0xff : off;
		}


		last = val;
		uavcan::simple::Output msg_out;
		msg_out.output_id = id;
		msg_out.out_mask = 1<<idx;
		msg_out.out[idx] = val;
		_pub_output_cmd.broadcast(msg_out);
	}
	void handleTimerEvent(const uavcan::TimerEvent& event) {
		(void)event;
		sendOutputCmd();
	}
};


class UavcanSimple : uavcan::Noncopyable
{
public:
	static const char *const NAME;

	UavcanSimple(uavcan::INode &node);

	const char *get_name() const { return NAME; }

	int init();

private:
	void simple_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ctrl::Command> &msg);
	void ambient_sub_cb(const uavcan::ReceivedDataStructure<uavcan::simple::Analog> &msg);

	typedef uavcan::MethodBinder < UavcanSimple *,
		void (UavcanSimple::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::ctrl::Command> &) >
		SimpleCbBinder;

	uavcan::Subscriber<uavcan::equipment::ctrl::Command, SimpleCbBinder> _sub_ctrl_cmd;
	uavcan::Publisher<uavcan::equipment::derailleur::Command>			 _pub_derailleur_cmd;

	typedef uavcan::MethodBinder < UavcanSimple *,
		void (UavcanSimple::*)
		(const uavcan::ReceivedDataStructure<uavcan::simple::Analog> &) >
		AnalogCbBinder;

	uavcan::Subscriber<uavcan::simple::Analog, AnalogCbBinder> _sub_ambient;

	typedef void (UavcanSimple::*input_handler)(const uavcan::ReceivedDataStructure<uavcan::equipment::ctrl::Command> &msg);

	typedef struct {
		uint8_t id;
		uint8_t id_mask;
		uint8_t src;
		uint8_t src_mask;
		uint8_t value_mask;
		input_handler handler;
	} ctrl_decode_t;

	enum inputs {
		UNKNOWN 		= 0,
		SHIFT_DWN 		= 0x01,
		SHIFT_UP 		= 0x02,
		SIGNAL_LEFT 	= 0x04,
		SIGNAL_RIGHT 	= 0x08,
		SIGNAL_BRAKE 	= 0x10,
		SWITCH1 		= 0x20,
		SWITCH2 		= 0x40,
	};

	UavcanSimple::ctrl_decode_t ctrl_decode[5] {
		{ 0, 0, 0, 0, SHIFT_DWN |SHIFT_UP| SWITCH1 | SWITCH2, &UavcanSimple::shift_handler },
		{ 0, 0, 0, 0, SIGNAL_LEFT | SIGNAL_RIGHT | SIGNAL_BRAKE, &UavcanSimple::signal_handler },
	};

	void shift_handler(const uavcan::ReceivedDataStructure<uavcan::equipment::ctrl::Command> &msg);
	void signal_handler(const uavcan::ReceivedDataStructure<uavcan::equipment::ctrl::Command> &msg);

	uint8_t singal_value_last;

    /**
     * Implement this method in your class to receive callbacks.
     */
    void handleTimerEvent(const uavcan::TimerEvent& event);

    /* Output state */
    SimpleSignalOutput _headlight;
    SimpleSignalOutput _sig_front_left;
    SimpleSignalOutput _sig_front_right;
    SimpleSignalOutput _sig_rear_left;
    SimpleSignalOutput _sig_rear_right;
};
