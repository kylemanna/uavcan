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

class SimpleOutputSignal : uavcan::Noncopyable
{
public:
	uint8_t			_idx;
	uint8_t			_duty_off;
	uint8_t			_duty_night;
	uint8_t			_duty_on;
	uint8_t			_last;

	bool			_enabled;
	bool			_flash;

	SimpleOutputSignal()
	:	_idx(0), _duty_off(0), _duty_night(0), _duty_on(0), _last(0),
		_enabled(false), _flash(false)
	{
	}

	void init(uint8_t idx, uint8_t off, uint8_t night, uint8_t on)
	{
		_idx = idx;
		_duty_off = off;
		_duty_night = night;
		_duty_on = on;
	}

	bool setEnable(bool newEnable)
	{
		if (_enabled == newEnable)
			return false;

		_enabled = newEnable;
		return true;
	}

	bool toggleFlash() {
		_flash = !_flash;
		return true;
	}
};

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a)  (sizeof(a)/sizeof(a[0]))
#endif

class SimpleOutput : uavcan::TimerBase
{
private:
	uavcan::Publisher<uavcan::simple::Output> _pub_output_cmd;

	const uint8_t			_id;
	SimpleOutputSignal		_out[4];
	bool					_flash;
	bool					_flash_last;
	bool					_night;

public:
	SimpleOutput(uavcan::INode &node, uint8_t id)
	: 	uavcan::TimerBase::TimerBase(node), _pub_output_cmd(node),
		_id(id), _flash(false), _flash_last(false), _night(false)
	{ }

	bool sendOutputCmd(bool force);
	bool sendOutputCmd(void) { return sendOutputCmd(false); }

	void config(uint8_t idx, uint8_t off, uint8_t night, uint8_t on);
	void canIOFlags(uavcan::CanIOFlags flags);
	void handleTimerEvent(const uavcan::TimerEvent& event);
	bool getNight();
	bool setNight(bool newNight, bool sendNow);
	bool setEnable(uint8_t idx, bool newEnable, bool sendNow);
	void toggleFlash(int idx);
};

class UavcanSimple : uavcan::Noncopyable
{
public:
	static const char *const NAME;

	UavcanSimple(uavcan::INode &node);

	const char *get_name() const { return NAME; }

	int init(uavcan::CanIOFlags flags = 0);

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
    SimpleOutput _front;
    SimpleOutput _rear;
};
