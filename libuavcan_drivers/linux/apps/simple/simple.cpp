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

#include "simple.hpp"
#include <cmath>

const char *const UavcanSimple::NAME = "simple";

/*
 * Assume
 * output_id = 0x1 = front with headlight, signal left, signal right
 * output_id = 0x2 = rear with combined brake and signal left, signal right
 *
 *
 */

/* Board location */
#define OUT_UNDEF      0
#define OUT_FRONT      1
#define OUT_REAR       2

/* Index on board */
#define OUT_CENTER_ID  0
#define OUT_LEFT_ID    1
#define OUT_RIGHT_ID   2

UavcanSimple::UavcanSimple(uavcan::INode &node) :
	_sub_ctrl_cmd(node),
	_pub_derailleur_cmd(node),
	_sub_ambient(node),
	_front(node, OUT_FRONT),
	_rear(node, OUT_REAR)
{
}

int UavcanSimple::init()
{
	int res;

	res = _sub_ctrl_cmd.start(SimpleCbBinder(this, &UavcanSimple::simple_sub_cb));

	if (res < 0) {
		fprintf(stderr, "failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_ambient.start(AnalogCbBinder(this, &UavcanSimple::ambient_sub_cb));

	if (res < 0) {
		fprintf(stderr, "failed to start uavcan sub: %d", res);
		return res;
	}

	_front.config(OUT_CENTER_ID, 0x0, 0xff, 0xff);
	_front.config(OUT_LEFT_ID,   0x0, 0x04, 0xff);
	_front.config(OUT_RIGHT_ID,  0x0, 0x04, 0xff);

	_rear.config(OUT_LEFT_ID,    0x0, 0x20, 0xff);
	_rear.config(OUT_RIGHT_ID,   0x0, 0x20, 0xff);

	_front.sendOutputCmd(true);
	_rear.sendOutputCmd(true);

	return 0;
}

void UavcanSimple::shift_handler(const uavcan::ReceivedDataStructure<uavcan::equipment::ctrl::Command> &msg)
{
	uavcan::equipment::derailleur::Command msg_out;

	msg_out.derailleur_id = 0x0;
	msg_out.command_type = msg_out.COMMAND_TYPE_RELATIVE;
	msg_out.command_value = 0;

	if (msg.value_mask & (SHIFT_UP | SWITCH2))
		msg_out.command_value = 1;
	else if (msg.value_mask &  (SHIFT_DWN | SWITCH1))
		msg_out.command_value = -1;

	if (msg_out.command_value) {
		_pub_derailleur_cmd.broadcast(msg_out);
	}
}


void UavcanSimple::signal_handler(const uavcan::ReceivedDataStructure<uavcan::equipment::ctrl::Command> &msg)
{
	if (singal_value_last == msg.value_mask)
		return;

	if (msg.value_mask & (SIGNAL_LEFT)) {
		_front.toggleFlash(OUT_LEFT_ID);
		_rear.toggleFlash(OUT_LEFT_ID);
	}

	if (msg.value_mask & (SIGNAL_RIGHT)) {
		_front.toggleFlash(OUT_RIGHT_ID);
		_rear.toggleFlash(OUT_RIGHT_ID);
	}

	bool brake = (msg.value_mask & SIGNAL_BRAKE) ? true : false;

	_rear.setEnable(OUT_LEFT_ID, brake, true);
	_rear.setEnable(OUT_RIGHT_ID, brake, true);

	singal_value_last = msg.value_mask;
}

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a)  sizeof(a)/(sizeof(a[0]))
#endif

#define CALL_MEMBER_FN(object,ptrToMember)  ((object).*(ptrToMember))

void UavcanSimple::simple_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::ctrl::Command> &msg)
{
	/* Iterate over decode array to determine value */
	for (size_t i = 0; i < ARRAY_SIZE(ctrl_decode); i++) {
		const ctrl_decode_t *p = &ctrl_decode[i];

		if ((msg.ctrl_id & p->id_mask) != (p->id & p->id_mask))
			continue;

		if ((msg.getSrcNodeID().get() & p->src_mask) != (p->src & p->src_mask))
			continue;

		if (p->handler) {
			CALL_MEMBER_FN(*this, p->handler)(msg);
		}
	}
}

void UavcanSimple::ambient_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::simple::Analog> &msg)
{
	if (msg.analog_id == 0x1) {
		bool night = (msg.analog > 0xd00) ? true : false;

		_front.setNight(night, true);
		_rear.setNight(night, true);
	}
}


bool SimpleOutput::sendOutputCmd(bool force)
{
	bool updated = false;
	bool invert_flash_last = false;
	uavcan::simple::Output msg_out;
	msg_out.output_id = _id;

	for (unsigned i = 0; i < ARRAY_SIZE(_out); i++) {

		uint8_t val;
		uint8_t off = (_night) ? _out[i]._duty_night : _out[i]._duty_off;
		uint8_t on = _out[i]._duty_on;

		if (_out[i]._flash) {
			val = (_flash_last && _out[i]._last == on) ? off : on;
			invert_flash_last = true;
		} else {
			val = (_out[i]._enabled) ? on : off;
		}

		if (_out[i]._last != val || force) {
			updated = true;
			_out[i]._last = val;

			msg_out.out_mask |= 1<<_out[i]._idx;
			msg_out.out[_out[i]._idx] = val;
		}
	}

	if (invert_flash_last)
		_flash_last = !_flash_last;

	if (updated)
		_pub_output_cmd.broadcast(msg_out);

	return true;
}


bool SimpleOutput::setNight(bool newNight, bool sendNow)
{
	_night = newNight;

	if (sendNow) {
		return sendOutputCmd();
	}
	return false;
}


bool SimpleOutput::setEnable(uint8_t idx, bool newEnable, bool sendNow)
{
	bool updated = _out[idx].setEnable(newEnable);

	if (updated && sendNow) {
		sendOutputCmd();
	}
	return updated;
}


void SimpleOutput::toggleFlash(int idx)
{
	_out[idx].toggleFlash();

	bool flash = false;
	for (unsigned i = 0; i < ARRAY_SIZE(_out); i++) {
		if (_out[i]._flash) {
			flash = true;
			break;
		}
	}

	if (flash != _flash) {
		_flash = flash;

		if (flash) {
			startPeriodic(uavcan::MonotonicDuration::fromMSec(200));
		} else {
			stop();
		}
		sendOutputCmd();
	}
}

void SimpleOutput::handleTimerEvent(const uavcan::TimerEvent& event)
{
	(void)event;
	sendOutputCmd();
}

void SimpleOutput::config(uint8_t idx, uint8_t off, uint8_t night, uint8_t on)
{
	_out[idx].init(idx, off, night, on);
}
