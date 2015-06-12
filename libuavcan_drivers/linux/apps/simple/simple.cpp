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

UavcanSimple::UavcanSimple(uavcan::INode &node) :
	_sub_ctrl_cmd(node),
	_pub_derailleur_cmd(node),
	_pub_output_cmd(node),
	_sub_ambient(node)
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

	return 0;
}

typedef struct {
	uint8_t id;
	uint8_t id_mask;
	uint8_t src;
	uint8_t src_mask;
	uint8_t value_mask;
	int8_t direction;
} ctrl_decode_t;

static ctrl_decode_t ctrl_decode[] {
		{ 0, 0, 0, 0, 0x42,  1 }, /* up shift */
		{ 0, 0, 0, 0, 0x21, -1 }  /* down shift */
};

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a)  sizeof(a)/(sizeof(a[0]))
#endif

void UavcanSimple::simple_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::ctrl::Command> &msg)
{
	value = msg.value_mask;

	uavcan::equipment::derailleur::Command msg_out;

	msg_out.derailleur_id = 0x0;
	msg_out.command_type = msg_out.COMMAND_TYPE_RELATIVE;
	msg_out.command_value = 0;

	/* Iterate over decode array to determine value */
	for (size_t i = 0; i < ARRAY_SIZE(ctrl_decode); i++) {
		const ctrl_decode_t *p = &ctrl_decode[i];

		if ((msg.ctrl_id & p->id_mask) != (p->id & p->id_mask))
			continue;

		if ((msg.getSrcNodeID().get() & p->src_mask) != (p->src & p->src_mask))
			continue;

		if (msg.value_mask & p->value_mask) {
			msg_out.command_value = p->direction;
			break;
		}
	}

	if (msg_out.command_value) {
		_pub_derailleur_cmd.broadcast(msg_out);
	}
}

void UavcanSimple::ambient_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::simple::Analog> &msg)
{
	uavcan::simple::Output msg_out;

	msg_out.out[0] = msg.analog >> 4;
	msg_out.out_mask = 0x1;

	_pub_output_cmd.broadcast(msg_out);
}
