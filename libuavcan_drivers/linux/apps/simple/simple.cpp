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
	_pub_ctrl_cmd(node)
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

	return 0;
}


void UavcanSimple::simple_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::ctrl::Command> &msg)
{
	value = msg.value_mask;

	uavcan::equipment::ctrl::Command out;

	out.ctrl_id = 0x1;
	out.value_mask = ~msg.value_mask;

	_pub_ctrl_cmd.broadcast(out);
}

