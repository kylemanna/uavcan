/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/param_server.hpp>
#include <uavcan/debug.hpp>
#include <cassert>
#include <cstdlib>

namespace uavcan
{

void ParamServer::handleGetSet(const protocol::param::GetSet::Request& in, protocol::param::GetSet::Response& out)
{
    UAVCAN_ASSERT(manager_ != NULL);

    // Recover the name from index
    if (in.name.empty())
    {
        manager_->getParamNameByIndex(in.index, out.name);
        UAVCAN_TRACE("ParamServer", "GetSet: Index %i --> '%s'", int(in.index), out.name.c_str());
    }
    else
    {
        out.name = in.name;
    }

    if (out.name.empty())
    {
        UAVCAN_TRACE("ParamServer", "GetSet: Can't resolve parameter name, index=%i", int(in.index));
        return;
    }

    // Assign if needed, read back
    if (!IParamManager::isValueEmpty(in.value))
    {
        manager_->assignParamValue(out.name, in.value);
    }
    manager_->readParamValue(out.name, out.value);

    // Check if the value is OK, otherwise reset the name to indicate that we have no idea what is it all about
    if (!IParamManager::isValueEmpty(out.value))
    {
        manager_->readParamDefaultMaxMin(out.name, out.default_value, out.max_value, out.min_value);
    }
    else
    {
        UAVCAN_TRACE("ParamServer", "GetSet: Unknown param: index=%i name='%s'", int(in.index), out.name.c_str());
        out.name.clear();
    }
}

void ParamServer::handleExecuteOpcode(const protocol::param::ExecuteOpcode::Request& in,
                                      protocol::param::ExecuteOpcode::Response& out)
{
    UAVCAN_ASSERT(manager_ != NULL);

    if (in.opcode == protocol::param::ExecuteOpcode::Request::OPCODE_SAVE)
    {
        out.ok = manager_->saveAllParams() >= 0;
    }
    else if (in.opcode == protocol::param::ExecuteOpcode::Request::OPCODE_ERASE)
    {
        out.ok = manager_->eraseAllParams() >= 0;
    }
    else
    {
        UAVCAN_TRACE("ParamServer", "ExecuteOpcode: invalid opcode %i", int(in.opcode));
        out.ok = false;
    }
}

int ParamServer::start(IParamManager* manager)
{
    if (manager == NULL)
    {
        return -ErrInvalidParam;
    }
    manager_ = manager;

    int res = get_set_srv_.start(GetSetCallback(this, &ParamServer::handleGetSet));
    if (res < 0)
    {
        return res;
    }

    res = save_erase_srv_.start(ExecuteOpcodeCallback(this, &ParamServer::handleExecuteOpcode));
    if (res < 0)
    {
        get_set_srv_.stop();
    }
    return res;
}

}
