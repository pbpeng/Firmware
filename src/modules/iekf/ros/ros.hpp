#pragma once

#include <px4_posix.h>
#include "uORB/Subscription.hpp"
#include "uORB/Publication.hpp"
#include "hash/HashMap.h"

///
// ROS to uORB API translator
//

#define ROS_INFO PX4_INFO
#define ROS_WARN PX4_WARN
#define ROS_ERROR PX4_ERR
#define ROS_DEBUG PX4_DEBUG

namespace ros
{

// forward declarations
class Node;
class Rate;
class Subscriber;
class Publisher;
class Callback;
class CallbackInterface;

class Time
{
public:
	// TODO ROS uses sec (uint32_t) and nsec (uint32_t)
	Time();
	Time(uint64_t nsec);
	static Time fromNSec(uint64_t nsec);
	uint64_t toNSec();
	static Time now();
private:
	uint64_t _nsec;
};

// node for this process declared in ros.cpp
extern Node *_node;

/***
 * Check if any callbacks are ready to fire and call them
 * for this process
 */
void spin();

/**
 * Initializes node interface
 */
void init(int argc, char **argv, const char *node_name);

/**
 * There is one node per process and it's job is to run communication
 * with the rest of the system via uORB
 */
class Node
{
public:
	Node();
	void spin();
	void addSubscriber(Subscriber *sub);
	bool getTopicMeta(const char *topic, const struct orb_metadata **meta);
private:
	Subscriber *_subListHead;
	HashMap<const char *, const struct orb_metadata *> _hmap;
};

/**
 * This sleeps until a deadline to meet a
 * desired frequency
 */
class Rate
{
public:
	Rate(float frequency);
	void sleep();
private:
	float _frequency;
	uint64_t _wake_timestamp;
};

/**
 * This class represents one subscribe and contains the required
 * hooks for the callback
 */
class Subscriber
{
public:
	Subscriber();
	Subscriber(CallbackInterface *cb);
	virtual ~Subscriber();
	void callback();
	Subscriber *next;
private:
	CallbackInterface *_callbackPtr;
};

/**
 * A publisher.
 */
class Publisher
{
public:
	Publisher(uORB::PublicationTiny *pub);
	virtual ~Publisher();

	template <class T>
	void publish(const T &msg)
	{
		if (_pub != NULL) {
			_pub->update((void *)(&msg));

		} else {
			ROS_INFO("publication is NULL");
		};
	}

private:
	uORB::PublicationTiny *_pub;
};

/**
 * The abstract interface to the callback.
 */
class CallbackInterface
{
public:
	virtual void callback() = 0;
};

/**
 * The callback implementation. It contains
 * an object and a function pointre to call a member
 * function.
 */
template <class MsgType, class ObjType>
class CallbackImpl : public CallbackInterface
{
public:
	CallbackImpl(const struct orb_metadata *meta,
		     unsigned interval,
		     int instance,
		     void (ObjType::*callbackFuncPtr)(const MsgType *msg),
		     ObjType *obj) :
		_sub(meta, interval, instance),
		_callbackFuncPtr(callbackFuncPtr),
		_objPtr(obj)
	{
	}

	virtual void callback()
	{
		if (_sub.getHandle() < 0) {
			ROS_INFO("subscription handle not valid");
			return;
		}

		if (_sub.updated()) {
			MsgType msg;
			_sub.update(&msg);
			(_objPtr->*_callbackFuncPtr)(&msg);
		}
	}
private:
	uORB::SubscriptionTiny _sub;
	void (ObjType::*_callbackFuncPtr)(const MsgType *msg);
	ObjType *_objPtr;
};

/**
 * This defines the standard ROS node handle that
 * ROS users expect.
 *
 * Only difference is interval and instance and meta passed
 * instead of topic name.
 */
class NodeHandle
{
public:

	bool ok();

	void param(const char *name, float val, const char *topic);

	template <class MsgType, class ObjType>
	Subscriber subscribe(const char *topic, size_t queue_size,
			     void (ObjType::*cb)(const MsgType *msg), ObjType *obj)
	{
		const struct orb_metadata *meta = NULL;

		if (_node  == NULL) {
			ROS_ERROR("node not initialized");
			return Subscriber();
		}

		if (!_node->getTopicMeta(topic, &meta)) {
			ROS_ERROR("error, topic not found %s\n", topic);
			return Subscriber();
		}

		CallbackInterface *cb_impl = new CallbackImpl<MsgType, ObjType>(
			meta, 0, 0, cb, obj);
		return Subscriber(cb_impl);
	}

	template <class T>
	Publisher advertise(const char *topic, size_t queue_size, int priority = -1)
	{
		const struct orb_metadata *meta = NULL;

		if (_node  == NULL) {
			ROS_ERROR("node not initialized");
			return Publisher(NULL);
		}

		if (!_node->getTopicMeta(topic, &meta)) {
			ROS_ERROR("error, topic not found %s\n", topic);
			return Publisher(NULL);
		}

		return Publisher(new uORB::PublicationTiny(meta, priority));
	}

};

}
