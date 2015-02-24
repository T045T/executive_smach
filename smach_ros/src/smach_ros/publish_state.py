import roslib; roslib.load_manifest('smach_ros')
import rospy

import threading
import traceback

import smach

__all__ = ['PublishState']

class PublishState(smach.State):
    """
    A state that will publish a single message to a given ROS topic.
    The message may either be supplied directly to the constructor,
    via user data or constructed on-the-fly via a callback
    """
    def __init__(self,
                 topic,
                 msg_type,
                 msg = None,
                 msg_key = None,
                 msg_slots = [],
                 msg_cb = None,  
                 msg_cb_args = [],
                 msg_cb_kwargs = {},
                 input_keys = [],
                 output_keys=[]):
        smach.State.__init__(self,outcomes=['succeeded', 'aborted', 'preempted'], input_keys = input_keys,output_keys = output_keys)
        """
        Constructor for PublishState
        @type msg: ros msg
        @param msg: a static message to publish
        
        @type msg_key: string
        @param msg_key: a user data key containing a ready-made message object,
        
        @type msg_slots: list of string
        @param msg_slots: a number of user data keys pointing to (correctly named!!)
        values of a message (i.e. std_msgs/Bool has a 'data' field, so the UserData key
        should be 'data', and nothing else!)
        
        @type msg_cb: callable
        @param msg_cb: a callback that will construct a message object at runtime

        @type msg_cb_args: list
        @param msg_cb_args: a list of arguments to be supplied to msg_cb

        @type msg_cb_kwargs: dictionary
        @param msg_cb_kwargs: a dictionary containing keyword arguments for msg_cb
        """

        self._topic = topic
        self._msg_type = msg_type

        # Store request policy
        if msg is None:
            self._msg = msg_type()
        else:
            self._msg = msg


        if msg_cb is not None and not hasattr(msg_cb, '__call__'):
            raise smach.InvalidStateError("Message callback object given to PublishState that IS NOT a function object")

        self._msg_cb = msg_cb
        self._msg_cb_args = msg_cb_args
        self._msg_cb_kwargs = msg_cb_kwargs
        if smach.has_smach_interface(msg_cb):
            self._msg_cb_input_keys = msg_cb.get_registered_input_keys()
            self._msg_cb_output_keys = msg_cb.get_registered_output_keys()

            self.register_input_keys(self._msg_cb_input_keys)
            self.register_output_keys(self._msg_cb_output_keys)
        else:
            self._msg_cb_input_keys = input_keys
            self._msg_cb_output_keys = output_keys

        self._msg_key = msg_key
        if msg_key is not None:
            self.register_input_keys([msg_key])

        self._msg_slots = msg_slots
        self.register_input_keys(msg_slots)
        
        self._pub = rospy.Publisher(self._topic, self._msg_type, queue_size=2)

    def execute(self, ud):
        # If prempted before even getting a chance, give up.
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # Grab message key if set
        if self._msg_key is not None:
            if self._msg_key in ud:
                self._msg = ud[self._msg_key]
            else:
                rospy.logerr("Requested message key '%s' not in userdata struture. Available keys are: %s" % (self._msg_key, str(list(ud.keys()))))
                return 'aborted'

        # Write message fields from userdata if set
        for key in self._msg_slots:
            if key in ud:
                setattr(self._msg,key,ud[key])
            else:
                rospy.logerr("Requested message slot key '%s' is not in userdata strcture. Available keys are: %s" % (key, str(list(ud.keys()))))
                return 'aborted'

        # Call user-supplied callback, if set, to get a message
        if self._msg_cb is not None:
            try:
                msg_update = self._msg_cb(
                    smach.Remapper(
                        ud,
                        self._msg_cb_input_keys,
                        self._msg_cb_output_keys,
                        []),
                    self._msg,
                    *self._msg_cb_args,
                    **self._msg_cb_kwargs)
                if msg_update is not None:
                    self._msg = msg_update
            except:
                rospy.logerr("Could not execute message callback: "+traceback.format_exc())
                return 'aborted'

        if self._msg is None:
            rospy.logerr("Publish on topic "+self._topic+" with no message")
            return 'aborted'

        self._pub.publish(self._msg)

        return 'succeeded'

    def request_preempt(self):
        smach.State.request_preempt(self)
