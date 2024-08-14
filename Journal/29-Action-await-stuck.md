

For action to be handled using await within a callback, it need to be in a different callback group.

A simple MutuallyExclusiveCallbackGroup is ok, even without using concurrent executor. The idea for using await is the co-routine/eventloop style will yield when awaiting for action response. I assume the default Mutually execlusive cb group lock the action client out while awaiting inside the timer callback.