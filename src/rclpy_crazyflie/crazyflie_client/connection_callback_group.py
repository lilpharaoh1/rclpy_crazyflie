from rclpy.callback_groups import CallbackGroup
from std_msgs.msg import Bool

class ConnectionCallbackGroup(CallbackGroup):
    def __init__(self, name):
        CallbackGroup.__init__(self)
        self._name = name
        self.connection = False
    
    def __call__(self, connected : Bool):
        self.connection = connected.data


    def can_execute(self, entity) -> bool:
        # print("can execute : ", entity)
        return self.connection

    def beginning_execution(self, entity) -> bool:
        # print("beginning_execution : ", entity)
        return True 

    def ending_execution(self, entity) -> None:
        # print("ending_execution : ", entity)
        pass