from rclpy.callback_groups import CallbackGroup

class ConnectCallbackGroup(CallbackGroup):
    def __init__(self, connect_type):
        CallbackGroup.__init__(self)
        self.connect_type = connect_type
    
    def can_execute(self, entity) -> bool:
        print("can execute : ", entity)
        return True

    def beginning_execution(self, entity) -> bool:
        print("beginning_execution : ", entity)
        return True 

    def ending_execution(self, entity) -> None:
        print("ending_execution : ", entity)
        pass