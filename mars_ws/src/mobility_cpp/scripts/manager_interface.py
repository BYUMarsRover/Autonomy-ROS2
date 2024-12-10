class Manager:
    def __init__(self, manager_name=""):
        self.manager_name = manager_name  # Instance-specific name
        self.enabled = False  # Instance-specific state

    def enable(self, msg, response=None):
        """
        Enable or disable the manager.
        
        Args:
            msg: An object with a 'data' attribute indicating enable (True) or disable (False).
            response: An optional response object to populate.

        Returns:
            Updated response object or None if no response is provided.
        """
        print(f'In {self.manager_name} enable')
        self.enabled = msg.data

        if self.enabled:
            print(f"{self.manager_name} Manager is enabled!")
        else:
            print(f"{self.manager_name} is disabled!")

        if response:
            response.success = True
            response.message = f"{self.manager_name} is {'enabled' if self.enabled else 'disabled'}!"
            return response

        # Return a simple dictionary when no ROS response is provided
        return {
            "success": True,
            "message": f"{self.manager_name} is {'enabled' if self.enabled else 'disabled'}!"
        }

        