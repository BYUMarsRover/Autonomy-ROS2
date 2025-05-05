from std_srvs.srv import SetBool

class ServiceCaller:
    """A thread-safe helper class to manage ROS 2 service calls with state tracking.
    
    Features:
    - Tracks enabled/disabled state internally
    - Cancels pending requests only if the new command differs
    - Validates responses against configurable success/failure messages
    - Implements retry logic for unavailable services
    """
    def __init__(self, client, node, name, enable_msgs=None, disable_msgs=None, max_retries=5):
        """Initialize the service caller.
        
        Args:
            client (rclpy.Client): Initialized ROS 2 service client
            node (rclpy.Node): Parent node for logging and context
            enable_msgs (list): Response messages that indicate "enabled" state
            disable_msgs (list): Response messages that indicate "disabled" state 
            max_retries (int): Maximum retry attempts for unavailable services
        """
        # ROS 2 infrastructure
        self.client = client          # Service client (SetBool type)
        self.node = node              # Parent node for logging
        self.name = name            # Service name for logging
        
        # Configuration
        self.enable_msgs = enable_msgs or []  # Messages that enable the service
        self.disable_msgs = disable_msgs or [] # Messages that disable the service
        self.max_retries = max_retries  # Max service connection attempts
        
        # State tracking
        self.retry_count = 0         # Current retry attempt count
        self.active_future = None    # Tracks the latest service call Future
        self.is_enabled = False      # Current enabled state
        self.last_request_data = None # The last requested state (True/False)

    def toggle(self, data):
        """Main interface - sends enable/disable requests with smart cancellation.
        
        Args:
            data (bool): True to enable service, False to disable
        """
        # Check if it is already in the toggled state
        if self.is_enabled == data:
            self.node.get_logger().debug(f"{self.name} Service already in requested state: {data}")
            return

        # Check service availability first
        if self.client.service_is_ready():
            # Only proceed if there's no pending request OR the new command differs
            if self.active_future and not self.active_future.done():
                if self.last_request_data != data:
                    self.active_future.cancel()
                    self.node.get_logger().warn(
                        f"Cancelled pending {self.last_request_data} request for new {data} command on {self.name} service"
                    )
                else:
                    self.node.get_logger().debug("Duplicate request - skipping")
                    return
            
            # Send the new request
            self._send_request(data)
            self.retry_count = 0  # Reset retry counter on success
        else:
            self._handle_unavailable_service()

    def _send_request(self, data):
        """Internal method to send service requests and configure callbacks."""
        request = SetBool.Request()
        request.data = data
        
        # Store the request state before sending to avoid race conditions
        self.last_request_data = data  
        self.active_future = self.client.call_async(request)
        
        # Attach response handler with error logging
        self.active_future.add_done_callback(self._handle_response)
        
        self.node.get_logger().info(
            f"{self.name} Service request sent (enable={data})"
        )

    def _handle_response(self, future):
        """Callback that processes service responses and updates internal state.
        
        Note: Called automatically when the service call completes.
        """
        try:
            response = future.result()
            
            # Log full response for debugging
            self.node.get_logger().debug(
                f"{self.name} Service response - Success: {response.success}, "
                f"Message: '{response.message}'"
            )
            
            # Update state based on configured messages
            if response.message in self.enable_msgs:
                self.is_enabled = True
                self.node.get_logger().info(f"{self.name} Service enabled confirmed")
            elif response.message in self.disable_msgs:
                self.is_enabled = False
                self.node.get_logger().info(f"{self.name} Service disabled confirmed")
            else:
                self.node.get_logger().warning(
                    f"Unrecognized response message from {self.name} service: '{response.message}'. "
                    f"Expected enable: {self.enable_msgs}, disable: {self.disable_msgs}"
                )
                
        except Exception as e:
            # Handle service call failures (e.g., timeouts, cancellations)
            self.node.get_logger().error(
                f"{self.name} Service call failed: {str(e)}", 
                throttle_duration_sec=10  # Prevents log spam
            )
        finally:
            # Cleanup regardless of success/failure
            self.active_future = None

    def _handle_unavailable_service(self):
        """Implements retry logic when services are temporarily unavailable."""
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.node.get_logger().warn(
                f"{self.name} Service unavailable. Retry {self.retry_count}/{self.max_retries}..."
            )
        else:
            self.node.get_logger().error(
                f"{self.name} Service permanently unavailable after retries for enabling and disabling", throttle_duration_sec=10
            )
