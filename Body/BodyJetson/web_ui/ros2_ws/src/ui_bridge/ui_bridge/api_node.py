# ui_bridge/api_node.py
import threading
import queue

import uvicorn

from .config import HOST, PORT
from .ros_types import RosCmd
from .ros_bridge import ros_thread_main
from .teleop import TeleopState
from .web_app import create_app

def main() -> None:
    ros_cmd_q: "queue.Queue[RosCmd]" = queue.Queue(maxsize=300)
    teleop = TeleopState()

    # Start ROS in background thread
    t = threading.Thread(target=ros_thread_main, args=(ros_cmd_q, teleop), daemon=True)
    t.start()

    # Start FastAPI
    app = create_app(ros_cmd_q, teleop)
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")

if __name__ == "__main__":
    main()
