#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, render_template, send_from_directory
from flask_socketio import SocketIO
import os
from ament_index_python.packages import get_package_share_directory

class WebDashboardServer(Node):
    def __init__(self):
        super().__init__('web_dashboard_server')
        
        # Get package directory
        pkg_dir = get_package_share_directory('flexy_webdashboard')
        web_dir = os.path.join(pkg_dir, 'web')
        
        # Initialize Flask
        self.app = Flask(__name__, 
                        template_folder=web_dir,
                        static_folder=web_dir)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading')
        
        # Routes
        @self.app.route('/')
        def index():
            return send_from_directory(web_dir, 'index.html')
        
        @self.app.route('/<path:filename>')
        def serve_static(filename):
            return send_from_directory(web_dir, filename)
        
        self.get_logger().info('Web Dashboard Server initialized')
        self.get_logger().info('Access dashboard at: http://localhost:5000')
    
    def run(self):
        # Add allow_unsafe_werkzeug=True to fix the RuntimeError
        self.socketio.run(
            self.app, 
            host='0.0.0.0', 
            port=5000, 
            debug=False,
            allow_unsafe_werkzeug=True
        )

def main(args=None):
    rclpy.init(args=args)
    server = WebDashboardServer()
    
    try:
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()