#!/bin/bash

# Quick fix for flexy_webdashboard package errors

echo "Fixing flexy_webdashboard package..."

# Navigate to package directory
cd ~/ros2_ws/src/flexy_webdashboard || {
    echo "Error: Package directory not found!"
    echo "Please check if the package name is 'flexy_webdashboard' or 'flexy_web_dashboard'"
    exit 1
}

echo "Found package at: $(pwd)"

# Fix package.xml
echo "Fixing package.xml..."
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>flexy_webdashboard</name>
  <version>1.0.0</version>
  <description>Web dashboard for Flexy Robot</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

echo "✓ package.xml fixed"

# Fix setup.py
echo "Fixing setup.py..."
cat > setup.py << 'EOF'
from setuptools import setup
from glob import glob
import os

package_name = 'flexy_webdashboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
        (os.path.join('share', package_name, 'web/js'), glob('web/js/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Web dashboard for Flexy Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = flexy_webdashboard.web_server:main',
        ],
    },
)
EOF

echo "✓ setup.py fixed"

# Create necessary directories
echo "Creating directory structure..."
mkdir -p flexy_webdashboard
mkdir -p web/js
mkdir -p launch
mkdir -p resource

# Create __init__.py
touch flexy_webdashboard/__init__.py

# Create resource marker
touch resource/flexy_webdashboard

echo "✓ Directory structure created"

# Create web_server.py if it doesn't exist
if [ ! -f "flexy_webdashboard/web_server.py" ]; then
    echo "Creating web_server.py..."
    cat > flexy_webdashboard/web_server.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, send_from_directory
from flask_socketio import SocketIO
import os
from ament_index_python.packages import get_package_share_directory

class WebDashboardServer(Node):
    def __init__(self):
        super().__init__('web_dashboard_server')
        
        try:
            pkg_dir = get_package_share_directory('flexy_webdashboard')
            web_dir = os.path.join(pkg_dir, 'web')
        except:
            # Fallback to local directory during development
            web_dir = os.path.join(os.path.dirname(__file__), '..', 'web')
        
        self.app = Flask(__name__, 
                        template_folder=web_dir,
                        static_folder=web_dir)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        @self.app.route('/')
        def index():
            return send_from_directory(web_dir, 'index.html')
        
        @self.app.route('/<path:filename>')
        def serve_static(filename):
            return send_from_directory(web_dir, filename)
        
        self.get_logger().info('Web Dashboard Server initialized')
        self.get_logger().info('Access dashboard at: http://localhost:5000')
    
    def run(self):
        self.socketio.run(self.app, host='0.0.0.0', port=5000, debug=False)

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
EOF
    chmod +x flexy_webdashboard/web_server.py
    echo "✓ web_server.py created"
fi

# Create launch file if it doesn't exist
if [ ! -f "launch/web_dashboard.launch.py" ]; then
    echo "Creating launch file..."
    cat > launch/web_dashboard.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': 8080,
            'address': '0.0.0.0',
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    web_dashboard_server = Node(
        package='flexy_webdashboard',
        executable='web_server',
        name='web_dashboard_server',
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time'
        ),
        rosbridge_server,
        web_video_server,
        web_dashboard_server,
    ])
EOF
    echo "✓ Launch file created"
fi

# Create placeholder HTML if it doesn't exist
if [ ! -f "web/index.html" ]; then
    echo "Creating placeholder HTML..."
    cat > web/index.html << 'EOF'
<!DOCTYPE html>
<html>
<head>
    <title>Flexy Robot Dashboard</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #1a1a2e;
            color: white;
            padding: 50px;
            text-align: center;
        }
        h1 { color: #00f2fe; }
        pre {
            background: #0f0f0f;
            padding: 20px;
            border-radius: 10px;
            display: inline-block;
            text-align: left;
        }
    </style>
</head>
<body>
    <h1>Flexy Robot Dashboard - Setup Required</h1>
    <p>Please replace this file with the HTML from the artifact!</p>
    <p>File location:</p>
    <pre>~/ros2_ws/src/flexy_webdashboard/web/index.html</pre>
    <p>The dashboard HTML was provided in the first artifact.</p>
</body>
</html>
EOF
    echo "✓ Placeholder HTML created"
fi

echo ""
echo "════════════════════════════════════════════════"
echo "✓ Package fixed successfully!"
echo "════════════════════════════════════════════════"
echo ""
echo "Next steps:"
echo ""
echo "1. Replace the HTML file with the artifact:"
echo "   nano ~/ros2_ws/src/flexy_webdashboard/web/index.html"
echo ""
echo "2. Build the package:"
echo "   cd ~/ros2_ws"
echo "   colcon build --packages-select flexy_webdashboard"
echo "   source install/setup.bash"
echo ""
echo "3. Launch the dashboard:"
echo "   ros2 launch flexy_webdashboard web_dashboard.launch.py"
echo ""
