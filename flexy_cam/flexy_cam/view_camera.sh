#!/bin/bash

# Flexy Robot Camera Viewer Launcher
# This script checks dependencies and launches the camera viewer

echo ""
echo "=========================================="
echo "   FLEXY ROBOT CAMERA VIEWER LAUNCHER"
echo "=========================================="
echo ""

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ Error: ROS 2 is not sourced!"
    echo "   Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "✓ ROS 2 Distribution: $ROS_DISTRO"

# Check if camera topic exists
echo "🔍 Checking for camera topic..."
if ros2 topic list | grep -q "/camera/image"; then
    echo "✓ Camera topic found: /camera/image"
else
    echo "⚠️  Warning: /camera/image topic not found!"
    echo "   Make sure your robot is spawned in Gazebo"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check for rqt_image_view
echo ""
echo "Checking available viewers..."

if command -v rqt_image_view &> /dev/null; then
    echo "✓ rqt_image_view found"
    HAS_RQT=true
else
    echo "⚠️  rqt_image_view not found"
    HAS_RQT=false
fi

# Check for Python dependencies
if python3 -c "import cv2" 2>/dev/null; then
    echo "✓ OpenCV (cv2) found"
    HAS_OPENCV=true
else
    echo "⚠️  OpenCV not found"
    HAS_OPENCV=false
fi

if python3 -c "import cv_bridge" 2>/dev/null; then
    echo "✓ cv_bridge found"
    HAS_CVBRIDGE=true
else
    echo "⚠️  cv_bridge not found"
    HAS_CVBRIDGE=false
fi

echo ""
echo "=========================================="
echo "   SELECT VIEWER METHOD"
echo "=========================================="
echo ""
echo "1) rqt_image_view (GUI - Recommended)"
echo "2) Python viewer (OpenCV - Feature-rich)"
echo "3) Install missing dependencies"
echo "4) Show diagnostics"
echo "5) Exit"
echo ""

read -p "Enter choice [1-5]: " choice

case $choice in
    1)
        if [ "$HAS_RQT" = true ]; then
            echo ""
            echo "🚀 Launching rqt_image_view..."
            echo "   Topic: /camera/image"
            echo ""
            ros2 run rqt_image_view rqt_image_view /camera/image
        else
            echo ""
            echo "❌ rqt_image_view not installed!"
            echo "   Install with: sudo apt install ros-$ROS_DISTRO-rqt-image-view"
        fi
        ;;
    2)
        if [ "$HAS_OPENCV" = true ] && [ "$HAS_CVBRIDGE" = true ]; then
            echo ""
            echo "🚀 Launching Python camera viewer..."
            echo ""
            # Check if view_camera.py exists
            if [ -f "view_camera.py" ]; then
                python3 view_camera.py
            else
                echo "❌ view_camera.py not found in current directory!"
                echo "   Please create it first."
            fi
        else
            echo ""
            echo "❌ Missing dependencies!"
            if [ "$HAS_OPENCV" = false ]; then
                echo "   - OpenCV: sudo apt install python3-opencv"
            fi
            if [ "$HAS_CVBRIDGE" = false ]; then
                echo "   - cv_bridge: sudo apt install ros-$ROS_DISTRO-cv-bridge"
            fi
        fi
        ;;
    3)
        echo ""
        echo "📦 Installing dependencies..."
        echo ""
        sudo apt update
        sudo apt install -y \
            ros-$ROS_DISTRO-rqt-image-view \
            ros-$ROS_DISTRO-cv-bridge \
            python3-opencv
        echo ""
        echo "✓ Installation complete!"
        echo "   Run this script again to launch the viewer."
        ;;
    4)
        echo ""
        echo "=========================================="
        echo "   CAMERA DIAGNOSTICS"
        echo "=========================================="
        echo ""
        echo "📋 Available topics:"
        ros2 topic list | grep -E "(camera|image)" || echo "  No camera topics found"
        echo ""
        echo "📊 Camera image info:"
        timeout 2 ros2 topic info /camera/image 2>/dev/null || echo "  Topic not available"
        echo ""
        echo "🔄 Camera image rate:"
        timeout 5 ros2 topic hz /camera/image 2>/dev/null || echo "  No messages received"
        echo ""
        ;;
    5)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid choice!"
        exit 1
        ;;
esac

echo ""
echo "✓ Done"
echo ""